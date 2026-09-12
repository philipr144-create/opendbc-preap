"""One-shot Pre-AP tap requests and signal ownership. No steering tuning."""

import json
import math
import os
import time
import uuid
from collections import deque
from pathlib import Path

from opendbc.car.tesla.preap.parked_signal_test import crc8

TAP_MAX = (
  0.5  # A half-stalk gesture is classified on release, without a dwell afterwards.
)
SPEED_MIN = 40 * 0.44704
STALK_MAX_AGE = 0.15
LINK_MAX_AGE = 0.25
MANEUVER_MAX = 10.0
REQUEST_PATH = "/dev/shm/nap_tap_lane_change_request.json"
ACK_PATH = "/dev/shm/nap_tap_lane_change_ack.json"
NAV_SIGNAL_REQUEST_PATH = "/dev/shm/nap_navigation_signal_request.json"
NAV_SIGNAL_STATUS_PATH = "/dev/shm/nap_navigation_signal_status.json"


class Snapshot:
  def __init__(self, path):
    self.path = Path(path)

  def read(self, now):
    try:
      with self.path.open() as stream:
        raw = stream.read(4097)
      data = json.loads(raw) if len(raw) <= 4096 else None
      stamp = data["time"]
      if (
        type(stamp) not in (int, float)
        or not math.isfinite(stamp)
        or not 0 <= now - stamp <= LINK_MAX_AGE
      ):
        return None
      return data
    except (OSError, ValueError, TypeError, KeyError):
      return None

  def write(self, data, now):
    temp = self.path.with_name(self.path.name + ".tmp")
    try:
      temp.write_text(json.dumps(dict(data, time=now), allow_nan=False))
      os.replace(temp, self.path)
      return True
    except (OSError, ValueError, TypeError):
      return False


class PhysicalStalk:
  """Only physical bus-0 frames create gestures; Panda TX receipts use bus 128.

  Retain every edge in a CAN batch, not just CANParser's final signal value.
  A stale stream, startup with stalk held, or invalid frame cannot create a tap.
  """

  def __init__(self):
    self.raw = None
    self.time = -math.inf
    self.direction = None
    self.started = None
    self.gesture = 0
    self.events = deque(maxlen=128)
    self.sent = deque(maxlen=128)

  def remember_tx(self, frame, now):
    if frame[0] == 0x45:
      self.sent.append((now, bytes(frame[1])))

  def feed(self, packets):
    for nanos, frames in packets:
      now = nanos * 1e-9
      for address, data, bus in frames:
        if address != 0x45 or bus != 0:
          continue
        raw = bytes(data)
        # Also exclude an exact recently transmitted frame if a CAN bridge
        # reflects it onto RX bus 0. Never use TX to refresh physical freshness.
        if any(
          0 <= now - stamp <= STALK_MAX_AGE and raw == sent for stamp, sent in self.sent
        ):
          continue
        if len(raw) != 8 or crc8(raw[:7]) != raw[7] or raw[2] & 3 == 3:
          self.raw, self.direction, self.started = None, None, None
          self.events.append(("invalid", self.gesture, 0, now))
          continue
        direction = raw[2] & 3
        if not 0 <= now - self.time <= STALK_MAX_AGE:
          self.direction, self.started = None, None
          self.events.append(("invalid", self.gesture, 0, now))
        previous = self.direction
        self.raw, self.time, self.direction = raw, now, direction
        if previous is None:
          continue  # Must observe neutral before a new press can be armed.
        if direction != previous:
          if direction:
            self.gesture += 1
            self.started = now if previous == 0 and not raw[0] & 63 else None
            self.events.append(("press", self.gesture, direction, now))
          else:
            is_tap = self.started is not None and 0 < now - self.started <= TAP_MAX
            self.events.append(
              ("tap" if is_tap else "release", self.gesture, previous, now)
            )
            self.started = None
        if raw[0] & 63:
          self.started = None

  def fresh(self, now):
    return self.raw is not None and 0 <= now - self.time <= STALK_MAX_AGE


def signal_frame(raw, direction):
  """Preserve all live switch bits except indicator, counter and CRC."""
  if len(raw) != 8 or crc8(raw[:7]) != raw[7] or direction not in (0, 1, 2):
    raise ValueError("Invalid signal frame")
  if raw[0] & 63 or raw[2] & 3:
    raise ValueError("Physical cruise/indicator input has priority")
  out = bytearray(raw)
  out[2] = (out[2] & 252) | direction
  out[6] = (out[6] & 15) | ((((out[6] >> 4) + 1) % 16) << 4)
  out[7] = crc8(out[:7])
  return (0x45, bytes(out), 0)


class TapController:
  """Car-side owner. Model feedback can accept/finish only the current request."""

  def __init__(self, request_path=REQUEST_PATH, ack_path=ACK_PATH, session=None):
    self.request = Snapshot(request_path)
    self.ack = Snapshot(ack_path)
    self.session = session or uuid.uuid4().hex
    self.request_id = 0
    self.direction = 0
    self.phase = "idle"
    self.reason = "off"
    self.started = 0.0
    self.model = None
    self.eligible_press = None
    self.suppress = False
    self.held_sent = False
    self.cleanup = deque()
    self.cleanup_deadline = 0.0
    self.last_raw_time = -math.inf

  def cancel(self, reason, now, manual=False):
    if self.phase in ("pending", "active"):
      self.phase = "cancelled"
      self.reason = reason
      self.suppress = True
      if self.held_sent and not manual:
        # Release held input, issue one cancellation tap, release that tap.
        self.cleanup = deque((0, self.direction, 0))
        self.cleanup_deadline = now + LINK_MAX_AGE
      self.held_sent = False
    if manual:
      self.cleanup.clear()  # Never counteract a new physical stalk action.
    self.eligible_press = None

  def update(
    self, stalk, cs, *, enabled, lateral_active, overriding, existing=(), now=None
  ):
    now = time.monotonic() if now is None else now
    owned_at_start = self.phase in ("pending", "active") or bool(self.cleanup)
    feedback = self.ack.read(now)
    link = (
      feedback is not None
      and feedback.get("controller") == self.session
      and isinstance(feedback.get("model"), str)
      and bool(feedback["model"])
      and feedback.get("status")
      in (
        "idle",
        "waiting",
        "starting",
        "finishing",
        "complete",
        "cancelled",
        "blocked",
      )
    )
    if not hasattr(self, 'params'):
      from openpilot.common.params import Params
      self.params = Params()
    
    if self.params.get_bool("NapTapLaneChange") is False:
      enabled = False

    eligible = (
      enabled
      and lateral_active
      and not overriding
      and cs.canValid
      and stalk.fresh(now)
      and math.isfinite(cs.vEgo)
      and cs.vEgo > SPEED_MIN
      and not (cs.leftBlinker and cs.rightBlinker)
      and str(cs.gearShifter) in ("drive", "low", "sport", "eco")
      and link
    )
    if not eligible:
      self.cancel(
        "disabled, inactive, takeover, speed, CAN or model freshness gate", now
      )
    if self.phase in ("pending", "active"):
      if feedback["model"] != self.model or now - self.started >= MANEUVER_MAX:
        self.cancel("model restarted or request timed out", now)
      elif feedback.get("id") == self.request_id:
        status = feedback.get("status")
        if status in ("waiting", "starting", "finishing"):
          self.phase = "active"
        elif status in ("complete", "cancelled", "blocked"):
          self.cancel(status, now)
          if status == "complete":
            self.phase = "complete"
      elif self.phase == "active" or now - self.started > LINK_MAX_AGE:
        self.cancel("request acknowledgement lost", now)
    for event, gesture, direction, stamp in stalk.events:
      if event == "invalid":
        self.cancel("physical stalk stream interrupted", now, manual=True)
      elif event == "press":
        was_active = (
          owned_at_start or self.phase in ("pending", "active") or bool(self.cleanup)
        )
        if was_active:
          self.cancel("physical stalk cancellation", now, manual=True)
        else:
          self.suppress = False
          self.eligible_press = (
            gesture if eligible and now - stamp <= STALK_MAX_AGE else None
          )
      elif event == "tap":
        if (
          eligible
          and gesture == self.eligible_press
          and not self.cleanup
          and self.phase not in ("pending", "active")
          and now - stamp <= STALK_MAX_AGE
        ):
          self.request_id, self.direction = gesture, direction
          self.phase, self.reason = "pending", "fresh physical half-stalk tap"
          self.started, self.model = now, feedback["model"]
          self.suppress = True
        self.eligible_press = None
      elif event == "release":
        self.eligible_press = None
    stalk.events.clear()

    # A physical action always wins, including during post-maneuver cleanup.
    if (stalk.direction and self.phase in ("pending", "active")) or (
      stalk.raw is not None and stalk.raw[0] & 63
    ):
      self.cancel("physical stalk input", now, manual=True)
    if self.cleanup and now > self.cleanup_deadline:
      self.cleanup.clear()
    sends = []
    free_bus = not any(msg[0] == 0x45 for msg in existing)
    if (
      cs.canValid
      and stalk.fresh(now)
      and stalk.direction == 0
      and not stalk.raw[0] & 63
      and free_bus
      and stalk.time != self.last_raw_time
    ):
      if self.cleanup:
        sends.append(signal_frame(stalk.raw, self.cleanup.popleft()))
      elif self.phase == "active":
        sends.append(signal_frame(stalk.raw, self.direction))
        self.held_sent = True
      if sends:
        self.last_raw_time = stalk.time
    if self.phase == "active" and not free_bus:
      self.cancel("higher-priority STW sender has priority", now)
    for msg in (*existing, *sends):
      stalk.remember_tx(msg, now)
    published = self.request.write(
      {
        "controller": self.session,
        "id": self.request_id,
        "direction": self.direction,
        "phase": self.phase,
        "enabled": bool(enabled),
        "suppress": self.suppress,
        # `suppress` is a replay/cooldown latch, not signal ownership.  Publish
        # the actual synthetic-output lifetime separately so modeld cannot
        # mistake an enabled, completed, or cancelled tap for an active owner.
        "signal_active": self.phase == "active" or bool(self.cleanup),
        "gesture": stalk.gesture,
        "physical_direction": stalk.direction,
        "reason": self.reason,
      },
      now,
    )
    if not published:
      self.cancel("request publication failed", now)
      return []
    return sends


class NavigationSignalController:
  """Announce an already-authorized navigation maneuver on STW_ACTN_RQ.

  This controller never creates a model desire.  It only consumes the separate
  modeld request, and physical stalk/cruise input always wins.
  """

  def __init__(self, request_path=NAV_SIGNAL_REQUEST_PATH, status_path=NAV_SIGNAL_STATUS_PATH):
    self.request = Snapshot(request_path)
    self.status = Snapshot(status_path)
    self.direction = 0
    self.request_key = None
    self.held_sent = False
    self.cleanup = deque()
    self.cleanup_deadline = 0.0
    self.last_raw_time = -math.inf

  def _stop(self, now, manual=False):
    old_direction = self.direction
    if self.held_sent and old_direction and not manual:
      # Release the held stalk request, cancel Tesla's latched indicator, then
      # release the cancellation pulse. This is the same sequence tap uses.
      self.cleanup = deque((0, old_direction, 0))
      self.cleanup_deadline = now + LINK_MAX_AGE
    if manual:
      self.cleanup.clear()
    self.direction = 0
    self.request_key = None
    self.held_sent = False

  def update(self, stalk, cs, *, lateral_active, overriding, existing=(), now=None):
    now = time.monotonic() if now is None else now
    request = self.request.read(now)
    requested_direction = request.get("direction") if request is not None else 0
    request_key = request.get("maneuver_id") if request is not None else None
    requested = (
      request is not None
      and request.get("active") is True
      and requested_direction in (1, 2)
      and isinstance(request_key, str)
      and bool(request_key)
      and cs.canValid
      and lateral_active
      and not overriding
    )

    physical_input = (
      stalk.fresh(now)
      and stalk.raw is not None
      and (stalk.direction not in (None, 0) or bool(stalk.raw[0] & 63))
    )
    if physical_input:
      self._stop(now, manual=True)
    elif not requested:
      if self.direction:
        self._stop(now)
    elif self.direction and (requested_direction != self.direction or request_key != self.request_key):
      self._stop(now)

    if requested and not physical_input and not self.cleanup and not self.direction:
      self.direction = requested_direction
      self.request_key = request_key

    if self.cleanup and now > self.cleanup_deadline:
      self.cleanup.clear()

    sends = []
    free_bus = not any(msg[0] == 0x45 for msg in existing)
    if (
      cs.canValid
      and stalk.fresh(now)
      and stalk.direction == 0
      and stalk.raw is not None
      and not stalk.raw[0] & 63
      and free_bus
      and stalk.time != self.last_raw_time
    ):
      if self.cleanup:
        sends.append(signal_frame(stalk.raw, self.cleanup.popleft()))
      elif self.direction:
        sends.append(signal_frame(stalk.raw, self.direction))
        self.held_sent = True
      if sends:
        self.last_raw_time = stalk.time

    for msg in sends:
      stalk.remember_tx(msg, now)
    self.status.write(
      {
        "active": bool(self.direction),
        "direction": self.direction,
        "maneuver_id": self.request_key or "",
        "cleanup": bool(self.cleanup),
        "physical_override": physical_input,
        "request_valid": request is not None,
      },
      now,
    )
    return sends
