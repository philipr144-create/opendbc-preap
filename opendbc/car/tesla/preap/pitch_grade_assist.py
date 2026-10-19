"""NAP Unified Pitch Assist V4.

OFF and MONITOR never change motor commands. ON requires independent sign
validation, a new engagement, and an authoritative planner permission gate.
This class does not perform filesystem/Params I/O on the 50 Hz control path.
"""
import json
import math
import threading
from pathlib import Path

PARAM_NAME = "NAPPitchAssistMode"
VERIFICATION_FILE = Path("/data/nap_settings.json")
MODE_NAMES = ("off", "monitor", "on")
G = 9.81
LP_TAU = 1.5
DEADZONE = .02  # exceed measured reference residual before adding support
MAX_SUPPORT = .10       # m/s^2, uphill only
MAX_SLEW_UP = .04       # m/s^3
MAX_SLEW_DOWN = .08     # m/s^3
MIN_SPEED = 8.0        # ~18 mph
MAX_STEADY_ACCEL = .12
SETTLE_SECONDS = 4.0
POLL_SECONDS = .25


def bound(x, lo, hi):
  return max(lo, min(x, hi))


class PitchGradeAssist:
  def __init__(self, dt=.02, params=None, verification_file=VERIFICATION_FILE,
               start_watcher=True):
    self.dt = float(dt)
    self.params = params
    self.verification_file = Path(verification_file)
    # A single tuple assignment makes settings atomic to the control thread.
    self._settings = ("off", False, 0.0)
    self.mode = "off"
    self.verified = False
    self.reference_offset = 0.0
    self.armed = False
    self.tick = 0
    self.pitch_rad = None
    self.stable_s = 0.0
    self.road_grade_mps2 = 0.0
    self.candidate_mps2 = 0.0
    self.applied_mps2 = 0.0
    self._stop = threading.Event()
    if start_watcher:
      # No synchronous disk/Params access in driving update() or reset().
      threading.Thread(target=self._watch_settings, name="nap_pitch_settings",
                       daemon=True).start()

  def _watch_settings(self):
    while not self._stop.is_set():
      try:
        self.refresh_settings()
      except Exception:
        # Failed watcher must never leave a previously ON mode armed.
        self._settings = ("off", False, 0.0)
      self._stop.wait(POLL_SECONDS)

  def close(self):
    self._stop.set()

  def refresh_settings(self):
    """Never invoke from a 50 Hz control callback. A failed read disarms."""
    try:
      if self.params is None:
        from openpilot.common.params import Params
        self.params = Params()
      value = int(self.params.get(PARAM_NAME, return_default=True))
      mode = MODE_NAMES[value] if value in (0, 1, 2) else "off"
    except (ImportError, AttributeError, TypeError, ValueError, OSError, KeyError):
      mode = "off"
    verified = False
    reference_offset = 0.0
    if mode != "off":
      try:
        config = json.loads(self.verification_file.read_text())
        reference = config.get("pitch_reference_offset_rad") if isinstance(config, dict) else None
        reference_valid = (type(reference) in (int, float) and math.isfinite(reference) and abs(reference) <= .08)
        verified = (isinstance(config, dict) and config.get("pitch_assist_verified") is True
                    and reference_valid)
        if reference_valid:
          reference_offset = float(reference)
      except (OSError, TypeError, ValueError):
        pass
    self._settings = (mode, verified, reference_offset)

  def reset(self):
    # Always fail closed if watcher has not completed its initial read.
    mode, verified, reference_offset = self._settings
    self.mode = mode
    self.verified = verified
    self.reference_offset = reference_offset
    self.armed = mode == "on" and verified
    self.pitch_rad = None
    self.stable_s = 0.0
    self.road_grade_mps2 = 0.0
    self.candidate_mps2 = 0.0
    self.applied_mps2 = 0.0

  def update(self, orientation_ned, v_ego, accel_cmd, allowed=False):
    self.tick += 1
    mode, verified, reference_offset = self._settings
    if mode != self.mode or verified != self.verified or reference_offset != self.reference_offset:
      # ON selected during a drive cannot arm until the *next* reset.
      if mode != "on" or not verified or self.mode != "on" or not self.verified or reference_offset != self.reference_offset:
        self.armed = False
        self.applied_mps2 = 0.0
      self.mode, self.verified, self.reference_offset = mode, verified, reference_offset

    try:
      pitch = float(orientation_ned[1])
      valid = math.isfinite(pitch) and abs(pitch) < .30
    except (IndexError, ValueError, TypeError):
      pitch, valid = 0.0, False
    if not valid:
      self.pitch_rad = None
      self.stable_s = 0.0
      self.road_grade_mps2 = 0.0
      self.candidate_mps2 = 0.0
      self.applied_mps2 = 0.0
      return 0.0

    pitch -= self.reference_offset
    if self.pitch_rad is None:
      self.pitch_rad = pitch
    else:
      self.pitch_rad += (self.dt / (LP_TAU + self.dt)) * (pitch - self.pitch_rad)
    self.stable_s += self.dt
    self.road_grade_mps2 = G * math.sin(self.pitch_rad)
    effective_uphill = max(0.0, self.pitch_rad - DEADZONE)
    self.candidate_mps2 = bound(G * math.sin(effective_uphill), 0.0, MAX_SUPPORT)

    steady = (math.isfinite(v_ego) and math.isfinite(accel_cmd)
              and v_ego >= MIN_SPEED and 0.0 <= accel_cmd <= MAX_STEADY_ACCEL
              and self.stable_s >= SETTLE_SECONDS)
    active = mode == "on" and verified and self.armed and allowed and steady
    if not active:
      self.applied_mps2 = 0.0
    else:
      self.applied_mps2 += bound(self.candidate_mps2 - self.applied_mps2,
                                -MAX_SLEW_DOWN * self.dt, MAX_SLEW_UP * self.dt)

    if self.tick % 100 == 0 and mode != "off":
      try:
        from opendbc.car.carlog import carlog
        carlog.info(
          "NAP_PITCH_V4 mode=%s verified=%d armed=%d allow=%d "
          "pitch=%.5f grade=%.4f proposed=%.4f applied=%.4f a_cmd=%.3f",
          mode, verified, self.armed, bool(allowed),
          self.pitch_rad, self.road_grade_mps2, self.candidate_mps2,
          self.applied_mps2, accel_cmd)
      except (ImportError, AttributeError):
        pass
    return self.applied_mps2


class OpenRoadPitchPermission:
  """Independent, fresh longitudinal-plan gate; missing data => no support.

  HUD lead state is an additional denial only, never authorization.
  """
  def __init__(self):
    self.sm = None
    self.cruise_source = None
    try:
      from cereal import messaging, log, car
      self.sm = messaging.SubMaster(['longitudinalPlan', 'modelV2'])
      self.cruise_source = log.LongitudinalPlan.LongitudinalPlanSource.cruise
      self.pid_state = car.CarControl.Actuators.LongControlState.pid
    except Exception:
      self.sm = None

  def allowed(self, CC, CS):
    if self.sm is None:
      return False
    try:
      import time
      self.sm.update(0)  # explicitly nonblocking; never wait for IPC
      if not self.sm.all_checks(['longitudinalPlan', 'modelV2']):
        return False
      times = self.sm.rcv_time
      now = time.monotonic()
      if any(not math.isfinite(times[n]) or now - times[n] > .5 or now < times[n]
             for n in ('longitudinalPlan', 'modelV2')):
        return False
      plan = self.sm['longitudinalPlan']
      model = self.sm['modelV2']
      if (plan.hasLead or plan.shouldStop or not plan.allowThrottle
          or plan.longitudinalPlanSource != self.cruise_source
          or model.action.shouldStop):
        return False
      if (not CC.longActive
          or CC.actuators.longControlState != self.pid_state
          or CC.hudControl.leadVisible
          or CS.out.gasPressed or CS.out.brakePressed
          or CS.out.leftBlinker or CS.out.rightBlinker):
        return False
      # Reject turns, including where the cruise source is still 'cruise'.
      # Do not override curvature/turn acceleration limits.
      if (not math.isfinite(CC.currentCurvature)
          or abs(CC.currentCurvature) > .0008
          or not math.isfinite(CS.out.steeringAngleDeg)
          or abs(CS.out.steeringAngleDeg) > 4.0):
        return False
      return True
    except Exception:
      return False
