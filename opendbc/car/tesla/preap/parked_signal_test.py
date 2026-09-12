"""Opt-in parked-only, one-frame indicator experiment; no navigation input."""
import json
import math
import os
import time
from pathlib import Path

COMMAND = Path('/dev/shm/nap_parked_signal_command.json')
STATUS = Path('/dev/shm/nap_parked_signal_status.json')

# Exact STW_ACTN_RQ layout in the inspected tesla_preap.dbc. Bit 51 is
# unmodelled. Reconstructing with zero there must match the original CRC;
# otherwise refuse the frame rather than guess that bit or any missing field.
FIELDS = [('SpdCtrlLvr_Stat', 0, 6), ('VSL_Enbl_Rq', 6, 1),
          ('SpdCtrlLvrStat_Inv', 7, 1), ('DTR_Dist_Rq', 8, 8),
          ('TurnIndLvr_Stat', 16, 2), ('HiBmLvr_Stat', 18, 2),
          ('WprWashSw_Psd', 20, 2), ('WprWash_R_Sw_Posn_V2', 22, 2),
          ('StW_Lvr_Stat', 24, 3), ('StW_Cond_Flt', 27, 1),
          ('StW_Cond_Psd', 28, 2), ('HrnSw_Psd', 30, 2)]
FIELDS += [(f'StW_Sw{i:02d}_Psd', 32+i, 1) for i in range(16)]
FIELDS += [('WprSw6Posn', 48, 3), ('MC_STW_ACTN_RQ', 52, 4),
           ('CRC_STW_ACTN_RQ', 56, 8)]


def crc8(data):
  value = 255
  for byte in data:
    value ^= byte
    for _ in range(8):
      value = ((value << 1) ^ 0x1d) & 255 if value & 128 else (value << 1) & 255
  return value ^ 255


def reconstruct(values):
  bits = 0
  for key, shift, width in FIELDS:
    v = values[key]
    if not math.isfinite(v) or int(v) != v or not 0 <= v < (1 << width):
      raise ValueError('Invalid stalk field')
    bits |= int(v) << shift
  raw = bits.to_bytes(8, 'little')
  if crc8(raw[:7]) != raw[7]:
    raise ValueError('Stalk reconstruction checksum mismatch')
  return raw


def make_signal(raw, direction):
  if direction not in ('left', 'right') or len(raw) != 8 or crc8(raw[:7]) != raw[7]:
    raise ValueError('Invalid signal frame')
  if raw[0] & 63 or raw[2] & 3:
    raise ValueError('Manual cruise or signal input')
  out = bytearray(raw)
  out[2] = (out[2] & 252) | (1 if direction == 'left' else 2)
  out[6] = (out[6] & 15) | ((((out[6] >> 4) + 1) % 16) << 4)
  out[7] = crc8(out[:7])
  return (0x45, bytes(out), 0)


class ParkedSignalTest:
  def __init__(self, command=COMMAND, status=STATUS, boot_id=None):
    self.command, self.status = Path(command), Path(status)
    try:
      self.boot_id = boot_id or Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
      self.boot_id = None  # Unavailable identity disables the experiment.
    self.last_id = None
    self.initialized = False
    self.counter = None
    self.counter_time = None
    self.consecutive = 0
    self.last_poll = -math.inf
    self.reason = 'Parked signal test is off'
    self.attempted = None
    self.last_attempt = -math.inf

  def publish(self, now):
    try:
      temp = self.status.with_suffix('.tmp')
      temp.write_text(json.dumps(dict(time=now, reason=self.reason,
                                     attempted=self.attempted, boot_id=self.boot_id)))
      os.replace(temp, self.status)
    except OSError:
      pass

  def update(self, CC, CS, existing, now=None):
    now = time.monotonic() if now is None else now
    try:
      raw = reconstruct(CS.msg_stw_actn_req)
      counter = raw[6] >> 4
      if counter != self.counter:
        if self.counter is not None and counter == (self.counter + 1) % 16:
          self.consecutive += 1
        else:
          self.consecutive = 0
        self.counter, self.counter_time = counter, now
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      raw = None
      self.consecutive = 0

    if now - self.last_poll < .1:
      return []
    self.last_poll = now
    try:
      with self.command.open() as f:
        command = json.loads(f.read(2049))
      if not isinstance(command, dict):
        raise ValueError('Invalid command')
      request = command.get('request')
      request_id = request.get('id') if isinstance(request, dict) else None
      if not hasattr(self, 'params'):
        from openpilot.common.params import Params
        self.params = Params()
      
      if self.params.get_bool("NAPParkedSignalTest") is False:
        self.reason = 'Disabled in NAP Advanced Settings'
        return []

      if not self.initialized:
        self.last_id = request_id  # Never replay a command on controller restart.
        self.initialized = True
        self.reason = 'Ready; enable the parked test and request one signal'
        return []
      if (not self.boot_id or command.get('enabled') is not True or command.get('boot_id') != self.boot_id
          or not 0 <= now - command['heartbeat'] <= .5):
        self.last_id = request_id
        self.reason = 'Parked signal test is off or its heartbeat expired'
        return []
      if not request_id or request_id == self.last_id:
        return []
      self.last_id = request_id  # A blocked request is consumed, never deferred.
      if now - self.last_attempt < 5:
        self.reason = 'Blocked: wait five seconds between tests and cancel the physical indicator'
        return []
      if not isinstance(request_id, str) or not 0 <= now - request['time'] <= 1:
        self.reason = 'Request expired'
        return []
      cs = CS.out
      if (str(cs.gearShifter) != 'park' or not cs.canValid
          or any(not math.isfinite(v) or abs(v) > .05 for v in (cs.vEgo, cs.vEgoRaw))
          or CC.enabled or CC.latActive or CC.longActive or cs.cruiseState.enabled):
        self.reason = 'Blocked: Park, zero speed, valid CAN and all controls disengaged required'
        return []
      if cs.leftBlinker or cs.rightBlinker or cs.gasPressed:
        self.reason = 'Blocked: cancel physical indicators and release accelerator first'
        return []
      if (raw is None or self.consecutive < 2 or self.counter_time is None
          or not 0 <= now - self.counter_time <= .15):
        self.reason = 'Blocked: fresh checksum-valid consecutive stalk frames required'
        return []
      if any(msg[0] == 0x45 for msg in existing):
        self.reason = 'Blocked: existing cruise stalk transmission takes priority'
        return []
      frame = make_signal(raw, request['direction'])
      self.last_attempt = now
      self.attempted = dict(id=request_id, direction=request['direction'], time=now)
      self.reason = 'One frame queued; physical operation UNVERIFIED. Cancel using the physical signal stalk.'
      return [frame]
    except (OSError, ValueError, KeyError, TypeError, AttributeError, OverflowError):
      self.reason = 'Parked test unavailable or request invalid'
      self.initialized = True
      return []
    finally:
      self.publish(now)
