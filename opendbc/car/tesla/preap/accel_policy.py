"""Shared Pre-AP positive acceleration policy; braking is never comfort-limited."""
import numpy as np

from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES

# Positive acceleration rise, m/s^3. This state is independent of plan source.
ACCEL_RISE = {0: 0.60, 1: 0.40, 2: 0.20}


def personality_index(personality):
  return int(getattr(personality, "raw", personality))


def accel_ceiling(speed, personality):
  profile = ACCEL_PREAP_PROFILES.get(personality_index(personality), ACCEL_PREAP_PROFILES[1])
  return float(np.interp(speed, ACCEL_PREAP_BP, profile))


def limit_positive_accel(request, previous, speed, personality, dt):
  """Bound added propulsion, without delaying reduced propulsion or braking.

  Do not reset previous on lead/cruise/model changes. After a stop/override,
  seed previous with zero so the positive ramp starts gently again. Negative
  requests pass unchanged; release from braking need not wait for this ramp.
  """
  if request <= 0.0:
    return request
  rise = ACCEL_RISE.get(personality_index(personality), ACCEL_RISE[1])
  return min(request, accel_ceiling(speed, personality), max(0.0, previous) + rise * dt)
