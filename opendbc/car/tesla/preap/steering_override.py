"""EPAS effort evidence respecting the existing low-speed takeover allowance."""
import math
from opendbc.car.tesla.values import STEER_THRESHOLD


def takeover_torque_threshold(speed_mps):
  speed_mph = speed_mps * 2.236936
  if not math.isfinite(speed_mph):
    return STEER_THRESHOLD
  if speed_mph <= 5.0:
    return 2.50
  if speed_mph < 10.0:
    return 2.50 - ((speed_mph - 5.0) / 5.0) * 0.50
  if speed_mph < 15.0:
    return 2.00 - ((speed_mph - 10.0) / 5.0) * 0.50
  if speed_mph < 20.0:
    return 1.50 - ((speed_mph - 15.0) / 5.0) * (1.50 - STEER_THRESHOLD)
  return STEER_THRESHOLD


def hands_on_override(level, torque, threshold=STEER_THRESHOLD):
  if not math.isfinite(torque) or not math.isfinite(threshold):
    return True
  # Light EPAS contact must not bypass the existing speed-dependent allowance.
  # Stronger EPAS effort retains immediate override authority.
  return level >= 2 or (level == 1 and abs(torque) > threshold)
