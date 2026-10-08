"""EPAS effort evidence shared by vehicle state and the steering controller."""
import math
from opendbc.car.tesla.values import STEER_THRESHOLD


def hands_on_override(level, torque):
  if not math.isfinite(torque):
    return True
  # Level 1 also appears with ordinary rack/hand-contact effort. It must
  # not independently cancel a turn below the existing torque threshold.
  # Stronger EPAS levels retain immediate override authority.
  return level >= 2 or (level == 1 and abs(torque) > STEER_THRESHOLD)
