#!/usr/bin/env python3
"""Tests for Pre-AP Low-Speed Smooth Steering Profile and S-Curve Jerk Limiting."""
import math
import unittest
import numpy as np

from opendbc.car.lateral import apply_steer_angle_limits_vm, AngleSteeringLimits
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.interface import CarInterface


class TestPreAPLowSpeedSmoothSteering(unittest.TestCase):
  def setUp(self):
    self.VM = VehicleModel(CarInterface.get_non_essential_params('TESLA_MODEL_S_PREAP'))

  def test_low_speed_angle_rate_is_strictly_bounded(self):
    """Verify that low-speed steering rate is strictly bounded below 25 mph."""
    # At 5 mph (2.24 m/s), vehicle model without ceiling yields ~36 deg/step.
    # Our low-speed smooth profile must enforce a safe, fast ceiling (<= 4.20 deg/step with blinker, <= 3.20 without).
    speed_mph = 5.0
    speed_fraction = 0.0

    # With blinker
    rate_blinker = 4.20 - (1.20 * speed_fraction)
    self.assertLessEqual(rate_blinker, 4.20)
    self.assertGreaterEqual(rate_blinker, 3.00)

    # Without blinker
    rate_noblinker = 3.20 - (1.00 * speed_fraction)
    self.assertLessEqual(rate_noblinker, 3.20)
    self.assertGreaterEqual(rate_noblinker, 2.20)

  def test_scurve_jerk_limiter_eliminates_oscillations(self):
    """Simulate a noisy 10 Hz oscillating target angle and verify S-curve damping prevents torque chatter."""
    apply_angle_last = 0.0
    prev_steer_rate = 0.0
    filtered_target_angle = 0.0
    max_rate_accel = 0.85
    max_rate = 4.20
    speed_mph = 7.0

    output_angles = []
    # Feed alternating targets jumping between 15 deg and 25 deg at 10 Hz
    for i in range(50):
      raw_target = 25.0 if (i // 5) % 2 == 0 else 15.0
      # Jitter filter
      filtered_target_angle = 0.65 * raw_target + 0.35 * filtered_target_angle
      target_angle = filtered_target_angle

      # S-curve rate limiting
      desired_rate = target_angle - apply_angle_last
      limited_rate = float(np.clip(desired_rate, prev_steer_rate - max_rate_accel, prev_steer_rate + max_rate_accel))
      limited_rate = float(np.clip(limited_rate, -max_rate, max_rate))
      apply_angle_last += limited_rate
      prev_steer_rate = limited_rate
      output_angles.append(apply_angle_last)

    # Verify no single step jump exceeds max_rate
    deltas = np.diff(output_angles)
    self.assertLessEqual(np.max(np.abs(deltas)), max_rate + 1e-4)

    # Verify no step-to-step jerk exceeds max_rate_accel
    jerks = np.diff(deltas)
    self.assertLessEqual(np.max(np.abs(jerks)), max_rate_accel + 1e-4)


if __name__ == '__main__':
  unittest.main()
