#!/usr/bin/env python3
"""Pre-AP carstate update tests.

Regression coverage for the class of bug where update_preap writes a field on
`ret` (the CarState capnp struct) without a matching schema entry in car.capnp.
These writes look like ordinary Python assignment but silently require the
schema to agree; the first update call crashes card with AttributeError, which
leaves the panda in elm327 safe mode and surfaces as 'Unknown Vehicle Variant'
(canError) in the UI.

See vault/lessons/agent-failure-modes/capnp-schema-write-without-field.md
and the regression report d0cdc986c5d023f5|4a5ffc1c21 (2026-04-20).
"""
import unittest

from opendbc.car.car_helpers import interfaces


class TestPreAPCarStateUpdate(unittest.TestCase):

  def _make_interface(self):
    CarInterface = interfaces["TESLA_MODEL_S_PREAP"]
    CP = CarInterface.get_params("TESLA_MODEL_S_PREAP",
                                 {i: {} for i in range(8)},
                                 [],
                                 alpha_long=False, is_release=False, docs=False)
    return CarInterface(CP)

  def test_update_runs_without_crashing(self):
    """update() with empty CAN must not raise — exercises every ret.X write path."""
    CI = self._make_interface()
    # Ten iterations; mirrors upstream test_car_interfaces pattern and catches
    # issues that only appear after state has accumulated.
    for _ in range(10):
      CI.update([])

  def test_nap_specific_fields_on_carstate(self):
    """NAP-specific booleans written by update_preap must exist on the schema."""
    CI = self._make_interface()
    CS = CI.update([])
    for field in ("teslaCCEngaged", "teslaCCDisengaged", "teslaCCNotArmed",
                  "pedalMaxRegen", "pedalLongActive"):
      self.assertTrue(hasattr(CS, field), f"CarState schema missing {field}")


if __name__ == "__main__":
  unittest.main()


def test_city_allowance_uses_blinkers_in_complete_state_update():
 from unittest.mock import patch
 from opendbc.car import Bus
 from opendbc.car.tesla.preap.carstate import update_preap
 ci=TestPreAPCarStateUpdate()._make_interface()
 ci.update([])
 chassis=ci.can_parsers[Bus.chassis]
 chassis.vl['GTW_carState']['BC_indicatorRStatus']=1
 chassis.vl['GTW_carState']['BC_indicatorLStatus']=0
 chassis.vl['EPAS_sysStatus']['EPAS_handsOnLevel']=0
 chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=1.75
 with patch.object(ci.CS,'update_speed_kf',return_value=(6.,0.)):
  for _ in range(5):state=update_preap(ci.CS,ci.can_parsers)
  assert state.rightBlinker and not state.leftBlinker
  assert not state.steeringPressed
  # Recorded turn effort must not cancel the actual initialized state.
  chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=4.1
  for _ in range(5):state=update_preap(ci.CS,ci.can_parsers)
  assert not state.steeringPressed
  chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=5.1
  for _ in range(5):state=update_preap(ci.CS,ci.can_parsers)
  assert state.steeringPressed
  chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=.1
  for _ in range(5):update_preap(ci.CS,ci.can_parsers)
  chassis.vl['GTW_carState']['BC_indicatorRStatus']=0
  chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=2.
  for _ in range(5):state=update_preap(ci.CS,ci.can_parsers)
  assert not state.rightBlinker and not state.steeringPressed
  chassis.vl['EPAS_sysStatus']['EPAS_torsionBarTorque']=5.1
  for _ in range(5):state=update_preap(ci.CS,ci.can_parsers)
  assert state.steeringPressed


def test_full_state_update_restored_filter_and_low_speed_allowance():
 from unittest.mock import patch
 from opendbc.car import Bus
 from opendbc.car.tesla.preap.carstate import update_preap
 from opendbc.car.tesla.preap.steering_override import takeover_torque_threshold
 ci=TestPreAPCarStateUpdate()._make_interface();ci.update([])
 chassis=ci.can_parsers[Bus.chassis];epas=chassis.vl['EPAS_sysStatus']
 for mph, effort in [(13.3,4.1),(27.5,1.09),(27.5,1.36),(27.,1.26),(65.,1.36)]:
  for signal in [0,1]:
   for sign in [-1,1]:
    chassis.vl['GTW_carState']['BC_indicatorLStatus']=signal
    chassis.vl['GTW_carState']['BC_indicatorRStatus']=0
    ci.CS.steering_pressed_cnt=0;epas['EPAS_handsOnLevel']=2
    with patch.object(ci.CS,'update_speed_kf',return_value=(mph/2.236936,0.)):
     epas['EPAS_torsionBarTorque']=sign*effort
     for _ in range(10):state=update_preap(ci.CS,ci.can_parsers)
     assert not state.steeringPressed, (mph,signal,effort)
     ci.CS.steering_pressed_cnt=0
     threshold=takeover_torque_threshold(mph/2.236936,city_turn=bool(signal))
     epas['EPAS_torsionBarTorque']=sign*(threshold+.1)
     for _ in range(51):state=update_preap(ci.CS,ci.can_parsers)
     assert state.steeringPressed, (mph,signal)
     epas['EPAS_torsionBarTorque']=0.
     for _ in range(105):state=update_preap(ci.CS,ci.can_parsers)
     assert not state.steeringPressed
