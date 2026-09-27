"""Final acceleration-command transition checks, without sending CAN."""
from unittest.mock import patch
import pytest

from opendbc.car.tesla.preap.accel_policy import ACCEL_RISE, REGEN_RELEASE_JERK, limit_regen_release
from opendbc.car.tesla.preap.virtual_das import VirtualDAS


@pytest.mark.parametrize('personality', [0,1,2])
def test_full_curve_cycle_at_final_accel_path(personality):
  with patch('opendbc.car.tesla.preap.virtual_das.get_zero_torque') as zero:
    zero.return_value.get.return_value=0.
    vdas=VirtualDAS(.02)
    for request in ([.4]*150+[-1.2]*100+[.4]*250+[-2.5]*100):
      previous=vdas.jerk_limiter.a_limited
      vdas.update(request,20.,vdas.prev_pedal_di,personality=personality)
      value=vdas.jerk_limiter.a_limited
      if request<previous:
        assert value==pytest.approx(max(request,previous-.05))
      elif previous<0.:
        assert 0. <= value-previous <= REGEN_RELEASE_JERK[personality]*.02+1e-9
      else:
        assert 0. <= value-previous <= ACCEL_RISE[personality]*.02+1e-9
    assert value==pytest.approx(-2.5)


@pytest.mark.parametrize('personality', [0,1,2])
def test_zero_crossing_and_low_speed_launch(personality):
  assert limit_regen_release(1.,-.001,personality,.02) <= ACCEL_RISE[personality]*.02
  assert limit_regen_release(-2.,-.8,personality,.02)==-2.
  vdas=VirtualDAS(.02)
  vdas.jerk_limiter.reset(-.8)
  vdas.update(.4,2.,0.,personality=personality)
  assert vdas.jerk_limiter.a_limited==pytest.approx(-.75)
