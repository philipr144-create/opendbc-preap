import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.tesla.preap.nap_conf import nap_conf, PEDAL_DI_MIN, PEDAL_DI_ZERO
from opendbc.car.tesla.preap.interface import get_preap_personality
from opendbc.car.tesla.pedal.controller import get_zero_torque
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS, CruiseButtons
from opendbc.car.carlog import carlog


def init_preap_can(dbc_names, packers):
  packers[CANBUS.autopilot_party] = CANPacker(dbc_names[Bus.party])
  tesla_can = TeslaCANPreAP(packers)
  tesla_can.pedal_can_bus = nap_conf.pedal_can_bus
  return tesla_can


class PreAPLongController:
  """Pedal-mode longitudinal: VirtualDAS, zero-torque, gas passthrough.

  Stalk-CC spoofs (CANCEL / SET_ACCEL) live in StockCCSpoofer.
  Communicates with the spoofer via CarState flags only.
  """

  def __init__(self):
    self.prev_pedal_di = 0.0
    self.prev_enable_long_control = False
    self.prev_requested_long = False
    self.prev_preap_long_active = False
    self.vdas = VirtualDAS(dt=0.02)

  def update(self, CC, CS, frame, tesla_can, can_bus_party):
    can_sends = []
    actuators = CC.actuators

    requested_long = CS.cruiseEnabled and CS.enableLongControl
    long_active = requested_long and CC.longActive
    use_pedal = nap_conf.use_pedal
    pedal_factor = float(nap_conf.pedal_factor)
    pedal_transform_valid = np.isfinite(pedal_factor) and abs(pedal_factor) > 1e-6
    pedal_long_allowed = use_pedal and pedal_transform_valid

    # For Pre-AP pedal, we use requested_long as our primary gate instead
    # of long_active. long_active is False during gas press (gasOverride
    # sets OVERRIDE_LONGITUDINAL), but we still need to track state and
    # send pedal commands for smooth transitions.
    pedal_engaged = requested_long and pedal_long_allowed

    # --- Engage transition ---
    requested_long_rising = (not self.prev_requested_long) and requested_long
    if requested_long_rising:
      zero_torque_di = get_zero_torque().get(CS.out.vEgo)
      self.prev_pedal_di = max(CS.pedal_interceptor_value, zero_torque_di)
      self.vdas.reset(a_init=float(CS.out.aEgo), pedal_di_init=self.prev_pedal_di)


    # --- Stock CC cancel triggers from pedal mode ---
    # Engage / disengage / button-press in pedal mode all need to drop stock
    # CC if it's running. Publish the request via CarState; StockCCSpoofer
    # consumes it and TXes the CANCEL frame.
    if pedal_long_allowed:
      pedal_button_press = (CS.cruise_buttons != CS.prev_cruise_buttons
                            and CS.cruise_buttons != CruiseButtons.IDLE)
      pedal_long_falling = self.prev_requested_long and not requested_long
      if requested_long_rising or pedal_long_falling or pedal_button_press:
        CS.preap_cc_cancel_needed = True

    self.prev_requested_long = requested_long
    pedal_responding = not CS.pedal_timeout

    if frame % 2 == 0:
      self.prev_enable_long_control = CS.enableLongControl

      # Update zero-torque learning
      if use_pedal:
        get_zero_torque().update(
          CS.pedal.torque_level, self.prev_pedal_di, CS.out.vEgo,
          valid=(self.prev_preap_long_active and long_active and pedal_responding
                 and getattr(CS.out, "canValid", False) and not CS.out.gasPressed
                 and not getattr(CS.out, "brakePressed", False)))

      if pedal_engaged:
        try:
          if CS.out.gasPressed:
            # Gas pressed: driver is controlling. Pass through their foot
            # directly (enable=0) but track their position for smooth resume.
            self.vdas.jerk_limiter.reset()
            self.prev_pedal_di = max(CS.pedal_interceptor_value, PEDAL_DI_ZERO)
            can_sends.append(tesla_can.create_pedal_command(0, enable=0))

          elif long_active:
            accel_request = float(actuators.accel)
            if not self.prev_preap_long_active:
              self.prev_pedal_di = float(CS.pedal_interceptor_value)
              self.vdas.reset(a_init=float(CS.out.aEgo), pedal_di_init=self.prev_pedal_di)

            self.prev_pedal_di = self.vdas.update(
              accel_request, CS.out.vEgo, self.prev_pedal_di,
              a_ego=CS.out.aEgo, freeze_integrator=False,
              orientation_ned=list(CC.orientationNED))
            if frame % 100 == 0:
              carlog.info("NAP actuator speed=%.3f personality=%d request=%.3f limited=%.3f motor=%.3f grade=%.3f pedal_di=%.3f zero_di=%.3f accel_slew=%d motor_slew=%d pedal_limit=%d pedal_slew=%d",
                          CS.out.vEgo, get_preap_personality(), accel_request,
                          self.vdas.jerk_limiter.a_limited, self.vdas.motor_accel,
                          self.vdas.grade_load, self.prev_pedal_di, get_zero_torque().get(CS.out.vEgo),
                          self.vdas.accel_slew_limited, self.vdas.motor_slew_limited,
                          self.vdas.pedal_limited, self.vdas.pedal_slew_limited)
            pedal_cmd = nap_conf.di_to_pedal(self.prev_pedal_di)
            can_sends.append(tesla_can.create_pedal_command(pedal_cmd, enable=1))

            if self.prev_pedal_di <= 0.95 * PEDAL_DI_MIN:
              CS.pccEvent = "pedalMaxRegen"

          else:
            # pedal_engaged but not long_active and not gasPressed:
            # IPC lag or transitioning. Hold at zero-torque.
            zero_torque_di = get_zero_torque().get(CS.out.vEgo)
            hold_pedal = nap_conf.di_to_pedal(zero_torque_di)
            can_sends.append(tesla_can.create_pedal_command(hold_pedal, enable=1))
            self.prev_pedal_di = zero_torque_di

        except Exception:
          carlog.exception("Pre-AP pedal command failed; sending disabled")
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          self.prev_pedal_di = 0.0

      elif use_pedal and not pedal_transform_valid:
        idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
        can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

      else:
        if use_pedal:
          idle_pedal = nap_conf.di_to_pedal(PEDAL_DI_ZERO)
          if pedal_responding:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
          elif frame % 100 == 0:
            can_sends.append(tesla_can.create_pedal_command(idle_pedal, enable=0))
        self.prev_pedal_di = 0.0

    if frame % 2 == 0:
      self.prev_preap_long_active = long_active and pedal_engaged and not CS.out.gasPressed
    return can_sends
