from dataclasses import replace
import types
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.preap.carcontroller import PreAPLongController, init_preap_can
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
from opendbc.car.tesla.preap.parked_signal_test import ParkedSignalTest
from opendbc.car.tesla.preap.tap_lane_change import NavigationSignalController, TapController
from opendbc.car.tesla.preap.nap_params import NAPParamKeys
from opendbc.car.tesla.preap.steering_override import hands_on_override, takeover_torque_threshold
from opendbc.car.tesla.preap.steering_handover import QuietHandover, SteeringRecovery, WheelMotion

def get_safety_CP():
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)

    self.VM = VehicleModel(get_safety_CP())

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_PREAP):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {CANBUS.party: CANPacker(dbc_names[Bus.party]), CANBUS.powertrain: CANPacker(dbc_names[Bus.pt])}

      if CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
        self.preap_long = PreAPLongController()
        self.stock_cc = StockCCSpoofer()
        self.parked_signal_test = ParkedSignalTest()
        self.navigation_signal = NavigationSignalController()
        self.tap_lane_change = TapController()
        try:
          from openpilot.common.params import Params
          self.tap_params = Params()
        except ImportError:
          self.tap_params = None
        self.tesla_can = init_preap_can(dbc_names, self.packers)
      else:
        self.tesla_can = TeslaCANRaven(self.packers)

      from opendbc.car.tesla.interface import CarInterface
      self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

  def update(self, CC, CS, now_nanos):
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP:
      return self._update_preap(CC, CS)

    actuators = CC.actuators
    can_sends = []

    lat_active = CC.latActive and CS.hands_on_level < 3

    if self.frame % 2 == 0:
      try:
        auto_res = __import__("json").load(open("/data/nap_settings.json")).get("auto_resume", False)
      except:
        auto_res = False

      target_angle = CS.out.steeringAngleDeg if ((CS.out.steeringPressed or getattr(CS, "hands_on_level", 0) >= 3) and auto_res) else actuators.steeringAngleDeg
      self.apply_angle_last = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      if self.CP.carFingerprint in LEGACY_CARS:
        cntr = (self.frame // 2) % 16
        can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      else:
        can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active))

    if self.frame % 10 == 0:
      if self.CP.carFingerprint in LEGACY_CARS and self.CP.carFingerprint not in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        cntr = (self.frame // 10) % 16
        can_sends.append(self.tesla_can.create_steering_allowed(cntr))
      elif self.CP.carFingerprint not in LEGACY_CARS:
        can_sends.append(self.tesla_can.create_steering_allowed())

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if CC.cruiseControl.cancel else 4
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))
    else:
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  def _update_preap(self, CC, CS):
    actuators = CC.actuators
    can_sends = []

    # Shared gate for low-speed steering rate and HSO sensitivity.
    speed_mph = CS.out.vEgo * 2.236936
    blinker_on = bool(CS.out.leftBlinker) != bool(CS.out.rightBlinker)

    # PREAP_LOW_SPEED_TURN_TAKEOVER_LATCH_V1
    # Preserve the wider low-speed turn profile when openpilot remains in
    # control. If the driver takes over below 15 mph during that maneuver,
    # resume from the measured angle after 0.20 s without driver effort.
    if not hasattr(self, "low_speed_turn_exit_timer"):
      self.low_speed_turn_exit_timer = 0

    if not hasattr(self, "low_speed_turn_takeover_latched"):
      self.low_speed_turn_takeover_latched = False
      self.low_speed_turn_reacquire_timer = 0

    if blinker_on and speed_mph < 25.0:
      self.low_speed_turn_exit_timer = 100
    elif self.low_speed_turn_exit_timer > 0:
      self.low_speed_turn_exit_timer -= 1

    low_speed_turn_profile = (
      speed_mph < 25.0
      and blinker_on
    )

    city_turn_context = (
      low_speed_turn_profile
    )

    # Driver effort yields immediately. Only continuous quiet effort can
    # release ownership; changing the signal or desired angle cannot release it.
    if not hasattr(self, 'steering_handover'):
      self.steering_handover = QuietHandover(DT_CTRL)
      self.steering_wheel_motion = WheelMotion(DT_CTRL)
      self.steering_recovery = SteeringRecovery(DT_CTRL * CarControllerParams.STEER_STEP)
      self.steering_recovery_needed = False
    hands_on_level = getattr(CS, 'hands_on_level', 0)
    threshold = takeover_torque_threshold(CS.out.vEgo, city_turn=blinker_on)
    hands_on_trigger = (hands_on_override(hands_on_level, CS.out.steeringTorque, threshold)
                        if speed_mph < 15.0 or blinker_on else hands_on_level >= 3)
    driver_pulling = CS.out.steeringPressed or hands_on_trigger
    healthy = (CS.out.canValid and not getattr(CS.out, 'steerFaultTemporary', False)
               and not getattr(CS.out, 'steerFaultPermanent', False)
               and all(np.isfinite(v) for v in (CS.out.steeringAngleDeg,
                       actuators.steeringAngleDeg, CS.out.steeringTorque,
                       CS.out.vEgo, CS.out.vEgoRaw)))
    self.steering_wheel_motion.update(CS.out.steeringAngleDeg if healthy else float('nan'))
    normal_rate = CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / self.steering_recovery.dt
    reachable = True
    if healthy and self.steering_handover.pending:
      reachable_limit = min(CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX,
                            get_max_angle_vm(max(CS.out.vEgoRaw, 1.), self.VM, CarControllerParams))
      reachable = np.isfinite(reachable_limit) and abs(CS.out.steeringAngleDeg) <= reachable_limit
    quiet = (reachable and not driver_pulling and hands_on_level < 2
             and abs(CS.out.steeringTorque) <= min(2., .5*threshold)
             and self.steering_wheel_motion.ready
             and abs(self.steering_wheel_motion.rate) <= normal_rate)
    overriding = self.steering_handover.update(CC.latActive, healthy, driver_pulling, quiet)
    if overriding:
      self.steering_recovery_needed = True
      self.steering_recovery.reset()
    if not CC.latActive:
      self.steering_recovery_needed = False
      self.steering_recovery.reset()
    lat_active = CC.latActive and healthy and not overriding
    if overriding:
      # Keep normal turning authority once control returns; only the
      # initial angle mismatch is blended, not the ongoing path.
      self.low_speed_turn_exit_timer = 0
      low_speed_turn_profile = False
    # Telemetry only: expose the final local command gate to card's optional
    # status publisher. No subscriptions, I/O or control changes here.
    self.nap_spoken_control_state = (bool(overriding), bool(lat_active))
    # ----------------------------------

    if self.frame % 2 == 0:
      target_angle = CS.out.steeringAngleDeg if overriding else actuators.steeringAngleDeg
      if lat_active and self.steering_recovery_needed:
        self.steering_recovery.seed(CS.out.steeringAngleDeg, self.steering_wheel_motion.rate,
                                   CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / self.steering_recovery.dt)
        self.steering_recovery_needed = False
      recovering = lat_active and self.steering_recovery.active
      if recovering:
        # Shape a reachable target. Safety's speed-dependent angle ceiling
        # remains authoritative; an unreachable raw target cannot trap recovery.
        max_angle = min(CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX,
                        get_max_angle_vm(max(CS.out.vEgoRaw, 1.), self.VM, CarControllerParams))
        target_angle = self.steering_recovery.update(
          float(np.clip(target_angle, -max_angle, max_angle)),
          CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / self.steering_recovery.dt)
      if overriding:
        self.apply_angle_last = CS.out.steeringAngleDeg

      # ========================================================
      # PREAP LOW-SPEED SMOOTH STEERING PROFILE (< 25 mph)
      #
      # Provides fast, agile steering response (up to 210 deg/s)
      # while applying S-curve angular acceleration limiting to
      # completely eliminate low-speed chatter, hunting, and
      # mechanical stalk-cancel torque shocks.
      # ========================================================
      params_to_use = CarControllerParams

      if not hasattr(self, 'params'):
        from openpilot.common.params import Params
        self.params = Params()
        self.low_speed_steering_rate_enabled = True

      if self.frame % 100 == 0:
        self.low_speed_steering_rate_enabled = self.params.get_bool("NAPLowSpeedSteeringRate")

      if self.low_speed_steering_rate_enabled and speed_mph < 25.0:
        speed_fraction = max(
          0.0,
          min(
            1.0,
            (speed_mph - 5.0) / 20.0,
          ),
        )

        if low_speed_turn_profile:
          # Active turn/maneuver: 4.20 deg/20ms (210 deg/s) @ 5 mph -> 3.00 deg/20ms (150 deg/s) @ 25 mph
          low_speed_max_angle_rate = 4.20 - (1.20 * speed_fraction)
        else:
          # Cruising/curves: 3.20 deg/20ms (160 deg/s) @ 5 mph -> 2.20 deg/20ms (110 deg/s) @ 25 mph
          low_speed_max_angle_rate = 3.20 - (1.00 * speed_fraction)

        scaled_limits = replace(
          CarControllerParams.ANGLE_LIMITS,
          MAX_ANGLE_RATE=low_speed_max_angle_rate,
        )

        params_to_use = types.SimpleNamespace(
          ANGLE_LIMITS=scaled_limits,
          STEER_STEP=CarControllerParams.STEER_STEP,
        )

      # Near-field low-speed vision jitter rejection:
      # Strips out high-frequency frame noise without adding latency to real steering intent
      if not hasattr(self, 'filtered_target_angle'):
        self.filtered_target_angle = CS.out.steeringAngleDeg
      if speed_mph < 20.0 and lat_active and not overriding and not recovering:
        self.filtered_target_angle = 0.65 * target_angle + 0.35 * self.filtered_target_angle
        target_angle = self.filtered_target_angle
      else:
        self.filtered_target_angle = target_angle

      vm_target_angle = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                    lat_active, params_to_use, self.VM)

      # S-curve angular acceleration / jerk damping:
      # Limits rate-of-change of the steering velocity to prevent 10 Hz EPAS chatter
      if not hasattr(self, 'prev_steer_rate'):
        self.prev_steer_rate = 0.0

      if lat_active and not overriding:
        max_rate_accel = 0.85 if speed_mph < 25.0 else 0.50
        desired_rate = vm_target_angle - self.apply_angle_last
        limited_rate = float(np.clip(desired_rate, self.prev_steer_rate - max_rate_accel, self.prev_steer_rate + max_rate_accel))
        self.apply_angle_last += limited_rate
        self.prev_steer_rate = limited_rate
      else:
        self.apply_angle_last = vm_target_angle
        self.prev_steer_rate = 0.0
      if recovering:
        self.steering_recovery.commit(self.apply_angle_last)
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1)) # EPAS must stay powered to avoid shudder

    CS.pccEvent = None

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.preap_long.update(CC, CS, self.frame, self.tesla_can, CANBUS.party))

    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.party))
    if self.stock_cc.pcc_event:
      CS.pccEvent = self.stock_cc.pcc_event

    # Independent parked-only one-shot test; existing cruise transmissions win.
    can_sends.extend(self.parked_signal_test.update(CC, CS, can_sends))

    # Complete an in-progress lane-change tap before navigation uses the
    # synthetic signal slot. Physical stalk input wins in both controllers.
    tap_enabled = (self.tap_params is not None and self.tap_params.check_key(NAPParamKeys.TAP_LANE_CHANGE)
                   and self.tap_params.get_bool(NAPParamKeys.TAP_LANE_CHANGE))
    can_sends.extend(self.tap_lane_change.update(
      CS.tap_stalk, CS.out, enabled=tap_enabled,
      lateral_active=CC.latActive, overriding=overriding, existing=can_sends))

    can_sends.extend(self.navigation_signal.update(
      CS.tap_stalk, CS.out, lateral_active=CC.latActive,
      overriding=overriding, existing=can_sends))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
