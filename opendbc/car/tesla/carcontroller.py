import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.preap.carcontroller import PreAPLongController, init_preap_can
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer

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

    # --- HUMAN STEERING OVERRIDE V5 ---
    if not hasattr(self, 'hso_timer'):
      self.hso_timer = 0

    # Instantly detect ANY touch (> 0) to yield before EPAS can fight you
    blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
    if blinker_on:
        # Allow light nudges (0.8 - 2.5 Nm) to confirm lane changes without dropping lat_active.
        # Only trip HSO if the driver firmly resists or swerves (> 2.5 Nm).
        driver_pulling = abs(CS.out.steeringTorque) > 2.5 or getattr(CS, "hands_on_level", 0) >= 3
    else:
        driver_pulling = CS.out.steeringPressed or getattr(CS, "hands_on_level", 0) >= 3
    if driver_pulling:
      self.hso_timer = 50
        
    overriding = self.hso_timer > 0
    if overriding:
      self.hso_timer -= 1
        
    lat_active = CC.latActive and not overriding
    # ----------------------------------

    if self.frame % 2 == 0:
      target_angle = CS.out.steeringAngleDeg if overriding else actuators.steeringAngleDeg
      if overriding:
        self.apply_angle_last = CS.out.steeringAngleDeg

      self.apply_angle_last = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1)) # EPAS must stay powered to avoid shudder

    CS.pccEvent = None

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.preap_long.update(CC, CS, self.frame, self.tesla_can, CANBUS.party))

    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.party))
    if self.stock_cc.pcc_event:
      CS.pccEvent = self.stock_cc.pcc_event

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
