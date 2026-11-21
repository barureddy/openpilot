import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.jve_acc_control = False
    # jvePilot: driver's max speed from the last engagement, restored when re-engaging with RES+
    self.jve_v_cruise_max = V_CRUISE_UNSET
    self.jve_resume_requested = False
    self.jve_engaged_prev = False
    self.jve_car_speed_prev = V_CRUISE_UNSET
    self.jve_driver_btn_frames = 0
    self.jve_driver_btn_type = None
    self.jve_button_synced = False
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  @property
  def jve_active(self) -> bool:
    return (self.jve_acc_control and self.CP.brand == "chrysler" and self.CP.pcmCruise and
            not self.CP.openpilotLongitudinalControl)

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      elif self.jve_active:
        # jvePilot hybrid ACC: while engaged, track driver's desired max cruise speed via physical
        # steering wheel buttons so CarController can modulate the car's actual ACC set speed below v_cruise_kph
        if not enabled or not CS.cruiseState.enabled or not self.v_cruise_initialized:
          # Standby/disengaged: remember whether the driver resumes (RES+) or sets a new speed (SET-)
          for b in CS.buttonEvents:
            if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise):
              self.jve_resume_requested = True
              self.jve_driver_btn_frames = 30
              self.jve_driver_btn_type = b.type
            elif b.type in (ButtonType.decelCruise, ButtonType.setCruise):
              self.jve_resume_requested = False
              self.jve_driver_btn_frames = 30
              self.jve_driver_btn_type = b.type
          self.jve_engaged_prev = False
          self.jve_button_synced = False
          self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
          if CS.cruiseState.speed == 0:
            self.v_cruise_kph = V_CRUISE_UNSET
            self.v_cruise_cluster_kph = V_CRUISE_UNSET
          elif CS.cruiseState.speed == -1:
            self.v_cruise_kph = -1
            self.v_cruise_cluster_kph = -1
          self.jve_car_speed_prev = self.v_cruise_kph
          self.update_button_timers(CS, enabled)
        else:
          car_v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
          for b in CS.buttonEvents:
            if b.type in (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.setCruise, ButtonType.resumeCruise):
              self.jve_driver_btn_frames = 30
              self.jve_driver_btn_type = b.type
              if b.pressed:
                self.jve_button_synced = False
          if any(t > 0 for t in self.button_timers.values()):
            self.jve_driver_btn_frames = 30

          car_speed_changed = (self.jve_car_speed_prev != V_CRUISE_UNSET and
                               abs(car_v_cruise_kph - self.jve_car_speed_prev) > 0.1)
          if car_speed_changed and self.jve_driver_btn_frames > 0 and not CS.gasPressed:
            if (self.jve_driver_btn_type in (ButtonType.decelCruise, ButtonType.setCruise) and
                car_v_cruise_kph < self.jve_car_speed_prev):
              self.v_cruise_kph = car_v_cruise_kph
              self.jve_button_synced = True
            elif (self.jve_driver_btn_type in (ButtonType.accelCruise, ButtonType.resumeCruise) and
                  car_v_cruise_kph > self.v_cruise_kph):
              self.v_cruise_kph = car_v_cruise_kph
              self.jve_button_synced = True

          if not self.jve_button_synced:
            self._update_v_cruise_non_pcm(CS, enabled, is_metric)
          if (self.jve_driver_btn_frames > 0 and
              car_v_cruise_kph > self.v_cruise_kph + 1.0 and
              self.jve_driver_btn_type not in (ButtonType.decelCruise, ButtonType.setCruise) and
              self.button_timers[ButtonType.decelCruise] == 0):
            self.v_cruise_kph = car_v_cruise_kph

          if self.jve_driver_btn_frames > 0:
            self.jve_driver_btn_frames -= 1
            if self.jve_driver_btn_frames == 0:
              self.jve_button_synced = False
          self.jve_car_speed_prev = car_v_cruise_kph
          self.v_cruise_cluster_kph = self.v_cruise_kph
          # Skip the first engaged frame so initialize_v_cruise can still restore the previous max speed
          if self.jve_engaged_prev:
            self.jve_v_cruise_max = self.v_cruise_kph
          self.jve_engaged_prev = True
          self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET
      # ACC main off: forget the jvePilot driver max speed
      self.jve_v_cruise_max = V_CRUISE_UNSET
      self.jve_resume_requested = False
      self.jve_engaged_prev = False
      self.jve_car_speed_prev = V_CRUISE_UNSET
      self.jve_driver_btn_frames = 0
      self.jve_driver_btn_type = None
      self.jve_button_synced = False

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    if self.jve_active and not is_metric:
      cur_mph = int(round(self.v_cruise_kph * CV.KPH_TO_MPH))
      if long_press:
        if cur_mph % 5 != 0:
          new_mph = CRUISE_NEAREST_FUNC[button_type](cur_mph / 5.0) * 5
        else:
          new_mph = cur_mph + 5 * CRUISE_INTERVAL_SIGN[button_type]
      else:
        new_mph = cur_mph + CRUISE_INTERVAL_SIGN[button_type]
      self.v_cruise_kph = float(round(new_mph * CV.MPH_TO_KPH, 1))
    else:
      v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
      if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
        self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
      else:
        self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      if self.jve_active:
        # jvePilot: SET- while pressing the gas syncs the driver max speed to the current speed (like stock FCA ACC)
        self.v_cruise_kph = max(CS.vEgo, CS.cruiseState.speed) * CV.MS_TO_KPH
      else:
        # If set is pressed while overriding, clip cruise speed to minimum of vEgo
        self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    if self.jve_active and CS.cruiseState.speed > 0:
      for b in CS.buttonEvents:
        if b.type in (ButtonType.accelCruise, ButtonType.resumeCruise):
          self.jve_resume_requested = True
        elif b.type in (ButtonType.decelCruise, ButtonType.setCruise):
          self.jve_resume_requested = False
      car_v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      if self.jve_resume_requested and self.jve_v_cruise_max != V_CRUISE_UNSET:
        # Resumed with RES+: restore the driver's previous max speed (stock ACC may hold a speed jvePilot lowered)
        self.v_cruise_kph = max(self.jve_v_cruise_max, car_v_cruise_kph)
      else:
        self.v_cruise_kph = car_v_cruise_kph
      self.v_cruise_cluster_kph = self.v_cruise_kph
      self.jve_car_speed_prev = car_v_cruise_kph
      self.jve_resume_requested = False
      return

    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_initialized:
      self.v_cruise_kph = self.v_cruise_kph_last
    else:
      self.v_cruise_kph = int(round(np.clip(CS.vEgo * CV.MS_TO_KPH, initial, V_CRUISE_MAX)))

    self.v_cruise_cluster_kph = self.v_cruise_kph
