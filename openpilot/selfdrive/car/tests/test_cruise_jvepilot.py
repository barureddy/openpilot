"""jvePilot VCruiseHelper behavior: driver max-speed tracking, restore on RES+, gas+SET- sync, and scoping."""
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_UNSET

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def chrysler_cp(**kwargs):
  params = {"brand": "chrysler", "pcmCruise": True, "openpilotLongitudinalControl": False, **kwargs}
  return car.CarParams(**params)


def cs(set_kph, enabled=True, available=True, v_ego=30.0, gas=False, buttons=()):
  return car.CarState(vEgo=v_ego, gasPressed=gas, buttonEvents=list(buttons),
                      cruiseState={"available": available, "enabled": enabled,
                                   "speed": set_kph * CV.KPH_TO_MS, "speedCluster": set_kph * CV.KPH_TO_MS})


def tap(button_type):
  return [ButtonEvent(type=button_type, pressed=True)], [ButtonEvent(type=button_type, pressed=False)]


class JveDriver:
  """Mimics card.py ordering: update_v_cruise, then initialize_v_cruise on the carControl.enabled rising edge."""
  def __init__(self, CP, jve=True):
    self.h = VCruiseHelper(CP)
    self.h.jve_acc_control = jve
    self.enabled_prev = False
    self.CS_prev = cs(0, enabled=False, available=False)

  def step(self, CS, enabled):
    self.h.update_v_cruise(CS, enabled, is_metric=False)
    if enabled and not self.enabled_prev:
      self.h.initialize_v_cruise(self.CS_prev, experimental_mode=False)
    self.enabled_prev = enabled
    self.CS_prev = CS
    return self.h.v_cruise_kph

  def engage(self, set_kph, buttons=()):
    # ACC goes active first (buttons released), openpilot enables on the next frame
    self.step(cs(set_kph, enabled=True, buttons=buttons), enabled=False)
    return self.step(cs(set_kph, enabled=True), enabled=True)


def test_scoping():
  assert VCruiseHelper(chrysler_cp()).jve_active is False  # toggle off by default
  for CP in (car.CarParams(brand="toyota", pcmCruise=True),
             chrysler_cp(openpilotLongitudinalControl=True),
             car.CarParams(brand="chrysler", pcmCruise=False)):
    h = VCruiseHelper(CP)
    h.jve_acc_control = True
    assert not h.jve_active, CP.brand


def test_stock_mode_mirrors_car_speed():
  d = JveDriver(chrysler_cp(), jve=False)
  d.engage(113)
  assert abs(d.step(cs(72), enabled=True) - 72) < 0.1


def test_driver_max_kept_when_jvepilot_lowers_acc_speed():
  d = JveDriver(chrysler_cp())
  assert abs(d.engage(113) - 113) < 0.1
  assert abs(d.step(cs(72), enabled=True) - 113) < 0.1  # car set speed lowered for a curve
  press, release = tap(ButtonType.accelCruise)
  d.step(cs(72, buttons=press), enabled=True)
  assert d.step(cs(72, buttons=release), enabled=True) > 113.5  # driver tap still raises the max


def test_restore_driver_max_after_brake_cancel_and_resume():
  d = JveDriver(chrysler_cp())
  d.engage(113)
  for _ in range(5):
    d.step(cs(72), enabled=True)
  # brake cancels ACC; car keeps the lowered 72 km/h set speed
  for _ in range(5):
    d.step(cs(72, enabled=False), enabled=False)
  press, release = tap(ButtonType.accelCruise)  # RES+
  d.step(cs(72, enabled=False, buttons=press), enabled=False)
  d.step(cs(72, enabled=False, buttons=release), enabled=False)
  assert abs(d.engage(72) - 113) < 0.1


def test_set_minus_after_cancel_uses_new_speed():
  d = JveDriver(chrysler_cp())
  d.engage(113)
  for _ in range(5):
    d.step(cs(72), enabled=True)
  d.step(cs(72, enabled=False), enabled=False)
  press, release = tap(ButtonType.decelCruise)  # SET- sets a new speed
  d.step(cs(72, enabled=False, buttons=press), enabled=False)
  d.step(cs(72, enabled=False, buttons=release), enabled=False)
  assert abs(d.engage(80) - 80) < 0.1


def test_acc_main_off_forgets_driver_max():
  d = JveDriver(chrysler_cp())
  d.engage(113)
  d.step(cs(113), enabled=True)
  d.step(cs(0, enabled=False, available=False), enabled=False)
  assert d.h.jve_v_cruise_max == V_CRUISE_UNSET
  press, release = tap(ButtonType.accelCruise)
  d.step(cs(72, enabled=False, buttons=press), enabled=False)
  d.step(cs(72, enabled=False, buttons=release), enabled=False)
  assert abs(d.engage(72) - 72) < 0.1


def test_gas_set_minus_syncs_to_current_speed():
  d = JveDriver(chrysler_cp())
  d.engage(80)
  for _ in range(3):
    d.step(cs(80), enabled=True)
  v_ego = 30.0  # 108 km/h on the gas, above the set speed
  press, release = tap(ButtonType.decelCruise)
  d.step(cs(80, v_ego=v_ego, gas=True, buttons=press), enabled=True)
  assert abs(d.step(cs(80, v_ego=v_ego, gas=True, buttons=release), enabled=True) - v_ego * CV.MS_TO_KPH) < 0.2


def test_imperial_long_press_set_minus_matches_fca_mph():
  d = JveDriver(chrysler_cp())
  d.engage(113)  # 70 mph
  press, release = tap(ButtonType.decelCruise)
  d.step(cs(113, buttons=press), enabled=True)
  for expected_mph in (65, 60, 55, 50):
    for _ in range(50):
      v_kph = d.step(cs(113), enabled=True)
    assert round(v_kph * CV.KPH_TO_MPH) == expected_mph
  d.step(cs(80, buttons=release), enabled=True)
  assert round(d.h.v_cruise_kph * CV.KPH_TO_MPH) == 50


def test_driver_set_minus_hold_syncs_downward_with_car_acc_hud():
  d = JveDriver(chrysler_cp())
  d.engage(97)  # 60 mph
  press, release = tap(ButtonType.decelCruise)
  d.step(cs(97, buttons=press), enabled=True)
  for _ in range(30):
    d.step(cs(97), enabled=True)
  # Jeep PCM jumps to 56 mph (90 km/h) after ~0.32s hold and driver releases SET-
  d.step(cs(90, buttons=release), enabled=True)
  assert abs(d.h.v_cruise_kph - 90) < 0.1
  assert round(d.h.v_cruise_cluster_kph * CV.KPH_TO_MPH) == 56


def test_set_minus_engage_on_rising_edge_and_delayed_hud():
  d = JveDriver(chrysler_cp())
  d.engage(113)
  for _ in range(5):
    d.step(cs(72), enabled=True)
  d.step(cs(72, enabled=False), enabled=False)
  # Earlier RES+ while disengaged, then driver engages by pressing SET- (still held on engage frame, HUD updates 10 frames later)
  res_press, res_release = tap(ButtonType.accelCruise)
  d.step(cs(90, enabled=False, buttons=res_press), enabled=False)
  d.step(cs(90, enabled=False, buttons=res_release), enabled=False)
  set_press, set_release = tap(ButtonType.decelCruise)
  d.step(cs(90, enabled=True, buttons=set_press), enabled=False)
  d.step(cs(90, enabled=True), enabled=True)
  for _ in range(8):
    d.step(cs(90, enabled=True), enabled=True)
  # 5 Hz ACC_HUD arrives with new 45 mph (72 km/h) SET- speed
  v_kph = d.step(cs(72, enabled=True, buttons=set_release), enabled=True)
  assert abs(v_kph - 72) < 0.1


def test_driver_set_plus_syncs_to_car_hud_when_max_was_higher():
  # SEG5 scenario: comma vCruise was at 65 mph while Jeep dashboard ACC_HUD was at 55 mph;
  # when driver holds SET+ and Jeep dashboard steps 55 -> 56 -> 60 mph, comma vCruise must sync to 60 mph (not 70 mph)
  d = JveDriver(chrysler_cp())
  d.engage(65 * CV.MPH_TO_KPH)
  for _ in range(5):
    d.step(cs(55 * CV.MPH_TO_KPH), enabled=True)
  press, release = tap(ButtonType.accelCruise)
  d.step(cs(55 * CV.MPH_TO_KPH, buttons=press), enabled=True)
  for _ in range(8):
    d.step(cs(55 * CV.MPH_TO_KPH), enabled=True)
  d.step(cs(56 * CV.MPH_TO_KPH), enabled=True)
  for _ in range(20):
    d.step(cs(56 * CV.MPH_TO_KPH), enabled=True)
  v_kph = d.step(cs(60 * CV.MPH_TO_KPH, buttons=release), enabled=True)
  assert round(v_kph * CV.KPH_TO_MPH) == 60
  assert round(d.h.v_cruise_cluster_kph * CV.KPH_TO_MPH) == 60
