from opendbc.car.structs import car
from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.jvepilot import (AUTO_FOLLOW_HYST, AUTO_FOLLOW_THRESHOLDS, BARS_TO_PERSONALITY, LEAD_MIN_PROB,
                                                       get_jve_target_speed, jve_supported, update_auto_follow_bars)


def test_jve_supported():
  assert jve_supported(car.CarParams(brand="chrysler", pcmCruise=True, openpilotLongitudinalControl=False))
  assert not jve_supported(car.CarParams(brand="chrysler", pcmCruise=True, openpilotLongitudinalControl=True))
  assert not jve_supported(car.CarParams(brand="toyota", pcmCruise=True, openpilotLongitudinalControl=False))


def test_auto_follow_steps_and_hysteresis():
  for profile, thresholds in AUTO_FOLLOW_THRESHOLDS.items():
    bars = 1
    for v in [x * 0.1 for x in range(400)]:  # accelerate 0 -> 40 m/s
      new = update_auto_follow_bars(bars, v, profile)
      assert new >= bars
      bars = new
    assert bars == 4

    # just below an up-threshold + hysteresis: no change; well below down-threshold: steps down
    assert update_auto_follow_bars(1, thresholds[0] + AUTO_FOLLOW_HYST - 0.01, profile) == 1
    assert update_auto_follow_bars(2, thresholds[0] - AUTO_FOLLOW_HYST + 0.01, profile) == 2
    assert update_auto_follow_bars(4, 0.0, profile) == 1


def test_auto_follow_off_profile_is_noop():
  assert update_auto_follow_bars(3, 30.0, 0) == 3


def test_bars_map_to_valid_personalities():
  valid = set(log.LongitudinalPersonality.schema.enumerants.values())
  assert set(BARS_TO_PERSONALITY) == {1, 2, 3, 4}
  assert all(p in valid for p in BARS_TO_PERSONALITY.values())


# ---- get_jve_target_speed (uses real capnp readers, like controlsd) ----


def _inputs(v_ego=30.0, v_cruise_kph=113.0, acc_set_kph=113.0, a_target=0.0, plan_v=None, source="cruise", lead_probs=()):
  CS = car.CarState(vEgo=v_ego, vCruise=v_cruise_kph, cruiseState={"speed": acc_set_kph * CV.KPH_TO_MS}).as_reader()
  plan = log.LongitudinalPlan.new_message()
  plan.aTarget = a_target
  plan.speeds = [plan_v if plan_v is not None else v_ego] * 17
  plan.longitudinalPlanSource = source
  model = log.ModelDataV2.new_message()
  leads = model.init("leadsV3", len(lead_probs))
  for i, p in enumerate(lead_probs):
    leads[i].prob = p
  return CS, plan.as_reader(), model.as_reader()


def test_target_holds_driver_max_when_cruising():
  v = get_jve_target_speed(*_inputs(v_ego=31.0, plan_v=31.4), eco=1, acc_braking=True)
  assert abs(v - 113 * CV.KPH_TO_MS) < 1e-3


def test_target_never_exceeds_driver_max():
  v = get_jve_target_speed(*_inputs(v_ego=20.0, a_target=1.0, plan_v=60.0), eco=0, acc_braking=True)
  assert v <= 113 * CV.KPH_TO_MS + 1e-6


def test_eco_limits_headroom():
  for eco, headroom in ((0, 5.0), (1, 3.0), (2, 1.8)):
    v = get_jve_target_speed(*_inputs(v_ego=20.0, a_target=1.0, plan_v=25.0), eco=eco, acc_braking=True)
    assert v <= 20.0 + headroom + 1e-6


def test_acc_braking_toggle_undershoots_only_when_enabled():
  args = _inputs(v_ego=25.0, a_target=-1.0, plan_v=24.0, source="lead0", lead_probs=(0.95,))
  with_braking = get_jve_target_speed(*args, eco=1, acc_braking=True)
  without = get_jve_target_speed(*args, eco=1, acc_braking=False)
  assert abs(with_braking - (25.0 - 2.5)) < 1e-3
  assert abs(without - (25.0 - 1.5)) < 1e-3  # v_plan = min(24, vEgo + 1.5 * aTarget)


def test_low_confidence_lead_does_not_lower_set_speed():
  acc_set = 100.0
  for source, probs in (("lead0", (LEAD_MIN_PROB - 0.2,)), ("lead1", (0.95, 0.3)), ("lead0", ())):
    v = get_jve_target_speed(*_inputs(v_ego=27.0, acc_set_kph=acc_set, a_target=-1.5, plan_v=24.0,
                                      source=source, lead_probs=probs), eco=1, acc_braking=True)
    assert v >= acc_set * CV.KPH_TO_MS - 1e-6, (source, probs)


def test_radar_lead_distance_overrides_low_vision_prob():
  v = get_jve_target_speed(*_inputs(v_ego=27.0, acc_set_kph=100.0, a_target=-1.5, plan_v=24.0,
                                    source="lead0", lead_probs=(0.2,)), eco=1, acc_braking=True,
                           radar_lead_distance=20.0)
  assert v < 100.0 * CV.KPH_TO_MS


def test_fast_highway_lead_does_not_slow_below_planner_target():
  v = get_jve_target_speed(*_inputs(v_ego=27.0, acc_set_kph=105.0, a_target=-0.3, plan_v=25.5,
                                    source="lead0", lead_probs=(0.95,)), eco=1, acc_braking=True,
                           radar_lead_distance=23.0)
  assert v >= 25.0 - 2.5 * 0.3 - 1e-6


def test_standstill_holds_current_acc_set_speed():
  CS, plan, model = _inputs(v_ego=0.0, v_cruise_kph=105.0, acc_set_kph=48.0)
  CS_rw = car.CarState.new_message(**CS.to_dict())
  CS_rw.cruiseState.standstill = True
  v = get_jve_target_speed(CS_rw.as_reader(), plan, model, eco=1, acc_braking=True)
  assert abs(v - 48.0 * CV.KPH_TO_MS) < 1e-3


def test_confident_lead_lowers_set_speed():
  v = get_jve_target_speed(*_inputs(v_ego=27.0, acc_set_kph=100.0, a_target=-1.5, plan_v=24.0,
                                    source="lead1", lead_probs=(0.2, 0.9)), eco=1, acc_braking=True)
  assert v < 100.0 * CV.KPH_TO_MS


def test_curve_slowdown_not_affected_by_lead_gate():
  # cruise-sourced deceleration (e.g. slow-in-turns) still lowers the set speed with no lead present
  v = get_jve_target_speed(*_inputs(v_ego=30.0, a_target=-1.0, plan_v=28.0, source="cruise"), eco=1, acc_braking=False)
  assert v < 30.0


def test_moderate_confidence_lead_lowers_set_speed():
  # 0.50-0.69 prob leads tracked by longitudinal MPC must still lower the set speed
  v = get_jve_target_speed(*_inputs(v_ego=20.0, v_cruise_kph=80.0, acc_set_kph=80.0, a_target=-0.8, plan_v=16.0,
                                    source="lead0", lead_probs=(0.55,)), eco=1, acc_braking=True)
  assert v < 80.0 * CV.KPH_TO_MS


def test_decel_never_raises_set_speed_above_current_acc_speed():
  # When vCruise drifted above the current ACC set speed, braking for a lead must never command a higher set speed (RES+)
  v = get_jve_target_speed(*_inputs(v_ego=25.0, v_cruise_kph=113.0, acc_set_kph=72.0, a_target=-0.4, plan_v=24.0,
                                    source="lead0", lead_probs=(0.8,)), eco=1, acc_braking=False)
  assert v <= 72.0 * CV.KPH_TO_MS + 1e-6


def test_confident_lead_caps_target_speed_before_hard_braking():
  # SEG16 10:49:55 scenario: slow lead ahead caps planned trajectory at ~5.8 m/s (13 mph) while aTarget transitions +0.4 -> -0.05
  for a_target in (0.4, 0.05, -0.05):
    v = get_jve_target_speed(*_inputs(v_ego=7.5, v_cruise_kph=95.0, acc_set_kph=95.0, a_target=a_target, plan_v=5.8,
                                      source="lead1", lead_probs=(0.58, 0.85)), eco=1, acc_braking=True)
    assert v <= 5.8 + 1e-6, a_target


def test_desire_helper_aborts_lane_change_on_blindspot():
  import sys
  from types import SimpleNamespace
  sys.modules.setdefault("setproctitle", SimpleNamespace(getproctitle=lambda: "", setproctitle=lambda _: None))
  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
  dh = DesireHelper()
  v_ego = 25.0
  # Blinker on -> preLaneChange
  dh.update(car.CarState(vEgo=v_ego, leftBlinker=True), lateral_active=True, lane_change_prob=1.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  # Steering nudge with clear blindspot -> laneChangeStarting
  dh.update(car.CarState(vEgo=v_ego, leftBlinker=True, steeringPressed=True, steeringTorque=10.0),
            lateral_active=True, lane_change_prob=1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.desire == log.Desire.laneChangeLeft
  # Car detected in blindspot mid-lane-change -> immediately aborts back to preLaneChange and clears desire
  dh.update(car.CarState(vEgo=v_ego, leftBlinker=True, leftBlindspot=True), lateral_active=True, lane_change_prob=1.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.desire == log.Desire.none
