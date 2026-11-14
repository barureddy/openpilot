"""Shared jvePilot hybrid-ACC helpers.

jvePilot only modulates the stock ACC set speed / follow distance through virtual steering-wheel
button presses, so every feature here is scoped to Chrysler/FCA cars running stock ACC
(pcmCruise and no openpilot longitudinal control). Other cars must behave exactly like upstream.
"""
from openpilot.cereal import log
from openpilot.common.constants import CV

LongitudinalPersonality = log.LongitudinalPersonality
LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource

# ACC Eco: max target-speed lead over current speed while accelerating (m/s), by setting 0=Off, 1=Eco 1, 2=Eco 2
ECO_HEADROOM = {0: 5.0, 1: 3.0, 2: 1.8}

# Auto Follow: speed thresholds (m/s) for stepping stock ACC follow-distance bars 1->2, 2->3 and 3->4
AUTO_FOLLOW_THRESHOLDS = {
  1: (5.4, 11.2, 18.8),   # Relaxed profile: 12 / 25 / 42 mph (longer distance sooner)
  2: (6.7, 13.4, 22.4),   # Standard profile: 15 / 30 / 50 mph
  3: (8.0, 16.1, 25.9),   # Aggressive profile: 18 / 36 / 58 mph (closer following)
}
AUTO_FOLLOW_HYST = 0.8  # m/s (~1.8 mph)

# The planner MPC follow gap tracks the stock ACC bar setting so both agree on how closely to follow
BARS_TO_PERSONALITY = {
  1: LongitudinalPersonality.aggressive,
  2: LongitudinalPersonality.standard,
  3: LongitudinalPersonality.relaxed,
  4: LongitudinalPersonality.relaxed,
}

# Only lower the stock ACC set speed for a lead the driving model is confident about. On radarless
# CUSW (vision-only leads) this avoids phantom slowdowns; stock ACC radar still handles real leads.
LEAD_MIN_PROB = 0.7


def jve_supported(CP) -> bool:
  return CP.brand == "chrysler" and CP.pcmCruise and not CP.openpilotLongitudinalControl


def update_auto_follow_bars(bars: int, v_ego: float, profile: int) -> int:
  """Step follow-distance bars (1..4) up/down with speed, using hysteresis. Returns the new bar count."""
  thresholds = AUTO_FOLLOW_THRESHOLDS.get(profile)
  if thresholds is None:
    return bars
  bars = min(max(int(bars), 1), 4)
  while bars < 4 and v_ego > thresholds[bars - 1] + AUTO_FOLLOW_HYST:
    bars += 1
  while bars > 1 and v_ego < thresholds[bars - 2] - AUTO_FOLLOW_HYST:
    bars -= 1
  return bars


def get_jve_target_speed(CS, long_plan, model_v2, eco: int, acc_braking: bool) -> float:
  """Target stock-ACC set speed (m/s) that CarController steers toward with virtual button presses.

  Never exceeds the driver's max (CS.vCruise). Lowers the set speed for curves/traffic from the plan, optionally
  undershoots while braking (ACC Braking), caps how far above current speed it asks for (ACC Eco), and refuses to
  lower the set speed for a low-confidence lead.
  """
  v_cruise = CS.vCruise * CV.KPH_TO_MS
  if len(long_plan.speeds) == 0:
    return float(max(0.0, v_cruise))

  a_target = float(long_plan.aTarget)
  v_ego = float(CS.vEgo)
  lookahead_idx = min(5, len(long_plan.speeds) - 1)
  v_plan = min(float(long_plan.speeds[lookahead_idx]), v_ego + a_target * 1.5)

  if a_target < -0.1:
    v_target = min(v_cruise, v_plan)
    if acc_braking:
      # ACC Braking: set the cruise speed below the target so the car starts slowing sooner
      v_target = min(v_target, v_ego + a_target * 2.5)
  elif v_plan >= v_cruise - 0.5 and v_ego >= v_cruise - 2.0:
    v_target = v_cruise
  else:
    v_target = min(v_cruise, max(v_plan, v_ego + a_target))
    if a_target > 0:
      v_target = min(v_target, v_ego + ECO_HEADROOM.get(eco, ECO_HEADROOM[1]))

  # Don't lower the stock ACC set speed for a low-confidence (possibly phantom) lead.
  # NOTE: capnp enum readers don't hash like ints, so compare with == rather than using a dict lookup.
  source = long_plan.longitudinalPlanSource
  lead_idx = 0 if source == LongitudinalPlanSource.lead0 else (1 if source == LongitudinalPlanSource.lead1 else None)
  if lead_idx is not None:
    leads = model_v2.leadsV3
    lead_prob = float(leads[lead_idx].prob) if len(leads) > lead_idx else 0.0
    if lead_prob < LEAD_MIN_PROB:
      v_target = max(v_target, float(CS.cruiseState.speed))

  return float(max(0.0, v_target))
