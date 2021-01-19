from cereal import car
from selfdrive.car import make_can_msg


GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_lkas_command(packer, apply_steer, lkas_active, frame):
  # LKAS_COMMAND 0x1f6 (502) Lane-keeping signal to turn the wheel.
  values = {
    "LKAS_STEERING_TORQUE": apply_steer,
    "LKAS_STATE": 2 if lkas_active else 0,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)
  
def create_lkas_hud_command(packer, enabled, left_lane_visible, right_lane_visible):
  # Chrysler came up with this scheme, not me
  if enabled:
    if left_lane_visible:
      if right_lane_visible:
        lane_visibility_signal = 0x6  # Both sides white
      else:
        lane_visibility_signal = 0x2  # Left only white
    elif right_lane_visible:
      lane_visibility_signal = 0x3    # Right only white
    else:
      lane_visibility_signal = 0x1    # Neither lane border shown
  else:
    lane_visibility_signal = 0x1      # Neither lane border shown

  values = {
    "SET_ME_0XAC": 0xAC,
    "LKAS_ICON_COLOR": 2 if enabled else 1,
    "LKAS_LANE_LINES": lane_visibility_signal,
  }

  return packer.make_can_msg("LKAS_HUD", 0, values)



def create_wheel_buttons(packer, frame, cancel=False):
  # WHEEL_BUTTONS (762) Message sent to cancel ACC.
  values = {
    "ACC_CANCEL": cancel,
    "COUNTER": frame % 10
  }
  return packer.make_can_msg("WHEEL_BUTTONS", 0, values)
