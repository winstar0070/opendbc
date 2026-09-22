import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


# Temporary, stationary-only left/right lane handoff demo. Disable after on-cluster validation.
CCNC_DEV_STOPPED_GREEN_LANES_TEST = True


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering:
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam


def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_torque, lkas_icon):
  values = {
    "LKA_OptUsmSta": 2,
    "LKA_SysIndReq": lkas_icon,
    "StrTqReqVal": apply_torque,
    "LKA_SysWrn": 0,
    "ActToiSta": 1 if lat_active else 0,
    "LKA_UsmMod": 0,  # hide LKAS settings
    "LKA_RcgSta": 0,
    "Damping_Gain": 100,  # can potentially tuned for better perf [3, 200]
  }

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret


def create_suppress_lfa(packer, CAN, lfa_block_msg, lka_steering_alt):
  suppress_msg = "CAM_0x362" if lka_steering_alt else "CAM_0x2a4"
  msg_bytes = 32 if lka_steering_alt else 24

  values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)


def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # CAN FD camera-based SCC requires additional signals to be preserved
  # verbatim from the previous SCC_CONTROL frame to avoid checksum or
  # state validation faults. Classic CAN SCC only validates a subset.
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, lfa_icon):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def ccnc_stopped_lane_handoff(frame):
  # Each direction takes 200 source frames (~10s at 20Hz). Do not use the
  # stock 8-bit COUNTER as a clock: it wraps before a full left/right cycle.
  phase = frame % 200
  left_change = (frame // 200) % 2 == 0
  sliding = 40 <= phase < 120
  transferring = 120 <= phase < 124
  rebinding = 122 <= phase < 124
  central_fill = 120 <= phase < 184

  left_position = right_position = 15
  left_color = right_color = 6
  if sliding or (transferring and not rebinding):
    progress = min(1.0, (phase - 40) / 79.0)
    eased = progress * progress * (3.0 - 2.0 * progress)
    shift = round(9.0 * eased)
    inner, outer = 15 - shift, 15 + shift
    inner_color = 1 if inner <= 8 else 6  # Hide before the boundary reaches the car.
    if left_change:
      left_position, right_position = inner, outer
      left_color = inner_color
    else:
      left_position, right_position = outer, inner
      right_color = inner_color

  if rebinding:
    # The central fill stays on while both boundaries are briefly hidden.
    # Rebind to the new lane at 15/15; never animate the old lines backward.
    left_color = right_color = 1

  # Keep the central green floor after entering the new lane. Hold green borders
  # for 1s, blink white/green twice over 2s, then settle to white and clear the fill. These phases
  # are clocked by source frames, so COUNTER wrap cannot truncate the effect.
  if phase < 40 or phase >= 184:
    left_color = right_color = 2
  elif 144 <= phase < 184:
    left_color = right_color = 2 if ((phase - 144) // 10) % 2 == 0 else 6

  return {
    "LANELINE_LEFT": left_color,
    "LANELINE_RIGHT": right_color,
    "LANELINE_LEFT_POSITION": left_position,
    "LANELINE_RIGHT_POSITION": right_position,
    "LANELINE_CURVATURE": 15,
    "CENTERLINE": 0,
    "LANE_LEFT": int(sliding and left_change),
    "LANE_RIGHT": int(sliding and not left_change),
    "LANE_HIGHLIGHT": int(central_fill),
    "LANE_HIGHLIGHT_DISTANCE": 60.0 if central_fill else 0.0,
  }


def create_ccnc(packer, CAN, openpilot_longitudinal_control, enabled, hud, left_blinker, right_blinker, msg_161, msg_162, msg_1b5,
                is_metric, out, main_cruise_enabled, lfa_icon, send_161=True, send_162=True, disp_state=None,
                lane_change_state=0, lane_change_direction=0):
  for f in {"FAULT_LSS", "FAULT_HDA", "FAULT_DAS", "FAULT_LFA", "FAULT_DAW", "FAULT_ESS"}:
    msg_162[f] = 0

  if msg_161["ALERTS_2"] == 5:
    msg_161.update({"ALERTS_2": 0, "SOUNDS_2": 0})
  if msg_161["ALERTS_3"] == 17:
    msg_161["ALERTS_3"] = 0
  if msg_161["ALERTS_5"] in (2, 5):
    msg_161["ALERTS_5"] = 0
  if msg_161["SOUNDS_4"] == 2 and msg_161["LFA_ICON"] in (0, 3):
    msg_161["SOUNDS_4"] = 0

  LANE_CHANGE_SPEED_MIN = 8.9408  # 20 mph

  # Keep the existing source cadence/counters; never force an extra 0x161 frame.
  stopped_green_lanes_test = CCNC_DEV_STOPPED_GREEN_LANES_TEST and 0.0 <= out.vEgo < 0.1
  if stopped_green_lanes_test:
    lfa_icon = 2
    lane_change_state = 0
    lane_change_direction = 0
  elif disp_state is not None:
    # Start again from the centered view after moving or disabling the demo.
    disp_state.pop("green_test_frame", None)
    disp_state.pop("green_test_values", None)

  any_blinker = left_blinker or right_blinker

  curvature = {
    i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i
    for i in range(-15, 16)
  }

  # ---------------------------------------------------------------------------
  # Lane change HUD animation
  #
  # CHANGING:
  #   - target side LANE_LEFT or LANE_RIGHT only
  #   - lane pair slides sideways
  #
  # LANDED_FILL:
  #   - side LANE_* goes OFF
  #   - LANE_HIGHLIGHT becomes GREEN
  #   - CENTERLINE hidden
  #   - lane pair smoothly recenters
  #
  # LANDED_LINES:
  #   - central fill goes OFF
  #   - left/right lane lines stay GREEN briefly
  #
  # DONE:
  #   - normal white lane
  # ---------------------------------------------------------------------------
  PROG_STEP = 0.01              # ~5 sec at 20 Hz
  LAND_FILL_FRAMES = 30         # ~1.5 sec central green fill
  LAND_LINE_FRAMES = 20         # ~1.0 sec green lane lines
  LAND_TOTAL_FRAMES = LAND_FILL_FRAMES + LAND_LINE_FRAMES

  # Physical metres, not 0..2047 raw.
  LANE_HIGHLIGHT_DISTANCE = 60.0

  changing = lane_change_state in (2, 3)

  if changing and lane_change_direction in (1, 2):
    cur_dir = lane_change_direction
  else:
    cur_dir = 0

  if disp_state is not None:
    prog = disp_state.get("lc_prog", 0.0)
    held_dir = disp_state.get("lc_dir", 0)
    landed_frames = disp_state.get("lc_landed_frames", 0)
    done = disp_state.get("lc_done", False)

    if send_161:
      if changing:
        if cur_dir in (1, 2):
          held_dir = cur_dir

        if not done:
          if landed_frames > 0:
            # Already reached target lane:
            # central highlight -> green lane lines -> done.
            landed_frames += 1

            if landed_frames > LAND_TOTAL_FRAMES:
              landed_frames = 0
              done = True

          else:
            # Slide target lane toward the ego position.
            prog = min(1.0, prog + PROG_STEP)

            if prog >= 1.0:
              prog = 1.0
              landed_frames = 1

      else:
        # Lane-change state ended. Reset animation for the next change.
        prog = 0.0
        held_dir = 0
        landed_frames = 0
        done = False

      disp_state["lc_prog"] = prog
      disp_state["lc_dir"] = held_dir
      disp_state["lc_landed_frames"] = landed_frames
      disp_state["lc_done"] = done

  else:
    # Fallback when no persistent display state is supplied.
    held_dir = cur_dir
    done = False

    if lane_change_state == 2:
      prog = 0.5
      landed_frames = 0
    elif lane_change_state == 3:
      prog = 1.0
      landed_frames = 1
    else:
      prog = 0.0
      landed_frames = 0

  landing_fill = 0 < landed_frames <= LAND_FILL_FRAMES
  landing_lines = LAND_FILL_FRAMES < landed_frames <= LAND_TOTAL_FRAMES

  changing_visual = (
    changing
    and held_dir in (1, 2)
    and landed_frames == 0
    and not done
  )

  # ---------------------------------------------------------------------------
  # Target-side green area
  #
  # IMPORTANT:
  # LANE_LEFT + LANE_RIGHT are NEVER used together to fake the ego lane.
  # ---------------------------------------------------------------------------
  fill_left = changing_visual and held_dir == 1
  fill_right = changing_visual and held_dir == 2

  # Central ego-lane fill after crossing.
  lane_highlight = 1 if (lfa_icon and landing_fill) else 0
  lane_highlight_distance = LANE_HIGHLIGHT_DISTANCE if lane_highlight else 0.0

  # Do not leave the middle CENTERLINE through the green lane.
  lane_anim_active = changing_visual or landing_fill or landing_lines
  centerline = 0 if lane_anim_active else (1 if lfa_icon else 0)

  # ---------------------------------------------------------------------------
  # Lane line colours
  # ---------------------------------------------------------------------------
  def _laneline_color(visible, depart):
    if not lfa_icon:
      return 0

    if depart:
      return 4

    # While crossing, green area is the main visual.
    if changing_visual:
      return 1  # hidden

    # Target reached:
    # show green borders during central fill and for a short period afterwards.
    if landing_fill or landing_lines:
      return 6  # green

    if not visible:
      return 1

    return 2  # white

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": 0,
    "LFA_ICON": 2 if lfa_icon else 0,

    "CENTERLINE": centerline,

    "LANELINE_CURVATURE": (
      curvature[max(-15, min(int(out.steeringAngleDeg / 4.5), 15))]
      if lfa_icon and not any_blinker else 15
    ),

    "LANELINE_LEFT": _laneline_color(
      hud.leftLaneVisible,
      hud.leftLaneDepart,
    ),

    "LANELINE_RIGHT": _laneline_color(
      hud.rightLaneVisible,
      hud.rightLaneDepart,
    ),

    "LCA_LEFT_ICON": (
      0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN
      else 1 if out.leftBlindspot
      else 2 if any_blinker
      else 4
    ),

    "LCA_RIGHT_ICON": (
      0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN
      else 1 if out.rightBlindspot
      else 2 if any_blinker
      else 4
    ),

    "LCA_LEFT_ARROW": 2 if left_blinker else 0,
    "LCA_RIGHT_ARROW": 2 if right_blinker else 0,

    # Adjacent/target lane only.
    "LANE_LEFT": 1 if (lfa_icon and fill_left) else 0,
    "LANE_RIGHT": 1 if (lfa_icon and fill_right) else 0,

    # Ego/central lane highlight.
    "LANE_HIGHLIGHT": lane_highlight,
    "LANE_HIGHLIGHT_DISTANCE": lane_highlight_distance,
  })

  # ---------------------------------------------------------------------------
  # Lane geometry
  #
  # Car stays visually centered.
  #
  # During crossing:
  #   lane pair moves sideways.
  #
  # After target reached:
  #   smoothly recenter while LANE_HIGHLIGHT is active.
  # ---------------------------------------------------------------------------
  if lfa_icon:
    LANE_POS_CENTER = 15.0
    LANE_POS_BIAS = 12.0
    LANE_POS_SIGN = -1

    dir_sign = (
      1.0 if held_dir == 1
      else -1.0 if held_dir == 2
      else 0.0
    )

    if changing_visual:
      shift = (
        LANE_POS_SIGN
        * dir_sign
        * prog
        * LANE_POS_BIAS
      )

    elif landing_fill and held_dir in (1, 2):
      # At the instant of crossing we are at full shift.
      # Bring the new ego lane smoothly back to screen center while
      # the central green highlight is displayed.
      settle = min(
        1.0,
        landed_frames / float(LAND_FILL_FRAMES),
      )

      shift = (
        LANE_POS_SIGN
        * dir_sign
        * LANE_POS_BIAS
        * (1.0 - settle)
      )

    else:
      shift = 0.0

    left_lane = int(round(
      min(
        30.0,
        max(
          0.0,
          LANE_POS_CENTER + shift,
        ),
      )
    ))

    right_lane = int(round(
      min(
        30.0,
        max(
          0.0,
          LANE_POS_CENTER - shift,
        ),
      )
    ))

    msg_161["LANELINE_LEFT_POSITION"] = left_lane
    msg_161["LANELINE_RIGHT_POSITION"] = right_lane

  if stopped_green_lanes_test:
    if disp_state is not None:
      if send_161:
        frame = disp_state.get("green_test_frame", 0)
        disp_state["green_test_values"] = ccnc_stopped_lane_handoff(frame)
        disp_state["green_test_frame"] = (frame + 1) % 400
      values = disp_state.get("green_test_values", ccnc_stopped_lane_handoff(0))
    else:
      values = ccnc_stopped_lane_handoff(0)
    msg_161.update(values)
    # Departure warnings retain priority even during the hidden-boundary phase.
    if hud.leftLaneDepart:
      msg_161["LANELINE_LEFT"] = 4
    if hud.rightLaneDepart:
      msg_161["LANELINE_RIGHT"] = 4

  if hud.leftLaneDepart or hud.rightLaneDepart:
    msg_162["VIBRATE"] = 1

  if openpilot_longitudinal_control:
    if msg_161["ALERTS_3"] in (1, 2, 3, 4, 7, 8, 9, 10):
      msg_161["ALERTS_3"] = 0

    if msg_161["ALERTS_5"] == 4:
      msg_161["ALERTS_5"] = 0

    if msg_161["SOUNDS_3"] == 5:
      msg_161["SOUNDS_3"] = 0

    msg_161.update({
      "SETSPEED": 3 if enabled else 1,

      "SETSPEED_HUD": (
        0 if not main_cruise_enabled
        else 2 if enabled
        else 1
      ),

      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled
        else (40 if is_metric else 25)
        if (
          s := round(
            out.vCruiseCluster
            * (1 if is_metric else CV.KPH_TO_MPH)
          )
        ) > (145 if is_metric else 90)
        else s
      ),

      "DISTANCE": hud.leadDistanceBars,

      "DISTANCE_SPACING": (
        0 if not main_cruise_enabled
        else 1 if enabled
        else 3
      ),

      "DISTANCE_LEAD": (
        0 if not main_cruise_enabled
        else 2 if enabled and hud.leadVisible
        else 1 if hud.leadVisible
        else 0
      ),

      "DISTANCE_CAR": (
        0 if not main_cruise_enabled
        else 2 if enabled
        else 1
      ),

      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = (
      0 if not main_cruise_enabled
      else 2 if enabled
      else 1
    )

    msg_162["LEAD_DISTANCE"] = msg_1b5["Longitudinal_Distance"]

  messages = []

  if send_161:
    messages.append(
      packer.make_can_msg(
        "CCNC_0x161",
        CAN.ECAN,
        msg_161,
      )
    )

  if send_162:
    messages.append(
      packer.make_can_msg(
        "CCNC_0x162",
        CAN.ECAN,
        msg_162,
      )
    )

  return messages


def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control,
                       lead_data: CanFdLeadData, main_cruise_enabled, tuning, cruise_info=None):
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel  # noqa: F841
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)  # noqa: F841

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1 if main_cruise_enabled else 0,
    "StopReq": 1 if tuning.stopping else 0,
    "aReqValue": tuning.actual_accel,
    "aReqRaw": tuning.actual_accel,
    "VSetDis": set_speed,
    "JerkLowerLimit": tuning.jerk_lower,
    "JerkUpperLimit": tuning.jerk_upper,

    "ACC_ObjDist": int(lead_data.lead_distance),
    "ACC_ObjRelSpd": lead_data.lead_rel_speed,
    "ObjValid": int(not lead_data.lead_visible),
    "SCC_ObjSta": 0 if not (enabled and lead_data.lead_visible) else (1 if gas_override else 2),
    "SET_ME_2": 0x4,
    "SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars,
  }
  if cruise_info:
    values.update({s: cruise_info[s] for s in ["ACC_ObjDist", "ACC_ObjRelSpd"]})

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(packer, CAN, frame):
  ret = []

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret


def create_adrv_messages(packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("ADRV_0x51", CAN.ACAN, values))

  ret.extend(create_fca_warning_light(packer, CAN, frame))

  if frame % 5 == 0:
    values = {
      'SET_ME_1C': 0x1c,
      'SET_ME_FF': 0xff,
      'SET_ME_TMP_F': 0xf,
      'SET_ME_TMP_F_2': 0xf,
    }
    ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

    values = {
      'SET_ME_E1': 0xe1,
      'SET_ME_3A': 0x3a,
    }
    ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

  if frame % 20 == 0:
    values = {
      'SET_ME_15': 0x15,
    }
    ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

  if frame % 100 == 0:
    values = {
      'SET_ME_22': 0x22,
      'SET_ME_41': 0x41,
    }
    ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret


def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc
