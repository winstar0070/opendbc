import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.lead_data_ext import CanFdLeadData


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

  # TEMP-DEV (REVERT BEFORE VEHICLE USE): stopped-only lane-change animation test,
  # AUTOMATIC (no blinker needed). This car has no factory auto-lane-change, so
  # while parked we synthesize lane_change_state on a timer and auto-cycle:
  #   off -> LEFT(starting,finishing) -> off -> RIGHT(starting,finishing) -> off
  # driven by the stock 0x161 COUNTER. Forces lfa_icon + send_161.
  # Display (HUD) only; no steering/control effect. REVERT before vehicle use.
  # Only active while essentially parked; auto-disables once the car is moving so
  # it never overlays the real lane display while driving.
  CCNC_DEV_STOPPED_LANECHANGE_TEST = True
  CCNC_DEV_TEST_MAX_SPEED = 2.0  # m/s (~7 km/h); above this the test is off
  if CCNC_DEV_STOPPED_LANECHANGE_TEST and out.vEgo < CCNC_DEV_TEST_MAX_SPEED:
    lfa_icon = 2
    send_161 = True
    _cnt = int(msg_161["COUNTER"])
    _stage = (_cnt // 40) % 8   # 8 stages, ~2s each at ~20Hz
    # 0 off, 1 L-start, 2 L-finish, 3 off, 4 off, 5 R-start, 6 R-finish, 7 off
    if _stage in (1, 2):        # LEFT: starting(2), finishing(3)
      lane_change_state = 2 if _stage == 1 else 3
      lane_change_direction = 1
      left_blinker, right_blinker = True, False   # show left arrow
    elif _stage in (5, 6):      # RIGHT: starting(2), finishing(3)
      lane_change_state = 2 if _stage == 5 else 3
      lane_change_direction = 2
      left_blinker, right_blinker = False, True   # show right arrow
    else:
      lane_change_state, lane_change_direction = 0, 0
      left_blinker, right_blinker = False, False

  any_blinker = left_blinker or right_blinker
  curvature = {i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i for i in range(-15, 16)}

  # Green-fill direction: held while the lane-change animation is active OR easing
  # back (disp_state), else the current lane_change_direction. Keeps green lit
  # through the ease-out.
  _green_dir = 0
  if disp_state is not None and not disp_state.get("lc_landed", False) \
     and (disp_state.get("lc_prog", 0.0) > 0.0 or disp_state.get("lc_dir", 0)):
    # green only while crossing; once landed the lane solidifies (green off)
    _green_dir = disp_state.get("lc_dir", 0)
  elif disp_state is None and lane_change_state in (2, 3):
    _green_dir = lane_change_direction

  # Lane-line COLOUR for the just-crossed-into lane. Normal driving = WHITE (2);
  # while a change is IN PROGRESS the lines are GREEN (6). After LANDING (crossed
  # lane is now the ego lane) blink the lines WHITE<->GREEN on a ~1s cadence as a
  # "transition complete" cue -- this must WIN over the blinker's green-hold, since
  # the blinker is usually still on right after landing. 0x161 runs ~20 Hz, so
  # (COUNTER // 20) % 2 toggles about once per second.
  _landed_now = disp_state is not None and disp_state.get("lc_landed", False)
  _blink_phase_green = (int(msg_161["COUNTER"]) // 20) % 2 == 0
  # Line colour: hidden if not visible, orange on depart; after landing the
  # blink decides white/green; while still crossing it's solid green; else white.

  def _laneline_color(visible, depart):
    if not lfa_icon:
      return 0
    if not visible:
      return 1
    if depart:
      return 4
    if _landed_now:
      return 6 if _blink_phase_green else 2   # post-landing white<->green blink
    if any_blinker:
      return 6                                 # solid green while crossing
    return 2                                   # normal white

  msg_161.update({
    "DAW_ICON": 0,
    "LKA_ICON": 0,
    "LFA_ICON": 2 if lfa_icon else 0,
    "CENTERLINE": 1 if lfa_icon else 0,
    "LANELINE_CURVATURE": curvature[max(-15, min(int(out.steeringAngleDeg / 4.5), 15))] if lfa_icon and not any_blinker else 15,
    "LANELINE_LEFT": _laneline_color(hud.leftLaneVisible, hud.leftLaneDepart),
    "LANELINE_RIGHT": _laneline_color(hud.rightLaneVisible, hud.rightLaneDepart),
    "LCA_LEFT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.leftBlindspot else 2 if any_blinker else 4),
    "LCA_RIGHT_ICON": (0 if not lfa_icon or out.vEgo < LANE_CHANGE_SPEED_MIN else 1 if out.rightBlindspot else 2 if any_blinker else 4),
    "LCA_LEFT_ARROW": 2 if left_blinker else 0,
    "LCA_RIGHT_ARROW": 2 if right_blinker else 0,
    # Fill the target lane area green during the change AND while it eases back to
    # center (so the green persists until the lane visually recenters). Uses the
    # held direction/progress tracked in disp_state (prior frame) so it stays lit
    # through the ease-out, not just while lane_change_state is 2/3.
    "LANE_LEFT": 1 if (lfa_icon and _green_dir == 1) else 0,
    "LANE_RIGHT": 1 if (lfa_icon and _green_dir == 2) else 0,
  })

  # Lane-change lane animation (car icon stays centered; the LANES move).
  # Driven purely by the plumbed lane-change state -- this car has no factory
  # auto-lane-change, so there is no stock source geometry to follow.
  #   left change:  push lanes RIGHT, green fills the LEFT lane.
  #   right change: push lanes LEFT,  green fills the RIGHT lane.
  # Keep pushing for the WHOLE change (starting(2) AND finishing(3)) so the green
  # target lane reaches center; on the change ending (off) snap-reset so the green
  # lane reads as the new ego lane. LANE_POS_SIGN flips the on-screen direction if
  # the cluster maps position the opposite way (confirm on-vehicle).
  # LANELINE_LEFT_POSITION is 6-bit (0..63); we keep left+right centered on 30.
  if lfa_icon:
    # Linear-progress animation so the slide REACHES the edge (not just approaches
    # it) and the recenter is smooth instead of a hard snap.
    #   prog 0..1 ramps UP while changing (green target lane slides to center),
    #   then ramps DOWN to 0 after the change ends (the green lane, now centered,
    #   eases back to the neutral ego view). Position = 15 + dir_sign * prog * 15,
    #   so prog=1 hits the edge (0 or 30) exactly.
    # LANE_POS_SIGN = -1 (on-vehicle: slide direction was inverted).
    LANE_POS_SIGN = 1
    PROG_STEP = 0.03         # per-0x161-frame ramp; ~33 frames (~1.7s) end to end (slower/smoother)
    changing = lane_change_state in (2, 3)
    # direction of travel: left(1) or right(2); hold last direction while easing out
    if changing and lane_change_direction in (1, 2):
      cur_dir = lane_change_direction
    else:
      cur_dir = 0

    if disp_state is not None:
      if send_161:
        prog = disp_state.get("lc_prog", 0.0)
        held_dir = disp_state.get("lc_dir", 0)
        landed = disp_state.get("lc_landed", False)
        if changing and not landed:
          held_dir = cur_dir
          prog = min(1.0, prog + PROG_STEP)      # slide toward the target lane
          if prog >= 1.0:
            # reached the target lane -> LAND: the green lane becomes the ego lane.
            # Solidify (green off) and treat this spot as the new center; do NOT
            # glide back, which would look like returning instead of crossing.
            landed = True
        if not changing:
          # change ended -> settled on the new lane; clear for the next change.
          prog = 0.0
          held_dir = 0
          landed = False
        disp_state["lc_prog"] = prog
        disp_state["lc_dir"] = held_dir
        disp_state["lc_landed"] = landed
      else:
        prog = disp_state.get("lc_prog", 0.0)
        held_dir = disp_state.get("lc_dir", 0)
        landed = disp_state.get("lc_landed", False)
    else:
      prog = 1.0 if changing else 0.0
      held_dir = cur_dir
      landed = False

    # held_dir: 1=left change (target lane is on the LEFT), 2=right change.
    # Rest pose = 15/15 (normal lane width, centered). During a change we use the
    # 6-bit headroom (0..63) so the ORIGINAL ego lane slides fully off-screen and
    # the target (green) lane reaches center -- a real crossing, not just a nudge.
    #   The lanes are NOT summed on a constant: at prog=1 the inner line goes to 0
    #   and the outer line goes to LANE_POS_MAX, so the whole pair rides off toward
    #   one side (the original lane exits, the target lane centers). Rest (prog=0)
    #   stays a symmetric 15/15 so straight driving looks normal.
    # LANE_POS_SIGN = 1 (on-vehicle: correct after the inner/outer rework; the
    # 6-bit-headroom split reversed the earlier -1 mapping).
    LANE_POS_REST = 15.0     # symmetric rest position (normal width)
    LANE_POS_INNER_END = 0.0    # inner line slides to the screen center at prog=1
    LANE_POS_OUTER_END = 60.0   # outer line rides out near the 6-bit max (<=63)
    if landed:
      # Landed: the crossed-into (green) lane is now the ego lane. Hold the pushed
      # pose (do NOT snap back to 15/15, which reads as returning). Green turns off
      # via _green_dir so the new lane solidifies in place.
      _p = 1.0
    else:
      _p = prog
    dir_sign = 1.0 if held_dir == 1 else -1.0 if held_dir == 2 else 0.0
    # inner/outer targets ramp from the 15/15 rest pose out to the crossing pose.
    _inner = LANE_POS_REST + _p * (LANE_POS_INNER_END - LANE_POS_REST)   # 15 -> 0
    _outer = LANE_POS_REST + _p * (LANE_POS_OUTER_END - LANE_POS_REST)   # 15 -> 60
    if LANE_POS_SIGN * dir_sign >= 0:
      # left line is the inner (screen-center) line, right line rides out
      left_lane = int(round(min(63.0, max(0.0, _inner))))
      right_lane = int(round(min(63.0, max(0.0, _outer))))
    else:
      left_lane = int(round(min(63.0, max(0.0, _outer))))
      right_lane = int(round(min(63.0, max(0.0, _inner))))
    msg_161["LANELINE_LEFT_POSITION"] = left_lane
    msg_161["LANELINE_RIGHT_POSITION"] = right_lane
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
      "SETSPEED_HUD": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SETSPEED_SPEED": (
        255 if not main_cruise_enabled else
        (40 if is_metric else 25) if (s := round(out.vCruiseCluster * (1 if is_metric else CV.KPH_TO_MPH))) > (145 if is_metric else 90) else s
      ),
      "DISTANCE": hud.leadDistanceBars,
      "DISTANCE_SPACING": 0 if not main_cruise_enabled else 1 if enabled else 3,
      "DISTANCE_LEAD": 0 if not main_cruise_enabled else 2 if enabled and hud.leadVisible else 1 if hud.leadVisible else 0,
      "DISTANCE_CAR": 0 if not main_cruise_enabled else 2 if enabled else 1,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": 0,
    })

    msg_162["LEAD"] = 0 if not main_cruise_enabled else 2 if enabled else 1
    msg_162["LEAD_DISTANCE"] = msg_1b5["Longitudinal_Distance"]

  messages = []
  if send_161:
    messages.append(packer.make_can_msg("CCNC_0x161", CAN.ECAN, msg_161))
  if send_162:
    messages.append(packer.make_can_msg("CCNC_0x162", CAN.ECAN, msg_162))
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
