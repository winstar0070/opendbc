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
    _stage = (_cnt // 120) % 8   # 8 stages, ~6s each at ~20Hz (holds the ~5s slide + margin)
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
  # Green BACKGROUND fill (LANE_LEFT/RIGHT = the coloured area inside the lane, not
  # the lines). User's model: the green starts on the TARGET side and fills toward
  # CENTER as the change progresses, so by completion the car sits in the middle
  # of the green. LANE_* has no position, only left/right half on/off -- so we
  # approximate "left -> center" as: target half first, then BOTH halves once the
  # slide passes FILL_PROG (green has reached center), and both when landed.
  FILL_PROG = 0.5   # past this the green has filled to center -> light both halves
  _fill_left = False
  _fill_right = False
  if disp_state is not None:
    _lc_landed = disp_state.get("lc_landed", False)
    _lc_prog = disp_state.get("lc_prog", 0.0)
    _lc_dir = disp_state.get("lc_dir", 0)
    _changing = (_lc_prog > 0.0 or _lc_dir)
    if _lc_landed and _lc_dir:
      _fill_left = _fill_right = True             # complete: green centered
    elif _changing:
      _reached_center = _lc_prog >= FILL_PROG
      if _lc_dir == 1:                            # left change
        _fill_left = True
        _fill_right = _reached_center             # fills to center in 2nd half
      elif _lc_dir == 2:                          # right change
        _fill_right = True
        _fill_left = _reached_center
  elif lane_change_state in (2, 3):
    if lane_change_direction == 1:
      _fill_left = True
    elif lane_change_direction == 2:
      _fill_right = True

  # Lane-line visibility/colour rule (simple):
  #   normal      -> WHITE (2)
  #   CHANGING    -> BOTH lines HIDDEN (1). Once the green is up we hide the lines
  #                  entirely; the green background carries the motion.
  #   LANDED      -> BOTH lines RETURN and blink WHITE<->GREEN (~1s) as the
  #                  "change complete" cue, then settle to white.
  # 0x161 ~20 Hz -> (COUNTER // 20) % 2 toggles ~1 Hz.
  _landed_now = disp_state is not None and disp_state.get("lc_landed", False)
  _blink_phase_green = (int(msg_161["COUNTER"]) // 20) % 2 == 0
  _prev_prog = disp_state.get("lc_prog", 0.0) if disp_state is not None else 0.0
  _prev_dir = disp_state.get("lc_dir", 0) if disp_state is not None else 0
  _changing_now = (_prev_prog > 0.0 or _prev_dir) and not _landed_now

  def _laneline_color(visible, depart):
    if not lfa_icon:
      return 0
    if not visible:
      return 1
    if depart:
      return 4
    if _changing_now:
      return 1                                 # HIDDEN while the green is sliding
    if _landed_now:
      return 6 if _blink_phase_green else 2   # returned + white<->green blink
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
    # Green background fill: target side first, then both halves as it reaches
    # center (see _fill_left/_fill_right), so the green slides from the target
    # side into the middle where the car sits.
    "LANE_LEFT": 1 if (lfa_icon and _fill_left) else 0,
    "LANE_RIGHT": 1 if (lfa_icon and _fill_right) else 0,
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
    PROG_STEP = 0.01         # per-0x161-frame ramp; 100 frames (~5s) end to end (a lane change takes ~5s)
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
    # Rest pose = 15/15. During a change we SHIFT the lane pair to one side while
    # keeping the sum on 30 (constant width) -- an asymmetric slide, not a spread:
    # one line moves toward center, the other away by the same amount, so the pair
    # rides sideways (the crossing) without the lanes widening or leaving the
    # screen. BIAS caps how far it shifts so the inner line never reaches the car
    # center (0) and the outer line never rides off-screen (30) -- both of those
    # extremes left the car with no lane around it on-vehicle.
    # On LANDING we return to 15/15: the crossed-into lane is now the ego lane and
    # must wrap the car at normal width (holding the shifted pose left the car with
    # one line under it and the other off-screen). Green->white blink marks it.
    # LANE_POS_SIGN sets which screen side a left/right change shifts toward
    # (re-verify with a frame sim whenever this mapping changes).
    LANE_POS_CENTER = 15.0
    LANE_POS_BIAS = 12.0     # max shift; inner->3, outer->27 at full push
    LANE_POS_SIGN = -1       # (user did not ask to flip; restored to -1)
    if landed:
      # crossed-into lane becomes the ego lane: normal-width pair around the car.
      left_lane = 15
      right_lane = 15
    else:
      dir_sign = 1.0 if held_dir == 1 else -1.0 if held_dir == 2 else 0.0
      shift = LANE_POS_SIGN * dir_sign * prog * LANE_POS_BIAS
      # left line moves by +shift, right by -shift: sum stays 30 (width constant),
      # the pair slides toward one side.
      left_lane = int(round(min(30.0, max(0.0, LANE_POS_CENTER + shift))))
      right_lane = int(round(min(30.0, max(0.0, LANE_POS_CENTER - shift))))
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
