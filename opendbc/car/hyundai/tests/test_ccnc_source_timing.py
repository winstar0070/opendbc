import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags


DBC = "hyundai_canfd_generated"

# Stock frames from route 428c22dc6061cee7|00000115--9f58c41b0b.
STOCK_161 = "b7cce84000000000c0fff0c003000040000000000000000000ff000000000100"
STOCK_162 = "c9f3e82100000000c0ff00000000000000000000000000000000000802040000"


def decode(name, address, data):
  parser = CANParser(DBC, [(name, 20)], 2)
  parser.update([(1, [(address, bytes.fromhex(data), 2)])])
  return copy.copy(parser.vl[name])


class TestCcncSourceTiming(unittest.TestCase):
  def setUp(self):
    self.msg_161 = decode("CCNC_0x161", 0x161, STOCK_161)
    self.msg_162 = decode("CCNC_0x162", 0x162, STOCK_162)
    self.hud = SimpleNamespace(
      leftLaneVisible=True,
      rightLaneVisible=True,
      leftLaneDepart=False,
      rightLaneDepart=False,
      leadDistanceBars=3,
      leadVisible=False,
    )
    self.cs = SimpleNamespace(
      ccnc_0x161_updated=False,
      ccnc_0x162_updated=False,
      msg_161=copy.copy(self.msg_161),
      msg_162=copy.copy(self.msg_162),
      msg_1b5={
        "Info_LftLnPosVal": 1.0,
        "Info_RtLnPosVal": 2.0,
        "Info_LftLnQualSta": 3,
        "Info_RtLnQualSta": 3,
        "Longitudinal_Distance": 22.6,
      },
      is_metric=True,
      out=SimpleNamespace(
        steeringAngleDeg=0.0,
        vEgo=20.0,
        leftBlindspot=False,
        rightBlindspot=False,
        vCruiseCluster=0.0,
      ),
      main_cruise_enabled=False,
      buttons_counter=0,
      cruise_info={},
    )
    self.cc = SimpleNamespace(
      enabled=False,
      hudControl=self.hud,
      leftBlinker=False,
      rightBlinker=False,
      cruiseControl=SimpleNamespace(override=False, cancel=False, resume=False),
    )

    self.controller = CarController.__new__(CarController)
    self.controller.CP = SimpleNamespace(flags=HyundaiFlags.CCNC, openpilotLongitudinalControl=False)
    self.controller.CAN = CanBus(None, fingerprint={})
    self.controller.packer = CANPacker(DBC)
    self.controller.frame = 0
    self.controller.lkas_icon = 0
    self.controller.lfa_icon = 2
    self.controller.last_button_frame = 0
    self.controller.ccnc_disp = {}
    self.controller.lane_change_state = 0
    self.controller.lane_change_direction = 0

  def display_messages(self, frame, updated_161=False, updated_162=False):
    self.controller.frame = frame
    self.cs.ccnc_0x161_updated = updated_161
    self.cs.ccnc_0x162_updated = updated_162
    messages = self.controller.create_canfd_msgs(True, 20, 0.0, 0.0, False, self.hud, self.cs, self.cc)
    return [msg for msg in messages if msg[0] in (0x161, 0x162)]

  def test_sends_only_the_source_message_updated_this_control_cycle(self):
    for speed in (0.0, 0.099, 0.1, 20.0):
      with self.subTest(speed=speed):
        self.cs.out.vEgo = speed
        self.assertEqual([msg[0] for msg in self.display_messages(0, updated_161=True)], [0x161])
        self.assertEqual(self.display_messages(1), [])
        self.assertEqual([msg[0] for msg in self.display_messages(2, updated_162=True)], [0x162])
        self.assertEqual(self.display_messages(5), [])
        self.assertEqual([msg[0] for msg in self.display_messages(6, updated_161=True, updated_162=True)], [0x161, 0x162])

  def test_stopped_transition_is_mirrored_and_hides_lines_before_rebinding(self):
    self.cs.out.vEgo = 0.0
    self.controller.lfa_icon = 0
    self.hud.leftLaneVisible = self.hud.rightLaneVisible = False
    outputs = []
    for frame in range(1200):
      self.cs.msg_161 = copy.copy(self.msg_161)
      self.cs.msg_161["COUNTER"] = frame % 256
      _, data, _ = self.display_messages(frame, updated_161=True)[0]
      values = decode("CCNC_0x161", 0x161, data.hex())
      outputs.append(values)
      with self.subTest(frame=frame):
        self.assertEqual(values["COUNTER"], frame % 256)
        self.assertGreaterEqual(values["LANELINE_LEFT_POSITION"] + values["LANELINE_RIGHT_POSITION"], 30)
        self.assertLessEqual(values["LANELINE_LEFT_POSITION"] + values["LANELINE_RIGHT_POSITION"], 60)
        self.assertEqual(values["CENTERLINE"], 0)
        self.assertEqual(values["LANELINE_CURVATURE"], 15)
        for side in ("LEFT", "RIGHT"):
          if values[f"LANELINE_{side}_POSITION"] <= 8:
            self.assertEqual(values[f"LANELINE_{side}"], 1)
        if values["LANELINE_LEFT"] == values["LANELINE_RIGHT"] == 6:
          self.assertEqual(values["LANELINE_LEFT_POSITION"] + values["LANELINE_RIGHT_POSITION"], 30)
        phase = frame % 300
        moving = 40 <= phase < 220
        self.assertEqual(values["LCA_LEFT_ARROW"], 2 if moving and (frame // 300) % 2 == 0 else 0)
        self.assertEqual(values["LCA_RIGHT_ARROW"], 2 if moving and (frame // 300) % 2 == 1 else 0)
        if 40 <= phase < 125:
          self.assertEqual(values["LANE_HIGHLIGHT"], 0)
          self.assertEqual(values["LANE_LEFT"], int((frame // 300) % 2 == 0))
          self.assertEqual(values["LANE_RIGHT"], int((frame // 300) % 2 == 1))
        elif 125 <= phase < 135:
          self.assertEqual(values["LANE_HIGHLIGHT"], 1)
          self.assertEqual(values["LANE_HIGHLIGHT_DISTANCE"], 60.0)
          self.assertEqual(values["LANE_LEFT"], int((frame // 300) % 2 == 0))
          self.assertEqual(values["LANE_RIGHT"], int((frame // 300) % 2 == 1))
        else:
          highlight = 135 <= phase < 280
          self.assertEqual(values["LANE_HIGHLIGHT"], int(highlight))
          self.assertEqual(values["LANE_HIGHLIGHT_DISTANCE"], 60.0 if highlight else 0.0)
          self.assertEqual((values["LANE_LEFT"], values["LANE_RIGHT"]), (0, 0))
        if phase in (129, 130):
          self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (1, 1))
        elif phase >= 220 or phase < 40:
          color = 2 if phase < 40 or phase >= 280 else 6
          if 240 <= phase < 280:
            color = 2 if ((phase - 240) // 10) % 2 == 0 else 6
          self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (color, color))
          self.assertEqual((values["LANELINE_LEFT_POSITION"], values["LANELINE_RIGHT_POSITION"]), (15, 15))
    for frame in range(300):
      for key in ("LANELINE_LEFT", "LANELINE_LEFT_POSITION", "LANE_LEFT", "LCA_LEFT_ARROW"):
        self.assertEqual(outputs[frame][key], outputs[frame + 300][key.replace("LEFT", "RIGHT")])
      for key in outputs[frame]:
        if key not in ("COUNTER", "CHECKSUM"):
          self.assertEqual(outputs[frame][key], outputs[frame + 600][key])

    # Both directions travel through a complete lane, then keep travelling in
    # the same direction after rebinding instead of snapping back to 15/15.
    for start in (0, 300):
      side = "LEFT" if start == 0 else "RIGHT"
      positions = [outputs[i][f"LANELINE_{side}_POSITION"] for i in range(start + 40, start + 220)]
      self.assertEqual((min(positions), max(positions)), (0, 60))
      travel = 0
      wraps = 0
      for i in range(start + 41, start + 220):
        before, after = outputs[i - 1], outputs[i]
        delta = after[f"LANELINE_{side}_POSITION"] - before[f"LANELINE_{side}_POSITION"]
        if delta > 30:
          wraps += 1
          delta -= 60
          for values in (before, after):
            self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (1, 1))
            self.assertTrue(values["LANE_LEFT"] or values["LANE_RIGHT"] or values["LANE_HIGHLIGHT"])
        self.assertIn(delta, (-1, 0))
        travel -= delta
      self.assertEqual(wraps, 1)
      self.assertEqual(travel, 60)

  def test_stopped_push_range_is_adjustable_without_changing_rest_position(self):
    self.cs.out.vEgo = 0.0
    for radius in (30, 45):
      with self.subTest(radius=radius), patch("opendbc.car.hyundai.hyundaicanfd.CCNC_LANE_DEMO_OUTER_POSITION", radius):
        self.controller.ccnc_disp.clear()
        positions = []
        for frame in range(300):
          _, data, _ = self.display_messages(frame, updated_161=True)[0]
          values = decode("CCNC_0x161", 0x161, data.hex())
          positions.append(values["LANELINE_LEFT_POSITION"])
        self.assertEqual((min(positions), max(positions)), (0, radius))
        self.assertEqual((positions[0], positions[-1]), (15, 15))

  def test_stopped_animation_only_advances_on_source_161(self):
    self.cs.out.vEgo = 0.0
    for frame in range(80):
      self.display_messages(frame, updated_161=True)
    state = copy.deepcopy(self.controller.ccnc_disp)
    for frame in range(80, 90):
      self.assertEqual([m[0] for m in self.display_messages(frame, updated_162=True)], [0x162])
    self.assertEqual(self.controller.ccnc_disp, state)

  def test_stopped_animation_restarts_after_moving(self):
    self.cs.out.vEgo = 0.0
    for frame in range(100):
      self.display_messages(frame, updated_161=True)
    self.cs.out.vEgo = 0.1
    self.display_messages(100, updated_162=True)
    self.cs.out.vEgo = 0.0
    _, data, _ = self.display_messages(101, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    self.assertEqual((values["LANELINE_LEFT_POSITION"], values["LANELINE_RIGHT_POSITION"]), (15, 15))
    self.assertEqual((values["LANE_LEFT"], values["LANE_RIGHT"], values["LANE_HIGHLIGHT"]), (0, 0, 0))

  def test_green_line_test_stops_when_vehicle_moves(self):
    self.controller.lfa_icon = 0
    for frame, (speed, color) in enumerate(((0.099, 2), (0.1, 0), (1.0, 0), (20.0, 0), (0.0, 2))):
      with self.subTest(speed=speed):
        self.cs.out.vEgo = speed
        _, data, _ = self.display_messages(frame, updated_161=True)[0]
        values = decode("CCNC_0x161", 0x161, data.hex())
        self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (color, color))
        self.assertEqual(values["LANE_HIGHLIGHT"], 0)
        self.assertEqual(values["LANE_HIGHLIGHT_DISTANCE"], 0.0)

  def test_green_line_test_can_be_disabled(self):
    self.cs.out.vEgo = 0.0
    self.controller.lfa_icon = 0
    with patch("opendbc.car.hyundai.hyundaicanfd.CCNC_DEV_STOPPED_GREEN_LANES_TEST", False):
      _, data, _ = self.display_messages(0, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    self.assertEqual((values["LFA_ICON"], values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (0, 0, 0))

  def test_green_line_test_preserves_departure_warnings(self):
    self.cs.out.vEgo = 0.0
    for phase in (0, 130):
      for left_depart, right_depart in ((True, False), (False, True), (True, True)):
        with self.subTest(phase=phase, left=left_depart, right=right_depart):
          self.controller.ccnc_disp["green_test_frame"] = phase
          self.hud.leftLaneDepart = left_depart
          self.hud.rightLaneDepart = right_depart
          messages = self.display_messages(0, updated_161=True, updated_162=True)
          values = decode("CCNC_0x161", 0x161, messages[0][1].hex())
          normal_color = 2 if phase == 0 else 1
          self.assertEqual(values["LANELINE_LEFT"], 4 if left_depart else normal_color)
          self.assertEqual(values["LANELINE_RIGHT"], 4 if right_depart else normal_color)
          self.assertEqual(decode("CCNC_0x162", 0x162, messages[1][1].hex())["VIBRATE"], 1)

  def test_preserves_stock_phase_and_source_counters(self):
    expected = [(0, 0x161, 10), (2, 0x162, 40), (5, 0x161, 11), (7, 0x162, 41)]
    actual = []

    for frame, address, counter in expected:
      if address == 0x161:
        self.cs.msg_161 = copy.copy(self.msg_161)
        self.cs.msg_161["COUNTER"] = counter
      else:
        self.cs.msg_162 = copy.copy(self.msg_162)
        self.cs.msg_162["COUNTER"] = counter

      for msg_address, data, _ in self.display_messages(frame, address == 0x161, address == 0x162):
        name = "CCNC_0x161" if msg_address == 0x161 else "CCNC_0x162"
        actual.append((frame, msg_address, int(decode(name, msg_address, data.hex())["COUNTER"])))

    self.assertEqual(actual, expected)


if __name__ == "__main__":
  unittest.main()
