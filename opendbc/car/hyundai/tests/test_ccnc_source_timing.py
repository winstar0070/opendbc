import copy
from types import SimpleNamespace
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.ccnc_model import CcncLaneDisplay, LaneModelSample
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
    self.controller.ccnc_display = CcncLaneDisplay()
    self.controller.ccnc_model = None

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

  def test_stationary_demo_is_removed(self):
    self.cs.out.vEgo = 0.0
    self.controller.lfa_icon = 0
    for frame in range(600):
      _, data, _ = self.display_messages(frame, updated_161=True)[0]
      values = decode("CCNC_0x161", 0x161, data.hex())
      self.assertEqual((values["LFA_ICON"], values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (0, 0, 0))
      self.assertEqual((values["LANE_LEFT"], values["LANE_RIGHT"], values["LANE_HIGHLIGHT"]), (0, 0, 0))

  def test_model_geometry_and_stock_fallback(self):
    self.controller.ccnc_model = LaneModelSample(1.0, (-4.8, -1.2, 2.4, 6.0), 0, 0)
    _, data, _ = self.display_messages(0, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    self.assertEqual((values["LANELINE_LEFT_POSITION"], values["LANELINE_RIGHT_POSITION"]), (10, 20))
    self.assertEqual(values["LANELINE_CURVATURE"], self.msg_161["LANELINE_CURVATURE"])
    self.controller.ccnc_model = None
    self.cs.msg_161 = copy.copy(self.msg_161)
    _, data, _ = self.display_messages(1, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    for key in ("LANELINE_LEFT_POSITION", "LANELINE_RIGHT_POSITION", "LANELINE_LEFT", "LANE_HIGHLIGHT"):
      self.assertEqual(values[key], self.msg_161[key])

  def test_source_162_does_not_advance_model_display(self):
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    self.display_messages(0, updated_161=True)
    state = copy.deepcopy(vars(self.controller.ccnc_display))
    self.controller.ccnc_model = LaneModelSample(1.05, (-5.0, -1.4, 2.2, 5.8), 2, 1)
    self.display_messages(1, updated_162=True)
    self.assertEqual(vars(self.controller.ccnc_display), state)

  def test_model_display_preserves_departure_warnings_and_arrows(self):
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    self.hud.leftLaneDepart = self.hud.rightLaneDepart = True
    self.cc.leftBlinker = True
    messages = self.display_messages(0, updated_161=True, updated_162=True)
    values = decode("CCNC_0x161", 0x161, messages[0][1].hex())
    self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (4, 4))
    self.assertEqual((values["LCA_LEFT_ARROW"], values["LCA_RIGHT_ARROW"]), (2, 0))
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
