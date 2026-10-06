import copy
from types import SimpleNamespace

import itertools
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.hyundaicanfd import create_ccnc, hkg_can_fd_checksum


DBC = "hyundai_canfd_generated"


def decode(name, message):
  address, data, bus = message
  parser = CANParser(DBC, [(name, 0)], bus)
  parser.update([(1, [(address, data, bus)])])
  return dict(parser.vl[name])


class TestCcncLeadAvatar(unittest.TestCase):
  def test_front_lead_avatar_preserves_gates_distance_and_other_slots(self):
    for openpilot_long, (main_cruise, enabled), lead_visible, distance in itertools.product(
      [False, True], [(False, False), (False, True), (True, False), (True, True)], [False, True], [0., 22.6, 204.7],
    ):
      with self.subTest(openpilot_long=openpilot_long, main_cruise=main_cruise, enabled=enabled,
                        lead_visible=lead_visible, distance=distance):
        self.check_avatar(openpilot_long, main_cruise, enabled, lead_visible, distance)

  def check_avatar(self, openpilot_long, main_cruise, enabled, lead_visible, distance):
    packer = CANPacker(DBC)
    msg_161 = decode("CCNC_0x161", packer.make_can_msg("CCNC_0x161", 0, {}))
    stock = {
      "LEAD": 7, "LEAD_DISTANCE": 13.2, "LEAD_LATERAL": 1.1,
      "LEAD_ALT": 2, "LEAD_ALT_DISTANCE": 33.2, "LEAD_ALT_LATERAL": 1.2,
      "LEAD_LEFT": 2, "LEAD_LEFT_DISTANCE": 19.1, "LEAD_LEFT_LATERAL": 3.4,
      "LEAD_RIGHT": 1, "LEAD_RIGHT_DISTANCE": 24.3, "LEAD_RIGHT_LATERAL": 3.1,
      "LEAD_LEFT_REAR_STATUS": 1, "LEAD_RIGHT_REAR_STATUS": 2,
    }
    msg_162 = decode("CCNC_0x162", packer.make_can_msg("CCNC_0x162", 0, stock))
    camera = {"Longitudinal_Distance": distance}
    originals = copy.deepcopy((msg_161, msg_162, camera))
    hud = SimpleNamespace(leftLaneDepart=False, rightLaneDepart=False, leadDistanceBars=3, leadVisible=lead_visible)
    out = SimpleNamespace(vEgo=20., vCruiseCluster=80., leftBlindspot=False, rightBlindspot=False)
    messages = create_ccnc(packer, SimpleNamespace(ECAN=0), openpilot_long, enabled, hud, False, False,
                           msg_161, msg_162, camera, True, out, main_cruise, 0)
    assert [m[0] for m in messages] == [0x161, 0x162]
    result = decode("CCNC_0x162", messages[1])
    expected = (0 if not main_cruise else 4 if enabled else 3) if openpilot_long else 7
    assert result["LEAD"] == expected
    self.assertAlmostEqual(result["LEAD_DISTANCE"], distance if openpilot_long else 13.2)
    for key in stock.keys() - {"LEAD", "LEAD_DISTANCE"}:
      self.assertAlmostEqual(result[key], stock[key])
    assert (msg_161, msg_162, camera) == originals
    for address, data, bus in messages:
      assert bus == 0
      assert int.from_bytes(data[:2], "little") == hkg_can_fd_checksum(address, None, data)
