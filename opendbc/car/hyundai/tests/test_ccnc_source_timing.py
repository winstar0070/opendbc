import copy
from types import SimpleNamespace
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.ccnc_model import CcncLaneDisplay, LaneModelSample
from opendbc.car.hyundai.ccnc_objects import CcncObjectDisplay, ObjectLanes, RadarObject, RadarObjects
from opendbc.car.hyundai.values import CAR, HyundaiFlags


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
      ccnc_camera_time_nanos=1_000_000_000,
      ccnc_display_time_nanos=1_000_000_000,
      msg_161=copy.copy(self.msg_161),
      msg_162=copy.copy(self.msg_162),
      msg_1b5={
        "Info_LftLnPosVal": -1.8,
        "Info_RtLnPosVal": 1.8,
        "Info_LftLnQualSta": 3,
        "Info_RtLnQualSta": 3,
        "Longitudinal_Distance": 22.6,
      },
      is_metric=True,
      out=SimpleNamespace(
        steeringAngleDeg=0.0,
        vEgo=20.0,
        leftBlinker=False,
        rightBlinker=False,
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
      latActive=False,
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
    self.controller.ccnc_object_display = CcncObjectDisplay()
    self.controller.ccnc_radar = None
    self.controller.ccnc_object_lanes = None

  def display_messages(self, frame, updated_161=False, updated_162=False):
    self.controller.frame = frame
    self.cs.ccnc_0x161_updated = updated_161
    self.cs.ccnc_0x162_updated = updated_162
    messages = self.controller.create_canfd_msgs(True, 20, 0.0, 0.0, False, self.hud, self.cs, self.cc,
                                                now_nanos=1_000_000_000 + frame * 10_000_000)
    return [msg for msg in messages if msg[0] in (0x161, 0x162)]

  def test_completion_hold_raw_packet_and_live_gates(self):
    for gate in ('expiry', 'lat', 'icon', 'speed', 'new'):
      self.setUp()
      self.cc.latActive = True
      self.cs.out.leftBlinker = True
      for i in range(73):
        offset = i * .05
        lanes = tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4))
        if offset >= 1.8:
          lanes = (lanes[0] - 3.6, *lanes[:3])
        self.controller.ccnc_model = LaneModelSample(1 + i * .05, lanes, 3 if i >= 70 else 2, 1)
        self.display_messages(i * 5, updated_161=True)
      # The turn signal can remain on after completing the maneuver.
      self.controller.ccnc_model = LaneModelSample(4.65, (-5.4, -1.8, 1.8, 5.4), 0, 0)
      msg = self.display_messages(365, updated_161=True)[0]
      packed = decode('CCNC_0x161', 0x161, msg[1].hex())
      self.assertEqual(packed['LANE_HIGHLIGHT'], 1)
      self.assertEqual(packed['LCA_LEFT_ARROW'], 0)
      self.assertEqual(packed['LCA_RIGHT_ARROW'], 0)
      self.assertNotEqual(packed['LCA_LEFT_ICON'], 2)
      frame = 370
      if gate == 'lat':
        self.cc.latActive = False
      elif gate == 'icon':
        self.controller.lfa_icon = 0
      elif gate == 'speed':
        self.cs.out.vEgo = 0
      elif gate == 'new':
        self.cs.out.rightBlinker = True
      else:
        # Advance fresh samples throughout the hold, rather than simulating
        # a stale source expiry when testing the one-second deadline.
        for frame in range(370, 465, 5):
          self.controller.ccnc_model = LaneModelSample(1 + frame * .01, (-5.4, -1.8, 1.8, 5.4), 0, 0)
          self.display_messages(frame, updated_161=True)
        frame = 465
      self.controller.ccnc_model = LaneModelSample(1 + frame * .01, (-5.4, -1.8, 1.8, 5.4), 1 if gate == 'new' else 0, 2 if gate == 'new' else 0)
      msg = self.display_messages(frame, updated_161=True)[0]
      self.assertIsNone(self.controller.ccnc_display.values)
      self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LANE_HIGHLIGHT'], self.msg_161['LANE_HIGHLIGHT'])

  def test_missing_model_does_not_rearm_after_a_long_gap(self):
    self.cc.latActive = True
    self.cs.out.leftBlinker = True
    self.cs.ccnc_camera_time_nanos = 0
    self.controller.ccnc_model = LaneModelSample(1., (-3.7, -.1, 3.5, 7.1), 2, 1)
    self.display_messages(0, updated_161=True)
    self.controller.ccnc_model = None
    self.display_messages(30, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.35, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    msg = self.display_messages(35, updated_161=True)[0]
    self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LANE_LEFT'], 0)
    # A known cancellation, unlike missing data, ends the maneuver.
    self.controller.ccnc_model = LaneModelSample(1.4, (-5.4, -1.8, 1.8, 5.4), 0, 0)
    self.display_messages(40, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.45, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    msg = self.display_messages(45, updated_161=True)[0]
    self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LANE_LEFT'], 1)

  def test_brief_missing_model_retains_target_across_index_switch(self):
    self.cc.latActive = True
    self.cs.out.leftBlinker = True
    self.cs.ccnc_camera_time_nanos = 0
    self.controller.ccnc_model = LaneModelSample(1., (-3.75, -.15, 3.45, 7.05), 2, 1)
    self.display_messages(0, updated_161=True)
    self.controller.ccnc_model = None
    self.display_messages(5, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.1, (-7.3, -3.7, -.1, 3.5), 2, 1)
    msg = self.display_messages(10, updated_161=True)[0]
    values = decode('CCNC_0x161', 0x161, msg[1].hex())
    self.assertEqual(values['LANE_LEFT'], 1)
    self.assertLess(values['LANELINE_LEFT_POSITION'], 5)

  def test_eligibility_gap_retains_association_and_republishes_held_sample(self):
    for gate in ('blinker', 'lat', 'speed', 'lfa'):
      with self.subTest(gate=gate):
        self.setUp()
        self.cc.latActive = True
        self.cs.out.leftBlinker = True
        self.cs.ccnc_camera_time_nanos = 0
        self.controller.ccnc_model = LaneModelSample(1., (-3.75, -.15, 3.45, 7.05), 2, 1)
        self.display_messages(0, updated_161=True)
        if gate == 'blinker':
          self.cs.out.leftBlinker = False
        elif gate == 'lat':
          self.cc.latActive = False
        elif gate == 'speed':
          self.cs.out.vEgo = 0.
        else:
          self.controller.lfa_icon = 0
        self.controller.ccnc_model = LaneModelSample(1.05, (-7.3, -3.7, -.1, 3.5), 2, 1)
        self.display_messages(5, updated_161=True)
        self.assertIsNone(self.controller.ccnc_display.values)
        self.cc.latActive = self.cs.out.leftBlinker = True
        self.cs.out.vEgo, self.controller.lfa_icon = 20., 2
        # Same model timestamp, only publication eligibility changed.
        msg = self.display_messages(6, updated_161=True)[0]
        values = decode('CCNC_0x161', 0x161, msg[1].hex())
        self.assertEqual(values['LANE_LEFT'], 1)
        self.assertLess(values['LANELINE_LEFT_POSITION'], 5)

  def test_stale_or_future_cancellation_cannot_unlock_a_lost_target(self):
    for timestamp in (1., 3.):
      with self.subTest(timestamp=timestamp):
        self.setUp()
        self.cc.latActive = self.cs.out.leftBlinker = True
        self.cs.ccnc_camera_time_nanos = 0
        self.controller.ccnc_model = LaneModelSample(1., (-5.4, -1.8, 1.8, 5.4), 2, 1)
        self.display_messages(0, updated_161=True)
        self.controller.ccnc_model = None
        self.display_messages(30, updated_161=True)
        self.controller.ccnc_model = LaneModelSample(timestamp, (-5.4, -1.8, 1.8, 5.4), 0, 0)
        self.display_messages(35, updated_161=True)
        self.controller.ccnc_model = LaneModelSample(1.4, (-5.4, -1.8, 1.8, 5.4), 2, 1)
        msg = self.display_messages(40, updated_161=True)[0]
        self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LANE_LEFT'], 0)

  def test_fresh_age_but_out_of_order_cancel_cannot_rearm_display(self):
    self.cc.latActive = self.cs.out.leftBlinker = True
    self.cs.ccnc_camera_time_nanos = 0
    self.controller.ccnc_model = LaneModelSample(1.05, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    self.display_messages(5, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.1, (), 2, 1)
    self.display_messages(10, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.15, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    self.display_messages(15, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.1, (-5.4, -1.8, 1.8, 5.4), 0, 0)
    self.display_messages(20, updated_161=True)
    self.controller.ccnc_model = LaneModelSample(1.25, (-5.4, -1.8, 1.8, 5.4), 2, 1)
    msg = self.display_messages(25, updated_161=True)[0]
    self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LANE_LEFT'], 0)

  def test_native_adjacent_and_rear_objects_survive_without_radar(self):
    for side in ('LEFT', 'RIGHT'):
      self.cs.msg_162.update({f'LEAD_{side}': 4, f'LEAD_{side}_DISTANCE': 12., f'LEAD_{side}_LATERAL': 3.5,
                             f'LEAD_{side}_REAR_STATUS': 2, f'LEAD_{side}_REAR_DISTANCE': 5.,
                             f'LEAD_{side}_REAR_LATERAL': 3.5})
    msg = self.display_messages(0, updated_162=True)[0]
    values = decode('CCNC_0x162', 0x162, msg[1].hex())
    for key, value in self.cs.msg_162.items():
      if key.startswith(('LEAD_LEFT', 'LEAD_RIGHT')):
        self.assertAlmostEqual(values[key], value)

  def test_cutin_fallback_uses_camera_source_age(self):
    for camera_time, expected in ((1_000_000_000, 4), (700_000_000, 4), (1_100_000_000, 4), (0, 4)):
      with self.subTest(camera_time=camera_time):
        self.setUp()
        self.controller.CP.openpilotLongitudinalControl = True
        self.cc.enabled = True
        self.cs.main_cruise_enabled = True
        self.cs.msg_162['LEAD'] = 0
        lines = tuple(((0., y), (80., y)) for y in (-5.4, -1.8, 1.8, 5.4))
        self.controller.ccnc_radar = RadarObjects(1.01, (RadarObject(1, 20., -3.6),))
        self.controller.ccnc_object_lanes = ObjectLanes(1.01, lines)
        self.display_messages(1, updated_162=True)
        self.cs.ccnc_camera_time_nanos = camera_time
        self.controller.ccnc_radar = RadarObjects(1.05, (RadarObject(1, 20., -.3),))
        self.controller.ccnc_object_lanes = ObjectLanes(1.05, lines)
        msg = self.display_messages(5, updated_162=True)[0]
        result = decode('CCNC_0x162', 0x162, msg[1].hex())
        self.assertEqual(result['LEAD'], expected)
        self.assertAlmostEqual(result['LEAD_DISTANCE'], 22.6 if camera_time == 1_000_000_000 else 20.)

  def test_camera_animation_without_model_and_with_missing_outer_lane(self):
    self.cs.out.leftBlinker = True
    self.cc.latActive = True
    for model in (None, LaneModelSample(1., (None, -1.8, 1.8, 5.4), 2, 1)):
      self.controller.ccnc_model = model
      self.cs.msg_1b5.update({"Info_LftLnPosVal": -1.2, "Info_RtLnPosVal": 2.4})
      msg = self.display_messages(0, updated_161=True)[0]
      values = decode('CCNC_0x161', 0x161, msg[1].hex())
      self.assertEqual((values['LANELINE_LEFT_POSITION'], values['LANELINE_RIGHT_POSITION']), (10, 20))
      self.assertEqual((values['LCA_LEFT_ARROW'], values['LCA_RIGHT_ARROW']), (2, 0))
      self.assertEqual(values['LANELINE_LEFT'], 6)

  def test_camera_fallback_follows_motion_and_clears_with_blinker(self):
    for direction in (1, 2):
      self.cs.out.leftBlinker, self.cs.out.rightBlinker = direction == 1, direction == 2
      for frame, left in enumerate((-1.8, -1.2, -0.6)):
        self.cs.msg_1b5.update({'Info_LftLnPosVal': left, 'Info_RtLnPosVal': left + 3.6})
        msg = self.display_messages(frame, updated_161=True)[0]
        values = decode('CCNC_0x161', 0x161, msg[1].hex())
        self.assertEqual(values['LANELINE_LEFT_POSITION'], (15, 10, 5)[frame])
        self.assertEqual(values['LANE_HIGHLIGHT'], 0)
      for speed, lfa, left, right in ((0., 2, True, False), (8., 2, True, False),
                                    (20., 0, True, False), (20., 2, True, True), (20., 2, False, False)):
        self.cs.out.vEgo, self.controller.lfa_icon = speed, lfa
        self.cs.out.leftBlinker, self.cs.out.rightBlinker = left, right
        msg = self.display_messages(0, updated_161=True)[0]
        values = decode('CCNC_0x161', 0x161, msg[1].hex())
        self.assertEqual((values['LCA_LEFT_ARROW'], values['LCA_RIGHT_ARROW']), (0, 0))
      self.cs.out.vEgo, self.controller.lfa_icon = 20., 2

  def test_camera_fallback_requires_fresh_valid_geometry(self):
    self.cs.out.leftBlinker = True
    for camera_time, quality in ((0, 3), (700_000_000, 3), (1_100_000_000, 3), (1_000_000_000, 0)):
      self.cs.ccnc_camera_time_nanos = camera_time
      self.cs.msg_1b5['Info_LftLnQualSta'] = quality
      msg = self.display_messages(0, updated_161=True)[0]
      self.assertEqual(decode('CCNC_0x161', 0x161, msg[1].hex())['LCA_LEFT_ARROW'], 0)

  def test_adjacent_objects_pack_without_mutating_stock_or_requiring_blinkers(self):
    self.cs.msg_162.update({'LEAD_LEFT': 0, 'LEAD_RIGHT': 0})
    stock = self.cs.msg_162.copy()
    self.controller.ccnc_radar = RadarObjects(1., (RadarObject(1, 25., -3.6), RadarObject(2, 40., 3.5)))
    self.controller.ccnc_object_lanes = ObjectLanes(1., tuple(((0., y), (80., y)) for y in (-5.4, -1.8, 1.8, 5.4)))
    messages = self.display_messages(0, updated_162=True)
    values = decode('CCNC_0x162', 0x162, messages[0][1].hex())
    for side, distance, lateral in (('LEFT', 25., 3.6), ('RIGHT', 40., 3.5)):
      self.assertEqual(values[f'LEAD_{side}'], 2)
      self.assertEqual(values[f'LEAD_{side}_DISTANCE'], distance)
      self.assertAlmostEqual(values[f'LEAD_{side}_LATERAL'], lateral)
    self.assertEqual(self.cs.msg_162, stock)
    for key in ('COUNTER', 'LEAD_LEFT_REAR_STATUS', 'LEAD_RIGHT_REAR_STATUS', 'LEAD_ALT', 'VIBRATE'):
      self.assertEqual(values[key], stock[key])
    self.assertEqual(self.display_messages(1), [])
    # Only new camera frames are sent, even when the radar changes.
    self.assertEqual([m[0] for m in self.display_messages(2, updated_161=True)], [0x161])
    expired = decode('CCNC_0x162', 0x162, self.display_messages(26, updated_162=True)[0][1].hex())
    self.assertEqual((expired['LEAD_LEFT'], expired['LEAD_RIGHT']), (0, 0))

  def test_stock_classified_objects_take_precedence_over_radar_boxes(self):
    self.cs.msg_162.update({'LEAD_LEFT': 6, 'LEAD_LEFT_DISTANCE': 35., 'LEAD_LEFT_LATERAL': 4.2})
    self.controller.ccnc_radar = RadarObjects(1., (RadarObject(1, 25., -3.6),))
    self.controller.ccnc_object_lanes = ObjectLanes(1., tuple(((0., y), (80., y)) for y in (-5.4, -1.8, 1.8, 5.4)))
    values = decode('CCNC_0x162', 0x162, self.display_messages(0, updated_162=True)[0][1].hex())
    self.assertEqual(values['LEAD_LEFT'], 6)
    self.assertEqual(values['LEAD_LEFT_DISTANCE'], 35.)
    self.assertAlmostEqual(values['LEAD_LEFT_LATERAL'], 4.2)

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
    self.cc.latActive = True
    self.cs.out.leftBlinker = True
    self.cs.msg_1b5.update({"Info_LftLnPosVal": -1.2, "Info_RtLnPosVal": 2.4})
    self.controller.ccnc_model = LaneModelSample(1.0, (-4.8, -1.2, 2.4, 6.0), 2, 1, edges=(-9.0, 9.0))
    _, data, _ = self.display_messages(0, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    self.assertEqual((values["LANELINE_LEFT_POSITION"], values["LANELINE_RIGHT_POSITION"]), (10, 20))
    self.assertEqual(values["LANELINE_CURVATURE"], self.msg_161["LANELINE_CURVATURE"])
    self.controller.ccnc_model = None
    self.cs.ccnc_camera_time_nanos = 0  # Both sources absent: preserve stock.
    self.cs.msg_161 = copy.copy(self.msg_161)
    _, data, _ = self.display_messages(1, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    for key in ("LANELINE_LEFT_POSITION", "LANELINE_RIGHT_POSITION", "LANELINE_LEFT", "LANE_HIGHLIGHT"):
      self.assertEqual(values[key], self.msg_161[key])

  def test_source_162_does_not_advance_model_display(self):
    self.cc.latActive = True
    self.cs.out.leftBlinker = True
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 1, edges=(-9.0, 9.0))
    self.display_messages(0, updated_161=True)
    state = copy.deepcopy(vars(self.controller.ccnc_display))
    self.controller.ccnc_model = LaneModelSample(1.05, (-5.0, -1.4, 2.2, 5.8), 2, 1, edges=(-9.0, 9.0))
    self.display_messages(1, updated_162=True)
    self.assertEqual(vars(self.controller.ccnc_display), state)

  def test_model_display_preserves_departure_warnings_and_arrows(self):
    self.cc.latActive = True
    self.cs.out.leftBlinker = True
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 1, edges=(-9.0, 9.0))
    self.hud.leftLaneDepart = self.hud.rightLaneDepart = True
    self.cc.leftBlinker = True
    messages = self.display_messages(0, updated_161=True, updated_162=True)
    values = decode("CCNC_0x161", 0x161, messages[0][1].hex())
    self.assertEqual((values["LANELINE_LEFT"], values["LANELINE_RIGHT"]), (4, 4))
    self.assertEqual((values["LCA_LEFT_ARROW"], values["LCA_RIGHT_ARROW"]), (2, 0))
    self.assertEqual(decode("CCNC_0x162", 0x162, messages[1][1].hex())["VIBRATE"], 1)

  def test_only_confirmed_lane_change_overrides_stock_geometry(self):
    # Isolate model enhancement from the independent stock camera fallback.
    self.cs.ccnc_camera_time_nanos = 0
    # Model/requested blinkers alone must not activate the custom display.
    self.cc.leftBlinker = True
    cases = [
      (0, 1, True, True, False, 20.0),
      (0, 2, True, False, True, 20.0),
      (1, 2, True, False, True, 20.0),
      (1, 1, True, True, False, 20.0),
      (2, 1, True, False, False, 20.0),
      (2, 1, True, False, True, 20.0),
      (2, 1, True, True, True, 20.0),
      (2, 1, False, True, False, 20.0),
      (2, 1, True, True, False, 0.0),
      (2, 1, True, True, False, 8.0),
      (2, 0, True, True, False, 20.0),
    ]
    for state, direction, lat_active, left, right, speed in cases:
      with self.subTest(state=state, direction=direction, lat_active=lat_active, blinkers=(left, right), speed=speed):
        self.cc.latActive = lat_active
        self.cs.out.leftBlinker, self.cs.out.rightBlinker = left, right
        self.cs.out.vEgo = speed
        for frame in range(6):
          offset = 0.6 if frame % 2 else -0.6
          self.controller.ccnc_model = LaneModelSample(1. + frame * .01, tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4)), state, direction, edges=(-9.0, 9.0))
          self.cs.msg_161 = copy.copy(self.msg_161)
          _, data, _ = self.display_messages(frame, updated_161=True)[0]
          values = decode("CCNC_0x161", 0x161, data.hex())
          for key in ("LANELINE_LEFT_POSITION", "LANELINE_RIGHT_POSITION", "LANE_LEFT", "LANE_RIGHT", "LANE_HIGHLIGHT"):
            self.assertEqual(values[key], self.msg_161[key])
          self.assertEqual((values["LCA_LEFT_ARROW"], values["LCA_RIGHT_ARROW"]), (0, 0))

  def test_model_arrows_clear_without_camera_when_change_ends(self):
    self.cs.ccnc_camera_time_nanos = 0
    self.cc.latActive = True
    for direction in (1, 2):
      for hazards in (False, True):
        with self.subTest(direction=direction, hazards=hazards):
          self.controller.ccnc_display.reset()
          for frame in range(2):
            self.cs.msg_161 = copy.copy(self.msg_161)
            self.cs.out.leftBlinker = direction == 1 or (frame == 1 and hazards)
            self.cs.out.rightBlinker = direction == 2 or (frame == 1 and hazards)
            state = 2 if frame == 0 or hazards else 0
            self.controller.ccnc_model = LaneModelSample(1. + frame * .01, (-5.4, -1.8, 1.8, 5.4), state, direction, edges=(-9.0, 9.0))
            _, data, _ = self.display_messages(frame, updated_161=True)[0]
            values = decode("CCNC_0x161", 0x161, data.hex())
            expected = (2 if direction == 1 else 0, 2 if direction == 2 else 0) if frame == 0 else (0, 0)
            self.assertEqual((values["LCA_LEFT_ARROW"], values["LCA_RIGHT_ARROW"]), expected)
            if frame == 1:
              self.assertEqual((values["LCA_LEFT_ICON"], values["LCA_RIGHT_ICON"]), (4, 4))

  def test_road_edge_blocks_model_enhancement(self):
    self.cs.ccnc_camera_time_nanos = 0
    self.cc.latActive = True
    self.cs.out.rightBlinker = True
    for i in range(10):
      self.cs.msg_161 = copy.copy(self.msg_161)
      self.controller.ccnc_model = LaneModelSample(1. + i * .01, (-5.4, -1.2, 2.4, 6.0), 2, 2, edges=(-9.0, 2.6))
      _, data, _ = self.display_messages(i, updated_161=True)[0]
      values = decode("CCNC_0x161", 0x161, data.hex())
      for key in ("LANELINE_LEFT_POSITION", "LANELINE_RIGHT_POSITION", "LANE_RIGHT", "LANE_HIGHLIGHT"):
        self.assertEqual(values[key], self.msg_161[key])

      self.assertEqual((values["LCA_LEFT_ARROW"], values["LCA_RIGHT_ARROW"]), (0, 0))

  def test_comma_change_does_not_require_stock_camera_agreement(self):
    self.cc.latActive = True
    self.cs.out.rightBlinker = True
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 2, edges=(None, None))
    baseline = copy.copy(self.cs.msg_1b5)
    for change, camera_time in (({"Info_RtLnQualSta": 1}, 1_000_000_000),
                                ({"Info_LftLnQualSta": 7}, 1_000_000_000),
                                ({"Info_LftLnPosVal": -0.3, "Info_RtLnPosVal": 3.3}, 1_000_000_000),
                                ({}, 700_000_000), ({}, 0)):
      with self.subTest(change=change, camera_time=camera_time):
        self.cs.msg_1b5 = baseline | change
        self.cs.ccnc_camera_time_nanos = camera_time
        self.cs.msg_161 = copy.copy(self.msg_161)
        _, data, _ = self.display_messages(0, updated_161=True)[0]
        values = decode("CCNC_0x161", 0x161, data.hex())
        self.assertEqual(values["LANE_RIGHT"], 1)
        self.assertEqual(values["LCA_RIGHT_ARROW"], 2)
        self.assertIsNotNone(self.controller.ccnc_display.target)

  def test_blinker_cancel_clears_published_display(self):
    self.cc.latActive = True
    self.cs.out.rightBlinker = True
    self.controller.ccnc_model = LaneModelSample(1.0, (-5.4, -1.8, 1.8, 5.4), 2, 2, edges=(-9.0, 9.0))
    _, data, _ = self.display_messages(0, updated_161=True)[0]
    self.assertEqual(decode("CCNC_0x161", 0x161, data.hex())["LANE_RIGHT"], 1)
    self.cs.out.rightBlinker = False
    self.cs.msg_161 = copy.copy(self.msg_161)
    _, data, _ = self.display_messages(1, updated_161=True)[0]
    values = decode("CCNC_0x161", 0x161, data.hex())
    self.assertEqual(values["LANE_RIGHT"], self.msg_161["LANE_RIGHT"])
    self.assertIsNone(self.controller.ccnc_display.values)

  def test_boot_first_display_frames_are_captured_in_either_order(self):
    cp = SimpleNamespace(flags=HyundaiFlags.CANFD | HyundaiFlags.CCNC,
                         carFingerprint=CAR.HYUNDAI_SONATA_2024, safetyConfigs=[None])
    for order in (((0x161, STOCK_161), (0x162, STOCK_162)), ((0x162, STOCK_162), (0x161, STOCK_161))):
      with self.subTest(order=[address for address, _ in order]):
        parser = CarState.__new__(CarState).get_can_parsers_canfd(cp)[Bus.cam]
        for frame, (address, raw) in enumerate(order):
          parser.update([(1_000_000_000 + frame * 20_000_000, [(address, bytes.fromhex(raw), 2)])])
          # Match carstate's first read, after the initial CAN batch was parsed.
          self.cs.msg_161 = copy.copy(parser.vl["CCNC_0x161"])
          self.cs.msg_162 = copy.copy(parser.vl["CCNC_0x162"])
          updated_161 = bool(parser.vl_all["CCNC_0x161"]["COUNTER"])
          updated_162 = bool(parser.vl_all["CCNC_0x162"]["COUNTER"])
          messages = self.display_messages(frame, updated_161, updated_162)
          self.assertEqual([m[0] for m in messages], [address])
          name = "CCNC_0x161" if address == 0x161 else "CCNC_0x162"
          self.assertEqual(decode(name, address, messages[0][1].hex())["COUNTER"], decode(name, address, raw)["COUNTER"])
          parser.update([(1_010_000_000 + frame * 20_000_000, [])])
          self.assertEqual(self.display_messages(frame, bool(parser.vl_all["CCNC_0x161"]["COUNTER"]),
                                                 bool(parser.vl_all["CCNC_0x162"]["COUNTER"])), [])

  def test_boot_camera_lane_message_is_captured_before_first_read(self):
    cp = SimpleNamespace(flags=HyundaiFlags.CANFD | HyundaiFlags.CCNC,
                         carFingerprint=CAR.HYUNDAI_SONATA_2024, safetyConfigs=[None])
    parser = CarState.__new__(CarState).get_can_parsers_canfd(cp)[Bus.cam]
    address, data, _ = self.controller.packer.make_can_msg("FR_CMR_03_50ms", 2, self.cs.msg_1b5)
    parser.update([(1_000_000_000, [(address, data, 2)])])
    self.assertEqual(parser.vl["FR_CMR_03_50ms"]["Info_LftLnQualSta"], 3)
    self.assertEqual(parser.ts_nanos["FR_CMR_03_50ms"]["Info_LftLnPosVal"], 1_000_000_000)

  def test_other_canfd_variants_do_not_preregister_ccnc(self):
    for flags in (HyundaiFlags.CANFD, HyundaiFlags.CANFD | HyundaiFlags.CCNC | HyundaiFlags.CANFD_LKA_STEER_MSG):
      cp = SimpleNamespace(flags=flags, carFingerprint=CAR.HYUNDAI_SONATA_2024, safetyConfigs=[None])
      parser = CarState.__new__(CarState).get_can_parsers_canfd(cp)[Bus.cam]
      self.assertNotIn(0x161, parser.addresses)
      self.assertNotIn(0x162, parser.addresses)

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
