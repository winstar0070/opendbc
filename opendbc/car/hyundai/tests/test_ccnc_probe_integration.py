import copy
from types import SimpleNamespace
import unittest

from opendbc.can import CANParser
from opendbc.car import structs, Bus
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.hyundai.ccnc_probe import PROBE_FIELDS, SLOT_SIGNALS
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.tests import test_ccnc_source_timing as fixtures


class TestCcncProbeIntegration(unittest.TestCase):
  def make_fixture(self, request=None):
    f = fixtures.TestCcncSourceTiming()
    f.setUp()
    f.cs.out.gearShifter = structs.CarState.GearShifter.park
    f.cs.out.vEgo = 0.
    f.cs.out.canValid = True
    f.cs.ccnc_0x162_time_nanos = 1_000_000_000
    f.cc.longActive = False
    f.controller.CP.openpilotLongitudinalControl = True
    f.controller.accel_last = 0.
    f.controller.tuning = SimpleNamespace(stopping=False, actual_accel=0., jerk_lower=1., jerk_upper=1.)
    f.controller.object_gap = 0
    f.controller.lead_distance = 10.
    f.controller.lead_rel_speed = 0.
    f.controller.lead_visible = False
    f.controller.ccnc_probe_request = request
    return f

  def request(self, slot='LEFT', token='a', issued=1_000_000_000, expires=3_000_000_000):
    return dict(token=token * 32, issued_ns=issued, expires_ns=expires, slot=slot, status=2, distance=12.3, lateral=3.4)

  def send(self, f, now=1_000_000_000, updated=True):
    f.cs.ccnc_0x161_updated = updated
    f.cs.ccnc_0x162_updated = updated
    return f.controller.create_canfd_msgs(False, 0, 0., 0., False, f.hud, f.cs, f.cc, now_nanos=now)

  def values(self, messages):
    message = next(m for m in messages if m[0] == 0x162)
    parser = CANParser(fixtures.DBC, [('CCNC_0x162', 0)], message[2])
    parser.update([(1, [message])])
    return dict(parser.vl['CCNC_0x162'])

  def test_every_slot_changes_only_object_fields_and_crc_all_packets_preserved(self):
    for slot, fields in SLOT_SIGNALS.items():
      with self.subTest(slot=slot):
        baseline, active = self.make_fixture(), self.make_fixture(self.request(slot))
        original = copy.deepcopy((active.cs.msg_161, active.cs.msg_162))
        normal, diagnostic = self.send(baseline), self.send(active)
        self.assertEqual([m[0] for m in normal], [m[0] for m in diagnostic])
        self.assertIn(0x12a, [m[0] for m in diagnostic])
        self.assertIn(0x1a0, [m[0] for m in diagnostic])
        self.assertEqual([m for m in normal if m[0] != 0x162], [m for m in diagnostic if m[0] != 0x162])
        before, after = self.values(normal), self.values(diagnostic)
        self.assertEqual({k: v for k, v in before.items() if k not in (*PROBE_FIELDS, 'CHECKSUM')},
                         {k: v for k, v in after.items() if k not in (*PROBE_FIELDS, 'CHECKSUM')})
        for key in PROBE_FIELDS:
          expected = (2, 12.3, 3.4)[fields.index(key)] if key in fields else 0
          self.assertAlmostEqual(after[key], expected)
        self.assertEqual((active.cs.msg_161, active.cs.msg_162), original)
        for addr, data, _ in diagnostic:
          self.assertEqual(int.from_bytes(data[:2], 'little'), hkg_can_fd_checksum(addr, None, data))

  def test_unsafe_transition_without_hud_frame_burns_request(self):
    for gate in ('gear', 'speed', 'enabled', 'latActive', 'longActive', 'canValid', 'display'):
      with self.subTest(gate=gate):
        f = self.make_fixture(self.request())
        self.assertEqual(self.values(self.send(f))['LEAD_LEFT'], 2)
        if gate == 'gear':
          f.cs.out.gearShifter = structs.CarState.GearShifter.drive
        elif gate == 'speed':
          f.cs.out.vEgo = .101
        elif gate in ('enabled', 'latActive', 'longActive'):
          setattr(f.cc, gate, True)
        elif gate == 'canValid':
          f.cs.out.canValid = False
        else:
          f.cs.ccnc_0x162_time_nanos = 0
        self.assertNotIn(0x162, [m[0] for m in self.send(f, 1_010_000_000, updated=False)])
        restored = self.make_fixture(self.request())
        restored.controller.ccnc_probe = f.controller.ccnc_probe
        self.assertEqual(self.send(restored, 1_020_000_000), self.send(self.make_fixture(), 1_020_000_000))

  def test_normal_between_frames_does_not_burn_lease_and_expiry_restores(self):
    f = self.make_fixture(self.request(expires=1_100_000_000))
    normal = self.make_fixture()
    self.send(f)
    self.send(normal)
    self.send(f, 1_010_000_000, updated=False)
    self.send(normal, 1_010_000_000, updated=False)
    self.assertEqual(self.values(self.send(f, 1_050_000_000))['LEAD_LEFT'], 2)
    self.send(normal, 1_050_000_000)
    self.assertEqual(self.send(f, 1_100_000_000), self.send(normal, 1_100_000_000))

  def test_actual_162_freshness_not_161_or_updated_flag(self):
    for timestamp in (0, 749_999_999, 1_000_000_001):
      f = self.make_fixture(self.request())
      f.cs.ccnc_0x162_time_nanos = timestamp
      f.cs.ccnc_display_time_nanos = 1_000_000_000
      self.assertEqual(self.send(f), self.send(self.make_fixture()))
    f = self.make_fixture(self.request())
    f.cs.ccnc_0x162_time_nanos = 750_000_000
    self.assertEqual(self.values(self.send(f))['LEAD_LEFT'], 2)

  def test_missing_fixture_attrs_and_nonfinite_speed_fail_closed(self):
    for field in ('gearShifter', 'canValid', 'vEgo'):
      f = self.make_fixture(self.request())
      delattr(f.cs.out, field)
      if field == 'vEgo':
        # Existing lane HUD code needs vEgo; no HUD frame still evaluates probe.
        self.send(f, updated=False)
        f.cs.out.vEgo = 0.
      else:
        self.send(f)
      self.assertIsNone(f.controller.ccnc_probe._active)
    for speed in (float('nan'), float('inf'), -.101):
      f = self.make_fixture(self.request())
      f.cs.out.vEgo = speed
      self.assertEqual(self.values(self.send(f))['LEAD_LEFT'], 0)

  def test_carstate_162_timestamp_follows_actual_source_not_161(self):
    cp = structs.CarParams(carFingerprint=CAR.HYUNDAI_SONATA_2024,
                           flags=int(HyundaiFlags.CCNC | HyundaiFlags.CANFD), wheelSpeedFactor=1.)
    cp.safetyConfigs = [structs.CarParams.SafetyConfig()]
    state = CarState(cp, structs.CarParamsSP())
    self.assertEqual(state.ccnc_0x162_time_nanos, 0)
    parsers = state.get_can_parsers_canfd(cp)
    packer = self.make_fixture().controller.packer
    parsers[Bus.cam].update([(1_000_000_000, [packer.make_can_msg('CCNC_0x162', 2, {})])])
    state.update_canfd(parsers)
    self.assertEqual(state.ccnc_0x162_time_nanos, 1_000_000_000)
    parsers[Bus.cam].update([(1_100_000_000, [packer.make_can_msg('CCNC_0x161', 2, {})])])
    state.update_canfd(parsers)
    self.assertEqual(state.ccnc_display_time_nanos, 1_100_000_000)
    self.assertEqual(state.ccnc_0x162_time_nanos, 1_000_000_000)
    self.assertFalse(state.ccnc_0x162_updated)

  def test_sender_rejects_nonobject_or_malformed_probe_fields(self):
    f = self.make_fixture()

    def render(probe):
      return hyundaicanfd.create_ccnc(f.controller.packer, f.controller.CAN, False, False, f.hud, False, False,
                                     f.cs.msg_161, f.cs.msg_162, f.cs.msg_1b5, True, f.cs.out, False, 0,
                                     probe_values=probe)
    normal = render(None)
    fields = dict.fromkeys(PROBE_FIELDS, 0.)
    for invalid in ({}, fields | {'SETSPEED': 2}, fields | {'LEAD': 99}, fields | {'LEAD_DISTANCE': 26.},
                    fields | {'LEAD_LATERAL': float('nan')}, fields | {'LEAD_DISTANCE': 10 ** 1000},
                    fields | {'LEAD': True}, 'bad'):
      self.assertEqual(render(invalid), normal)

  def test_request_none_restores_and_same_token_cannot_restart(self):
    f = self.make_fixture(self.request())
    normal = self.make_fixture()
    self.send(f)
    self.send(normal)
    f.controller.ccnc_probe_request = None
    self.send(f, 1_010_000_000, updated=False)
    self.send(normal, 1_010_000_000, updated=False)
    f.controller.ccnc_probe_request = self.request()
    self.assertEqual(self.send(f, 1_020_000_000), self.send(normal, 1_020_000_000))
