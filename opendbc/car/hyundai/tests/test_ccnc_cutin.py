import copy
from types import SimpleNamespace
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai.ccnc_objects import CcncObjectDisplay, ObjectLanes, RadarObject, RadarObjects
from opendbc.car.hyundai.hyundaicanfd import create_ccnc, hkg_can_fd_checksum


def decode(name, msg):
  parser = CANParser('hyundai_canfd_generated', [(name, 0)], 0)
  parser.update([(1, [msg])])
  return dict(parser.vl[name])


class TestCcncCutInPacking(unittest.TestCase):
  def send(self, front=None, native=0, camera=0., longitudinal=True, main=True, camera_fresh=True, native_alt=0):
    packer = CANPacker('hyundai_canfd_generated')
    stock_161 = decode('CCNC_0x161', packer.make_can_msg('CCNC_0x161', 0, {}))
    stock_162 = decode('CCNC_0x162', packer.make_can_msg('CCNC_0x162', 0, {'LEAD': native, 'LEAD_DISTANCE': 42.,
                                                                     'LEAD_ALT': native_alt, 'LEAD_ALT_DISTANCE': 55.}))
    originals = copy.deepcopy((stock_161, stock_162))
    hud = SimpleNamespace(leftLaneDepart=False, rightLaneDepart=False, leadDistanceBars=3, leadVisible=False)
    out = SimpleNamespace(vEgo=20., vCruiseCluster=80., leftBlindspot=False, rightBlindspot=False)
    messages = create_ccnc(packer, SimpleNamespace(ECAN=0), longitudinal, True, hud, False, False,
                           stock_161, stock_162, {} if camera is None else {'Longitudinal_Distance': camera}, True, out, main, 2,
                           object_values=front, camera_fresh=camera_fresh)
    self.assertEqual(originals, (stock_161, stock_162))
    for addr, data, _ in messages:
      self.assertEqual(int.from_bytes(data[:2], 'little'), hkg_can_fd_checksum(addr, None, data))
    return decode('CCNC_0x162', messages[-1])

  def test_tracked_cutin_fills_missing_camera_front(self):
    front = {'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}
    for longitudinal in (False, True):
      result = self.send(front, longitudinal=longitudinal)
      self.assertNotEqual(result['LEAD'], 0)
      self.assertAlmostEqual(result['LEAD_DISTANCE'], 20.)
      self.assertAlmostEqual(result['LEAD_LATERAL'], .3)

  def test_camera_and_native_front_take_precedence(self):
    front = {'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}
    for longitudinal in (False, True):
      self.assertAlmostEqual(self.send(front, camera=30., longitudinal=longitudinal)['LEAD_DISTANCE'],
                             30. if longitudinal else 20.)
      self.assertAlmostEqual(self.send(front, native=4, camera=30., longitudinal=longitudinal)['LEAD_DISTANCE'],
                             30. if longitudinal else 42.)

  def test_unknown_camera_range_allows_fallback_and_native_always_wins(self):
    front = {'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}
    for distance in (200., 204.75):
      self.assertAlmostEqual(self.send(front, camera=distance)['LEAD_DISTANCE'], 20.)
    self.assertEqual(self.send(front, native=4, longitudinal=False)['LEAD_DISTANCE'], 42.)
    for key, value in (('LEAD_DISTANCE', float('nan')), ('LEAD_DISTANCE', -1.), ('LEAD_LATERAL', float('inf'))):
      invalid = dict(front, **{key: value})
      self.assertEqual(self.send(invalid)['LEAD_DISTANCE'], 0.)

  def test_cutin_sequence_has_one_packed_object_and_camera_takes_over(self):
    for sign, side in ((-1, 'LEFT'), (1, 'RIGHT')):
      display = CcncObjectDisplay()
      lines = tuple(((0., y), (80., y)) for y in (-5.4, -1.8, 1.8, 5.4))
      for i, lateral in enumerate((3.6, 2.1, 1.99, 1.81, 1.8, 1.7, .3)):
        t = 1 + i * .05
        values = display.update(RadarObjects(t, (RadarObject(1, 20., sign * lateral),)), ObjectLanes(t, lines), t)
        result = self.send(values, longitudinal=False)
        self.assertEqual(sum(result[k] != 0 for k in ('LEAD', 'LEAD_LEFT', 'LEAD_RIGHT')), 1)
        key = f'LEAD_{side}' if lateral > 1.8 else 'LEAD'
        self.assertNotEqual(result[key], 0)
        self.assertAlmostEqual(result[f'{key}_DISTANCE'], 20.)
      takeover = self.send(values, native=4, camera=19.5)
      self.assertEqual(takeover['LEAD'], 4)
      self.assertAlmostEqual(takeover['LEAD_DISTANCE'], 19.5)

  def test_loss_returns_to_original_front_and_respects_cruise_gate(self):
    result = self.send(None, camera=0.)
    self.assertEqual(result['LEAD_DISTANCE'], 0.)
    self.assertEqual(self.send({'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}, main=False)['LEAD'], 0)

  def test_stale_camera_cannot_block_fresh_cutin(self):
    front = {'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}
    self.assertEqual(self.send(front, camera=30., camera_fresh=False)['LEAD_DISTANCE'], 20.)

  def test_stock_longitudinal_accepts_missing_camera_distance(self):
    front = {'LEAD': 2, 'LEAD_DISTANCE': 20., 'LEAD_LATERAL': .3}
    self.assertEqual(self.send(front, camera=None, longitudinal=False)['LEAD_DISTANCE'], 20.)


if __name__ == '__main__':
  unittest.main()
