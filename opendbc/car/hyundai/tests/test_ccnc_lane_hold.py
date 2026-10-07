import unittest

from opendbc.car.hyundai.ccnc_objects import CcncObjectDisplay, read_object_lanes, read_radar_objects
from opendbc.car.hyundai.tests import test_ccnc_cutin as packing_tests
from opendbc.car.hyundai.tests.test_ccnc_objects import model, radar


class TestCcncLaneHold(unittest.TestCase):
  def setUp(self):
    self.display = CcncObjectDisplay()

  def update(self, t, *points, missing=(), geometry=None, now=None):
    m = geometry or model()
    for index in missing:
      m.laneLineProbs[index] = .1
    return self.display.update(read_radar_objects(radar(*points), t), read_object_lanes(m, t), t if now is None else now)

  def test_both_sides_keep_live_distance_for_six_second_outer_dropout(self):
    for side, sign, outer in (('LEFT', 1, 0), ('RIGHT', -1, 3)):
      self.display.reset()
      self.update(1., (1, 8., sign * 3.4))
      distances = []
      for i in range(1, 121):
        values = self.update(1. + i * .05, (1, 8. + i * .1, sign * 3.4), missing=(outer,))
        distances.append(values[f'LEAD_{side}_DISTANCE'])
      self.assertTrue(all(b > a for a, b in zip(distances, distances[1:], strict=False)))
      self.assertGreater(distances[-1], 19.)

  def test_hold_expires_without_renewal_and_recovery_can_reacquire(self):
    self.update(1., (1, 20., 3.6))
    for i in range(1, 141):
      self.assertIn('LEAD_LEFT', self.update(1. + i * .05, (1, 20., 3.6), missing=(0,)))
    self.assertEqual(self.update(8.05, (1, 20., 3.6), missing=(0,)), {})
    self.assertIn('LEAD_LEFT', self.update(8.1, (1, 20., 3.6)))

  def test_missing_outer_does_not_acquire_new_tracks_or_reuse_lost_track(self):
    self.update(1., (1, 20., 3.6))
    self.assertEqual(self.update(1.05, (2, 20., 3.6), missing=(0,)), {})
    self.assertEqual(self.update(1.1, (1, 20., 3.6), missing=(0,)), {})

  def test_missing_inner_stale_input_and_track_discontinuity_clear_hold(self):
    for kind in ('inner', 'stale', 'lateral_jump', 'distance_jump', 'gap', 'regression'):
      with self.subTest(kind=kind):
        self.display.reset()
        self.update(1., (1, 20., 3.6))
        t = .95 if kind == 'regression' else 1.25 if kind == 'gap' else 1.05
        x = 40. if kind == 'distance_jump' else 20.
        y = 4.5 if kind == 'lateral_jump' else 3.6
        missing = (0, 1) if kind == 'inner' else (0,)
        self.assertEqual(self.update(t, (1, x, y), missing=missing, now=1.4 if kind == 'stale' else t), {})
        if kind != 'inner':
          self.assertEqual(self.update(t + .05, (1, x, y), missing=(0,)), {})

  def test_inner_dropout_hides_then_recovers_same_track_without_renewing_lease(self):
    self.update(1., (1, 20., 3.6))
    for i in range(1, 21):
      self.assertEqual(self.update(1. + i * .05, (1, 20. + i * .1, 3.6), missing=(0, 1)), {})
    self.assertIn('LEAD_LEFT', self.update(2.05, (1, 22., 3.6), missing=(0,)))
    # Motion toward the center while the inner line is unknown drops the memory.
    self.assertEqual(self.update(2.1, (1, 22., 2.7), missing=(0, 1)), {})
    self.assertEqual(self.update(2.15, (1, 22., 2.7), missing=(0,)), {})

  def test_dormant_track_loss_and_expiry_cannot_reappear(self):
    for kind in ('loss', 'expiry'):
      self.display.reset()
      self.update(1., (1, 20., 3.6))
      for i in range(1, 142):
        points = () if kind == 'loss' and i == 10 else ((1, 20., 3.6),)
        self.assertEqual(self.update(1. + i * .05, *points, missing=(0, 1)), {})
      self.assertEqual(self.update(8.1, (1, 20., 3.6), missing=(0,)), {})

  def test_cutin_uses_current_inner_boundary_during_outer_dropout(self):
    for side, sign, outer in (('LEFT', 1, 0), ('RIGHT', -1, 3)):
      self.display.reset()
      self.update(1., (1, 20., sign * 2.1))
      self.assertIn(f'LEAD_{side}', self.update(1.05, (1, 20., sign * 1.9), missing=(outer,)))
      values = self.update(1.1, (1, 20., sign * 1.7), missing=(outer,))
      self.assertIn('LEAD', values)
      self.assertNotIn(f'LEAD_{side}', values)
      # A cut-in does not carry the old side lease back into an unknown lane.
      self.assertEqual(self.update(1.15, (1, 20., sign * 2.1), missing=(outer,)), {})

  def test_live_inner_boundary_moves_hold_and_valid_outer_can_reject_it(self):
    self.update(1., (1, 20., 3.6))
    shifted = model()
    shifted.laneLines[1].y = [-3.7] * 3
    self.assertEqual(self.update(1.05, (1, 20., 3.6), missing=(0,), geometry=shifted), {})
    self.update(1.1, (1, 20., 3.6))
    narrow = model()
    narrow.laneLines[0].y = [-2.0] * 3
    self.assertEqual(self.update(1.15, (1, 20., 3.6), geometry=narrow), {})

  def test_repeated_radar_sample_and_opposite_side_loss_do_not_extend_lease(self):
    self.update(1., (1, 20., 3.6))
    self.assertIn('LEAD_LEFT', self.update(1.05, (1, 20., 3.6), missing=(0, 2, 3)))
    for i in range(2, 141):
      t = 1. + i * .05
      self.update(t, (1, 20., 3.6), missing=(0,))
      self.update(t, (1, 20., 3.6), missing=(0,), now=t + .01)
    self.assertEqual(self.update(8.05, (1, 20., 3.6), missing=(0,)), {})

  def test_held_vehicle_and_cutin_pack_one_slot_with_valid_crc(self):
    packer_test = packing_tests.TestCcncCutInPacking()
    self.update(1., (1, 20., 2.1))
    for i, y in enumerate((2.05, 1.95, 1.85, 1.75, 1.6), 1):
      values = self.update(1. + i * .05, (1, 20. + i, y), missing=(0,))
      packed = packer_test.send(values, longitudinal=False)
      slot = 'LEAD_LEFT' if y > 1.8 else 'LEAD'
      self.assertEqual(packed[slot], 2)
      self.assertEqual(sum(packed[k] != 0 for k in ('LEAD', 'LEAD_ALT', 'LEAD_LEFT', 'LEAD_RIGHT')), 1)
      self.assertAlmostEqual(packed[slot + '_DISTANCE'], values[slot + '_DISTANCE'], delta=.1)


if __name__ == '__main__':
  unittest.main()
