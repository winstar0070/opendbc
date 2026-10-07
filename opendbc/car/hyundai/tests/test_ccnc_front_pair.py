import unittest

from opendbc.car.hyundai.tests import test_ccnc_cutin


class TestCcncFrontPair(unittest.TestCase):
  def setUp(self):
    self.fixture = test_ccnc_cutin.TestCcncCutInPacking()
    self.pair = {'LEAD': 2, 'LEAD_DISTANCE': 10., 'LEAD_LATERAL': .1,
                 'LEAD_ALT': 2, 'LEAD_ALT_DISTANCE': 20., 'LEAD_ALT_LATERAL': .2}

  def test_pair_fills_both_empty_front_slots_with_actual_radar_positions(self):
    for longitudinal in (False, True):
      result = self.fixture.send(self.pair, camera=30., longitudinal=longitudinal)
      self.assertEqual(result['LEAD'], 4 if longitudinal else 2)
      self.assertEqual(result['LEAD_ALT'], 2)  # 3/4 in ALT mean cones, not cars.
      for k in ('LEAD_DISTANCE', 'LEAD_LATERAL', 'LEAD_ALT_DISTANCE', 'LEAD_ALT_LATERAL'):
        self.assertAlmostEqual(result[k], self.pair[k])

  def test_native_front_or_alt_prevents_pair_replacement(self):
    for longitudinal in (False, True):
      for native, native_alt in ((4, 0), (0, 2), (4, 2), (0, 31)):
        baseline = self.fixture.send(None, native=native, native_alt=native_alt, camera=30., longitudinal=longitudinal)
        result = self.fixture.send(self.pair, native=native, native_alt=native_alt, camera=30., longitudinal=longitudinal)
        for k in ('LEAD_ALT', 'LEAD_ALT_DISTANCE', 'LEAD_ALT_LATERAL'):
          self.assertEqual(result[k], baseline[k])
        if native:
          for k in ('LEAD', 'LEAD_DISTANCE', 'LEAD_LATERAL'):
            self.assertEqual(result[k], baseline[k])

  def test_pair_is_disabled_with_cruise_and_returns_to_camera_after_loss(self):
    result = self.fixture.send(self.pair, main=False)
    self.assertEqual(result['LEAD'], 0)
    self.assertEqual(result['LEAD_ALT'], 0)
    result = self.fixture.send(None, camera=30.)
    self.assertEqual(result['LEAD_ALT'], 0)
    self.assertEqual(result['LEAD_DISTANCE'], 30.)

  def test_incomplete_or_invalid_alt_does_not_activate_pair(self):
    for key, value in (('LEAD_ALT', 3), ('LEAD_ALT_DISTANCE', -1.), ('LEAD_ALT_DISTANCE', float('nan')),
                       ('LEAD_ALT_DISTANCE', 200.), ('LEAD_ALT_LATERAL', float('inf')), ('LEAD_ALT_LATERAL', 12.7)):
      invalid = dict(self.pair, **{key: value})
      result = self.fixture.send(invalid, camera=30.)
      self.assertEqual(result['LEAD_ALT'], 0)
      self.assertEqual(result['LEAD_DISTANCE'], 30.)
    incomplete = self.pair.copy()
    del incomplete['LEAD_ALT_LATERAL']
    self.assertEqual(self.fixture.send(incomplete)['LEAD_ALT'], 0)


if __name__ == '__main__':
  unittest.main()
