import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.ccnc_objects import CcncObjectDisplay, read_object_lanes, read_radar_objects


def model(curve=0.0):
  return SimpleNamespace(laneLines=[SimpleNamespace(x=[0., 40., 80.], y=[y, y + curve, y + curve * 2])
                                    for y in (-5.4, -1.8, 1.8, 5.4)],
                         laneLineProbs=[.9] * 4, laneLineStds=[.1] * 4)


def radar(*points, fault=False):
  result = structs.RadarData()
  result.errors.canError = fault
  result.points = [structs.RadarData.RadarPoint(trackId=i, dRel=x, yRel=y, vRel=0.) for i, x, y in points]
  return result


class TestCcncObjects(unittest.TestCase):
  def setUp(self):
    self.display = CcncObjectDisplay()

  def values(self, *points, t=1., geometry=None, now=None):
    return self.display.update(read_radar_objects(radar(*points), t), read_object_lanes(geometry or model(), t), t if now is None else now)

  def test_both_sides_use_measured_positions_and_unclassified_boxes(self):
    values = self.values((1, 25., 3.6), (2, 40., -3.5), (3, 15., 0.))
    self.assertEqual({k: round(v, 3) for k, v in values.items()}, {'LEAD_LEFT': 2, 'LEAD_LEFT_DISTANCE': 25., 'LEAD_LEFT_LATERAL': 3.6,
                              'LEAD_RIGHT': 2, 'LEAD_RIGHT_DISTANCE': 40., 'LEAD_RIGHT_LATERAL': 3.5})

  def test_curved_lane_is_evaluated_at_object_distance(self):
    values = self.values((1, 40., -.4), (2, 40., -4.), (3, 40., -7.6), geometry=model(curve=4.))
    self.assertEqual(values['LEAD_LEFT_DISTANCE'], 40.)
    self.assertAlmostEqual(values['LEAD_RIGHT_LATERAL'], 7.6, places=5)
    self.assertEqual(len(values), 6)
    self.assertNotIn('LEAD_LEFT', self.values((1, 40., 1.12), geometry=model(curve=4.)))

  def test_time_regression_resets_object_position_filter(self):
    self.values((1, 20., 3.6), t=2.)
    self.assertEqual(self.values((1, 40., 3.6), t=1.)['LEAD_LEFT_DISTANCE'], 40.)

  def test_all_radar_faults_reject_the_snapshot(self):
    for field in ('canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'):
      source = radar((1, 20., 3.6))
      setattr(source.errors, field, True)
      self.assertIsNone(read_radar_objects(source, 1.))

  def test_nearest_track_selection_keeps_identity_until_clearly_closer(self):
    self.values((1, 20., 3.6), (2, 22., 3.6))
    values = self.values((1, 22., 3.6), (2, 20., 3.6), t=1.05)
    self.assertGreater(values['LEAD_LEFT_DISTANCE'], 20.)
    values = self.values((1, 30., 3.6), (2, 20., 3.6), t=1.1)
    self.assertEqual(values['LEAD_LEFT_DISTANCE'], 20.)

  def test_disappearance_and_side_change_do_not_leave_ghosts(self):
    self.values((1, 20., 3.6))
    self.assertEqual(self.values(t=1.05), {})
    self.assertNotIn('LEAD_LEFT', self.values((1, 20., 0.), t=1.1))
    self.assertIn('LEAD_RIGHT', self.values((1, 20., -3.6), t=1.15))

  def test_bad_positions_rear_objects_and_out_of_model_range_are_ignored(self):
    self.assertEqual(self.values((1, 20., float('nan')), (2, -5., 3.6), (3, 100., 3.6),
                                 (4, float('inf'), -3.6), (5, 20., 15.), (6, 20., 1.8)), {})

  def test_stale_future_missing_and_faulted_data_clear_overrides(self):
    r = read_radar_objects(radar((1, 20., 3.6)), 1.)
    lanes = read_object_lanes(model(), 1.)
    for now in (1.251, .999, float('nan')):
      self.assertEqual(self.display.update(r, lanes, now), {})
    self.assertEqual(self.display.update(None, lanes, 1.), {})
    self.assertEqual(self.display.update(r, None, 1.), {})
    self.assertIsNone(read_radar_objects(radar(fault=True), 1.))
    self.assertIsNone(read_radar_objects(radar(), float('nan')))
    self.assertEqual(self.display.update(r, read_object_lanes(model(), 1.2), 1.2), {})

  def test_low_confidence_malformed_and_missing_lane_geometry(self):
    for kind in ('prob', 'std', 'ordering', 'finite', 'length', 'width'):
      m = model()
      if kind == 'prob':
        m.laneLineProbs[0] = .1
      elif kind == 'std':
        m.laneLineStds[0] = float('nan')
      elif kind == 'ordering':
        m.laneLines[0].x = [0., 80., 40.]
      elif kind == 'finite':
        m.laneLines[0].y[1] = float('nan')
      elif kind == 'length':
        m.laneLines[0].y = []
      else:
        m.laneLines[0].y = [-10.] * 3
      self.assertNotIn('LEAD_LEFT', self.values((1, 20., 3.6), geometry=m))
    m.laneLines = []
    self.assertIsNone(read_object_lanes(m, 1.))
    self.assertIsNone(read_object_lanes(model(), float('nan')))

  def test_snapshots_copy_input_and_repeated_sample_does_not_resmooth(self):
    source = radar((1, 20., 3.6))
    r = read_radar_objects(source, 1.)
    m = model()
    lanes = read_object_lanes(m, 1.)
    source.points[0].dRel = 90.
    m.laneLines[0].y[1] = 20.
    before = self.display.update(r, lanes, 1.)
    self.assertEqual(before['LEAD_LEFT_DISTANCE'], 20.)
    self.assertEqual(self.display.update(r, lanes, 1.1), before)


if __name__ == '__main__':
  unittest.main()
