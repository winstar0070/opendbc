import unittest
from types import SimpleNamespace

from opendbc.car.hyundai.ccnc_model import CcncLaneDisplay, LaneModelSample, read_model_lanes


def sample(t, offset=0.0, state=0, direction=0, lanes=None, edges=(-9.0, 9.0)):
  return LaneModelSample(t, tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4)) if lanes is None else lanes, state, direction, edges)


class TestModelLanes(unittest.TestCase):
  def test_reader_rejects_bad_points_and_probabilities(self):
    model = SimpleNamespace(laneLines=[SimpleNamespace(x=[0.0], y=[y]) for y in (-5.4, -1.8, 1.8, 5.4)],
                            laneLineProbs=[0.9] * 4, laneLineStds=[0.1] * 4,
                            roadEdges=[SimpleNamespace(x=[0.0], y=[y]) for y in (-9.0, 9.0)], roadEdgeStds=[0.1, 0.1],
                            meta=SimpleNamespace(laneChangeState=SimpleNamespace(raw=2), laneChangeDirection=SimpleNamespace(raw=1)))
    self.assertEqual(read_model_lanes(model, 1.0), sample(1.0, state=2, direction=1))
    model.laneLines[1].y = [float('nan')]
    model.laneLineProbs[2] = 0.1
    self.assertEqual(read_model_lanes(model, 2.0).lanes[1:3], (None, None))
    model.laneLines = []
    self.assertIsNone(read_model_lanes(model, 3.0))

  def test_edge_without_adjacent_lane_blocks_both_directions(self):
    for direction, edges in ((1, (-2.1, 9.0)), (2, (-9.0, 2.1)),
                             (1, (None, 9.0)), (2, (-9.0, None))):
      display = CcncLaneDisplay()
      for i in range(8):
        self.assertIsNone(display.update(sample(i * .05, offset=.05 * i, state=2, direction=direction, edges=edges)))

  def test_initial_missing_target_or_edge_recovers_when_data_arrives(self):
    for direction in (1, 2):
      for missing in ('lane', 'edge'):
        with self.subTest(direction=direction, missing=missing):
          display = CcncLaneDisplay()
          lanes = [-5.4, -1.8, 1.8, 5.4]
          edges = [-9.0, 9.0]
          if missing == 'lane':
            lanes[0 if direction == 1 else 3] = None
          else:
            edges[0 if direction == 1 else 1] = None
          self.assertIsNone(display.update(sample(0, state=2, direction=direction, lanes=tuple(lanes), edges=tuple(edges))))
          recovered = display.update(sample(.05, state=2, direction=direction))
          self.assertIsNotNone(recovered)
          self.assertEqual(recovered['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_pending_target_does_not_acquire_another_lane_after_index_switch(self):
    display = CcncLaneDisplay()
    self.assertIsNone(display.update(sample(0, state=2, direction=1, lanes=(None, -.1, 3.5, 7.1))))
    self.assertIsNone(display.update(sample(.05, state=2, direction=1, lanes=(-5.25, -1.65, 1.95, 5.55))))

  def test_confirmed_edge_rejection_does_not_rearm_during_same_change(self):
    display = CcncLaneDisplay()
    self.assertIsNone(display.update(sample(0, state=2, direction=2, edges=(-9.0, 2.0))))
    self.assertIsNone(display.update(sample(.05, state=2, direction=2)))

  def test_edge_loss_during_change_clears_display_and_does_not_rearm(self):
    display = CcncLaneDisplay()
    self.assertIsNotNone(display.update(sample(0, state=2, direction=2)))
    self.assertIsNone(display.update(sample(.05, state=2, direction=2, edges=(-9.0, 2.0))))
    self.assertIsNone(display.update(sample(.1, state=2, direction=2)))

  def test_reader_drops_uncertain_or_malformed_road_edges(self):
    model = SimpleNamespace(laneLines=[SimpleNamespace(x=[0.0], y=[y]) for y in (-5.4, -1.8, 1.8, 5.4)],
                            laneLineProbs=[.9] * 4, laneLineStds=[.1] * 4,
                            roadEdges=[SimpleNamespace(x=[0.0], y=[-9.0]), SimpleNamespace(x=[0.0], y=[9.0])],
                            roadEdgeStds=[.8, .1],
                            meta=SimpleNamespace(laneChangeState=SimpleNamespace(raw=2), laneChangeDirection=SimpleNamespace(raw=1)))
    self.assertEqual(read_model_lanes(model, 1).edges, (None, 9.0))
    model.roadEdges[1].y = [float('nan')]
    self.assertEqual(read_model_lanes(model, 2).edges, (None, None))
    model.roadEdges = []
    self.assertEqual(read_model_lanes(model, 3).edges, (None, None))

  def test_stationary_geometry_does_not_animate_with_time(self):
    display = CcncLaneDisplay()
    for i in range(250):
      result = display.update(sample(i * 0.05, state=2, direction=1))
      self.assertEqual((result['LANELINE_LEFT_POSITION'], result['LANELINE_RIGHT_POSITION']), (15, 15))
      self.assertEqual(result['LANE_HIGHLIGHT'], 0)
      self.assertEqual(result['LANE_LEFT'], 1)

  def test_jitter_is_filtered_and_same_sample_is_not_filtered_twice(self):
    display = CcncLaneDisplay()
    display.update(sample(0))
    result = display.update(sample(0.05, offset=0.1))
    self.assertEqual(display.update(sample(0.05, offset=0.1)), result)
    self.assertLess(abs(display.ego[0] + 1.8), 0.1)

  def test_crossing_tracks_target_across_model_index_change_and_mirrors(self):
    results = []
    for direction in (1, 2):
      display = CcncLaneDisplay()
      sequence = []
      for i in range(181):
        offset = min(i * 0.02, 3.6)
        lanes = tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4))
        if offset >= 1.8:
          lanes = (lanes[0] - 3.6, *lanes[:3])
        if direction == 2:
          lanes = tuple(-y for y in reversed(lanes))
        result = display.update(sample(i * 0.05, state=2, direction=direction, lanes=lanes))
        self.assertIsNotNone(result)
        sequence.append(result)
      # A lane index switch must create exactly one boundary exchange, at fill
      # activation, without an earlier jump or a later corrective snap.
      outer = 'LEFT' if direction == 1 else 'RIGHT'
      wraps = 0
      for before, after in zip(sequence, sequence[1:], strict=False):
        delta = after[f'LANELINE_{outer}_POSITION'] - before[f'LANELINE_{outer}_POSITION']
        if delta > 15:
          wraps += 1
          self.assertEqual((before['LANE_HIGHLIGHT'], after['LANE_HIGHLIGHT']), (0, 1))
          delta -= 30
        self.assertIn(delta, (-1, 0))
      self.assertEqual(wraps, 1)
      results.append(sequence)
      incoming = 'RIGHT' if direction == 1 else 'LEFT'
      for v in sequence:
        self.assertEqual(v[f'LANELINE_{outer}'], 6)
        hidden = not v['LANE_HIGHLIGHT'] or v[f'LANELINE_{incoming}_POSITION'] < 12
        self.assertEqual(v[f'LANELINE_{incoming}'], 1 if hidden else 6)
      highlight = [v for v in sequence if v['LANE_HIGHLIGHT']]
      self.assertTrue(highlight)
      self.assertEqual(highlight[0][f'LANELINE_{incoming}'], 1)
      self.assertEqual(highlight[-1][f'LANELINE_{incoming}'], 6)
      for v in highlight:
        self.assertEqual((v['LANE_LEFT'], v['LANE_RIGHT']), (0, 0))
        self.assertEqual(v['LANELINE_LEFT_POSITION'] + v['LANELINE_RIGHT_POSITION'], 30)
      self.assertEqual(sequence[-1][f'LANELINE_{incoming}_POSITION'], 15)
    for left, right in zip(*results, strict=True):
      self.assertEqual(left['LANELINE_LEFT_POSITION'], right['LANELINE_RIGHT_POSITION'])
      self.assertEqual(left['LANELINE_RIGHT'], right['LANELINE_LEFT'])

  def test_invalid_abort_and_restart_do_not_keep_green_fill(self):
    display = CcncLaneDisplay()
    display.update(sample(0, state=2, direction=1))
    self.assertIsNone(display.update(None))
    self.assertIsNone(display.update(sample(1, lanes=(None,) * 4)))
    result = display.update(sample(2))
    self.assertEqual((result['LANE_LEFT'], result['LANE_HIGHLIGHT']), (0, 0))
    self.assertEqual((result['LANELINE_LEFT'], result['LANELINE_RIGHT']), (2, 2))
    display.update(sample(3, state=2, direction=1))
    result = display.update(sample(3.05, state=1, direction=1))
    self.assertEqual(result['LANE_LEFT'], 0)

  def test_reversing_mid_change_restores_target_side_without_completion(self):
    display = CcncLaneDisplay()
    offsets = [i * 0.03 for i in range(81)] + [i * 0.03 for i in range(80, -1, -1)]
    results = [display.update(sample(i * 0.05, offset=o, state=2, direction=1)) for i, o in enumerate(offsets)]
    self.assertTrue(any(v['LANE_HIGHLIGHT'] for v in results))
    self.assertEqual(results[-1]['LANE_HIGHLIGHT'], 0)
    self.assertEqual(results[-1]['LANE_LEFT'], 1)
    result = display.update(sample(len(offsets) * 0.05, state=0))
    self.assertEqual(result['LANE_LEFT'], 0)

  def test_timestamp_gap_and_regression_reset_filter(self):
    display = CcncLaneDisplay()
    display.update(sample(1))
    result = display.update(sample(2, offset=0.6))
    self.assertEqual(result['LANELINE_LEFT_POSITION'], 10)
    result = display.update(sample(0, offset=-0.6))
    self.assertEqual(result['LANELINE_LEFT_POSITION'], 20)
    self.assertIsNone(display.update(sample(float('nan'))))

  def test_finishing_without_tracked_target_does_not_select_another_lane(self):
    display = CcncLaneDisplay()
    result = display.update(sample(1.0, state=3, direction=1))
    self.assertIsNone(result)

  def test_unmatched_target_does_not_invent_completion(self):
    display = CcncLaneDisplay()
    display.update(sample(0, state=2, direction=1))
    result = display.update(sample(0.05, lanes=(-8.0, -1.8, 1.8, 8.0), state=2, direction=1))
    self.assertIsNone(result)


if __name__ == '__main__':
  unittest.main()
