import unittest
import math
from types import SimpleNamespace

from opendbc.car.hyundai.ccnc_model import CcncLaneDisplay, LaneModelSample, read_model_lanes, read_camera_lanes


def sample(t, offset=0.0, state=0, direction=0, lanes=None, edges=(-9.0, 9.0)):
  return LaneModelSample(t, tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4)) if lanes is None else lanes, state, direction, edges)


class TestModelLanes(unittest.TestCase):
  def test_in_window_out_of_order_cancel_does_not_unlock_target(self):
    display = CcncLaneDisplay()
    display.update(sample(1.05, state=2, direction=1), now=1.05)
    self.assertIsNone(display.update(sample(1.1, state=2, direction=1, lanes=()), now=1.1))
    self.assertIsNone(display.update(sample(1.15, state=2, direction=1), now=1.15))
    # Age 100 ms passes freshness, but this off metadata predates the last
    # accepted active metadata and cannot prove that the maneuver ended.
    self.assertIsNone(display.update(sample(1.1, state=0), now=1.2))
    self.assertIsNone(display.update(sample(1.25, state=2, direction=1), now=1.25))
    display.update(sample(1.3, state=0), now=1.3)
    self.assertEqual(display.update(sample(1.35, state=2, direction=1), now=1.35)['LANE_LEFT'], 1)

  def test_clock_rollback_requires_reset_before_a_new_maneuver(self):
    display = CcncLaneDisplay()
    display.update(sample(2., state=2, direction=1), now=2.)
    self.assertIsNone(display.update(sample(1., state=0), now=1.))
    self.assertIsNone(display.update(sample(1.05, state=2, direction=1), now=1.05))
    display.reset()
    self.assertEqual(display.update(sample(1.1, state=2, direction=1), now=1.1)['LANE_LEFT'], 1)

  def test_missing_sample_without_a_trustworthy_clock_does_not_rearm(self):
    for now in (None, float('nan'), .5):
      with self.subTest(now=now):
        display = CcncLaneDisplay()
        display.update(sample(1., state=2, direction=1), now=1.)
        self.assertIsNone(display.update(None, now=now))
        self.assertIsNone(display.update(sample(1.1, state=2, direction=1), now=1.1))

  def test_nonfinite_clock_or_sample_cannot_end_the_maneuver(self):
    for timestamp, now in ((1.05, float('nan')), (float('nan'), 1.05)):
      with self.subTest(timestamp=timestamp, now=now):
        display = CcncLaneDisplay()
        display.update(sample(1., state=2, direction=1), now=1.)
        self.assertIsNone(display.update(sample(timestamp, state=0), now=now))
        self.assertIsNone(display.update(sample(1.1, state=2, direction=1), now=1.1))

  def test_ego_quality_loss_does_not_reacquire_next_lane_during_same_change(self):
    for direction in (1, 2):
      with self.subTest(direction=direction):
        display = CcncLaneDisplay()
        self.assertIsNotNone(display.update(sample(1., state=2, direction=direction)))
        # The model loses ego boundaries while moving, then reindexes around
        # the destination lane. Its new adjacent lane is not our old target.
        self.assertIsNone(display.update(sample(1.4, state=2, direction=direction, lanes=(None,) * 4)))
        self.assertIsNone(display.update(sample(1.45, state=2, direction=direction)))
        self.assertIsNone(display.update(sample(1.5, state=3, direction=direction)))
        display.update(sample(1.55, state=0))
        restarted = display.update(sample(1.6, state=2, direction=direction))
        self.assertEqual(restarted['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_brief_ego_quality_loss_recovers_the_existing_target(self):
    for direction in (1, 2):
      with self.subTest(direction=direction):
        display = CcncLaneDisplay()
        self.assertIsNotNone(display.update(sample(1., state=2, direction=direction)))
        lanes = [-5.4, -1.8, 1.8, 5.4]
        # The far ego boundary drops below confidence while the original
        # target pair remains measured, as in f8/5's 50 ms quality dip.
        lanes[2 if direction == 1 else 1] = None
        self.assertIsNone(display.update(sample(1.05, state=2, direction=direction, lanes=tuple(lanes))))
        recovered = display.update(sample(1.1, offset=.03, state=2, direction=direction))
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_repeated_ego_loss_expires_the_original_association(self):
    display = CcncLaneDisplay()
    display.update(sample(1., state=2, direction=1))
    for i in range(1, 8):
      self.assertIsNone(display.update(sample(1. + i * .05, state=2, direction=1, lanes=(None,) * 4)))
    self.assertIsNone(display.update(sample(1.4, state=2, direction=1)))

  def test_invalid_sample_does_not_rearm_an_active_change(self):
    for bad in (sample(1.05, state=2, direction=1, lanes=()), sample(float('nan'), state=2, direction=1)):
      with self.subTest(sample=bad):
        display = CcncLaneDisplay()
        self.assertIsNotNone(display.update(sample(1., state=2, direction=1)))
        self.assertIsNone(display.update(bad))
        self.assertIsNone(display.update(sample(1.1, state=2, direction=1)))

  def test_timestamp_discontinuity_does_not_rearm_an_active_change(self):
    for timestamp in (1.3, .5):
      with self.subTest(timestamp=timestamp):
        display = CcncLaneDisplay()
        self.assertIsNotNone(display.update(sample(1., state=2, direction=1)))
        self.assertIsNone(display.update(sample(timestamp, state=2, direction=1)))
        self.assertIsNone(display.update(sample(timestamp + .05, state=2, direction=1)))

  def test_cancel_with_invalid_geometry_allows_a_new_change(self):
    display = CcncLaneDisplay()
    display.update(sample(1., state=2, direction=1))
    self.assertIsNone(display.update(sample(1.05, state=2, direction=1, lanes=(None,) * 4)))
    self.assertIsNone(display.update(sample(1.1, state=1, direction=1, lanes=(None,) * 4)))
    self.assertEqual(display.update(sample(1.15, state=2, direction=1))['LANE_LEFT'], 1)

  def test_direction_change_after_ego_loss_can_acquire_its_own_target(self):
    display = CcncLaneDisplay()
    display.update(sample(1., state=2, direction=1))
    self.assertIsNone(display.update(sample(1.05, state=2, direction=1, lanes=(None,) * 4)))
    self.assertEqual(display.update(sample(1.1, state=2, direction=2))['LANE_RIGHT'], 1)

  def test_pending_target_is_not_rebased_after_ego_loss(self):
    display = CcncLaneDisplay()
    self.assertIsNone(display.update(sample(1., state=2, direction=1, lanes=(None, -.1, 3.5, 7.1))))
    self.assertIsNone(display.update(sample(1.05, state=2, direction=1, lanes=(None,) * 4)))
    self.assertIsNone(display.update(sample(1.1, state=2, direction=1, lanes=(-5.25, -1.65, 1.95, 5.55))))

  def test_camera_reader_rejects_unusable_geometry(self):
    camera = {'Info_LftLnPosVal': -1.8, 'Info_RtLnPosVal': 1.8, 'Info_LftLnQualSta': 3, 'Info_RtLnQualSta': 3}
    self.assertEqual(read_camera_lanes(camera, 1_000_000_000, 1_000_000_000)['LANELINE_LEFT_POSITION'], 15)
    for changes in ({'Info_LftLnPosVal': None}, {'Info_RtLnPosVal': float('nan')},
                    {'Info_LftLnPosVal': float('inf')}, {'Info_RtLnQualSta': 0},
                    {'Info_RtLnPosVal': -1.8}, {'Info_RtLnPosVal': 9.}):
      self.assertIsNone(read_camera_lanes(camera | changes, 1_000_000_000, 1_000_000_000))

  def test_target_loss_timeout_does_not_select_a_new_lane(self):
    display = CcncLaneDisplay()
    display.update(sample(1., state=2, direction=1))
    for i in range(1, 8):
      self.assertIsNone(display.update(sample(1. + i * .05, state=2, direction=1, lanes=(None, -1.8, 1.8, 5.4))))
    self.assertIsNone(display.update(sample(1.4, state=2, direction=1)))

  def test_temporary_target_loss_recovers_original_lane(self):
    for direction in (1, 2):
      display = CcncLaneDisplay()
      self.assertIsNotNone(display.update(sample(1., state=2, direction=direction)))
      lanes = [-5.4, -1.8, 1.8, 5.4]
      lanes[0 if direction == 1 else 3] = None
      self.assertIsNone(display.update(sample(1.05, state=2, direction=direction, lanes=tuple(lanes))))
      recovered = display.update(sample(1.1, state=2, direction=direction))
      self.assertIsNotNone(recovered)
      self.assertEqual(recovered['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_crossing_jitter_does_not_toggle_fill_in_either_direction(self):
    for direction in (1, 2):
      display = CcncLaneDisplay()
      results = []
      offsets = [i * .04 for i in range(51)] + [1.8 + .04 * math.sin(i * .4) for i in range(100)]
      for i, offset in enumerate(offsets):
        results.append(display.update(sample(i * .05, offset=offset if direction == 1 else -offset, state=2, direction=direction)))
      self.assertTrue(all(v['LANE_HIGHLIGHT'] == 1 for v in results[50:]))

  def test_revealed_boundary_does_not_flicker_near_reveal_threshold(self):
    for direction in (1, 2):
      display = CcncLaneDisplay()
      incoming = 'RIGHT' if direction == 1 else 'LEFT'
      offsets = [i * .04 for i in range(86)] + [3.18 + .04 * math.sin(i * .4) for i in range(100)]
      results = [display.update(sample(i * .05, offset=o if direction == 1 else -o, state=2, direction=direction))
                 for i, o in enumerate(offsets)]
      self.assertTrue(all(v[f'LANELINE_{incoming}'] == 6 for v in results[85:]))

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
    for direction, edges in ((1, (-2.1, 9.0)), (2, (-9.0, 2.1))):
      display = CcncLaneDisplay()
      for i in range(8):
        self.assertIsNone(display.update(sample(i * .05, offset=.05 * i, state=2, direction=direction, edges=edges)))

  def test_initial_missing_target_recovers_when_data_arrives(self):
    for direction in (1, 2):
      with self.subTest(direction=direction):
        display = CcncLaneDisplay()
        lanes = [-5.4, -1.8, 1.8, 5.4]
        lanes[0 if direction == 1 else 3] = None
        self.assertIsNone(display.update(sample(0, state=2, direction=direction, lanes=tuple(lanes))))
        recovered = display.update(sample(.05, state=2, direction=direction))
        self.assertIsNotNone(recovered)
        self.assertEqual(recovered['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_comma_change_accepts_missing_edge_and_continues_when_edge_is_lost(self):
    for direction in (1, 2):
      for edges in ((None, None), (-9.0, 9.0)):
        with self.subTest(direction=direction, edges=edges):
          display = CcncLaneDisplay()
          result = display.update(sample(0, state=2, direction=direction, edges=edges))
          self.assertEqual(result['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)
          result = display.update(sample(.05, state=2, direction=direction, edges=(None, None)))
          self.assertEqual(result['LANE_LEFT' if direction == 1 else 'LANE_RIGHT'], 1)

  def test_pending_target_does_not_acquire_another_lane_after_index_switch(self):
    display = CcncLaneDisplay()
    self.assertIsNone(display.update(sample(0, state=2, direction=1, lanes=(None, -.1, 3.5, 7.1))))
    self.assertIsNone(display.update(sample(.05, state=2, direction=1, lanes=(-5.25, -1.65, 1.95, 5.55))))

  def test_confirmed_edge_rejection_does_not_rearm_during_same_change(self):
    display = CcncLaneDisplay()
    self.assertIsNone(display.update(sample(0, state=2, direction=2, edges=(-9.0, 2.0))))
    self.assertIsNone(display.update(sample(.05, state=2, direction=2)))

  def test_detected_road_edge_inside_target_clears_display_and_does_not_rearm(self):
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

class TestCompletionHold(unittest.TestCase):
  def completed(self, direction=1, finishing=True):
    display = CcncLaneDisplay()
    for i in range(73):
      offset = i * .05
      lanes = tuple(y + offset for y in (-5.4, -1.8, 1.8, 5.4))
      if offset >= 1.8:
        lanes = (lanes[0] - 3.6, *lanes[:3])
      if direction == 2:
        lanes = tuple(-y for y in reversed(lanes))
      display.update(sample(i * .05, state=3 if finishing and i >= 70 else 2, direction=direction, lanes=lanes),
                     now=i * .05, completion_eligible=True)
    return display

  def test_hold_tracks_fresh_center_then_expires(self):
    for direction in (1, 2):
      display = self.completed(direction)
      for i in range(20):
        t = 3.65 + i * .05
        result = display.update(sample(t, offset=.1), eligible=False, completion_eligible=True, now=t)
        self.assertEqual(result['LANE_HIGHLIGHT'], 1)
        self.assertEqual(result['LANE_LEFT'], 0)
        self.assertEqual(result['LANELINE_LEFT_POSITION'], 14)
      self.assertIsNone(display.update(sample(4.65), eligible=False, completion_eligible=True, now=4.65))

  def test_cancel_without_finishing_and_untracked_finishing_do_not_hold(self):
    for display in (self.completed(finishing=False), CcncLaneDisplay()):
      self.assertIsNone(display.update(sample(3.65), eligible=False, completion_eligible=True, now=3.65))
    display = CcncLaneDisplay()
    display.update(sample(3.6, state=3, direction=1), completion_eligible=True, now=3.6)
    self.assertIsNone(display.update(sample(3.65), eligible=False, completion_eligible=True, now=3.65))

  def test_hold_clears_on_loss_disengagement_or_new_maneuver(self):
    for cause in ('missing', 'quality', 'stale', 'permission', 'new', 'opposite', 'unmatched', 'uncentered'):
      display = self.completed()
      self.assertIsNotNone(display.update(sample(3.65), eligible=False, completion_eligible=True, now=3.65))
      md = sample(3.7)
      if cause == 'missing':
        md = None
      elif cause == 'quality':
        md = sample(3.7, lanes=(None,) * 4)
      elif cause == 'stale':
        md = sample(3.3)
      elif cause == 'new':
        md = sample(3.7, state=1, direction=1)
      elif cause == 'opposite':
        md = sample(3.7, direction=2)
      elif cause == 'uncentered':
        md = sample(3.7, offset=.4)
      elif cause == 'unmatched':
        md = sample(3.7, offset=1.)
      self.assertIsNone(display.update(md, eligible=False, completion_eligible=cause != 'permission', now=3.7))
      self.assertIsNone(display.update(sample(3.75), eligible=False, completion_eligible=True, now=3.75))

  def test_hold_does_not_lock_a_new_maneuver_without_prechange(self):
    display = self.completed()
    for t in (3.65, 3.7, 3.75, 3.8, 3.85):
      display.update(sample(t), eligible=False, completion_eligible=True, now=t)
    result = display.update(sample(3.9, state=2, direction=1), completion_eligible=True, now=3.9)
    self.assertEqual(result['LANE_LEFT'], 1)

  def test_hold_stops_on_wall_clock_rollback(self):
    display = self.completed()
    display.update(sample(3.65), eligible=False, completion_eligible=True, now=3.8)
    self.assertIsNone(display.update(sample(3.65), eligible=False, completion_eligible=True, now=3.7))
