import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
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

  def packed_lateral(self, values, side="LEFT"):
    message = CANPacker("hyundai_canfd_generated").make_can_msg("CCNC_0x162", 0, values)
    parser = CANParser("hyundai_canfd_generated", [("CCNC_0x162", 0)], 0)
    parser.update([(1, [message])])
    prefix = "LEAD" if side == "FRONT" else f"LEAD_{side}"
    return parser.vl["CCNC_0x162"][f"{prefix}_LATERAL"]

  def test_small_lateral_jitter_does_not_toggle_packed_can(self):
    for side, sign in (("LEFT", 1), ("RIGHT", -1)):
      self.display.reset()
      packed = []
      for i in range(40):
        # Radar 5cm grid: +/-5cm around the 3.65m CAN rounding boundary.
        measured = 3.6 if i % 2 == 0 else 3.7
        values = self.values((1, 20., sign * measured), t=1. + i * .05)
        packed.append(self.packed_lateral(values, side))
      self.assertEqual(len(set(packed)), 1)
      self.assertNotEqual(abs(self.display.selected[side].lateral), packed[-1])

  def test_adjacent_jitter_is_stable_while_longitudinal_motion_continues(self):
    for side, sign in (("LEFT", 1), ("RIGHT", -1)):
      self.display.reset()
      packed = []
      distances = []
      for i in range(81):
        # Slow enough that the old 100ms coordinate EMA follows the wobble.
        measured = 3.6 if i == 0 else 3.6 + (.25 if (i // 5) % 2 else -.25)
        values = self.values((1, 50. - i * .3, sign * measured), t=1. + i * .05)
        packed.append(self.packed_lateral(values, side))
        distances.append(values[f'LEAD_{side}_DISTANCE'])
      self.assertLessEqual(max(packed) - min(packed), .1 + 1e-9)
      self.assertTrue(all(a > b for a, b in zip(distances, distances[1:], strict=False)))
      self.assertLess(distances[-1], 27.)

  def test_lateral_hysteresis_follows_sustained_motion_without_changing_distance(self):
    self.values((1, 20., 3.6))
    previous_x = 20.
    previous_y = -3.6
    for i in range(1, 21):
      measured = 3.6 + i * .05
      values = self.values((1, 20. + i, measured), t=1. + i * .05)
      previous_x += (20. + i - previous_x) / 3.
      previous_y += (-measured - previous_y) / 3.
      self.assertAlmostEqual(values['LEAD_LEFT_DISTANCE'], previous_x)
      self.assertAlmostEqual(self.display.selected['LEFT'].lateral, previous_y, places=6)
      self.assertLessEqual(abs(self.packed_lateral(values) - self.display.filtered_lateral['LEFT']), .075 + 1e-9)
      self.assertLessEqual(abs(self.packed_lateral(values) - abs(previous_y)), .45)
    # A sustained displacement must eventually move the painted car, while
    # allowing the deliberate deadband instead of demanding sensor jitter.
    for i in range(21, 41):
      values = self.values((1, 40., 4.6), t=1. + i * .05)
    self.assertGreaterEqual(self.packed_lateral(values), 4.3)

  def test_lateral_hysteresis_clears_on_track_side_loss_and_time_reset(self):
    for reset_kind in ('track', 'side', 'missing', 'stale', 'regression', 'explicit'):
      with self.subTest(reset_kind=reset_kind):
        self.display.reset()
        self.values((1, 20., 3.6), t=2.)
        self.values((1, 20., 3.7), t=2.05)
        track, sign, side, t = 1, 1, 'LEFT', 2.10
        if reset_kind == 'track':
          track = 2
        elif reset_kind == 'side':
          sign, side = -1, 'RIGHT'
        elif reset_kind == 'missing':
          self.values(t=2.075)
        elif reset_kind == 'stale':
          self.values((1, 20., 3.6), t=2.05, now=2.4)
        elif reset_kind == 'regression':
          t = 1.
        else:
          self.display.reset()
        values = self.values((track, 20., sign * 3.68), t=t)
        self.assertAlmostEqual(self.packed_lateral(values, side), 3.7)

  def test_tracked_cutin_crosses_shared_boundary_without_gap_and_returns(self):
    for side, sign in (("LEFT", 1), ("RIGHT", -1)):
      with self.subTest(side=side):
        self.display.reset()
        self.values((1, 20., sign * 2.1), t=1.)
        previous_x, previous_y = 20., -sign * 2.1
        for i, y in enumerate((1.95, 1.81, 1.79, 1.6, 1.81, 1.95), 1):
          values = self.values((1, 20. + i, sign * y), t=1. + i * .05)
          slot = side if y > 1.8 else 'FRONT'
          prefix = 'LEAD' if slot == 'FRONT' else 'LEAD_' + slot
          self.assertEqual(values[prefix], 2)
          self.assertEqual(list(self.display.selected), [slot])
          previous_x += (20. + i - previous_x) / 3.
          previous_y += (-sign * y - previous_y) / 3.
          self.assertAlmostEqual(values[prefix + '_DISTANCE'], previous_x)
          self.assertAlmostEqual(self.display.selected[slot].lateral, previous_y, places=6)
          self.assertEqual(len(values), 3)
          self.assertAlmostEqual(self.packed_lateral(values, slot), values[prefix + '_LATERAL'])

  def test_front_requires_previous_selected_track_and_valid_inner_geometry(self):
    self.assertEqual(self.values((9, 20., 0.)), {})
    self.values((1, 20., 2.1), t=1.05)
    self.assertEqual(self.values((9, 20., 0.), t=1.1), {})
    self.values((1, 20., 2.1), t=1.15)
    bad = model()
    bad.laneLineProbs[1] = .1
    self.assertEqual(self.values((1, 20., 1.7), t=1.2, geometry=bad), {})

  def test_cutin_keeps_painted_lateral_filter_across_slot_handoff(self):
    for sign, side in ((1, 'LEFT'), (-1, 'RIGHT')):
      self.display.reset()
      values = self.values((1, 20., sign * 2.1), t=1.)
      painted = values[f'LEAD_{side}_LATERAL']
      # Both lane membership changes and the raw measurement changes; the
      # center slot must inherit the displayed position, not jump to its EMA.
      values = self.values((1, 19.5, sign * 1.0), t=1.05)
      self.assertEqual(list(self.display.selected), ['FRONT'])
      self.assertLess(abs(values['LEAD_LATERAL'] - painted), .1 + 1e-9)
      self.assertGreater(abs(values['LEAD_LATERAL'] - abs(self.display.selected['FRONT'].lateral)), .2)
      for i in range(1, 41):
        values = self.values((1, 19.5, sign * 1.0), t=1.05 + i * .05)
      self.assertLessEqual(values['LEAD_LATERAL'], 1.3)

  def test_reused_slot_does_not_blend_unrelated_vehicle_positions(self):
    for x, y in ((45., 3.6), (20., 5.2)):
      self.display.reset()
      self.values((1, 20., 3.6), t=1.)
      values = self.values((1, x, y), t=1.05)
      self.assertAlmostEqual(values['LEAD_LEFT_DISTANCE'], x)
      self.assertAlmostEqual(values['LEAD_LEFT_LATERAL'], y)

  def test_front_competition_retains_near_previous_and_has_no_duplicates(self):
    self.values((1, 20., 2.1), (2, 22., -2.1))
    self.values((1, 20., 1.7), (2, 22., -2.1), t=1.05)
    self.values((1, 22., 1.6), (2, 20., -1.7), t=1.1)
    self.assertEqual(self.display.selected['FRONT'].track_id, 1)
    self.assertEqual(len(self.display.selected), 1)

  def test_front_competition_switches_to_clearly_closer_tracked_cutin(self):
    self.values((1, 20., 2.1), (2, 22., -2.1))
    self.values((1, 20., 1.7), (2, 22., -2.1), t=1.05)
    values = self.values((1, 30., 1.6), (2, 20., -1.7), t=1.1)
    self.assertEqual(self.display.selected['FRONT'].track_id, 2)
    self.assertEqual(len(values), 3)

  def test_front_uses_inner_lines_without_extrapolating_and_never_uses_new_center(self):
    self.values((1, 20., 2.1))
    geometry = model()
    geometry.laneLineProbs[0] = geometry.laneLineProbs[3] = 0.
    values = self.values((1, 20., 0.), (9, 5., 0.), t=1.05, geometry=geometry)
    self.assertEqual(self.display.selected['FRONT'].track_id, 1)
    self.assertEqual(values['LEAD'], 2)
    geometry.laneLines[1].x = geometry.laneLines[2].x = [0., 5., 10.]
    self.assertEqual(self.values((1, 20., 0.), t=1.1, geometry=geometry), {})

  def test_new_side_object_does_not_gain_tracked_boundary_margin(self):
    self.assertEqual(self.values((1, 20., 1.9), (2, 20., -1.9)), {})

  def test_cutin_permission_is_lost_on_loss_fault_or_quarter_second_gap(self):
    for kind in ('loss', 'fault', 'gap', 'regression'):
      with self.subTest(kind=kind):
        self.display.reset()
        self.values((1, 20., 2.1), t=2.)
        t = 2.05
        if kind == 'loss':
          self.values(t=2.025)
        elif kind == 'fault':
          self.display.update(None, read_object_lanes(model(), 2.025), 2.025)
        elif kind == 'gap':
          t = 2.25
        else:
          t = 1.9
        self.assertEqual(self.values((1, 20., 1.7), t=t), {})

  def pair_values(self, *points, t=1., multiple_front=True, geometry=None):
    return self.display.update(read_radar_objects(radar(*points), t), read_object_lanes(geometry or model(), t),
                               t, multiple_front=multiple_front)

  def test_front_pair_opt_in_selects_two_nearest_measured_center_tracks(self):
    points = ((1, 30., .5), (2, 20., -.5), (3, 40., 0.))
    self.assertEqual(self.values(*points), {})
    values = self.pair_values(*points)
    self.assertEqual([self.display.selected[k].track_id for k in ('FRONT', 'ALT')], [2, 1])
    self.assertEqual(values['LEAD'], 2)
    self.assertEqual(values['LEAD_ALT'], 2)
    self.assertEqual(values['LEAD_DISTANCE'], 20.)
    self.assertEqual(values['LEAD_ALT_DISTANCE'], 30.)
    self.assertAlmostEqual(self.packed_lateral(values, 'ALT'), .5)
    packed = CANPacker('hyundai_canfd_generated').make_can_msg('CCNC_0x162', 0, values)
    parser = CANParser('hyundai_canfd_generated', [('CCNC_0x162', 0)], 0)
    parser.update([(1, [packed])])
    self.assertEqual(parser.vl['CCNC_0x162']['LEAD_ALT'], 2)
    self.assertEqual(parser.vl['CCNC_0x162']['LEAD_ALT_DISTANCE'], 30.)

  def test_front_pair_retention_switch_and_ema_transfer_do_not_duplicate(self):
    self.pair_values((1, 20., .5), (2, 22., -.5))
    self.pair_values((1, 22., .5), (2, 20., -.5), t=1.05)
    self.assertEqual(self.display.selected['FRONT'].track_id, 1)
    before = dict(self.display.selected)
    # Six metres preserves a continuous track while still forcing a switch.
    self.pair_values((1, 28., .5), (2, 20., -.5), t=1.1)
    self.assertEqual([self.display.selected[k].track_id for k in ('FRONT', 'ALT')], [2, 1])
    self.assertAlmostEqual(self.display.selected['FRONT'].distance, before['ALT'].distance + (20. - before['ALT'].distance) / 3.)
    self.assertAlmostEqual(self.display.selected['ALT'].distance, before['FRONT'].distance + (28. - before['FRONT'].distance) / 3.)

  def test_front_pair_alt_transfers_to_side_and_mode_off_removes_alt(self):
    self.pair_values((1, 10., 0.), (2, 20., 1.7))
    values = self.pair_values((1, 10., 0.), (2, 20., 1.9), t=1.05)
    self.assertEqual(self.display.selected['LEFT'].track_id, 2)
    self.assertNotIn('LEAD_ALT', values)
    self.pair_values((1, 10., 0.), (2, 20., 1.7), t=1.1)
    self.assertEqual(self.display.selected['ALT'].track_id, 2)
    values = self.pair_values((1, 10., 0.), (2, 20., 1.7), t=1.15, multiple_front=False)
    self.assertNotIn('LEAD_ALT', values)
    self.assertNotIn('ALT', self.display.selected)

  def test_front_pair_loss_fault_invalid_geometry_and_one_candidate(self):
    values = self.pair_values((1, 20., 0.))
    self.assertIn('LEAD', values)
    self.assertNotIn('LEAD_ALT', values)
    self.assertEqual(self.pair_values(t=1.05), {})
    bad = model()
    bad.laneLineProbs[1] = .1
    self.assertEqual(self.pair_values((1, 20., 0.), t=1.1, geometry=bad), {})
    self.pair_values((1, 20., 0.), (2, 30., .5), t=1.15)
    self.assertEqual(self.display.update(None, read_object_lanes(model(), 1.2), 1.2, multiple_front=True), {})
    self.assertEqual(self.values((1, 20., 0.), t=1.25), {})

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
