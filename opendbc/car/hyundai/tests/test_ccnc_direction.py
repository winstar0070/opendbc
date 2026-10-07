import unittest

from opendbc.car.hyundai.ccnc_objects import CcncTrafficDirection, ObjectLanes, RadarObject, RadarObjects
from opendbc.car.hyundai.tests import test_ccnc_source_timing as fixtures


class TestCcncDirection(unittest.TestCase):
  def sequence(self, target_speed, ego_speed=20.):
    reader = CcncTrafficDirection()
    result = None
    for i in range(13):
      t = 1. + i * .05
      result = reader.update(RadarObjects(t, (RadarObject(1, 80. + (target_speed - ego_speed) * i * .05, -3.6),)), ego_speed, t)
    return result

  def test_opposing_traffic_hidden(self):
    self.assertEqual(self.sequence(-15.).objects, ())
    self.assertEqual(self.sequence(-10., ego_speed=0.).objects, ())
    self.assertEqual(self.sequence(-1.).objects, ())
    self.assertEqual(self.sequence(-.6).objects, ())

  def test_slower_same_direction_and_stopped_objects_retained(self):
    for speed in (0., 5., 20., 30.):
      with self.subTest(speed=speed):
        self.assertEqual(len(self.sequence(speed).objects), 1)

  def test_ego_acceleration_is_compensated(self):
    reader = CcncTrafficDirection()
    distance = 80.
    for i in range(13):
      speed = 10. + i
      if i:
        distance += (5. - (speed + speed - 1) / 2) * .05
      t = 1. + i * .05
      result = reader.update(RadarObjects(t, (RadarObject(1, distance, 3.6),)), speed, t)
    self.assertEqual(len(result.objects), 1)

  def test_unknown_and_held_sample_do_not_acquire_direction(self):
    reader = CcncTrafficDirection()
    sample = RadarObjects(1., (RadarObject(1, 20., -3.6),))
    for now in (1., 1.1, 1.2):
      self.assertEqual(reader.update(sample, 20., now).objects, ())

  def test_missing_stale_or_invalid_ego_resets(self):
    for bad in (None, float('nan'), float('inf'), -1.):
      reader = CcncTrafficDirection()
      sample = RadarObjects(1., (RadarObject(1, 20., -3.6),))
      self.assertIsNone(reader.update(sample, bad, 1.))
    reader = CcncTrafficDirection()
    self.assertIsNone(reader.update(None, 20., 1.))
    self.assertIsNone(reader.update(RadarObjects(1., ()), 20., 1.3))

  def test_disappearance_and_slot_reuse_require_new_observation(self):
    reader = CcncTrafficDirection()
    for i in range(9):
      t = 1. + i * .05
      result = reader.update(RadarObjects(t, (RadarObject(1, 20., -3.6),)), 20., t)
    self.assertEqual(len(result.objects), 1)
    self.assertEqual(reader.update(RadarObjects(1.45, ()), 20., 1.45).objects, ())
    self.assertEqual(reader.update(RadarObjects(1.5, (RadarObject(1, 70., -3.6),)), 20., 1.5).objects, ())

  def test_controller_hides_oncoming_and_preserves_slower_car(self):
    f = fixtures.TestCcncSourceTiming()
    f.setUp()
    f.controller.ccnc_traffic_direction = CcncTrafficDirection()
    f.cs.main_cruise_enabled = True
    f.cs.msg_162.update({'LEAD': 0, 'LEAD_ALT': 0, 'LEAD_LEFT': 0, 'LEAD_RIGHT': 0})
    lines = tuple(((0., y), (100., y)) for y in (-5.4, -1.8, 1.8, 5.4))
    for i in range(13):
      t = 1. + i * .05
      f.controller.ccnc_radar = RadarObjects(t, (RadarObject(1, 70. - 35 * i * .05, -3.6),
                                               RadarObject(2, 70. - 15 * i * .05, 3.6)))
      f.controller.ccnc_object_lanes = ObjectLanes(t, lines)
      message = f.display_messages(i * 5, updated_162=True)[0]
      values = fixtures.decode('CCNC_0x162', 0x162, message[1].hex())
      self.assertEqual(values['LEAD_LEFT'], 0)
    self.assertEqual(values['LEAD_RIGHT'], 2)

  def test_direction_reversal_removes_a_previously_visible_track(self):
    reader = CcncTrafficDirection()
    distance = 70.
    for i in range(9):
      t = 1. + i * .05
      result = reader.update(RadarObjects(t, (RadarObject(1, distance, -3.6),)), 20., t)
    self.assertEqual(len(result.objects), 1)
    for i in range(1, 9):
      distance -= 35 * .05
      t = 1.4 + i * .05
      result = reader.update(RadarObjects(t, (RadarObject(1, distance, -3.6),)), 20., t)
    self.assertEqual(result.objects, ())


if __name__ == '__main__':
  unittest.main()
