import unittest
from opendbc.car.hyundai.ccnc_radar import CcncRadarTracks
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum


def frame(addr, counter=1, lateral=3.2, state=3):
  n = counter << 16 | state << 52 | round(14.2 / .05) << 63 | (round(lateral / .05) & 4095) << 76
  b = bytearray(n.to_bytes(24, 'little'))
  b[:2] = hkg_can_fd_checksum(addr, None, b).to_bytes(2, 'little')
  return (addr, bytes(b), 1)


def cycle(counter=1):
  return [frame(a, counter) for a in range(0x3a5, 0x3c5)]


class TestCcncRadar(unittest.TestCase):
  def test_complete_cycle_coordinates_and_expiry(self):
    d = CcncRadarTracks()
    self.assertIsNone(d.update([(1, cycle()[:16])], 1.))
    r = d.update([(2, cycle()[16:])], 1.01)
    self.assertEqual(len(r.objects), 32)
    self.assertAlmostEqual(r.objects[0].distance, 14.2)
    self.assertAlmostEqual(r.objects[0].lateral, -3.2)
    self.assertEqual(d.update([], 1.1), r)
    self.assertIsNone(d.update([], 1.3))

  def test_bad_crc_wrong_bus_size_and_mixed_cycles_rejected(self):
    for mode in ('crc', 'bus', 'size', 'counter'):
      d = CcncRadarTracks()
      msgs = cycle()
      a, b, bus = msgs[0]
      if mode == 'crc':
        b = b[:4] + bytes([b[4] ^ 1]) + b[5:]
      if mode == 'bus':
        bus = 0
      if mode == 'size':
        b = b[:8]
      if mode == 'counter':
        a, b, bus = frame(a, 2)
      msgs[0] = (a, b, bus)
      self.assertIsNone(d.update([(1, msgs)], 1.))

  def test_empty_cycle_clears_objects_and_clock_reset(self):
    d = CcncRadarTracks()
    d.update([(1, cycle())], 1.)
    r = d.update([(2, [frame(a, 2, state=0) for a in range(0x3a5, 0x3c5)])], 1.05)
    self.assertEqual(r.objects, ())
    self.assertIsNone(d.update([], .5))

  def test_out_of_display_range_is_not_published(self):
    d = CcncRadarTracks()
    r = d.update([(1, [frame(a, lateral=20.) for a in range(0x3a5, 0x3c5)])], 1.)
    self.assertEqual(r.objects, ())

  def test_duplicate_cycle_cannot_refresh_snapshot(self):
    d = CcncRadarTracks()
    original = d.update([(1, cycle())], 1.)
    self.assertEqual(d.update([(2, cycle())], 1.1), original)
    self.assertIsNone(d.update([(3, cycle())], 1.26))
    self.assertIsNone(d.update([], float('nan')))

  def test_negative_lateral_and_incomplete_cycle_timeout(self):
    d = CcncRadarTracks()
    d.update([(1, cycle()[:16])], 1.)
    self.assertIsNone(d.update([(2, cycle()[16:])], 1.2))
    r = d.update([(3, [frame(a, 2, lateral=-3.2) for a in range(0x3a5, 0x3c5)])], 1.21)
    self.assertAlmostEqual(r.objects[0].lateral, 3.2)
