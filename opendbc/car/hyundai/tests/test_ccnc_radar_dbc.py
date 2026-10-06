import unittest

from opendbc.can import CANParser
from opendbc.can.dbc import DBC, SignalType
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum


DBC_NAME = 'hyundai_canfd_radar_generated'


class TestCcncRadarDBC(unittest.TestCase):
  def test_family_sizes_and_checksum(self):
    dbc = DBC(DBC_NAME)
    self.assertEqual(set(dbc.addr_to_msg), {0x3a0, *range(0x3a5, 0x3c5)})
    for addr, msg in dbc.addr_to_msg.items():
      self.assertEqual(msg.size, 32 if addr == 0x3a0 else 24)
      self.assertEqual(msg.sigs['CHECKSUM'].type, SignalType.HKG_CAN_FD_CHECKSUM)

  def test_recorded_relation_does_not_imply_active_target(self):
    # 139/24: CRC-valid source references a target slot that has just gone idle.
    parser = CANParser(DBC_NAME, [('RADAR_TRACK_3a6', 0), ('RADAR_TRACK_3bd', 0)], 1)
    packets = [(0x3a6, bytes.fromhex('de449b42243a30013eb003ba33ba9f0c64000000d0020000'), 1),
               (0x3bd, bytes.fromhex('f56b9b4000000000000000000000000000000000d0020000'), 1)]
    parser.update([(1_000_000_000, packets)])
    source, target = parser.vl['RADAR_TRACK_3a6'], parser.vl['RADAR_TRACK_3bd']
    self.assertEqual(source['RAW_130_6'], 25)
    self.assertEqual(target['STATE'], 0)
    self.assertAlmostEqual(source['LONG_DIST'], 6.2)
    self.assertEqual(target['LONG_DIST'], 0)

  def test_signed_sensor_coordinates_and_crc_rejection(self):
    parser = CANParser(DBC_NAME, [('RADAR_TRACK_3a5', 0)], 1)
    raw = 1 << 16 | 3 << 52 | 284 << 63 | ((-64) & 4095) << 76
    payload = bytearray(raw.to_bytes(24, 'little'))
    payload[:2] = hkg_can_fd_checksum(0x3a5, None, payload).to_bytes(2, 'little')
    parser.update([(1_000_000_000, [(0x3a5, bytes(payload), 1)])])
    self.assertAlmostEqual(parser.vl['RADAR_TRACK_3a5']['LONG_DIST'], 14.2)
    self.assertAlmostEqual(parser.vl['RADAR_TRACK_3a5']['LAT_DIST'], -3.2)
    payload[9] ^= 1
    self.assertEqual(parser.update([(1_010_000_000, [(0x3a5, bytes(payload), 1)])]), set())
    self.assertAlmostEqual(parser.vl['RADAR_TRACK_3a5']['LAT_DIST'], -3.2)

  def test_auxiliary_raw_windows_without_semantic_enums(self):
    raw = 7 << 16
    expected = {24: 422, 33: 100, 42: 419, 51: 40, 64: 422, 73: 200}
    for start, value in expected.items():
      raw |= value << start
    raw |= 2 << 94 | 1 << 134
    payload = bytearray(raw.to_bytes(32, 'little'))
    payload[:2] = hkg_can_fd_checksum(0x3a0, None, payload).to_bytes(2, 'little')
    parser = CANParser(DBC_NAME, [('RADAR_AUX_3a0', 0)], 1)
    parser.update([(1_000_000_000, [(0x3a0, bytes(payload), 1)])])
    values = parser.vl['RADAR_AUX_3a0']
    for start, value in expected.items():
      self.assertEqual(values[f'RAW_{start}_9'], value)
    self.assertEqual(values['RAW_94_2'], 2)
    self.assertEqual(values['RAW_134_1'], 1)


if __name__ == '__main__':
  unittest.main()
