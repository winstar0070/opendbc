"""Observed Sonata CAN-FD radar layout; raw research fields have no semantic enums."""
# ruff: noqa: INP001


def generate():
  # Keep a separate bus-1 DBC: 0x3C1 is also the bus-0 8-byte BLINKER_STALKS,
  # whereas this family uses 24 bytes. A single address map cannot hold both.
  parts = ['VERSION ""\nNS_ :\nBS_:\nBU_: FRONT_RADAR XXX\n']
  parts.append('''
BO_ 928 RADAR_AUX_3a0: 32 FRONT_RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
''')
  for start in (24, 33, 42, 51, 64, 73):
    parts.append(f' SG_ RAW_{start}_9 : {start}|9@1+ (1,0) [0|511] "" XXX\n')
  for start, width in ((87, 1), (94, 2), (120, 1), (134, 1)):
    parts.append(f' SG_ RAW_{start}_{width} : {start}|{width}@1+ (1,0) [0|{(1 << width) - 1}] "" XXX\n')

  parts.append('CM_ BO_ 928 "Bus 1, 32-byte Sonata frames. RAW research windows are not confirmed object records or physical quantities.";\n')
  for start, condition in ((24, 'bit87 nonzero'), (33, 'bits94..95 nonzero'), (64, 'bit120 nonzero'), (73, 'bit134 nonzero')):
    note = f'Observed raw!=422 iff {condition} across three drives/53 segments. '
    note += 'VALID, field boundaries, object absence and units remain unconfirmed.'
    parts.append(f'CM_ SG_ 928 RAW_{start}_9 "{note}";\n')

  # Same geometry layout already consumed by the passive ccNC display reader.
  # Reference: sunnypilot/opendbc c662942dcbe18b85c726af363b86e5fceb82b283,
  # hyundai_radar_3a5_3c4.py. Unknown/category/relation fields remain raw.
  for addr in range(0x3a5, 0x3c5):
    parts.append(f'''
BO_ {addr} RADAR_TRACK_{addr:x}: 24 FRONT_RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ RAW_40_8 : 40|8@1+ (1,0) [0|255] "" XXX
 SG_ STATE : 52|3@1+ (1,0) [0|7] "" XXX
 SG_ LONG_DIST : 63|13@1+ (0.05,0) [0|409.55] "m" XXX
 SG_ LAT_DIST : 76|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ RAW_88_14 : 88|14@1- (1,0) [-8192|8191] "" XXX
 SG_ RAW_102_2 : 102|2@1+ (1,0) [0|3] "" XXX
 SG_ RAW_130_6 : 130|6@1+ (1,0) [0|63] "" XXX
 SG_ RAW_138_5 : 138|5@1+ (1,0) [0|31] "" XXX
 SG_ RAW_144_8 : 144|8@1+ (1,0) [0|255] "" XXX
''')
    note = 'Bus 1, 24-byte complete 32-slot family; other buses have different layouts. '
    note += 'RAW boundaries and signedness are research views, not validated physical fields or classifications.'
    parts.append(f'CM_ BO_ {addr} "{note}";\n')
    parts.append(f'CM_ SG_ {addr} LAT_DIST "Sensor coordinate; display uses opposite lateral sign. No camera extrinsic offset.";\n')
    parts.append(f'CM_ SG_ {addr} RAW_102_2 "Not a validated car/truck classification. Values change within continuous tracks.";\n')
    note = 'Nonzero k observed referencing 0x3A5+k-1; target may be idle. Not a permanent ID or duplicate-removal rule.'
    parts.append(f'CM_ SG_ {addr} RAW_130_6 "{note}";\n')
  return {'hyundai_canfd_radar.dbc': ''.join(parts)}
