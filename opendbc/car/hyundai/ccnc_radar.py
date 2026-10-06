"""Passive Sonata CAN-FD radar reader, exclusively for cluster display.

Layout: sunnypilot/opendbc c662942dcbe18b85c726af363b86e5fceb82b283,
hyundai_radar_3a5_3c4.py. No diagnostic requests or control publications.
"""
import math
from opendbc.car.hyundai.ccnc_objects import RadarObject, RadarObjects, MAX_AGE
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum


class CcncRadarTracks:
  def __init__(self):
    self.reset()

  def reset(self):
    self.counter = None
    self.completed_counter = None
    self.started = 0.
    self.last_time = None
    self.pending = {}
    self.snapshot = None

  def update(self, can_packets, now):
    if not math.isfinite(now):
      self.reset()
      return None
    if self.last_time is not None and (now < self.last_time or now - self.last_time > MAX_AGE):
      self.reset()
    self.last_time = now
    if now - self.started > .1:
      self.pending.clear()
      self.counter = None
    if self.snapshot is not None and now - self.snapshot.timestamp > MAX_AGE:
      self.snapshot = None
    for _, messages in can_packets:
      for addr, data, bus in messages:
        # Bus 1 / 24-byte complete family is verified on the Sonata route.
        # Same addresses on E-CAN and transmission echoes are different data.
        if bus != 1 or not 0x3a5 <= addr <= 0x3c4 or len(data) != 24:
          continue
        if hkg_can_fd_checksum(addr, None, data) != int.from_bytes(data[:2], 'little'):
          continue
        counter = data[2]
        if counter == self.completed_counter:
          continue
        if counter != self.counter:
          self.counter, self.started = counter, now
          self.pending.clear()
        self.pending[addr] = data
        if len(self.pending) != 32:
          continue
        objects = []
        for address, payload in sorted(self.pending.items()):
          if (payload[6] >> 4) & 7 not in (3, 4):
            continue
          raw = int.from_bytes(payload, 'little')
          distance = ((raw >> 63) & 8191) * .05
          lateral_raw = (raw >> 76) & 4095
          lateral = (lateral_raw - 4096 if lateral_raw & 2048 else lateral_raw) * .05
          if 0 < distance < 200 and abs(lateral) < 12.7:
            objects.append(RadarObject(address, distance, -lateral))
        self.completed_counter = counter
        self.snapshot = RadarObjects(now, tuple(objects))
        self.pending.clear()
    return self.snapshot
