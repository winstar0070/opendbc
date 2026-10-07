"""Display-only adjacent objects. Never used for control or lane-change permission."""
from dataclasses import dataclass
import math


MAX_AGE = 0.25
MAX_SOURCE_SKEW = 0.15
# CAN lateral resolution is 10cm; retain the bin for an extra 2.5cm.
LATERAL_STEP = 0.1
LATERAL_HYSTERESIS = 0.025
# Same frame offset used by openpilot radard when matching model leads to radar.
RADAR_TO_MODEL = 1.52
LaneLine = tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class RadarObject:
  track_id: int
  distance: float
  lateral: float  # model convention: negative left, positive right


@dataclass(frozen=True)
class RadarObjects:
  timestamp: float
  objects: tuple[RadarObject, ...]


@dataclass(frozen=True)
class ObjectLanes:
  timestamp: float
  lines: tuple[LaneLine, ...]


class CcncTrafficDirection:
  """Estimate road-direction motion from track displacement plus ego travel.

  Raw Sonata velocity research fields are not treated as calibrated speeds.
  This filter changes display candidates only, never radar/control messages.
  """
  def __init__(self):
    self.reset()

  def reset(self):
    self.timestamp = None
    self.ego_speed = None
    self.ego_travel = 0.
    self.history = {}
    self.accepted = set()

  def update(self, radar, ego_speed, now):
    if (radar is None or type(ego_speed) not in (int, float) or not 0 <= ego_speed <= 100 or
        not math.isfinite(now) or not 0 <= now - radar.timestamp <= MAX_AGE):
      self.reset()
      return None
    dt = None if self.timestamp is None else radar.timestamp - self.timestamp
    if dt == 0:
      return RadarObjects(radar.timestamp, tuple(o for o in radar.objects if o.track_id in self.accepted))
    if dt is not None and not 0 < dt <= MAX_AGE:
      self.reset()
      dt = None
    if dt is not None:
      self.ego_travel += (self.ego_speed + ego_speed) * .5 * dt
    self.timestamp, self.ego_speed = radar.timestamp, ego_speed
    history, accepted = {}, set()
    for obj in radar.objects:
      samples = self.history.get(obj.track_id, [])
      if samples:
        prior = samples[-1]
        gap = radar.timestamp - prior[0]
        # Slots can be reused: discontinuous coordinates are a new observation.
        if (gap <= 0 or gap > MAX_AGE or abs(obj.lateral - prior[3]) > 1.5 or
            abs(obj.distance - prior[1]) > 120 * gap + 1.):
          samples = []
      samples = [s for s in samples if radar.timestamp - s[0] <= .6]
      samples.append((radar.timestamp, obj.distance, self.ego_travel, obj.lateral))
      history[obj.track_id] = samples
      first = samples[0]
      duration = radar.timestamp - first[0]
      if duration < .3:
        continue  # A new slot has no trustworthy direction yet.
      road_speed = (obj.distance - first[1] + self.ego_travel - first[2]) / duration
      # Use the recent interval too: a direction reversal must not remain
      # visible solely because an older same-direction sample dominates.
      recent = next(s for s in samples if radar.timestamp - s[0] <= .3)
      recent_duration = radar.timestamp - recent[0]
      recent_speed = ((obj.distance - recent[1] + self.ego_travel - recent[2]) / recent_duration
                      if recent_duration >= .15 else road_speed)
      # Half a metre/second tolerates distance quantization around a stopped
      # target, while rejecting even low-speed opposing traffic.
      if road_speed >= -.5 and recent_speed >= -.5:
        accepted.add(obj.track_id)
    self.history, self.accepted = history, accepted
    return RadarObjects(radar.timestamp, tuple(o for o in radar.objects if o.track_id in accepted))


def read_radar_objects(radar, timestamp: float) -> RadarObjects | None:
  if not math.isfinite(timestamp) or any(radar.errors.to_dict().values()):
    return None
  # SCC-only tracks have no lateral measurement. Do not turn those, or binary
  # blind-spot warnings, into made-up adjacent vehicle positions.
  objects = tuple(RadarObject(p.trackId, float(p.dRel), -float(p.yRel)) for p in radar.points
                  if math.isfinite(p.dRel) and math.isfinite(p.yRel) and 0 < p.dRel < 200 and abs(p.yRel) < 12.7)
  return RadarObjects(timestamp, objects)


def read_object_lanes(model, timestamp: float) -> ObjectLanes | None:
  if (not math.isfinite(timestamp) or len(model.laneLines) != 4 or
      len(model.laneLineProbs) != 4 or len(model.laneLineStds) != 4):
    return None
  lines = []
  for line, prob, std in zip(model.laneLines, model.laneLineProbs, model.laneLineStds, strict=True):
    points = ()
    if (math.isfinite(prob) and prob >= 0.5 and math.isfinite(std) and 0 <= std <= 0.5 and
        len(line.x) == len(line.y) and len(line.x) >= 2):
      candidate = tuple((float(x), float(y)) for x, y in zip(line.x, line.y, strict=True))
      if (all(math.isfinite(x) and math.isfinite(y) for x, y in candidate) and
          all(b[0] > a[0] for a, b in zip(candidate, candidate[1:], strict=False))):
        points = candidate
    lines.append(points)
  return ObjectLanes(timestamp, tuple(lines))


def lane_y(line: LaneLine, distance: float) -> float | None:
  # Never extrapolate a short/absent model line out to a distant radar target.
  if line and line[0][0] <= distance <= line[-1][0]:
    for (x0, y0), (x1, y1) in zip(line, line[1:], strict=False):
      if x0 <= distance <= x1:
        return y0 + (y1 - y0) * (distance - x0) / (x1 - x0)
  return None


class CcncObjectDisplay:
  def __init__(self):
    self.reset()

  def reset(self):
    self.timestamp = None
    self.selected: dict[str, RadarObject] = {}
    self.display_lateral: dict[str, float] = {}

  def update(self, radar: RadarObjects | None, lanes: ObjectLanes | None, now: float, multiple_front: bool = False) -> dict[str, float]:
    if (radar is None or lanes is None or not math.isfinite(now) or
        not 0 <= now - radar.timestamp <= MAX_AGE or not 0 <= now - lanes.timestamp <= MAX_AGE or
        abs(radar.timestamp - lanes.timestamp) > MAX_SOURCE_SKEW):
      self.reset()
      return {}
    if self.timestamp is not None and not 0 <= radar.timestamp - self.timestamp < MAX_AGE:
      self.reset()
    dt = 0.0 if self.timestamp is None else radar.timestamp - self.timestamp
    self.timestamp = radar.timestamp
    values = {}
    selected = {}
    display_lateral = {}
    if not multiple_front:
      self.selected.pop("ALT", None)
      self.display_lateral.pop("ALT", None)
    previous_slots = {obj.track_id: side for side, obj in self.selected.items()}
    candidates_by_side: dict[str, list[RadarObject]] = {"LEFT": [], "RIGHT": [], "FRONT": []}
    for obj in radar.objects:
      positions = [lane_y(line, obj.distance + RADAR_TO_MODEL) for line in lanes.lines]
      inner_left, inner_right = positions[1:3]
      inner_valid = (inner_left is not None and inner_right is not None and
                     2.4 <= inner_right - inner_left <= 4.8)
      previous_side = previous_slots.get(obj.track_id)
      if inner_valid and inner_left <= obj.lateral <= inner_right:
        # New center candidates are opt-in; default mode keeps tracked cut-ins only.
        if multiple_front or previous_side is not None:
          candidates_by_side["FRONT"].append(obj)
        continue
      for side, index in (("LEFT", 0), ("RIGHT", 2)):
        left, right = positions[index:index + 2]
        if left is None or right is None or not 2.4 <= right - left <= 4.8:
          continue
        retained = inner_valid and previous_side in (side, "FRONT", "ALT")
        # Share the actual inner boundary with FRONT, without two margin gaps.
        if retained:
          in_lane = left + 0.2 <= obj.lateral < right if side == "LEFT" else left < obj.lateral <= right - 0.2
        else:
          in_lane = left + 0.2 <= obj.lateral <= right - 0.2
        if in_lane:
          candidates_by_side[side].append(obj)
    if multiple_front:
      candidates_by_side["ALT"] = candidates_by_side["FRONT"]
    used_tracks = set()
    for side in candidates_by_side:
      candidates = [obj for obj in candidates_by_side[side] if obj.track_id not in used_tracks]
      if not candidates:
        continue
      nearest = min(candidates, key=lambda p: (p.distance, p.track_id))
      previous = self.selected.get(side)
      retained = next((p for p in candidates if previous is not None and p.track_id == previous.track_id), None)
      target = retained if retained is not None and retained.distance <= nearest.distance + 5.0 else nearest
      source_side = previous_slots.get(target.track_id)
      if source_side is not None and (source_side == side or source_side in ("FRONT", "ALT") or side in ("FRONT", "ALT")):
        previous = self.selected[source_side]
      else:
        previous = None
      used_tracks.add(target.track_id)
      if previous is not None:
        alpha = dt / (0.1 + dt)
        target = RadarObject(target.track_id, previous.distance + alpha * (target.distance - previous.distance),
                             previous.lateral + alpha * (target.lateral - previous.lateral))
      selected[side] = target
      # Keep the measured EMA separate from the quantized display position.
      lateral = abs(target.lateral)
      prior_lateral = self.display_lateral.get(source_side) if previous is not None else None
      if prior_lateral is not None and abs(lateral - prior_lateral) <= LATERAL_STEP / 2 + LATERAL_HYSTERESIS:
        lateral = prior_lateral
      else:
        lateral = math.floor(lateral / LATERAL_STEP + 0.5) * LATERAL_STEP
      display_lateral[side] = lateral
      # Radar tracks do not supply an object class. Use the existing white box
      # enum rather than asserting that a reflector is a car/truck/person.
      prefix = "LEAD" if side == "FRONT" else f"LEAD_{side}"
      values.update({prefix: 2, f"{prefix}_DISTANCE": target.distance, f"{prefix}_LATERAL": lateral})
    self.selected = selected
    self.display_lateral = display_lateral
    return values
