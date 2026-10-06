"""Display-only adjacent objects. Never used for control or lane-change permission."""
from dataclasses import dataclass
import math


MAX_AGE = 0.25
MAX_SOURCE_SKEW = 0.15
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

  def update(self, radar: RadarObjects | None, lanes: ObjectLanes | None, now: float) -> dict[str, float]:
    if (radar is None or lanes is None or not math.isfinite(now) or
        not 0 <= now - radar.timestamp <= MAX_AGE or not 0 <= now - lanes.timestamp <= MAX_AGE or
        abs(radar.timestamp - lanes.timestamp) > MAX_SOURCE_SKEW):
      self.reset()
      return {}
    if self.timestamp is not None and not 0 <= radar.timestamp - self.timestamp <= MAX_AGE:
      self.reset()
    dt = 0.0 if self.timestamp is None else radar.timestamp - self.timestamp
    self.timestamp = radar.timestamp
    values = {}
    selected = {}
    for side, index in (("LEFT", 0), ("RIGHT", 2)):
      candidates = []
      for obj in radar.objects:
        left, right = (lane_y(line, obj.distance + RADAR_TO_MODEL) for line in lanes.lines[index:index + 2])
        if (left is not None and right is not None and 2.4 <= right - left <= 4.8 and
            left + 0.2 <= obj.lateral <= right - 0.2):
          candidates.append(obj)
      if not candidates:
        continue
      nearest = min(candidates, key=lambda p: (p.distance, p.track_id))
      previous = self.selected.get(side)
      retained = next((p for p in candidates if previous is not None and p.track_id == previous.track_id), None)
      target = retained if retained is not None and retained.distance <= nearest.distance + 5.0 else nearest
      if previous is not None and target.track_id == previous.track_id:
        alpha = dt / (0.1 + dt)
        target = RadarObject(target.track_id, previous.distance + alpha * (target.distance - previous.distance),
                             previous.lateral + alpha * (target.lateral - previous.lateral))
      selected[side] = target
      # Radar tracks do not supply an object class. Use the existing white box
      # enum rather than asserting that a reflector is a car/truck/person.
      values.update({f"LEAD_{side}": 2, f"LEAD_{side}_DISTANCE": target.distance, f"LEAD_{side}_LATERAL": abs(target.lateral)})
    self.selected = selected
    return values
