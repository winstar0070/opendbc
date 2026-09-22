"""Display-only model lane geometry. Never used for steering or lane-change control."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LaneModelSample:
  timestamp: float
  lanes: tuple[float | None, ...]
  state: int
  direction: int


def read_model_lanes(model, timestamp: float) -> LaneModelSample | None:
  """Copy model-space y at the nearest point: negative left, positive right, metres."""
  if len(model.laneLines) != 4 or len(model.laneLineProbs) != 4 or len(model.laneLineStds) != 4:
    return None
  lanes = []
  for line, prob, std in zip(model.laneLines, model.laneLineProbs, model.laneLineStds, strict=True):
    valid = (len(line.x) > 0 and len(line.y) > 0 and math.isfinite(line.x[0]) and abs(line.x[0]) <= 5.0
             and math.isfinite(line.y[0]) and abs(line.y[0]) < 12.0
             and math.isfinite(prob) and prob >= 0.5 and math.isfinite(std) and 0 <= std <= 0.5)
    lanes.append(float(line.y[0]) if valid else None)
  return LaneModelSample(timestamp, tuple(lanes), model.meta.laneChangeState.raw, model.meta.laneChangeDirection.raw)


def lane_pair(lanes, index):
  left, right = lanes[index:index + 2]
  if left is None or right is None or not (math.isfinite(left) and math.isfinite(right)):
    return None
  return (left, right) if 2.4 <= right - left <= 4.8 else None


def smooth_pair(previous, current, dt):
  if previous is None:
    return current
  # Suppress small estimation jitter, but follow genuine lateral movement faster.
  error = max(abs(a - b) for a, b in zip(previous, current, strict=True))
  tau = 0.12 if error < 0.2 else 0.04
  alpha = dt / (tau + dt)
  return tuple(a + alpha * (b - a) for a, b in zip(previous, current, strict=True))


class CcncLaneDisplay:
  def __init__(self):
    self.reset()

  def reset(self):
    self.timestamp = None
    self.ego = None
    self.target = None
    self.direction = 0
    self.crossed = False
    self.values = None

  def update(self, sample: LaneModelSample | None):
    if sample is None or len(sample.lanes) != 4 or not math.isfinite(sample.timestamp):
      self.reset()
      return None
    ego = lane_pair(sample.lanes, 1)
    if ego is None:
      self.reset()
      return None
    if self.timestamp is not None and sample.timestamp == self.timestamp:
      return self.values
    if self.timestamp is not None and not 0 < sample.timestamp - self.timestamp <= 0.25:
      self.reset()
    dt = 0.05 if self.timestamp is None else sample.timestamp - self.timestamp
    self.timestamp = sample.timestamp

    # Model indices can switch to the new ego lane. Do not average across two
    # different lanes; the separately tracked target remains continuous instead.
    if self.ego is not None and abs(sum(ego) - sum(self.ego)) > (ego[1] - ego[0]):
      self.ego = None
    self.ego = smooth_pair(self.ego, ego, dt)
    direction = sample.direction if sample.state in (2, 3) and sample.direction in (1, 2) else 0
    if direction != self.direction:
      self.direction = direction
      self.crossed = False
      # A late/recovered finishing sample cannot identify the original target.
      self.target = lane_pair(sample.lanes, 0 if direction == 1 else 2) if direction and sample.state == 2 else None
    elif self.target is not None:
      candidates = [p for i in range(3) if (p := lane_pair(sample.lanes, i)) is not None]
      nearest = min(candidates, key=lambda p: abs(sum(p) - sum(self.target)))
      # Reject a lost/replaced boundary rather than animating to an unrelated lane.
      if max(abs(a - b) for a, b in zip(nearest, self.target, strict=True)) > 0.75:
        self.target = None
        self.crossed = False
      else:
        self.target = smooth_pair(self.target, nearest, dt)

    if self.target is not None:
      self.crossed = self.target[0] <= 0 <= self.target[1]
    active = bool(direction and self.target is not None)
    filled = active and self.crossed
    pair = self.target if filled else self.ego
    if active and not filled:
      # Anchor the departing lane to the tracked shared boundary. Otherwise an
      # early model index switch would jump geometry before the fill handoff.
      width = self.target[1] - self.target[0]
      pair = ((self.target[1], self.target[1] + width) if direction == 1
              else (self.target[0] - width, self.target[0]))
    width = pair[1] - pair[0]
    left = max(0, min(30, round(-30 * pair[0] / width)))
    right = 30 - left
    left_color = right_color = 6 if active else 2
    if filled:
      if direction == 1 and right < 12:
        right_color = 1
      elif direction == 2 and left < 12:
        left_color = 1
    elif active:
      if left <= 2:
        left_color = 1
      if right <= 2:
        right_color = 1
    self.values = {
      'LANELINE_LEFT_POSITION': left, 'LANELINE_RIGHT_POSITION': right,
      'LANELINE_LEFT': left_color, 'LANELINE_RIGHT': right_color,
      'LANE_LEFT': int(active and not filled and direction == 1),
      'LANE_RIGHT': int(active and not filled and direction == 2),
      'LANE_HIGHLIGHT': int(filled), 'LANE_HIGHLIGHT_DISTANCE': 60.0 if filled else 0.0,
    }
    return self.values
