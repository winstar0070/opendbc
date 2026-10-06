"""Display-only model lane geometry. Never used for steering or lane-change control."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LaneModelSample:
  timestamp: float
  lanes: tuple[float | None, ...]
  state: int
  direction: int
  edges: tuple[float | None, ...] = (None, None)


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
  edges = [None, None]
  if len(model.roadEdges) == 2 and len(model.roadEdgeStds) == 2:
    for i, (edge, std) in enumerate(zip(model.roadEdges, model.roadEdgeStds, strict=True)):
      if (len(edge.x) > 0 and len(edge.y) > 0 and math.isfinite(edge.x[0]) and abs(edge.x[0]) <= 5.0
          and math.isfinite(edge.y[0]) and abs(edge.y[0]) < 30.0 and math.isfinite(std) and 0 <= std <= 0.5):
        edges[i] = float(edge.y[0])
  return LaneModelSample(timestamp, tuple(lanes), model.meta.laneChangeState.raw, model.meta.laneChangeDirection.raw, tuple(edges))


def read_camera_lanes(camera, timestamp_ns, now_ns):
  """Normalize measured stock camera boundaries for the baseline animation."""
  if timestamp_ns <= 0 or not 0 <= now_ns - timestamp_ns <= 250_000_000:
    return None
  left, right = camera.get('Info_LftLnPosVal'), camera.get('Info_RtLnPosVal')
  if (camera.get('Info_LftLnQualSta') not in (2, 3) or camera.get('Info_RtLnQualSta') not in (2, 3) or
      left is None or right is None or not math.isfinite(left) or not math.isfinite(right) or
      not 2.4 <= right - left <= 4.8):
    return None
  position = max(0, min(30, round(-30 * left / (right - left))))
  return {'LANELINE_LEFT_POSITION': position, 'LANELINE_RIGHT_POSITION': 30 - position,
          'LANELINE_LEFT': 6, 'LANELINE_RIGHT': 6, 'LANELINE_CURVATURE': 15,
          'LANE_LEFT': 0, 'LANE_RIGHT': 0, 'LANE_HIGHLIGHT': 0, 'LANE_HIGHLIGHT_DISTANCE': 0.0}


def target_within_road(pair, direction, edges):
  if pair is None or direction not in (1, 2):
    return False
  edge = edges[0 if direction == 1 else 1] if len(edges) == 2 else None
  # Comma's active lane-change state is the trigger. Only a known road edge
  # vetoes the target; absent/uncertain edge evidence must not hide the display.
  if edge is None or not math.isfinite(edge):
    return True
  return edge <= pair[0] if direction == 1 else edge >= pair[1]


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
    self.metadata_timestamp = None
    self.completion_until = None
    self.completion_now = None
    self.completion_target = None
    self.previous_state = None
    self.ego = None
    self.target = None
    self.target_decided = False
    self.target_seen = None
    self.target_hint = None
    self.direction = 0
    self.crossed = False
    self.incoming_visible = False
    self.geometry_values = None
    self.values = None

  def _invalidate(self, direction):
    metadata_timestamp = self.metadata_timestamp
    self.reset()
    self.metadata_timestamp = metadata_timestamp
    # Geometry loss can hide a model lane-index switch. Without the original
    # association, the next adjacent pair may be one lane beyond our target.
    # Keep this change rejected until it ends or its direction changes.
    self.direction = direction
    self.target_decided = bool(direction)

  def _clear_completion(self):
    self.completion_until = None
    self.completion_now = None
    self.completion_target = None
    self.previous_state = None

  def _pause(self, now):
    self._clear_completion()
    if (self.timestamp is None or now is None or not math.isfinite(now) or
        not 0 <= now - self.timestamp <= 0.25):
      self._invalidate(self.direction)
    self.values = None
    return None

  def update(self, sample: LaneModelSample | None, *, eligible=True, now=None, completion_eligible=False):
    """Track fresh measurements independently from permission to display them."""
    if sample is None:
      return self._pause(now)
    if now is not None:
      if not math.isfinite(now) or not math.isfinite(sample.timestamp):
        return self._pause(None)
      if not 0 <= now - sample.timestamp <= 0.25:
        # Stale/future metadata is not evidence that this maneuver ended.
        return self._pause(now)
    if math.isfinite(sample.timestamp):
      if self.metadata_timestamp is not None and sample.timestamp < self.metadata_timestamp:
        if now is not None or self.direction:
          # A fresh-age sample can still arrive out of order. Geometry loss
          # must not let older cancellation metadata unlock this maneuver.
          return self._pause(None)
        # Legacy offline idle filtering may start a new clock without a HUD
        # clock, but never while an active maneuver association is retained.
        self.reset()
      self.metadata_timestamp = sample.timestamp
    hold = self._completion(sample, completion_eligible, now)
    if hold is not None:
      self.values = hold
      return hold
    values = self._update(sample)
    self.previous_state = sample.state if completion_eligible and values is not None else None
    self.values = values if eligible else None
    return self.values

  def _completion(self, sample, eligible, now):
    # Only a measured, centered target after finishing -> off is completion.
    # An off sample without finishing and centered-target evidence never arms it.
    if self.completion_until is not None and sample.state != 0:
      metadata_timestamp = self.metadata_timestamp
      self.reset()
      self.metadata_timestamp = metadata_timestamp
    if self.completion_now is not None and now is not None and now < self.completion_now:
      self._clear_completion()
      return None
    ego = lane_pair(sample.lanes, 1) if len(sample.lanes) == 4 else None
    if not eligible or now is None or sample.state != 0 or sample.direction not in (0, self.direction) or ego is None:
      self._clear_completion()
      return None
    target = self.completion_target if self.completion_until is not None else self.target
    matched = (target is not None and max(abs(a - b) for a, b in zip(ego, target, strict=True)) <= 0.75)
    # Completion requires the vehicle within 35 cm of the measured lane center.
    centered = abs(sum(ego) / 2) <= 0.35
    if not matched or not centered or not target_within_road(ego, self.direction, sample.edges):
      self._clear_completion()
      return None
    if self.completion_until is None:
      if self.previous_state != 3 or not self.crossed or self.target_seen is None or not 0 <= sample.timestamp - self.target_seen <= .25:
        self._clear_completion()
        return None
      self.completion_until = now + 1.0
      self.previous_state = None
    if now >= self.completion_until:
      self._clear_completion()
      return None
    self.completion_now = now
    self.completion_target = ego
    position = max(0, min(30, round(-30 * ego[0] / (ego[1] - ego[0]))))
    return {'LANELINE_LEFT_POSITION': position, 'LANELINE_RIGHT_POSITION': 30 - position,
            'LANELINE_LEFT': 6, 'LANELINE_RIGHT': 6, 'LANE_LEFT': 0, 'LANE_RIGHT': 0,
            'LANE_HIGHLIGHT': 1, 'LANE_HIGHLIGHT_DISTANCE': 60.0}

  def _update(self, sample: LaneModelSample):
    direction = sample.direction if sample.state in (2, 3) and sample.direction in (1, 2) else 0
    if len(sample.lanes) != 4 or not math.isfinite(sample.timestamp):
      self._invalidate(direction)
      return None
    ego = lane_pair(sample.lanes, 1)
    if ego is None:
      if (self.timestamp is not None and direction == self.direction and
          0 <= sample.timestamp - self.timestamp <= 0.25):
        # A brief ego-confidence dip does not discard the original target.
        # Publish nothing while missing; on recovery the existing target
        # matching and expiry checks still apply. Do not advance this clock.
        self.geometry_values = None
      else:
        self._invalidate(direction)
      return None
    if self.timestamp is not None and sample.timestamp == self.timestamp:
      return self.geometry_values
    if self.timestamp is not None and not 0 < sample.timestamp - self.timestamp <= 0.25:
      self._invalidate(direction)
    dt = 0.05 if self.timestamp is None else sample.timestamp - self.timestamp
    self.timestamp = sample.timestamp

    # Model indices can switch to the new ego lane. Do not average across two
    # different lanes; the separately tracked target remains continuous instead.
    if self.ego is not None and abs(sum(ego) - sum(self.ego)) > (ego[1] - ego[0]):
      self.ego = None
    self.ego = smooth_pair(self.ego, ego, dt)
    if direction != self.direction:
      self.direction = direction
      self.crossed = False
      self.incoming_visible = False
      self.target = None
      self.target_decided = False
      width = ego[1] - ego[0]
      self.target_hint = sum(ego) / 2 + (-width if direction == 1 else width) if direction else None
    if direction and not self.target_decided and sample.state == 2:
      # A late/recovered finishing sample cannot identify the original target.
      # Missing start-up evidence is pending, not a permanent rejection. Retry
      # until a target pair can actually be evaluated.
      candidate = lane_pair(sample.lanes, 0 if direction == 1 else 2)
      if candidate is not None:
        self.target_decided = True
        # Do not acquire the next lane if model indices switched while waiting.
        same_target = abs(sum(candidate) / 2 - self.target_hint) <= 0.75
        self.target = candidate if same_target and target_within_road(candidate, direction, sample.edges) else None
        self.target_seen = sample.timestamp if self.target is not None else None
    elif self.target is not None:
      candidates = [p for i in range(3) if (p := lane_pair(sample.lanes, i)) is not None]
      nearest = min(candidates, key=lambda p: abs(sum(p) - sum(self.target)))
      matched = max(abs(a - b) for a, b in zip(nearest, self.target, strict=True)) <= 0.75
      # Keep the last target briefly for association, but publish no model
      # geometry while it is missing. Never reacquire a neighbouring lane.
      if (not target_within_road(self.target, direction, sample.edges) or
          (matched and not target_within_road(nearest, direction, sample.edges)) or
          sample.timestamp - self.target_seen > 0.25):
        self.target = None
        self.crossed = False
      elif not matched:
        self.geometry_values = None
        return None
      else:
        self.target = smooth_pair(self.target, nearest, dt)
        self.target_seen = sample.timestamp

    if direction and not target_within_road(self.target, direction, sample.edges):
      # Do not fall through to model ego geometry: even white model lines could
      # shake during a turn. Keep stock display while evidence is pending or
      # the target has been rejected for this change.
      self.target = None
      self.crossed = False
      self.geometry_values = None
      return None

    if self.target is not None:
      # Separate entry and exit thresholds so centimetre-scale model jitter
      # cannot repeatedly exchange the side and central green areas. A genuine
      # reversal still restores the side area, and cancellation resets both.
      depth = min(-self.target[0], self.target[1])
      self.crossed = depth >= (-0.10 if self.crossed else 0.10)
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
    # Hide the incoming boundary from the start of the change, including while
    # the green area is still beside the car. Keep the opposite boundary visible.
    # Reveal only after entering the target lane and nearing its center.
    if active:
      incoming_position = right if direction == 1 else left
      self.incoming_visible = filled and incoming_position >= (10 if self.incoming_visible else 12)
      if direction == 1 and not self.incoming_visible:
        right_color = 1
      elif direction == 2 and not self.incoming_visible:
        left_color = 1
    self.geometry_values = {
      'LANELINE_LEFT_POSITION': left, 'LANELINE_RIGHT_POSITION': right,
      'LANELINE_LEFT': left_color, 'LANELINE_RIGHT': right_color,
      'LANE_LEFT': int(active and not filled and direction == 1),
      'LANE_RIGHT': int(active and not filled and direction == 2),
      'LANE_HIGHLIGHT': int(filled), 'LANE_HIGHLIGHT_DISTANCE': 60.0 if filled else 0.0,
    }
    return self.geometry_values
