"""Short, isolated PARK-only display probes. No CAN transmission or control logic."""
import math
import re


MAX_LEASE_NS = 10_000_000_000
SLOTS = ("FRONT", "ALT", "LEFT", "RIGHT", "LEFT_REAR", "RIGHT_REAR")
SLOT_SIGNALS = {}
for _slot in SLOTS:
  _prefix = "LEAD" if _slot == "FRONT" else f"LEAD_{_slot}"
  SLOT_SIGNALS[_slot] = (_prefix + "_STATUS" if _slot.endswith("_REAR") else _prefix,
                         _prefix + "_DISTANCE", _prefix + "_LATERAL")
PROBE_FIELDS = tuple(field for signals in SLOT_SIGNALS.values() for field in signals)
FIELDS = frozenset(("token", "issued_ns", "expires_ns", "slot", "status", "distance", "lateral"))


def _token(request):
  token = request.get("token") if type(request) is dict else None
  return token.lower() if type(token) is str and re.fullmatch(r"[0-9a-fA-F]{32}", token) else None


def _parse(request):
  if type(request) is not dict or request.keys() != FIELDS:
    return None
  token = _token(request)
  issued, expires = request["issued_ns"], request["expires_ns"]
  slot, status = request["slot"], request["status"]
  if (token is None or type(issued) is not int or type(expires) is not int or issued < 0 or
      not 0 < expires - issued <= MAX_LEASE_NS or type(slot) is not str or slot not in SLOTS or
      type(status) is not int or not 1 <= status <= (4 if slot.endswith("_REAR") else 2)):
    return None
  values = []
  for key, low, high in (("distance", .1, 25.5), ("lateral", 0., 12.7)):
    value = request[key]
    # Range-check before float conversion also rejects arbitrarily large ints.
    if type(value) not in (int, float) or not low <= value <= high or not math.isfinite(value):
      return None
    values.append(float(value))
  return token, issued, expires, slot, status, *values


class CcncSlotProbe:
  def __init__(self):
    self._active = None
    self._seen_tokens = set()
    self._last_issued = -1
    self._last_now = None

  def cancel(self):
    # Keep used-token and timestamp history when an external gate is lost.
    self._active = None

  def _invalidate(self, request):
    token = _token(request)
    if token is not None:
      self._seen_tokens.add(token)
    self._active = None
    return None

  def update(self, request: dict | None, *, now_ns: int, parked: bool, stationary: bool,
             controls_inactive: bool, can_valid: bool, display_fresh: bool) -> dict | None:
    if type(now_ns) is not int or now_ns < 0 or (self._last_now is not None and now_ns < self._last_now):
      return self._invalidate(request)
    self._last_now = now_ns
    parsed = _parse(request)
    if parsed is None:
      return self._invalidate(request)
    token, issued, expires, slot, status, distance, lateral = parsed
    if not issued <= now_ns < expires:
      return self._invalidate(request)
    if self._active is not None and token == self._active[0]:
      if parsed != self._active:
        return self._invalidate(request)
    else:
      self._active = None
      if token in self._seen_tokens or issued <= self._last_issued:
        return self._invalidate(request)
      self._seen_tokens.add(token)
      self._last_issued = issued
      self._active = parsed
    if not all(gate is True for gate in (parked, stationary, controls_inactive, can_valid, display_fresh)):
      return self._invalidate(request)
    # Isolate the chosen marker: overwrite every object slot, not only status.
    values = dict.fromkeys(PROBE_FIELDS, 0.)
    values.update(zip(SLOT_SIGNALS[slot], (status, distance, lateral), strict=True))
    return values
