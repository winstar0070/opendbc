import unittest

from opendbc.car.hyundai.ccnc_probe import CcncSlotProbe, PROBE_FIELDS, SLOT_SIGNALS


class TestCcncSlotProbe(unittest.TestCase):
  def setUp(self):
    self.probe = CcncSlotProbe()
    self.request = dict(token='a' * 32, issued_ns=1_000_000_000, expires_ns=3_000_000_000,
                        slot='LEFT', status=2, distance=10., lateral=3.)
    self.gates = dict(parked=True, stationary=True, controls_inactive=True, can_valid=True, display_fresh=True)

  def update(self, request=None, now=1_000_000_000, **gates):
    return self.probe.update(self.request if request is None else request, now_ns=now, **(self.gates | gates))

  def test_all_slots_isolate_exactly_eighteen_object_fields(self):
    for slot in ('FRONT', 'ALT', 'LEFT', 'RIGHT', 'LEFT_REAR', 'RIGHT_REAR'):
      with self.subTest(slot=slot):
        probe = CcncSlotProbe()
        values = probe.update(self.request | dict(slot=slot), now_ns=1_000_000_000, **self.gates)
        prefix = 'LEAD' if slot == 'FRONT' else 'LEAD_' + slot
        status = prefix + '_STATUS' if slot.endswith('_REAR') else prefix
        self.assertEqual(len(values), 18)
        self.assertEqual(values[status], 2)
        self.assertEqual(values[prefix + '_DISTANCE'], 10.)
        self.assertEqual(values[prefix + '_LATERAL'], 3.)
        self.assertEqual({k: v for k, v in values.items() if k not in (status, prefix + '_DISTANCE', prefix + '_LATERAL')},
                         {k: 0 for k in values if k not in (status, prefix + '_DISTANCE', prefix + '_LATERAL')})

  def test_exported_fields_and_request_output_mutation_cannot_change_lease(self):
    self.assertEqual(len(PROBE_FIELDS), 18)
    self.assertEqual(len(set(PROBE_FIELDS)), 18)
    self.assertEqual(SLOT_SIGNALS['LEFT_REAR'], ('LEAD_LEFT_REAR_STATUS', 'LEAD_LEFT_REAR_DISTANCE', 'LEAD_LEFT_REAR_LATERAL'))
    result = self.update()
    result['LEAD_LEFT_DISTANCE'] = 99.
    self.assertEqual(self.update()['LEAD_LEFT_DISTANCE'], 10.)
    self.request['distance'] = 20.
    self.assertIsNone(self.update(now=1_100_000_000))

  def test_maximum_lease_expires_and_token_case_cannot_bypass_retirement(self):
    request = self.request | dict(expires_ns=11_000_000_000)
    self.assertIsNotNone(self.update(request, now=10_999_999_999))
    self.assertIsNone(self.update(request, now=11_000_000_000))
    request.update(token='A' * 32, issued_ns=11_000_000_001, expires_ns=12_000_000_000)
    self.assertIsNone(self.update(request, now=11_000_000_001))

  def test_cancel_retires_token_without_erasing_history(self):
    self.update()
    self.probe.cancel()
    self.assertIsNone(self.update(now=1_100_000_000))
    self.assertIsNotNone(self.update(self.request | dict(token='b' * 32, issued_ns=1_200_000_000), now=1_200_000_000))

  def test_future_issue_does_not_poison_new_valid_request(self):
    future = self.request | dict(issued_ns=100_000_000_000, expires_ns=101_000_000_000)
    self.assertIsNone(self.update(future))
    self.assertIsNotNone(self.update(self.request | dict(token='b' * 32, issued_ns=1_100_000_000), now=1_100_000_000))

  def test_lease_and_identical_request_reuse(self):
    values = self.update()
    self.assertEqual(self.update(now=2_999_999_999), values)
    self.assertIsNone(self.update(now=3_000_000_000))
    self.assertIsNone(self.update(self.request | dict(issued_ns=3_000_000_001, expires_ns=4_000_000_000), now=3_000_000_001))

  def test_each_gate_loss_burns_token_even_when_first_request_is_unsafe(self):
    for name in self.gates:
      for first_safe in (False, True):
        with self.subTest(gate=name, first_safe=first_safe):
          self.probe = CcncSlotProbe()
          if first_safe:
            self.assertIsNotNone(self.update())
          self.assertIsNone(self.update(now=1_100_000_000, **{name: False}))
          self.assertIsNone(self.update(now=1_200_000_000))

  def test_none_restore_requires_new_token_and_strictly_newer_issue(self):
    self.update()
    self.assertIsNone(self.probe.update(None, now_ns=1_100_000_000, **self.gates))
    self.assertIsNone(self.update(now=1_200_000_000))
    self.assertIsNone(self.update(self.request | dict(token='b' * 32), now=1_200_000_000))
    self.assertIsNotNone(self.update(self.request | dict(token='c' * 32, issued_ns=1_300_000_000), now=1_300_000_000))
    self.assertIsNone(self.update(self.request | dict(issued_ns=1_400_000_000), now=1_400_000_000))

  def test_clock_regression_invalidates_and_does_not_allow_same_token(self):
    self.update(now=1_500_000_000)
    self.assertIsNone(self.update(now=1_400_000_000))
    self.assertIsNone(self.update(now=1_600_000_000))
    self.assertIsNone(self.update(self.request | dict(token='b' * 32, issued_ns=1_300_000_000), now=1_300_000_000))
    self.assertIsNotNone(self.update(self.request | dict(token='c' * 32, issued_ns=1_700_000_000), now=1_700_000_000))

  def test_active_request_fields_are_frozen(self):
    for field, value in [('issued_ns', 1_000_000_001), ('expires_ns', 4_000_000_000), ('slot', 'RIGHT'),
                         ('status', 1), ('distance', 11.), ('lateral', 4.)]:
      with self.subTest(field=field):
        self.probe = CcncSlotProbe()
        self.update()
        self.assertIsNone(self.update(self.request | {field: value}, now=1_100_000_000))
        self.assertIsNone(self.update(now=1_200_000_000))

  def test_malformed_inputs_fail_closed_and_invalidate_current_request(self):
    malformed = [[], 'x', {}, self.request | {'extra': 1}]
    for field in self.request:
      malformed.append({k: v for k, v in self.request.items() if k != field})
    for field, values in {
      'token': [None, 'a' * 31, 'g' * 32, 7], 'slot': [None, 'rear', [], 'UNKNOWN'],
      'issued_ns': [True, -1, 1., None], 'expires_ns': [False, 1_000_000_000, 12_000_000_001, None],
      'status': [True, 0, 3, 1., None], 'distance': [True, 0., 25.6, float('nan'), float('inf'), '1', None],
      'lateral': [False, -.1, 12.8, float('nan'), float('inf'), {}, None],
    }.items():
      malformed.extend(self.request | {field: value} for value in values)
    for request in malformed:
      with self.subTest(request=request):
        self.probe = CcncSlotProbe()
        self.update()
        self.assertIsNone(self.update(request, now=1_100_000_000))
        self.assertIsNone(self.update(now=1_200_000_000))

  def test_range_edges_and_rear_experimental_statuses(self):
    for slot, statuses in [('ALT', (1, 2)), ('LEFT_REAR', (1, 2, 3, 4)), ('RIGHT_REAR', (1, 2, 3, 4))]:
      for status in statuses:
        for distance, lateral in ((.1, 0.), (25.5, 12.7)):
          request = self.request | dict(slot=slot, status=status, distance=distance, lateral=lateral, expires_ns=11_000_000_000)
          self.assertIsNotNone(CcncSlotProbe().update(request, now_ns=1_000_000_000, **self.gates))
    self.assertIsNone(self.update(self.request | dict(issued_ns=1_100_000_000)))

  def test_unknown_clock_and_nonboolean_gate_values_fail_closed(self):
    for now in (None, True, -1, float('nan'), '1', []):
      self.assertIsNone(self.probe.update(self.request, now_ns=now, **self.gates))
    for gate in self.gates:
      self.probe = CcncSlotProbe()
      self.assertIsNone(self.update(**{gate: 1}))
