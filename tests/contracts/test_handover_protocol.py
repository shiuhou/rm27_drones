"""Non-actuating handover handshake and the two common JSONL readers."""

from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys
import unittest

from rm27_exchange.handover import HandoverProtocol, HandoverRegistry
from rm27_exchange.offline_contracts import Blocked
from rm27_exchange.validation import read_json


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / 'tests/contracts/fixtures/handover_synthetic.json'


class HandoverProtocolTests(unittest.TestCase):
    def setUp(self):
        self.fixture = read_json(FIXTURE)
        self.registry = HandoverRegistry.from_dict(self.fixture['registry'])
        self.request = deepcopy(self.fixture['request'])
        self.initial = deepcopy(self.fixture['initial_authority'])

    def protocol(self):
        return HandoverProtocol(self.registry, self.initial,
                                parent_reference_id=self.fixture['parent_reference_id'])

    def successor(self):
        return deepcopy(self.request['authority']['record']['handoff']['successor'])

    def complete(self, *, decision='ACCEPT', reference=None):
        protocol = self.protocol()
        request_id = self.request['request_id']
        self.assertEqual(protocol.request(self.request, now_s=10.01)['status'], 'REQUEST')
        self.assertEqual(protocol.decide(request_id, decision, now_s=10.012,
                                         reference=reference)['status'], decision)
        accepted = reference if reference is not None else self.request['reference']
        result = protocol.acknowledge(request_id, self.request['snapshot'],
                                      accepted['record']['reference_id'], self.successor(), now_s=10.015)
        self.assertEqual(result['status'], 'ACK')
        self.assertEqual(protocol.active, self.initial)
        self.assertEqual(protocol.tick(now_s=10.02)['status'], 'ACK')
        return protocol

    def active_reference(self, protocol, reference=None):
        wrapper = deepcopy(self.request['reference'] if reference is None else reference)
        wrapper['record']['authority'] = deepcopy(protocol.active)
        return wrapper

    def test_accept_needs_ack_and_effective_time_before_switch(self):
        protocol = self.protocol()
        request_id = self.request['request_id']
        protocol.request(self.request, now_s=10.01)
        protocol.decide(request_id, 'ACCEPT', now_s=10.012)
        self.assertEqual(protocol.active, self.initial)
        self.assertIsNone(protocol.boundary)
        protocol.acknowledge(request_id, self.request['snapshot'],
                             self.request['reference']['record']['reference_id'],
                             self.successor(), now_s=10.015)
        self.assertEqual(protocol.tick(now_s=10.019)['authority'], self.initial)
        self.assertIsNone(protocol.boundary)
        result = protocol.tick(now_s=10.02)
        expected = self.successor() | {'handoff_state': 'ACTIVE'}
        self.assertEqual(result, {'status': 'ACK', 'authority': expected})
        self.assertIsNone(protocol.pending)
        self.assertEqual(protocol.boundary, self.request['snapshot'])

    def test_adjust_commits_only_the_validated_revised_reference(self):
        revised = deepcopy(self.request['reference'])
        revised['record']['reference_id'] += ':adjusted'
        protocol = self.complete(decision='ADJUST', reference=revised)
        self.assertEqual(protocol.parent_reference_id, revised['record']['reference_id'])
        self.assertEqual(protocol.validate_command(self.active_reference(protocol, revised),
                                                   now_s=10.021)['status'], 'PASS')
        with self.assertRaisesRegex(Blocked, 'parent identity'):
            protocol.validate_command(self.active_reference(protocol), now_s=10.022)

    def test_abort_and_timeout_keep_predecessor_and_do_not_create_boundary(self):
        for decision in ('ABORT', 'TIMEOUT'):
            with self.subTest(decision=decision):
                protocol = self.protocol()
                protocol.request(self.request, now_s=10.01)
                if decision == 'ABORT':
                    result = protocol.decide(self.request['request_id'], 'ABORT', now_s=10.012)
                else:
                    protocol.decide(self.request['request_id'], 'ACCEPT', now_s=10.012)
                    result = protocol.tick(now_s=10.018)
                self.assertEqual(result['status'], decision)
                self.assertEqual(protocol.active, self.initial)
                self.assertIsNone(protocol.pending)
                self.assertIsNone(protocol.boundary)
                self.assertEqual(protocol.tick(now_s=10.02)['authority'], self.initial)
                with self.assertRaises(Blocked):
                    protocol.acknowledge(self.request['request_id'], self.request['snapshot'],
                                         self.request['reference']['record']['reference_id'],
                                         self.successor(), now_s=10.021)

    def test_stale_future_expired_invalid_authority_epoch_and_parent_requests_are_rejected(self):
        cases = []
        cases.append((deepcopy(self.request), 10.201, 'stale'))
        cases.append((deepcopy(self.request), 9.999, 'future'))
        request = deepcopy(self.request)
        request['reference']['record']['timing']['expires_at_s'] = 10.009
        cases.append((request, 10.01, 'expired'))
        request = deepcopy(self.request)
        request['authority']['record']['authority']['owner_id'] = 'unregistered-controller'
        cases.append((request, 10.01, 'authority'))
        request = deepcopy(self.request)
        request['snapshot']['state']['record']['metadata']['reset_epoch'] += 1
        cases.append((request, 10.01, 'epoch'))
        request = deepcopy(self.request)
        request['reference']['record']['source_state_id'] = 'wrong-source-state'
        cases.append((request, 10.01, 'parent'))
        request = deepcopy(self.request)
        request['reference']['record']['parent_id'] = 'wrong-parent-reference'
        cases.append((request, 10.01, 'parent'))
        for request, now, reason in cases:
            with self.subTest(reason=reason):
                protocol = self.protocol()
                with self.assertRaisesRegex(Blocked, reason):
                    protocol.request(request, now_s=now)
                self.assertEqual(protocol.active, self.initial)
                self.assertIsNone(protocol.pending)

    def test_duplicate_request_or_nonincreasing_source_sequence_is_rejected(self):
        protocol = self.protocol()
        protocol.request(self.request, now_s=10.01)
        protocol.decide(self.request['request_id'], 'ABORT', now_s=10.012)
        with self.assertRaisesRegex(Blocked, 'duplicate request'):
            protocol.request(self.request, now_s=10.013)
        request = deepcopy(self.request)
        request['request_id'] += ':retry'
        with self.assertRaisesRegex(Blocked, 'state sequence'):
            protocol.request(request, now_s=10.014)

    def test_ack_preserves_state_actuator_and_history_and_rejects_reset_boundary(self):
        protocol = self.protocol()
        before = deepcopy(self.request['snapshot'])
        protocol.request(self.request, now_s=10.01)
        protocol.decide(self.request['request_id'], 'ACCEPT', now_s=10.012)
        changed = deepcopy(before)
        changed['state']['record']['position_W_m'][0] += .1
        with self.assertRaisesRegex(Blocked, 'discontinuity'):
            protocol.acknowledge(self.request['request_id'], changed,
                                 self.request['reference']['record']['reference_id'],
                                 self.successor(), now_s=10.014)
        protocol.acknowledge(self.request['request_id'], before,
                             self.request['reference']['record']['reference_id'],
                             self.successor(), now_s=10.015)
        protocol.tick(now_s=10.02)
        self.assertEqual(protocol.boundary, before)
        self.assertEqual(protocol.boundary['state']['record']['actuator_state'],
                         before['state']['record']['actuator_state'])
        self.assertEqual(protocol.boundary['control_history'], before['control_history'])
        self.assertEqual(self.request['snapshot'], before)

    def test_ack_rejects_wrong_reference_or_successor_and_missing_decision(self):
        for stage in ('undecided', 'reference', 'authority'):
            with self.subTest(stage=stage):
                protocol = self.protocol()
                protocol.request(self.request, now_s=10.01)
                if stage != 'undecided':
                    protocol.decide(self.request['request_id'], 'ACCEPT', now_s=10.012)
                reference_id = self.request['reference']['record']['reference_id']
                successor = self.successor()
                if stage == 'reference':
                    reference_id += ':wrong'
                if stage == 'authority':
                    successor['owner_id'] = 'wrong-controller'
                with self.assertRaises(Blocked):
                    protocol.acknowledge(self.request['request_id'], self.request['snapshot'],
                                         reference_id, successor, now_s=10.015)
                self.assertEqual(protocol.active, self.initial)

    def test_switched_commands_reject_old_authority_expiry_and_duplicates(self):
        protocol = self.complete()
        old = self.active_reference(protocol)
        old['record']['authority'] = deepcopy(self.initial)
        old['record']['metadata']['source'] = self.initial['owner_id']
        with self.assertRaisesRegex(Blocked, 'authority conflict'):
            protocol.validate_command(old, now_s=10.021)
        valid = self.active_reference(protocol)
        admitted = protocol.validate_command(valid, now_s=10.022)
        self.assertEqual(admitted['status'], 'PASS')
        self.assertEqual(admitted['record'], valid['record'])
        with self.assertRaisesRegex(Blocked, 'duplicate/out-of-order'):
            protocol.validate_command(valid, now_s=10.023)
        expired = deepcopy(valid)
        expired['record']['metadata']['sequence'] += 1
        with self.assertRaisesRegex(Blocked, 'expired'):
            protocol.validate_command(expired, now_s=10.15)

    def test_epoch_reset_revokes_pending_active_and_future_command_admission(self):
        protocol = self.complete()
        reference = self.active_reference(protocol)
        result = protocol.invalidate_epoch(self.registry.epoch + 1)
        self.assertEqual(result['status'], 'ABORT')
        self.assertIsNone(result['authority'])
        self.assertIsNone(protocol.active)
        self.assertIsNone(protocol.pending)
        self.assertIsNone(protocol.boundary)
        with self.assertRaisesRegex(Blocked, 'no acknowledged'):
            protocol.validate_command(reference, now_s=10.021)
        with self.assertRaisesRegex(Blocked, 'no registered'):
            protocol.request(self.request, now_s=10.022)

    def test_reference_identity_cannot_be_reused_for_mutated_motion_or_timing(self):
        for mutation in ('motion', 'timing'):
            with self.subTest(mutation=mutation):
                protocol = self.complete()
                reference = self.active_reference(protocol)
                reference['record']['metadata']['sequence'] += 1
                reference['record']['metadata']['record_id'] += ':redelivery'
                if mutation == 'motion':
                    reference['record']['knots'][0]['position_ref_W_m'][0] += .1
                else:
                    reference['record']['timing']['expires_at_s'] += .01
                with self.assertRaisesRegex(Blocked, 'reference content changed'):
                    protocol.validate_command(reference, now_s=10.021)

        protocol = self.complete()
        redelivery = self.active_reference(protocol)
        redelivery['record']['metadata']['sequence'] += 1
        redelivery['record']['metadata']['record_id'] += ':redelivery'
        self.assertEqual(protocol.validate_command(redelivery, now_s=10.021)['status'], 'PASS')

    def test_both_cross_process_consumers_read_and_transfer_identical_snapshot(self):
        successor = self.successor()
        command = deepcopy(self.request['reference'])
        command['record']['authority'] = successor | {'handoff_state': 'ACTIVE'}
        old = deepcopy(command)
        old['record']['authority'] = deepcopy(self.initial)
        old['record']['metadata']['source'] = self.initial['owner_id']
        reference_id = self.request['reference']['record']['reference_id']
        messages = [
            {'operation': 'read', 'snapshot': self.request['snapshot'], 'now_s': 10.01},
            {'operation': 'request', 'request': self.request, 'now_s': 10.01},
            {'operation': 'decide', 'request_id': self.request['request_id'],
             'decision': 'ACCEPT', 'now_s': 10.012},
            {'operation': 'ack', 'request_id': self.request['request_id'],
             'snapshot': self.request['snapshot'], 'reference_id': reference_id,
             'successor': successor, 'now_s': 10.015},
            {'operation': 'tick', 'now_s': 10.019},
            {'operation': 'tick', 'now_s': 10.02},
            {'operation': 'command', 'command': command, 'now_s': 10.021},
            {'operation': 'command', 'command': command, 'now_s': 10.022},
            {'operation': 'command', 'command': old, 'now_s': 10.023},
        ]
        results = []
        for consumer in ('rl', 'non_rl'):
            completed = subprocess.run(
                [sys.executable, '-m', 'rm27_exchange.handover', '--fixture', str(FIXTURE),
                 '--consumer', consumer], cwd=ROOT,
                input=''.join(json.dumps(message) + '\n' for message in messages),
                capture_output=True, text=True, check=True, timeout=20,
            )
            rows = [json.loads(line) for line in completed.stdout.splitlines()]
            self.assertEqual([row['status'] for row in rows],
                             ['PASS', 'REQUEST', 'ACCEPT', 'ACK', 'UNCHANGED', 'ACK',
                              'PASS', 'BLOCKED', 'BLOCKED'])
            self.assertEqual(rows[0]['snapshot'], self.request['snapshot'])
            self.assertEqual(rows[0]['state'], self.request['snapshot']['state']['record'])
            self.assertEqual(rows[3]['authority'], self.initial)
            self.assertEqual(rows[4]['authority'], self.initial)
            self.assertEqual(rows[5]['authority'], command['record']['authority'])
            self.assertIn('G0_OFFLINE_COMMON_READER', completed.stderr)
            self.assertIn(str(ROOT / 'rm27_exchange/handover.py'), completed.stderr)
            for row in rows:
                self.assertEqual(row.pop('consumer'), consumer)
            results.append(rows)
        self.assertEqual(results[0], results[1])
