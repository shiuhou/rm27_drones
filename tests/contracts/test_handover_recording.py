"""Source-backed non-contact admission and non-actuating protocol replay."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from rm27_exchange.handover import (
    HandoverRegistry, HandoverProtocol, admit_snapshot, encode, digest,
)
from rm27_exchange.handover_recording import (
    EXTENSION, FIXTURE, rebuild_recorded_fixture, snapshot_from_row,
)
from rm27_exchange.offline_contracts import Blocked
from rm27_exchange.validation import read_json

ROOT = Path(__file__).resolve().parents[2]


def protocol_case(recording):
    """Explicitly synthetic exchange messages anchored to a recorded estimate.

    This reuses the existing no-actuation protocol test profile. It is neither
    the historical flight's authority nor a claimed feasible reference plan.
    """
    case = read_json(ROOT / 'tests/contracts/fixtures/handover_synthetic.json')
    case['registry'] = deepcopy(recording['registration'])
    row = recording['rows'][0]
    snapshot = row['snapshot']
    state = snapshot['state']['record']
    origin = state['state_time_s']
    shift = lambda time: round((time - 10. + origin) * 1e9) / 1e9
    request = case['request']
    request['snapshot'] = deepcopy(snapshot)
    request['ack_deadline_s'] = shift(request['ack_deadline_s'])
    case['initial_authority']['lease_expires_at_s'] = shift(case['initial_authority']['lease_expires_at_s'])
    for wrapper in (request['reference'], request['authority']):
        record = wrapper['record']
        owner = record['metadata']['source']
        record_id = record['metadata']['record_id']
        record['metadata'] = deepcopy(state['metadata'])
        record['metadata'].update(contract_id='rm27.control', source=owner,
                                  record_id=record_id, input_kind='SYNTHETIC', availability={})
        record['metadata'].pop('extensions', None)
        record['authority']['lease_expires_at_s'] = shift(record['authority']['lease_expires_at_s'])
        if 'timing' in record:
            for key in ('generated_time_s', 'apply_time_s', 'valid_from_s',
                        'expires_at_s', 'source_state_time_s'):
                record['timing'][key] = shift(record['timing'][key])
            record['source_state_id'] = state['metadata']['record_id']
        if 'handoff' in record:
            handoff = record['handoff']
            for key in ('predecessor', 'successor'):
                handoff[key]['lease_expires_at_s'] = shift(handoff[key]['lease_expires_at_s'])
            handoff['effective_apply_time_s'] = shift(handoff['effective_apply_time_s'])
        for knot in record.get('knots', []):
            knot.update(time_s=shift(knot['time_s']),
                        position_ref_W_m=deepcopy(state['position_W_m']),
                        velocity_ref_W_m_s=deepcopy(state['velocity_W_m_s']),
                        q_ref_WB_wxyz=deepcopy(state['q_WB_wxyz']),
                        body_rate_ref_B_rad_s=deepcopy(state['body_rates_B_rad_s']))
    return case


class RecordedNavigationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = read_json(ROOT / FIXTURE)

    def setUp(self):
        self.registry = HandoverRegistry.from_dict(self.fixture['registration'])
        self.row = deepcopy(self.fixture['rows'][0])

    def derive(self, payload):
        return snapshot_from_row(payload, self.fixture['source'], self.registry,
            source_line=self.row['source_line'], snapshot_id=self.row['scenario_id'])

    def test_rebuild_from_real_recording_and_historical_failure_is_retained(self):
        self.assertEqual(rebuild_recorded_fixture(ROOT), self.fixture)
        checks = self.fixture['evidence']['original_experiment_checks']
        self.assertEqual(checks['no_contact_safety'], 'PASS')
        self.assertEqual(checks['heading_at_transfer'], 'FAIL')
        for row in self.fixture['rows']:
            state = admit_snapshot(row['snapshot'], self.registry, now_s=row['replay_now_s'])
            self.assertEqual(state['metadata']['input_kind'], 'ESTIMATED')
            self.assertIn('GROUND_TRUTH', {x['input_kind'] for x in state['metadata']['lineage']})
            self.assertFalse(row['payload']['estimate']['world_attitude_trusted'])

    def rebuild_mutated_fixture(self, fixture):
        """Vary only receiver configuration; keep all archived bytes unchanged."""
        def read(path):
            return deepcopy(fixture) if Path(path) == ROOT / FIXTURE else read_json(path)
        with patch('rm27_exchange.handover_recording.read_json', side_effect=read):
            return rebuild_recorded_fixture(ROOT)

    def test_recorded_roles_and_artifacts_must_bind_to_verified_source_bytes(self):
        paths = (
            ('recorded_simulation', 'roles', 'estimator', 'identity', 'sha256'),
            ('recorded_simulation', 'roles', 'sensor_generation', 'identity', 'sha256'),
            ('recorded_simulation', 'roles', 'truth', 'identity', 'sha256'),
            ('recorded_simulation', 'registration', 'sha256'),
            ('recorded_simulation', 'registration', 'id'),
            ('recorded_simulation', 'source_run_id'),
            ('recorded_simulation', 'source_artifact', 'sha256'),
            ('recorded_simulation', 'source_artifact', 'id'),
            ('recorded_simulation', 'runtime_inputs', 0, 'artifact', 'sha256'),
            ('recorded_simulation', 'runtime_inputs', 0, 'artifact', 'id'),
            ('identity', 'config_hash'),
        )
        for keys in paths:
            fixture = deepcopy(self.fixture)
            value = fixture['registration']
            for key in keys[:-1]:
                value = value[key]
            value[keys[-1]] = '0' * 64 if keys[-1] in ('sha256', 'config_hash') else 'other-recording'
            with self.subTest(path=keys), self.assertRaises(Blocked):
                self.rebuild_mutated_fixture(fixture)

    def test_recorded_evidence_may_not_be_empty_missing_duplicated_or_relabelled(self):
        fixture = deepcopy(self.fixture)
        fixture['evidence']['files'] = []
        with self.assertRaises(Blocked):
            self.rebuild_mutated_fixture(fixture)
        for index, item in enumerate(self.fixture['evidence']['files']):
            fixture = deepcopy(self.fixture)
            del fixture['evidence']['files'][index]
            with self.subTest(missing=item['original_path']), self.assertRaises(Blocked):
                self.rebuild_mutated_fixture(fixture)
        fixture = deepcopy(self.fixture)
        fixture['evidence']['files'].append(deepcopy(fixture['evidence']['files'][0]))
        with self.assertRaises(Blocked):
            self.rebuild_mutated_fixture(fixture)
        fixture = deepcopy(self.fixture)
        fixture['evidence']['files'][0]['original_path'] = 'another-source/alignment.py'
        with self.assertRaises(Blocked):
            self.rebuild_mutated_fixture(fixture)

    def test_frame_time_and_raw_history_fidelity_without_estimated_activation(self):
        state = self.row['snapshot']['state']['record']
        native = self.row['payload']['estimate']
        axes = np.diag([1., -1., -1.])
        np.testing.assert_allclose(state['position_W_m'], axes @ native['position_m'], atol=0, rtol=0)
        np.testing.assert_allclose(state['velocity_W_m_s'], axes @ native['velocity_m_s'], atol=0, rtol=0)
        q = state['q_WB_wxyz']
        np.testing.assert_allclose(Rotation.from_quat([*q[1:], q[0]]).as_matrix(),
            axes @ native['rotation_body_to_local'] @ axes, atol=1e-15)
        np.testing.assert_allclose(state['body_rates_B_rad_s'], axes @ native['angular_velocity_body_rad_s'], atol=0, rtol=0)
        data = state['metadata']['extensions'][EXTENSION]['data']
        self.assertEqual(data['original_estimate'], native)
        self.assertGreater(native['channels']['attitude']['acquisition_s'], state['state_time_s'])
        self.assertEqual(state['measurement_times_s']['q_WB_wxyz']['time_s'], .82)
        self.assertEqual(data['raw_control_history']['records'][0]['applied_motor_command_N'],
                         self.row['payload']['applied_motor_command_N'])
        self.assertEqual(data['diagnostic_actuator_truth']['input_kind'], 'GROUND_TRUTH')
        self.assertNotIn('actuator_state', state)
        self.assertIsNone(state['uncertainty'])
        self.assertEqual(self.row['snapshot']['control_history']['availability']['status'], 'UNKNOWN')

    def test_missing_invalid_uninitialized_and_genuine_future_measurements_reject(self):
        for case in ('position', 'rates', 'epoch', 'alignment', 'trusted', 'missing_time', 'future', 'delivery'):
            raw = deepcopy(self.row['payload'])
            e = raw['estimate']
            if case == 'position': del e['position_m']
            elif case == 'rates': e['channels']['body_rate']['valid'] = False
            elif case == 'epoch': e['generation'] += 1
            elif case == 'alignment': e['world_alignment_observed'] = False
            elif case == 'trusted': e['channels']['position']['source'] = 'UNKNOWN'
            elif case == 'missing_time': del e['channels']['attitude']['acquisition_s']
            elif case == 'future': e['channels']['attitude']['acquisition_s'] = .82001
            else: e['channels']['position']['delivery_s'] = .9
            with self.subTest(case=case), self.assertRaises(Blocked):
                self.derive(raw)

    def test_truth_siblings_do_not_change_estimate_and_unknown_runtime_stays_blocked(self):
        raw = deepcopy(self.row['payload'])
        raw['position_offline'] = [999., 999., 999.]
        raw['velocity_offline'] = [999., 999., 999.]
        derived = self.derive(raw)
        self.assertEqual(derived, self.row['snapshot'])
        for registry, now in ((replace(self.registry, mode='RUNTIME_ESTIMATED'), .82),
                              (self.registry, 1.1),
                              (replace(self.registry, epoch=1), .82),
                              (replace(self.registry, frame=replace(self.registry.frame, body_origin='UNKNOWN')), .82)):
            with self.subTest(now=now, registry=registry.mode), self.assertRaises(Blocked):
                admit_snapshot(derived, registry, now_s=now)

    def test_recorded_snapshot_protocol_preserves_boundary_and_synthetic_test_authority(self):
        case = protocol_case(self.fixture)
        request = case['request']
        protocol = HandoverProtocol(self.registry, case['initial_authority'],
                                    parent_reference_id=case['parent_reference_id'])
        self.assertEqual(protocol.request(request, now_s=.83)['status'], 'REQUEST')
        self.assertEqual(protocol.decide(request['request_id'], 'ACCEPT', now_s=.832)['status'], 'ACCEPT')
        successor = request['authority']['record']['handoff']['successor']
        self.assertEqual(protocol.acknowledge(request['request_id'], request['snapshot'],
            request['reference']['record']['reference_id'], successor, now_s=.835)['status'], 'ACK')
        self.assertEqual(protocol.tick(now_s=.84)['status'], 'ACK')
        self.assertEqual(protocol.boundary, request['snapshot'])
        self.assertEqual(protocol.boundary['state']['record']['metadata']['extensions'],
                         self.row['snapshot']['state']['record']['metadata']['extensions'])
        old = deepcopy(request['reference'])
        old['record']['authority'] = case['initial_authority']
        with self.assertRaisesRegex(Blocked, 'authority conflict'):
            protocol.validate_command(old, now_s=.841)

    def test_recorded_request_expiry_authority_and_sequence_rejection(self):
        case = protocol_case(self.fixture)
        for defect in ('expiry', 'authority', 'parent', 'stale'):
            request = deepcopy(case['request'])
            now = .83
            if defect == 'expiry': request['reference']['record']['timing']['expires_at_s'] = .829
            elif defect == 'authority': request['authority']['record']['authority']['owner_id'] = 'unregistered'
            elif defect == 'parent': request['reference']['record']['source_state_id'] = 'unknown'
            else: now = 1.0
            protocol = HandoverProtocol(self.registry, case['initial_authority'],
                                        parent_reference_id=case['parent_reference_id'])
            with self.subTest(defect=defect), self.assertRaises(Blocked):
                protocol.request(request, now_s=now)
        protocol = HandoverProtocol(self.registry, case['initial_authority'],
                                    parent_reference_id=case['parent_reference_id'])
        protocol.request(case['request'], now_s=.83)
        protocol.decide(case['request']['request_id'], 'ABORT', now_s=.831)
        retry = deepcopy(case['request']); retry['request_id'] += ':retry'
        with self.assertRaisesRegex(Blocked, 'state sequence'):
            protocol.request(retry, now_s=.832)

    def test_both_jsonl_readers_admit_same_recorded_snapshot_and_ack(self):
        case = protocol_case(self.fixture)
        request = case['request']
        messages = [
            {'operation': 'read', 'snapshot': request['snapshot'], 'now_s': .82},
            {'operation': 'request', 'request': request, 'now_s': .83},
            {'operation': 'decide', 'request_id': request['request_id'], 'decision': 'ACCEPT', 'now_s': .832},
            {'operation': 'ack', 'request_id': request['request_id'], 'snapshot': request['snapshot'],
             'reference_id': request['reference']['record']['reference_id'],
             'successor': request['authority']['record']['handoff']['successor'], 'now_s': .835},
            {'operation': 'tick', 'now_s': .84},
        ]
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp)/'receiver.json'; config.write_text(encode(case))
            for consumer in ('rl', 'non_rl'):
                result = subprocess.run([sys.executable, '-m', 'rm27_exchange.handover',
                    '--consumer', consumer, '--fixture', str(config)], cwd=ROOT, text=True,
                    input=''.join(encode(m)+'\n' for m in messages), capture_output=True, timeout=30, check=True)
                rows = [json.loads(line) for line in result.stdout.splitlines()]
                self.assertEqual([r['status'] for r in rows], ['PASS','REQUEST','ACCEPT','ACK','ACK'])
                self.assertEqual(rows[0]['snapshot'], self.row['snapshot'])
                self.assertEqual(rows[0]['snapshot_sha256'], digest(self.row['snapshot']))


if __name__ == '__main__':
    unittest.main()
