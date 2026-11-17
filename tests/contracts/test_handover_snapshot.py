"""Snapshot conformance using frozen records and actual recorded navigation."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import unittest

from rm27_exchange.handover import (
    Blocked, CHANNELS, HandoverRegistry, admit_snapshot, assert_continuity,
    build_snapshot, decode_snapshot, digest, encode, validate_snapshot,
)
from rm27_exchange.handover_scenarios import replay_records, replay_snapshots
from rm27_exchange.offline_contracts import verify_local_asm2, verify_sha256
from rm27_exchange.recorded_attitude import RECORDING, RECORDING_SHA256, RUN_ID, replay_attitude
from rm27_exchange.validation import read_json

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / 'tests/contracts/fixtures/handover_synthetic.json'


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.fixture = read_json(FIXTURE)
        self.snapshot = deepcopy(self.fixture['snapshot'])
        self.registry = HandoverRegistry.from_dict(self.fixture['registry'])

    def admit(self, value=None, registry=None, now=10.01):
        return admit_snapshot(self.snapshot if value is None else value,
                              self.registry if registry is None else registry, now_s=now)

    def test_canonical_identity_and_synthetic_fixture_digest(self):
        identity = verify_local_asm2(ROOT)
        for key, value in identity.items():
            self.assertEqual(self.snapshot['identity'][key], value)
        manifest = read_json(ROOT / 'configs/handover_scenarios_v1.json')
        verify_sha256(FIXTURE, manifest['synthetic_fixture']['sha256'])

    def test_lossless_schema_semantic_roundtrip_and_no_zero_fill(self):
        snapshot = decode_snapshot(encode(self.snapshot))
        self.assertEqual(snapshot, self.snapshot)
        self.assertEqual(self.admit(), snapshot['state']['record'])
        self.assertEqual(digest(snapshot), digest(decode_snapshot(encode(snapshot))))
        self.assertNotIn('rotor_speed_rad_s', snapshot['state']['record']['actuator_state'])
        self.assertIsNone(snapshot['state']['record']['uncertainty'])
        self.assertEqual(snapshot['state']['record']['metadata']['availability']['uncertainty']['status'], 'UNKNOWN')

    def test_copy_ownership_does_not_mutate_original_or_previous_delivery(self):
        fields = {k: v for k, v in self.snapshot.items() if k != 'format'}
        copy = build_snapshot(**fields)
        copy['state']['record']['position_W_m'][0] = 999
        self.assertNotEqual(copy, self.snapshot)
        admitted = self.admit()
        admitted['actuator_state']['rotor_thrust_N'][0] = 999
        self.assertEqual(self.admit()['actuator_state']['rotor_thrust_N'][0], .308)

    def test_malformed_schema_nonfinite_duplicate_and_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            decode_snapshot('{"format":"x","format":"y"}')
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                encode({'time': value})
        for path, value in ((('state', 'record', 'q_WB_wxyz'), [2, 0, 0, 0]),
                            (('state', 'record', 'metadata', 'reset_epoch'), None),
                            (('state', 'record', 'frames', 'body'), 'FLU')):
            snapshot = deepcopy(self.snapshot)
            target = snapshot
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.admit(snapshot)
        snapshot = deepcopy(self.snapshot)
        del snapshot['state']['record']['metadata']['contract_version']
        with self.assertRaises(Blocked):
            validate_snapshot(snapshot)

    def test_missing_channels_validity_uncertainty_are_not_invented(self):
        for channel in CHANNELS:
            snapshot = deepcopy(self.snapshot)
            del snapshot['state']['record'][channel]
            validate_snapshot(snapshot)  # Structurally optional != admitted.
            with self.subTest(channel=channel), self.assertRaises(Blocked):
                self.admit(snapshot)
        for key in ('initialized', 'aligned', 'finite', 'current'):
            snapshot = deepcopy(self.snapshot)
            snapshot['state']['record']['validity'][CHANNELS[0]][key] = None
            with self.subTest(key=key), self.assertRaises(Blocked):
                self.admit(snapshot)

    def test_frame_clock_identity_epoch_and_validity_registration_fail_closed(self):
        alternatives = [
            replace(self.registry, epoch=3),
            replace(self.registry, clock=replace(self.registry.clock, destination_id='unknown-clock')),
            replace(self.registry, frame=replace(self.registry.frame, destination_id='unknown-frame')),
            replace(self.registry, frame=replace(self.registry.frame, body_origin='UNKNOWN')),
            replace(self.registry, frame=replace(self.registry.frame, source_valid_s=(0, 5))),
            replace(self.registry, frame=replace(self.registry.frame, source_valid_s=(10, 20))),
            replace(self.registry, clock=replace(self.registry.clock, source_valid_s=(0, 5))),
            replace(self.registry, frame=replace(self.registry.frame, source_clock_id='unknown')),
        ]
        for registry in alternatives:
            with self.subTest(registry=registry), self.assertRaises(Blocked):
                self.admit(registry=registry)
        for key in ('model', 'geometry', 'plant_profile', 'config_hash'):
            snapshot = deepcopy(self.snapshot)
            if key == 'config_hash':
                snapshot['identity'][key] = '0' * 64
            else:
                snapshot['identity'][key]['sha256'] = '0' * 64
            with self.subTest(key=key), self.assertRaises(Blocked):
                self.admit(snapshot)
        for key in ('model', 'geometry', 'config_hash'):
            snapshot = deepcopy(self.snapshot)
            snapshot['state']['record']['metadata'].pop(key)
            with self.subTest(state_identity=key), self.assertRaises(Blocked):
                self.admit(snapshot)

    def test_future_stale_and_unmapped_measurement_rejection(self):
        for now in (9.999, 10.3):
            with self.subTest(now=now), self.assertRaises(Blocked):
                self.admit(now=now)
        for time, clock in ((9.0, self.registry.clock.destination_id),
                            (10.005, self.registry.clock.destination_id), (9.995, 'unknown')):
            snapshot = deepcopy(self.snapshot)
            snapshot['state']['record']['measurement_times_s'][CHANNELS[0]] = {
                'time_s': time, 'clock_id': clock}
            with self.subTest(time=time, clock=clock), self.assertRaises(Blocked):
                self.admit(snapshot)
        snapshot = deepcopy(self.snapshot)
        del snapshot['state']['record']['measurement_times_s'][CHANNELS[0]]
        with self.assertRaises(Blocked):
            self.admit(snapshot)

    def test_prediction_cannot_refresh_informative_measurement_age(self):
        snapshot = deepcopy(self.snapshot)
        state = snapshot['state']['record']
        state['metadata']['input_kind'] = 'PREDICTED'
        state['metadata']['lineage'].append({'input_kind': 'ESTIMATED', 'source': 'synthetic-prior-estimate'})
        state['state_time_s'] = 10.1
        self.admit(snapshot, now=10.11)
        state['state_time_s'] = 10.3
        with self.assertRaisesRegex(Blocked, 'stale'):
            self.admit(snapshot, now=10.31)

    def test_storage_provenance_never_erases_truth(self):
        snapshot = deepcopy(self.snapshot)
        state = snapshot['state']['record']
        state['metadata']['lineage'].append({'input_kind': 'GROUND_TRUTH', 'source': 'simulator oracle'})
        with self.assertRaisesRegex(Blocked, 'ground truth'):
            self.admit(snapshot)
        snapshot['source']['input_kind'] = 'RECORDED'
        state['metadata']['lineage'].append({'input_kind': 'RECORDED', 'source': 'stored truth-derived output'})
        with self.assertRaisesRegex(Blocked, 'ground truth'):
            self.admit(snapshot, registry=replace(self.registry, mode='RUNTIME_ESTIMATED'))
        snapshot['state']['schema'] = 'state.schema.json#/$defs/GroundTruthState'
        state['metadata']['input_kind'] = 'GROUND_TRUTH'
        validate_snapshot(snapshot)  # May archive truth; may not give it authority.
        with self.assertRaises(Blocked):
            self.admit(snapshot)

    def test_synthetic_mode_is_explicitly_distinct_from_runtime(self):
        with self.assertRaisesRegex(Blocked, 'synthetic'):
            self.admit(registry=replace(self.registry, mode='RUNTIME_ESTIMATED'))
        snapshot = deepcopy(self.snapshot)
        snapshot['state']['record']['metadata']['lineage'] = []
        with self.assertRaisesRegex(Blocked, 'lineage'):
            self.admit(snapshot)

    def test_runtime_history_cannot_smuggle_oracle_or_synthetic_evidence(self):
        snapshot = deepcopy(self.snapshot)
        snapshot['source']['input_kind'] = 'RECORDED'
        meta = snapshot['state']['record']['metadata']
        for link in meta['lineage']:
            link['input_kind'] = 'ESTIMATED'
        meta['lineage'].append({'input_kind': 'RECORDED', 'source': 'test-only hypothetical sensor record'})
        with self.assertRaisesRegex(Blocked, 'oracle or synthetic ancestry'):
            self.admit(snapshot, registry=replace(self.registry, mode='RUNTIME_ESTIMATED'))

    def test_actuator_history_and_boundary_are_not_reset(self):
        snapshot = decode_snapshot(encode(self.snapshot))
        assert_continuity(self.snapshot, snapshot)
        snapshot['state']['record']['q_WB_wxyz'] = [-x for x in snapshot['state']['record']['q_WB_wxyz']]
        assert_continuity(self.snapshot, snapshot)
        for category in ('state', 'history', 'sequence', 'epoch'):
            snapshot = deepcopy(self.snapshot)
            if category == 'state':
                snapshot['state']['record']['actuator_state']['rotor_thrust_N'] = [0] * 4
            elif category == 'history':
                snapshot['control_history']['records'].pop(0)
            else:
                snapshot['state']['record']['metadata']['reset_epoch' if category == 'epoch' else 'sequence'] += 1
            with self.subTest(category=category), self.assertRaises(Blocked):
                assert_continuity(self.snapshot, snapshot)

    def test_history_order_lengths_clock_and_last_applied_consistency(self):
        for category in ('sequence', 'time', 'epoch', 'motor_length', 'last_applied', 'clock'):
            snapshot = deepcopy(self.snapshot)
            record = snapshot['control_history']['records'][-1]['record']
            if category == 'sequence':
                record['metadata']['sequence'] = 0
            elif category == 'time':
                record['timing']['apply_time_s'] = 9.0
            elif category == 'epoch':
                record['metadata']['reset_epoch'] = 1
            elif category == 'motor_length':
                record['motor_command_applied']['values'].pop()
            elif category == 'last_applied':
                record['motor_command_applied']['values'][0] = 0
            else:
                record['metadata']['clock_id'] = 'unmapped-clock'
            with self.subTest(category=category), self.assertRaises(Blocked):
                self.admit(snapshot)

    def test_history_sequence_is_scoped_to_producer_not_global(self):
        snapshot = deepcopy(self.snapshot)
        records = snapshot['control_history']['records']
        records[0]['record']['metadata'].update(source='other-allocator', sequence=1)
        self.admit(snapshot)  # Two producer streams may both have sequence=1.

    def test_unknown_actuator_history_stays_unknown(self):
        snapshot = deepcopy(self.snapshot)
        del snapshot['state']['record']['actuator_state']
        snapshot['control_history'] = {
            'availability': {'status': 'UNKNOWN', 'reason': 'No activation or command records available'},
            'records': [],
        }
        state = self.admit(snapshot)
        self.assertNotIn('actuator_state', state)
        self.assertEqual(decode_snapshot(encode(snapshot)), snapshot)

    def test_registered_target_geometry_only_and_optional(self):
        target = read_json(ROOT / 'contracts/v0.1.0/examples/valid/geometry-manifest-stable-patch-ids.json')
        target = deepcopy(target['record']['target'])
        target['target_frame_WT']['parent_frame_id'] = self.registry.frame.destination_id
        target['registration_epoch'] = self.registry.epoch
        wrapper = {'schema': 'geometry.schema.json#/$defs/TargetGeometry', 'record': target}
        snapshot = deepcopy(self.snapshot)
        snapshot['target_geometry'] = wrapper
        with self.assertRaises(Blocked):
            self.admit(snapshot)
        registry = replace(self.registry, targets={target['target_id']: deepcopy(target)})
        self.admit(snapshot, registry)
        snapshot['state']['record']['target_relative'] = {'target_id': 'different-target'}
        with self.assertRaisesRegex(Blocked, 'target-relative'):
            self.admit(snapshot, registry)
        del snapshot['state']['record']['target_relative']
        snapshot['target_geometry']['record']['geometry']['sha256'] = '0' * 64
        with self.assertRaises(Blocked):
            self.admit(snapshot, registry)

    def test_recorded_native_navigation_is_lossless_but_not_admitted(self):
        rows = list(replay_records(ROOT, verify_originals=True))
        self.assertEqual(len(rows), 3)
        for row in rows:
            source = row['source']
            snapshot = build_snapshot(
                snapshot_id=row['scenario_id'],
                source={'run_id': source['source_run_id'],
                        'trajectory_family_id': source['source_family_id'], 'input_kind': 'RECORDED',
                        'artifact': {'id': source['source_path'], 'version': 'recorded',
                                     'sha256': source['source_sha256']},
                        'record_locator': f"line:{row['source_line']}"}, native_record=row,
            )
            self.assertEqual(decode_snapshot(encode(snapshot))['native_record']['payload'], row['payload'])
            self.assertIsNone(snapshot['state'])
            with self.assertRaisesRegex(Blocked, 'native recording'):
                self.admit(snapshot)

    def test_scenario_cli_emits_the_same_diagnostic_snapshots(self):
        snapshots = list(replay_snapshots(ROOT, verify_originals=True))
        process = subprocess.run([sys.executable, '-m', 'rm27_exchange.handover_scenarios',
                                  '--snapshots', '--verify-originals'], cwd=ROOT,
                                 text=True, capture_output=True, timeout=20, check=True)
        self.assertEqual([decode_snapshot(line) for line in process.stdout.splitlines()], snapshots)
        self.assertTrue(all(snapshot['state'] is None for snapshot in snapshots))
        self.assertEqual(snapshots, list(replay_snapshots(ROOT)))

    def test_actual_ulog_state_keeps_measurement_time_and_missing_channels(self):
        records = replay_attitude(ROOT / RECORDING, receiver_boot_us=79_160_000, max_age_us=60_000)
        self.assertEqual(len(records), 9)
        for record in records:
            snapshot = build_snapshot(snapshot_id=record['metadata']['record_id'], source={
                'run_id': RUN_ID, 'trajectory_family_id': RUN_ID, 'input_kind': 'RECORDED',
                'artifact': {'id': str(RECORDING), 'version': 'original-ULog', 'sha256': RECORDING_SHA256},
                'record_locator': f"vehicle_attitude:{record['metadata']['sequence']}"},
                state={'schema': 'state.schema.json#/$defs/EstimatedState', 'record': record})
            self.assertEqual(decode_snapshot(encode(snapshot))['state']['record'], record)
            self.assertNotIn('position_W_m', snapshot['state']['record'])
            self.assertEqual(snapshot['source']['input_kind'], 'RECORDED')
            self.assertEqual(snapshot['state']['record']['metadata']['input_kind'], 'ESTIMATED')
            with self.assertRaises(Blocked):
                self.admit(snapshot, registry=replace(self.registry, mode='RUNTIME_ESTIMATED'), now=79.16)

    def test_two_consumers_inspect_same_recorded_snapshot_and_reject_runtime_use(self):
        row = next(replay_records(ROOT))
        source = row['source']
        snapshot = build_snapshot(snapshot_id=row['scenario_id'], source={
            'run_id': source['source_run_id'], 'trajectory_family_id': source['source_family_id'],
            'input_kind': 'RECORDED', 'artifact': {'id': source['source_path'], 'version': 'recorded',
                                                 'sha256': source['source_sha256']},
            'record_locator': f"line:{row['source_line']}"}, native_record=row)
        results = []
        for consumer in ('rl', 'non_rl'):
            process = subprocess.run([sys.executable, '-m', 'rm27_exchange.handover',
                                      '--fixture', str(FIXTURE), '--consumer', consumer],
                                     cwd=ROOT, input=encode({'operation': 'inspect', 'snapshot': snapshot,
                                                            'now_s': 23.55}) + '\n',
                                     text=True, capture_output=True, timeout=20, check=True)
            output = json.loads(process.stdout)
            self.assertEqual(output['status'], 'PASS')
            self.assertEqual(output['admission']['status'], 'BLOCKED')
            self.assertEqual(output['snapshot'], snapshot)
            results.append(output['snapshot_sha256'])
        self.assertEqual(results[0], results[1])


if __name__ == '__main__':
    unittest.main()
