"""Local adapter conformance without importing or stepping a simulator."""

from copy import deepcopy
from dataclasses import replace
from hashlib import sha256
from itertools import islice
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from rm27_exchange.offline_contracts import (
    Blocked, ClockRegistration, FrameRegistration, NativeContext, adapt_non_rl_sample,
    adapt_physical_ctbr, adapt_sim_truth, require_same_identity, verify_local_asm2, verify_sha256,
)
from test_schema_validation import BUNDLE, ROOT, read_json, validator_for
from test_semantics import control_errors, state_errors


class OfflineIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.identity = verify_local_asm2(ROOT)

    def setUp(self):
        # Independent NED yaw/origin registration plus a nontrivial body attitude.
        self.frame = FrameRegistration('test-frame-map', 'test-world', 'test-NED', 'test-native-clock',
                                       3, (1, 5), ((0, 1, 0), (1, 0, 0), (0, 0, -1)),
                                       (10, 20, 30), 'FLU', 'COM')
        self.clock = ClockRegistration('test-clock-map', 'test-native-clock', 'test-receiver-clock',
                                       3, (1, 5), 1.25, 8., .005)
        self.context = NativeContext('synthetic-run', 'mapped-state', 12, 'test-world',
                                     'test-native-clock', 3, 'synthetic-reset-record',
                                     'GROUND_TRUTH', 'native-record', 'synthetic-ego-fixture')
        self.sample = SimpleNamespace(position=[1, 2, 3], velocity=[4, 5, 6],
                                      rotation=[[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                                      omega_body=[.1, .2, .3], timestamp=2., source='MUJOCO_ORACLE')
        self.settings = dict(frame=self.frame, clock=self.clock, now_s=10.55, max_age_s=.2,
                             current_epoch=3, expected_identity=self.identity,
                             observed_identity=deepcopy(self.identity))

    def truth(self, **changes):
        return adapt_sim_truth(self.sample, self.context, **(self.settings | changes))

    def test_d1_local_identity_and_configuration_bytes(self):
        self.assertEqual(self.identity['model']['version'], '1.0.0')
        self.assertEqual(self.identity['geometry']['id'], 'ASM2-local-native-3522')
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'offline-config.json'
            original = b'{"profile":"fixture-only"}\n'
            expected = sha256(original).hexdigest()
            path.write_bytes(original)
            self.assertEqual(verify_sha256(path, expected), expected)
            path.write_bytes(original + b' ')
            with self.assertRaisesRegex(Blocked, 'SHA-256 mismatch'):
                verify_sha256(path, expected)

    def test_remote_and_historical_geometry_are_not_local(self):
        for version in ('remote-proxy-3521', 'historical-11874'):
            with self.subTest(version=version):
                other = deepcopy(self.identity)
                other['geometry']['id'] = version
                with self.assertRaisesRegex(Blocked, 'identity mismatch'):
                    self.truth(observed_identity=other)
        with self.assertRaisesRegex(Blocked, 'identity mismatch'):
            require_same_identity(self.identity | {'config_hash': 'a' * 64}, self.identity)

    def test_truth_uses_registered_rotation_translation_and_clock(self):
        record = self.truth()
        validator_for('state.schema.json#/$defs/GroundTruthState').validate(record)
        self.assertFalse(list(state_errors(record)))
        self.assertEqual(record['position_W_m'], [12, 21, 27])
        self.assertEqual(record['velocity_W_m_s'], [5, 4, -6])
        self.assertEqual(record['body_rates_B_rad_s'], [.1, -.2, -.3])
        w, x, y, z = record['q_WB_wxyz']
        np.testing.assert_allclose(Rotation.from_quat([x, y, z, w]).as_matrix(), np.eye(3), atol=1e-10)
        self.assertEqual(record['state_time_s'], 10.5)
        self.assertAlmostEqual(record['data_age_s']['state'], .05)
        self.assertEqual(record['metadata']['lineage'][0]['input_kind'], 'SYNTHETIC')
        self.assertEqual(record['metadata']['lineage'][1]['record_id'], 'native-record')
        self.assertEqual(record['metadata']['availability']['actuator_state']['status'], 'UNKNOWN')
        self.assertNotIn('actuator_state', record)
        with self.assertRaisesRegex(Blocked, 'truth lineage'):
            adapt_sim_truth(self.sample, replace(self.context, input_kind='ESTIMATED'), **self.settings)

    def test_attitude_is_hamilton_wxyz_with_nonzero_yaw(self):
        self.sample.rotation = np.eye(3)
        w, x, y, z = self.truth()['q_WB_wxyz']
        # +90 degree yaw about NED down, quaternion sign is immaterial.
        np.testing.assert_allclose(np.array([w, x, y, z]) * np.sign(w),
                                   [2**-.5, 0, 0, 2**-.5], atol=1e-10)

    def test_bad_or_unknown_registration_is_not_an_identity_transform(self):
        for changes in (
            {'clock': None}, {'clock': replace(self.clock, source_id='unknown-clock')},
            {'clock': replace(self.clock, scale=0)}, {'current_epoch': 4},
            {'frame': replace(self.frame, body_origin='camera')},
            {'frame': replace(self.frame, source_valid_s=(2.1, 3))},
            {'frame': replace(self.frame, rotation_ned_native=((1, 0, 0), (0, 1, 0), (0, 0, -1)))},
            {'now_s': 10.49}, {'now_s': 10.8}, {'now_s': 10.502}, {'now_s': 10.699},
        ):
            with self.subTest(changes=changes), self.assertRaises(Blocked):
                self.truth(**changes)

    def test_non_rl_channel_preserves_measurement_and_absence(self):
        context = replace(self.context, input_kind='ESTIMATED')
        sample = SimpleNamespace(quantity='position', value=[1, 2, 3], frame='world',
                                 source='IMPLEMENTED_VIO', acquisition_s=2., state_time_s=2.,
                                 delivery_s=2.02, epoch_id=3, max_age_s=.1, valid=True,
                                 propagated=False, clipped=False, gap=False)
        record = adapt_non_rl_sample(sample, context, **self.settings)
        validator_for('state.schema.json#/$defs/EstimatedState').validate(record)
        self.assertFalse(list(state_errors(record)))
        self.assertEqual(record['position_W_m'], [12, 21, 27])
        self.assertEqual(record['measurement_times_s']['position_W_m'],
                         {'clock_id': 'test-receiver-clock', 'time_s': 10.5})
        self.assertEqual(record['metadata']['availability']['velocity_W_m_s']['status'], 'UNKNOWN')
        self.assertNotIn('published_time_s', record)
        for overrides in ({'epoch_id': 2}, {'delivery_s': 1.99}, {'delivery_s': 2.1},
                          {'state_time_s': 2.01}, {'max_age_s': .01}, {'gap': True},
                          {'source': 'GROUND_TRUTH'}, {'source': 'TRUTH_GENERATED_SYNTHETIC'}):
            with self.subTest(overrides=overrides), self.assertRaises(Blocked):
                adapt_non_rl_sample(SimpleNamespace(**(vars(sample) | overrides)), context, **self.settings)

    def test_legacy_hover_projection_does_not_acquire_missing_registration(self):
        fixture = read_json(Path(__file__).parent / 'fixtures/passive_hover_legacy_ego.json')
        original = deepcopy(fixture)
        recording = ROOT / fixture['source_path']
        if recording.is_file():  # Copied fixture is portable; verify source when available.
            verify_sha256(recording, fixture['source_sha256'])
            index = fixture['source_line'] - 1
            with recording.open() as stream:
                row = json.loads(next(islice(stream, index, index + 1)))
            self.assertEqual(row['phase'], fixture['phase'])
            self.assertEqual({key: row['physical_state'][key] for key in fixture['physical_state']},
                             fixture['physical_state'])
        with self.assertRaisesRegex(Blocked, 'missing frame, clock or epoch'):
            adapt_sim_truth(SimpleNamespace(**fixture['physical_state']), None, **(
                self.settings | {'frame': None, 'clock': None}))
        self.assertEqual(fixture, original)

    def ctbr(self):
        state = self.truth()
        envelope = read_json(BUNDLE / 'examples/valid/control-ctbr.json')['record']
        envelope.pop('collective_thrust_N')
        envelope.pop('body_rate_sp_B_rad_s')
        meta = state['metadata']
        for key in ('run_id', 'frame_id', 'clock_id', 'model', 'geometry', 'reset_epoch'):
            envelope['metadata'][key] = deepcopy(meta[key])
        envelope['source_state_id'] = meta['record_id']
        envelope['target_id'] = None
        envelope['metadata']['source'] = envelope['authority']['owner_id']
        envelope['authority']['lease_expires_at_s'] = 11.
        envelope['timing'].update(source_state_time_s=10.5, generated_time_s=10.52,
                                  valid_from_s=10.54, apply_time_s=10.6, expires_at_s=10.8)
        return dict(representation='PHYSICAL_CTBR_SI', source_body='FLU', envelope=envelope,
                    source_state=state, active_authority=deepcopy(envelope['authority']),
                    current_epoch=3, now_s=10.55, max_state_age_s=.2)

    def test_physical_ctbr_records_n_and_frd_rates(self):
        kwargs = self.ctbr()
        original = deepcopy(kwargs['envelope'])
        record = adapt_physical_ctbr([1.2, .1, .2, -.3], **kwargs)
        validator_for('control.schema.json#/$defs/DirectCTBR').validate(record)
        self.assertFalse(list(control_errors(record, kwargs)))
        self.assertEqual(record['collective_thrust_N'], 1.2)
        self.assertEqual(record['body_rate_sp_B_rad_s'], [.1, -.2, .3])
        self.assertAlmostEqual(record['timing']['measurement_age_s'], .1)
        self.assertEqual(kwargs['envelope'], original)

    def test_ctbr_rejects_normalized_stale_and_conflicting_inputs(self):
        kwargs = self.ctbr()
        for overrides in ({'representation': 'NORMALIZED_ACTION'}, {'source_body': 'UNKNOWN'},
                          {'now_s': 10.8}, {'max_state_age_s': .01}, {'current_epoch': 4},
                          {'active_authority': kwargs['active_authority'] | {'generation': 3}}):
            with self.subTest(overrides=overrides), self.assertRaises(Blocked):
                adapt_physical_ctbr([1.2, .1, .2, -.3], **(kwargs | overrides))
        for channel, value in (('clock_id', 'other-clock'), ('geometry', {'id': 'other'}),
                               ('source', 'another-owner')):
            modified = deepcopy(kwargs)
            modified['envelope']['metadata'][channel] = value
            with self.subTest(channel=channel), self.assertRaises(Blocked):
                adapt_physical_ctbr([1.2, .1, .2, -.3], **modified)
        for value in ([-1., .1, .2, -.3], [1.2, .1, float('nan'), -.3]):
            with self.subTest(value=value), self.assertRaises(Blocked):
                adapt_physical_ctbr(value, **kwargs)
        no_ancestry = deepcopy(kwargs)
        no_ancestry['source_state']['metadata']['lineage'] = []
        with self.assertRaisesRegex(Blocked, 'synthetic target-free'):
            adapt_physical_ctbr([1.2, .1, .2, -.3], **no_ancestry)
        kwargs['envelope']['authority']['lease_expires_at_s'] = 10.6
        kwargs['active_authority']['lease_expires_at_s'] = 10.6
        with self.assertRaisesRegex(Blocked, 'lease expired'):
            adapt_physical_ctbr([1.2, .1, .2, -.3], **kwargs)
