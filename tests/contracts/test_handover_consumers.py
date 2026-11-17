"""Real native constructors, in separate installed component environments."""
from copy import deepcopy
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from rm27_exchange.handover import HandoverRegistry, digest, encode
from rm27_exchange.handover_consumers import project_native
from rm27_exchange.offline_contracts import Blocked
from rm27_exchange.validation import ROOT, loads_json, read_json


FIXTURE = ROOT / 'tests/contracts/fixtures/handover_synthetic.json'
INTERPRETERS = {
    'rl': ROOT / 'components/rm27_RL/.venv/bin/python',
    'non_rl': Path('/home/shiuhou/venvs/mujoco/bin/python'),
}


class NativeConsumerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = read_json(FIXTURE)

    def test_common_handover_worker_dispatches_to_actual_native_types(self):
        import sys
        for consumer in ('rl', 'non_rl'):
            process = subprocess.run(
                [sys.executable, '-m', 'rm27_exchange.handover', '--fixture', str(FIXTURE),
                 '--consumer', consumer], cwd=ROOT,
                input=encode({'operation': 'native-read', 'snapshot': self.fixture['snapshot'],
                              'now_s': 10.01}) + '\n',
                capture_output=True, text=True, timeout=30, check=True,
            )
            row = loads_json(process.stdout)
            self.assertEqual(row['status'], 'PASS', row)
            self.assertEqual(row['consumer'], consumer)
            self.assertEqual(row['snapshot'], self.fixture['snapshot'])
            self.assertEqual(row['snapshot_sha256'], digest(self.fixture['snapshot']))
            self.assertIn('native_type', row)

    def project(self, consumer, fixture=None):
        fixture = fixture or self.fixture
        python = INTERPRETERS[consumer]
        if not python.is_file():
            self.skipTest(f'BLOCKED: existing {consumer} interpreter unavailable: {python}')
        environment = {key: value for key, value in os.environ.items()
                       if key not in ('PYTHONPATH', 'PYTHONHOME')}
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        result = subprocess.run(
            [sys.executable, '-m', 'rm27_exchange.handover_consumers', '--consumer', consumer],
            cwd=ROOT, env=environment, input=encode({
                'snapshot': fixture['snapshot'], 'registry': fixture['registry'], 'now_s': 10.01,
            }) + '\n', text=True, capture_output=True, timeout=30, check=True,
        )
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 1, result.stdout)
        return loads_json(lines[0])

    def test_same_snapshot_constructs_actual_installed_types_in_both_processes(self):
        before = deepcopy(self.fixture)
        results = {consumer: self.project(consumer) for consumer in INTERPRETERS}
        for consumer, result in results.items():
            with self.subTest(consumer=consumer):
                self.assertEqual(result['status'], 'PASS', result)
                self.assertEqual(result['snapshot_sha256'], digest(self.fixture['snapshot']))
                self.assertEqual(result['source_metadata'], self.fixture['snapshot']['state']['record']['metadata'])
                self.assertEqual(Path(result['python']), INTERPRETERS[consumer])
                native = result['native_type']
                self.assertTrue(Path(native['path']).is_file())
                self.assertEqual(len(native['sha256']), 64)
                self.assertEqual(result['native_frame']['body'], 'FLU')
        self.assertEqual(results['rl']['native_type']['type'],
                         'rm27_contact_physics.model.VehicleEstimate')
        self.assertEqual(results['non_rl']['native_type']['type'],
                         'rm27_non_rl.asm2_estimation_integrity.StateSample')
        self.assertEqual(results['rl']['native_values']['source'],
                         'G0_OFFLINE_SYNTHETIC_RECONSTRUCTION')
        self.assertEqual(results['rl']['native_capability']['policy_observation']['status'], 'BLOCKED')
        for sample in results['non_rl']['native_values']:
            self.assertEqual(sample['source'], 'RECONSTRUCTED_SYNTHETIC')
            self.assertTrue(sample['propagated'])
            self.assertIsNone(sample['propagation_integrity_id'])
        self.assertEqual(self.fixture, before)

    def test_inverse_registered_frame_clock_and_body_rates(self):
        fixture = deepcopy(self.fixture)
        a = Rotation.from_euler('zy', [0.4, 0.2]).as_matrix()
        shift = np.array([0.1, -0.2, 0.3])
        fixture['registry']['frame']['rotation_ned_native'] = a.tolist()
        fixture['registry']['frame']['translation_ned_m'] = shift.tolist()
        fixture['registry']['clock'].update(scale=2., offset_s=1.)
        state = fixture['snapshot']['state']['record']
        result = self.project('rl', fixture)
        self.assertEqual(result['status'], 'PASS', result)
        native = result['native_values']
        np.testing.assert_allclose(native['position'], a.T @ (np.array(state['position_W_m']) - shift))
        np.testing.assert_allclose(native['velocity'], a.T @ state['velocity_W_m_s'])
        np.testing.assert_allclose(native['rotation'], a.T @ np.diag([1., -1., -1.]))
        np.testing.assert_allclose(native['omega_body'], [0.1, 0.2, -0.05])
        self.assertEqual(native['timestamp'], 4.5)
        non_rl = self.project('non_rl', fixture)
        self.assertEqual(non_rl['status'], 'PASS', non_rl)
        for sample in non_rl['native_values']:
            self.assertAlmostEqual(sample['acquisition_s'], 4.4975)
            self.assertAlmostEqual(sample['delivery_s'], 4.505)
            self.assertAlmostEqual(sample['state_time_s'], 4.5)
            self.assertAlmostEqual(sample['max_age_s'], 0.1)

    def test_reject_before_native_projection_on_missing_registration_or_stale_state(self):
        for changed in ('registration', 'stale'):
            fixture = deepcopy(self.fixture)
            if changed == 'registration':
                fixture['snapshot']['registration'] = None
            else:
                fixture['registry']['max_age_s'] = 0.001
            for consumer in INTERPRETERS:
                with self.subTest(case=changed, consumer=consumer):
                    self.assertEqual(self.project(consumer, fixture)['status'], 'BLOCKED')

    def test_runtime_and_predicted_state_do_not_gain_native_source_qualifications(self):
        fixture = deepcopy(self.fixture)
        fixture['snapshot']['state']['record']['metadata']['input_kind'] = 'PREDICTED'
        for consumer in INTERPRETERS:
            with self.subTest(consumer=consumer):
                result = self.project(consumer, fixture)
                self.assertEqual(result['status'], 'BLOCKED')
                self.assertIn('runtime/predicted', result['reason'])
        registry = HandoverRegistry.from_dict(self.fixture['registry'])
        with self.assertRaisesRegex(Blocked, 'unknown native consumer'):
            project_native(self.fixture['snapshot'], registry, 'unregistered', 10.01)

    def test_constructor_only_child_cannot_report_conformance_pass(self):
        payload = {'values': {'position': [0., 0., 0.], 'velocity': [0., 0., 0.],
                              'rotation': np.eye(3).tolist(), 'omega_body': [0., 0., 0.]},
                   'stamp': 0.}
        result = subprocess.run(
            [str(INTERPRETERS['rl']), '-m', 'rm27_exchange.handover_consumers',
             '--construct-only', '--consumer', 'rl'], cwd=ROOT,
            env={**{key: value for key, value in os.environ.items()
                    if key not in ('PYTHONPATH', 'PYTHONHOME')}, 'PYTHONDONTWRITEBYTECODE': '1'},
            input=encode(payload) + '\n', text=True, capture_output=True, timeout=30, check=True,
        )
        child = loads_json(result.stdout)
        self.assertNotEqual(child.get('status'), 'PASS')
        self.assertEqual(child['scope'], 'UNADMITTED_NATIVE_CONSTRUCTOR_ONLY')


if __name__ == '__main__':
    unittest.main()
