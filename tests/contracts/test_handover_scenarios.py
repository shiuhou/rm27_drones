"""Replay real local navigation evidence without inventing admissible state."""

from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from rm27_exchange.handover_scenarios import (
    DEFAULT_MANIFEST, load_scenario_set, replay_records, replay_snapshots,
)
from rm27_exchange.offline_contracts import Blocked
from rm27_exchange.validation import read_json


ROOT = Path(__file__).resolve().parents[2]
FIXTURE = 'tests/contracts/fixtures/handover_navigation_recorded.json'
ESTIMATED_FIXTURE = 'tests/contracts/fixtures/handover_navigation_estimated.json'


class HandoverScenarioTests(unittest.TestCase):
    def setUp(self):
        self.manifest = read_json(ROOT / DEFAULT_MANIFEST)
        self.fixture = read_json(ROOT / FIXTURE)
        # Legacy rejection cases remain about the original incomplete recording.
        self.manifest['sources'] = [source for source in self.manifest['sources']
                                    if source['fixture_path'] == FIXTURE]

    def write_set(self, root, *, manifest=None, fixture=None, copy_originals=False):
        manifest = deepcopy(self.manifest if manifest is None else manifest)
        fixture = deepcopy(self.fixture if fixture is None else fixture)
        path = root / FIXTURE
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps(fixture, allow_nan=False), encoding='utf-8')
        digest = sha256(path.read_bytes()).hexdigest()
        for source in manifest['sources']:
            source['fixture_sha256'] = digest
        manifest_path = root / DEFAULT_MANIFEST
        manifest_path.parent.mkdir(parents=True)
        manifest_path.write_text(json.dumps(manifest, allow_nan=False), encoding='utf-8')
        if copy_originals:
            for field in ('source_path', 'summary_path'):
                relative = self.fixture['source'][field]
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / relative, target)

    def test_actual_navigation_projection_matches_originals_and_preserves_unknowns(self):
        dataset = load_scenario_set(ROOT, verify_originals=True)
        self.assertTrue(dataset['originals_verified'])
        self.assertEqual(dataset['dataset_readiness'], 'BLOCKED')
        records = [record for record in dataset['records']
                   if record['source']['source_run_id'] == self.fixture['source']['source_run_id']]
        self.assertEqual(len(records), 3)
        self.assertEqual([row['source_line'] for row in records], [1176, 1177, 1178])
        for native, record in zip(self.fixture['rows'], records):
            self.assertEqual(record['payload'], native['payload'])
            self.assertEqual(record['source'], self.fixture['source'])
            self.assertEqual(record['input_kind'], 'RECORDED')
            self.assertEqual(record['split'], 'dev')
            self.assertTrue(all(value is None for value in record['registration'].values()))
            self.assertEqual([link['input_kind'] for link in record['lineage']],
                             ['SYNTHETIC', 'GROUND_TRUTH'])
            self.assertIn('physical_state', record['payload'])
            self.assertIn('controls', record['payload'])
            self.assertNotIn('state', record)
            self.assertNotIn('actuator_state', record)
        self.assertGreater(abs(records[0]['payload']['telemetry']['local_velocity'][0]), .2)
        self.assertTrue(dataset['reasons'])

    def test_replay_is_deterministic_and_does_not_mutate_or_reset_native_data(self):
        first, second = list(replay_records(ROOT)), list(replay_records(ROOT))
        self.assertEqual(first, second)
        before = (ROOT / FIXTURE).read_bytes()
        first[0]['payload']['controls'][0] = -123
        first[0]['registration']['clock_mapping'] = 'invented'
        first[0]['source']['source_run_id'] = 'changed'
        self.assertEqual(list(replay_records(ROOT)), second)
        self.assertEqual((ROOT / FIXTURE).read_bytes(), before)
        self.assertNotEqual(first, second)

    def test_cross_process_jsonl_replay_preserves_the_same_records(self):
        result = subprocess.run(
            [sys.executable, '-m', 'rm27_exchange.handover_scenarios', '--root', str(ROOT)],
            cwd=ROOT, capture_output=True, text=True, check=True, timeout=20,
        )
        self.assertEqual([json.loads(line) for line in result.stdout.splitlines()],
                         list(replay_records(ROOT)))
        self.assertIn('Dataset readiness BLOCKED', result.stderr)

    def test_family_leakage_and_relabeling_one_trace_as_multiple_families_are_rejected(self):
        for overrides, error in (({'split': 'train'}, 'split leakage'),
                                 ({'source_family_id': 'invented-independent-family'},
                                  'multiple family')):
            manifest = deepcopy(self.manifest)
            manifest['sources'].append(manifest['sources'][0] | overrides)
            with self.subTest(overrides=overrides), TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.write_set(root, manifest=manifest)
                with self.assertRaisesRegex(Blocked, error):
                    load_scenario_set(root)

    def test_fixture_identity_time_lineage_and_duplicate_rejections(self):
        cases = []
        changed = deepcopy(self.fixture)
        changed['source']['source_run_id'] = 'different-run'
        cases.append((changed, 'identity mismatch'))
        changed = deepcopy(self.fixture)
        changed['rows'][1]['payload']['simulation_s'] = changed['rows'][0]['payload']['simulation_s']
        cases.append((changed, 'timestamps'))
        changed = deepcopy(self.fixture)
        changed['rows'][1]['scenario_id'] = changed['rows'][0]['scenario_id']
        cases.append((changed, 'duplicate'))
        changed = deepcopy(self.fixture)
        changed['rows'][0]['input_kind'] = 'ESTIMATED'
        cases.append((changed, 'remain RECORDED'))
        changed = deepcopy(self.fixture)
        del changed['registration']['clock_mapping']
        cases.append((changed, 'missing registration'))
        changed = deepcopy(self.fixture)
        del changed['rows'][0]['payload']['controls']
        cases.append((changed, 'incomplete'))
        for fixture, error in cases:
            with self.subTest(error=error), TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.write_set(root, fixture=fixture)
                with self.assertRaisesRegex(Blocked, error):
                    load_scenario_set(root)

    def test_bad_hash_and_external_paths_are_rejected(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_set(root)
            with (root / FIXTURE).open('a') as stream:
                stream.write('\n')
            with self.assertRaisesRegex(Blocked, 'SHA-256 mismatch'):
                load_scenario_set(root)
            for manifest_path in ('../outside.json', str(ROOT / DEFAULT_MANIFEST)):
                with self.subTest(path=manifest_path), self.assertRaisesRegex(Blocked, 'root-relative|escapes'):
                    load_scenario_set(root, manifest_path)

    def test_rehashed_projection_cannot_disagree_with_native_recording(self):
        fixture = deepcopy(self.fixture)
        fixture['rows'][0]['payload']['controls'][0] += .01
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_set(root, fixture=fixture, copy_originals=True)
            # A valid fixture hash proves bytes, not fidelity to original inputs.
            self.assertEqual(load_scenario_set(root)['dataset_readiness'], 'BLOCKED')
            with self.assertRaisesRegex(Blocked, 'differs from original'):
                load_scenario_set(root, verify_originals=True)

    def test_tampered_original_source_is_rejected(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_set(root, copy_originals=True)
            with (root / self.fixture['source']['source_path']).open('a') as stream:
                stream.write('\n')
            with self.assertRaisesRegex(Blocked, 'SHA-256 mismatch'):
                load_scenario_set(root, verify_originals=True)

    def test_readable_evidence_does_not_promote_dataset_readiness(self):
        manifest = deepcopy(self.manifest)
        manifest['dataset_readiness'] = 'PASS'
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_set(root, manifest=manifest)
            with self.assertRaisesRegex(Blocked, 'does not establish dataset readiness'):
                load_scenario_set(root)


class RecordedEstimatorScenarioTests(unittest.TestCase):
    def setUp(self):
        self.manifest = read_json(ROOT / DEFAULT_MANIFEST)
        self.manifest['sources'] = [source for source in self.manifest['sources']
                                    if source['fixture_path'] == ESTIMATED_FIXTURE]
        self.fixture = read_json(ROOT / ESTIMATED_FIXTURE)

    def write_fixture(self, root, fixture):
        fixture_path = root / ESTIMATED_FIXTURE
        fixture_path.parent.mkdir(parents=True)
        fixture_path.write_text(json.dumps(fixture, allow_nan=False), encoding='utf-8')
        manifest = deepcopy(self.manifest)
        for source in manifest['sources']:
            source['fixture_sha256'] = sha256(fixture_path.read_bytes()).hexdigest()
        manifest_path = root / DEFAULT_MANIFEST
        manifest_path.parent.mkdir(parents=True)
        manifest_path.write_text(json.dumps(manifest, allow_nan=False), encoding='utf-8')

    def test_actual_estimator_recording_is_admitted_without_promoting_dataset(self):
        dataset = load_scenario_set(ROOT, verify_originals=True)
        records = [record for record in dataset['records'] if 'snapshot' in record]
        self.assertEqual(len(records), 3)
        self.assertEqual([record['source_line'] for record in records], [165, 166, 167])
        self.assertEqual(dataset['dataset_readiness'], 'BLOCKED')
        snapshots = list(replay_snapshots(ROOT, verify_originals=True))
        for original, record in zip(self.fixture['rows'], records):
            self.assertEqual(record['admission'], 'ADMITTED')
            self.assertEqual(record['payload'], original['payload'])
            self.assertEqual(record['snapshot'], original['snapshot'])
            self.assertIn(record['snapshot'], snapshots)
            self.assertEqual(record['split'], 'dev')
            self.assertEqual(record['snapshot']['source']['input_kind'], 'RECORDED')

    def test_rehashed_raw_payload_cannot_disagree_with_admitted_snapshot(self):
        fixture = deepcopy(self.fixture)
        fixture['rows'][0]['payload']['estimate']['position_m'][0] += .1
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_fixture(root, fixture)
            with self.assertRaisesRegex(Blocked, 'snapshot|projection|binding'):
                load_scenario_set(root)

    def test_source_locator_and_profile_cannot_be_relabelled(self):
        cases = []
        changed = deepcopy(self.fixture)
        changed['rows'][0]['snapshot']['source']['record_locator'] = 'line:1'
        cases.append((changed, 'source identity'))
        changed = deepcopy(self.fixture)
        changed['registration']['mode'] = 'RUNTIME_ESTIMATED'
        cases.append((changed, 'explicit offline profile'))
        changed = deepcopy(self.fixture)
        changed['rows'][0]['replay_now_s'] += 60
        cases.append((changed, 'stale|future'))
        for fixture, error in cases:
            with self.subTest(error=error), TemporaryDirectory() as tmp:
                root = Path(tmp)
                self.write_fixture(root, fixture)
                with self.assertRaisesRegex(Blocked, error):
                    load_scenario_set(root)

    def test_same_estimator_source_family_cannot_cross_splits(self):
        manifest = deepcopy(self.manifest)
        manifest['sources'].append(manifest['sources'][0] | {'split': 'held-out'})
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_fixture(root, self.fixture)
            path = root / DEFAULT_MANIFEST
            # Keep the actual fixture digest, then alter only the family split.
            manifest['sources'][0]['fixture_sha256'] = sha256(
                (root / ESTIMATED_FIXTURE).read_bytes()).hexdigest()
            manifest['sources'][1]['fixture_sha256'] = manifest['sources'][0]['fixture_sha256']
            path.write_text(json.dumps(manifest), encoding='utf-8')
            with self.assertRaisesRegex(Blocked, 'split leakage'):
                load_scenario_set(root)

    def test_original_verification_checks_retained_source_evidence_too(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_fixture(root, self.fixture)
            for field in ('source_path', 'summary_path'):
                relative = self.fixture['source'][field]
                target = root / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / relative, target)
            evidence = self.fixture['evidence']['files'][0]
            target = root / evidence['path']
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((ROOT / evidence['path']).read_bytes() + b'\nchanged\n')
            # Raw navigation and summary hashes still match. The retained source
            # bytes must independently match before source-backed replay passes.
            with self.assertRaisesRegex(Blocked, 'SHA-256 mismatch'):
                load_scenario_set(root, verify_originals=True)
