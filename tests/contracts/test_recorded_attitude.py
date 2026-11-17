"""Positive State conformance from an actual on-board recording."""

import json
import os
import subprocess
import sys
from tempfile import TemporaryDirectory
from pathlib import Path
import unittest

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation

from rm27_exchange.evidence import (
    EvidenceClaim, EvidenceContext, EvidenceRequirement, EvidenceSpan, assess_coverage,
)
from rm27_exchange.offline_contracts import Blocked
from rm27_exchange.recorded_attitude import (
    CLOCK, FRAME, RECORDING, RUN_ID, SOURCE, _adapt_attitude, replay_attitude,
)
from test_schema_validation import ROOT, validator_for
from test_semantics import state_errors


class RecordedAttitudeTests(unittest.TestCase):
    def test_ulog_cross_process_state_round_trip(self):
        result = subprocess.run(
            [sys.executable, '-m', 'rm27_exchange.recorded_attitude', str(RECORDING),
             '--receiver-boot-us', '79160000', '--max-age-us', '60000'],
            cwd=ROOT, env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'),
            capture_output=True, text=True, check=True, timeout=20,
        )
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([r['metadata']['sequence'] for r in rows], list(range(3500, 3509)))
        source = ULog(str(ROOT / RECORDING), ['vehicle_attitude']).get_dataset('vehicle_attitude').data
        validator = validator_for('state.schema.json#/$defs/EstimatedState')
        truth = validator_for('state.schema.json#/$defs/GroundTruthState')
        for record in rows:
            validator.validate(record)
            self.assertFalse(list(state_errors(record)))
            self.assertFalse(truth.is_valid(record))
            metadata = record['metadata']
            self.assertEqual((metadata['run_id'], metadata['clock_id'], metadata['frame_id']),
                             (RUN_ID, CLOCK, FRAME))
            self.assertEqual(metadata['reset_epoch'], 0)
            self.assertEqual(metadata['input_kind'], 'ESTIMATED')
            self.assertEqual([link['input_kind'] for link in metadata['lineage']], ['RECORDED'])
            self.assertEqual(record['frames'], {'world': 'NED', 'body': 'FRD', 'quaternion': 'HAMILTON_WXYZ'})
            self.assertEqual(record['measurement_times_s']['q_WB_wxyz']['clock_id'], CLOCK)
            self.assertEqual(record['published_clock_id'], CLOCK)
            self.assertLess(record['state_time_s'], record['published_time_s'])
            index = metadata['sequence']
            self.assertEqual(round(record['state_time_s'] * 1_000_000), source['timestamp_sample'][index])
            self.assertEqual(round(record['published_time_s'] * 1_000_000), source['timestamp'][index])
            np.testing.assert_allclose(record['q_WB_wxyz'], [source[f'q[{i}]'][index] for i in range(4)],
                                       atol=1e-7, rtol=0)
            self.assertTrue(record['validity']['q_WB_wxyz']['current'])
            self.assertTrue(record['validity']['q_WB_wxyz']['finite'])
            self.assertNotIn('received_time_s', record)
            self.assertNotIn('model', metadata)
            for missing in ('position_W_m', 'velocity_W_m_s', 'body_rates_B_rad_s', 'uncertainty', 'actuator_state'):
                self.assertNotIn(missing, record)
                self.assertEqual(metadata['availability'][missing]['status'], 'UNKNOWN')

        first = rows[0]
        self.assertEqual(first['state_time_s'], 79.111221)
        self.assertEqual(first['published_time_s'], 79.111482)
        self.assertEqual(first['data_age_s']['q_WB_wxyz'], .048779)
        native_q = [0.8433356285095215, -0.009058709256350994, 0.008734840899705887, -0.5372399091720581]
        w, x, y, z = first['q_WB_wxyz']
        np.testing.assert_allclose([w, x, y, z], native_q, atol=1e-7, rtol=0)
        np.testing.assert_allclose(Rotation.from_quat([x, y, z, w]).apply([1, 0, 0]),
                                   [.4225939997, -.9063053113, -.0049994045], atol=1e-8)

        # Valid point observations still cannot certify a continuous window.
        context = EvidenceContext(RUN_ID, 'recorded-attitude-coverage-test', CLOCK, 0)
        spans = tuple(EvidenceSpan(context, SOURCE, r['metadata']['record_id'],
                                   r['state_time_s'], r['state_time_s']) for r in rows)
        requirements = (EvidenceRequirement('continuous', SOURCE, spans[0].start_s, spans[-1].end_s),)
        self.assertEqual(assess_coverage(context, requirements, {'continuous': EvidenceClaim('PASS', spans)})[0],
                         'BLOCKED')

    def test_actual_log_cannot_be_replayed_before_publication_or_after_age_limit(self):
        for now, age, message in ((79_111_400, 60_000, 'future'), (79_160_000, 1_000, 'stale')):
            with self.subTest(now=now), self.assertRaisesRegex(Blocked, message):
                replay_attitude(ROOT / RECORDING, receiver_boot_us=now, max_age_us=age)

    def test_wrong_log_and_bad_attitude_are_rejected(self):
        with TemporaryDirectory() as tmp:
            altered = Path(tmp) / '1113_z.ulg'
            altered.write_bytes(b'unregistered bytes')
            with self.assertRaisesRegex(Blocked, 'SHA-256'):
                replay_attitude(altered, receiver_boot_us=79_160_000, max_age_us=60_000)

        row = {'timestamp_sample': 79_111_221, 'timestamp': 79_111_482, 'quat_reset_counter': 0,
               'q[0]': 1., 'q[1]': 0., 'q[2]': 0., 'q[3]': 0.}
        for overrides, message in (({'quat_reset_counter': 1}, 'epoch'), ({'q[0]': .1}, 'quaternion')):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(Blocked, message):
                _adapt_attitude(row | overrides, sequence=3500, receiver_boot_us=79_160_000, max_age_us=60_000)
