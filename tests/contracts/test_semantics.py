"""Cross-field fixture assertions; these do not admit commands or score flight."""
from copy import deepcopy
from itertools import product
import math
import unittest

from test_schema_validation import load_fixtures, structural_errors


VERDICT_ORDER = ('FAIL', 'BLOCKED', 'NOT_RUN', 'PASS')
COMPLETION_TERMS = ('Q_contact', 'Q_clearance', 'Q_separation', 'Q_stable')
FRAME_CONTEXT_KEYS = ('frame_id', 'clock_id', 'reset_epoch')


def required_verdict(verdicts):
    return min(verdicts, key=VERDICT_ORDER.index)


def ordered_intervals(value):
    if isinstance(value, dict):
        if {'start_s', 'end_s'} <= value.keys():
            yield value['start_s'] <= value['end_s']
        for item in value.values():
            yield from ordered_intervals(item)
    elif isinstance(value, list):
        for item in value:
            yield from ordered_intervals(item)


def unit_vector(value):
    # Numerical comparison tolerance only; not a model or motion threshold.
    return math.isclose(math.hypot(*value), 1.0, rel_tol=1e-8, abs_tol=1e-8)


def ordered_rotors(record):
    rotors = record['rotors']
    orders = [rotor['order'] for rotor in rotors]
    ids = [rotor['id'] for rotor in rotors]
    return orders == sorted(set(orders)) and len(set(ids)) == len(ids)


def coverage_complete(assessment):
    return assessment.get('coverage', {}).get('complete') is True


def state_errors(record):
    q = record.get('q_WB_wxyz')
    if q is not None and not unit_vector(q):
        yield 'quaternion_norm'
    target = record.get('target_relative') or {}
    q = (target.get('target_pose_WT') or {}).get('q_WT_wxyz')
    if q is not None and not unit_vector(q):
        yield 'quaternion_norm'
    actuator = record.get('actuator_state') or {}
    rotor_ids = actuator.get('rotor_ids')
    if rotor_ids:
        for field in ('rotor_speed_rad_s', 'rotor_thrust_N'):
            vector = actuator.get(field)
            if vector is not None and len(vector) != len(rotor_ids):
                yield 'rotor_vector_length'
    if 'shape' in record and math.prod(record['shape']) != len(record['values']):
        yield 'observation_shape_mismatch'


def geometry_errors(record):
    for key, value in record.items():
        if key == 'rotation_parent_child_wxyz' and value is not None and not unit_vector(value):
            yield 'quaternion_norm'
        elif isinstance(value, dict):
            yield from geometry_errors(value)
        elif isinstance(value, list):
            for child in value:
                if isinstance(child, dict):
                    yield from geometry_errors(child)


def control_errors(record, context):
    timing = record.get('timing')
    lease = record.get('authority')
    if timing:
        apply_at = timing['apply_time_s']
        if not timing['valid_from_s'] <= apply_at < timing['expires_at_s']:
            yield 'command_expired'
        if context.get('now_s', apply_at) >= timing['expires_at_s']:
            yield 'command_expired'
        if lease and max(apply_at, context.get('now_s', apply_at)) >= lease['lease_expires_at_s']:
            yield 'lease_expired'
        if timing['generated_time_s'] > apply_at or timing['source_state_time_s'] > apply_at:
            yield 'command_time_order'
    active = context.get('active_authority')
    if active and lease:
        keys = ('vehicle_id', 'allowed_layer', 'owner_id', 'lease_id', 'generation')
        if any(lease[key] != active[key] for key in keys):
            yield 'control_authority_conflict'
    if 'handoff' in record:
        handoff = record['handoff']
        successor = handoff['successor']
        if (successor['generation'] <= handoff['predecessor']['generation']
                or (successor['handoff_state'] == 'ACTIVE' and not handoff['acknowledged_successor'])):
            yield 'invalid_handoff'
    rotor_ids = record.get('rotor_ids')
    if rotor_ids:
        for field in ('motor_command_requested', 'motor_command_applied'):
            if field in record and len(record[field]['values']) != len(rotor_ids):
                yield 'rotor_vector_length'
        if 'rotor_thrust_request_N' in record and len(record['rotor_thrust_request_N']) != len(rotor_ids):
            yield 'rotor_vector_length'


def evaluation_errors(record):
    terms = [record[key] for key in COMPLETION_TERMS]
    expected = required_verdict(term['verdict'] for term in terms)
    if record['Q_complete']['verdict'] != expected:
        yield 'evaluation_verdict_aggregation'
    if record['Q_complete']['verdict'] == 'PASS' and not (
        record['window_coverage']['complete']
        and all(coverage_complete(item) for item in terms + [record['Q_complete']])
    ):
        yield 'evaluation_partial_success'
    first = record['first_contact']
    if record['Q_contact']['verdict'] == 'PASS' and not first:
        yield 'evaluation_contact_missing'
    if first and record['Q_contact']['verdict'] == 'PASS' and any(
        forbidden['interval']['start_s'] < first['interval']['end_s']
        for forbidden in record['forbidden_events']
    ):
        yield 'forbidden_event_first_contact'
    clearance = record['clearance']
    window = clearance['window']
    if window and record['Q_clearance']['verdict'] != 'FAIL' and any(
        forbidden['interval']['start_s'] < window['end_s']
        and forbidden['interval']['end_s'] > window['start_s']
        for forbidden in record['forbidden_events']
    ):
        yield 'forbidden_event_clearance'
    if record['Q_clearance']['verdict'] == 'PASS' and clearance['method'] == 'SAMPLED_ONLY':
        yield 'sampled_clearance_not_continuous'
    for result, key in (('clearance', 'Q_clearance'), ('separation', 'Q_separation'), ('stability', 'Q_stable')):
        if record[result]['assessment']['verdict'] != record[key]['verdict']:
            yield 'evaluation_component_verdict_mismatch'


def feasibility_errors(record):
    stage_evidence = record['evidence']
    required = set(record['required_stages'])
    expected = required_verdict(stage['verdict'] for stage in stage_evidence if stage['stage'] in required)
    recovery = record['recovery_assessed']
    outcome = recovery['predicted_recovery_outcome']
    if 4 in required and outcome == 'FAIL':
        expected = 'FAIL'
    if record['validation_status'] != expected:
        yield 'feasibility_verdict_aggregation'
    coverage = record['validation_level']['coverage']
    if record['validation_status'] == 'PASS':
        if not coverage['complete'] or any(
            not stage['coverage']['complete'] for stage in stage_evidence if stage['stage'] in required
        ):
            yield 'feasibility_partial_success'
        if record['validation_level']['scope'] == 'FULL_PHYSICAL' and (
            required != {1, 2, 3, 4}
            or record['terminal_state'] is None
            or record['contact_admissible']['assessment']['verdict'] != 'PASS'
            or record['clearance_valid']['assessment']['verdict'] != 'PASS'
            or recovery['assessment']['verdict'] != 'PASS'
            or recovery['stability_status'] != 'PASS'
        ):
            yield 'feasibility_partial_success'
    completed = [stage['stage'] for stage in stage_evidence if stage['verdict'] in ('PASS', 'FAIL')]
    if record['validation_level']['highest_completed_stage'] != max(completed, default=None):
        yield 'feasibility_stage_level_mismatch'
    native_stage = stage_evidence[3]
    feedback = native_stage['feedback_source']['kind']
    trigger = native_stage['contact_event_source']['kind']
    if recovery['execution_mode'] == 'SENSOR_DRIVEN' and (
        feedback not in ('SENSOR', 'ESTIMATED') or trigger != 'DETECTOR'
    ):
        yield 'feasibility_oracle_not_sensor'
    hold, horizon = recovery['required_stability_hold_s'], recovery['horizon_s']
    if recovery['stability_status'] == 'PASS' and (hold is None or horizon is None or horizon < hold):
        yield 'feasibility_recovery_hold'
    if (record['geometry_feasible']['method'] == 'SAMPLED_ONLY'
            and record['clearance_valid']['assessment']['verdict'] == 'PASS'
            and record['validation_level']['scope'] == 'FULL_PHYSICAL'):
        yield 'sampled_clearance_not_continuous'


def semantic_errors(fixture):
    """Check only fixture claims and their declared comparison context."""
    record = fixture['record']
    context = fixture.get('context', {})
    if not all(ordered_intervals(record)):
        yield 'interval_order'
    metadata = record.get('metadata') or {}
    if metadata.get('input_kind') == 'REAL_MEASURED' and any(
        item['input_kind'] == 'PREDICTED' for item in metadata.get('lineage', ())
    ):
        yield 'prediction_not_measured'
    for key in FRAME_CONTEXT_KEYS:
        if key in context and context[key] != metadata.get(key):
            yield {'frame_id': 'frame_identity_mismatch', 'clock_id': 'clock_identity_mismatch',
                   'reset_epoch': 'reset_epoch_mismatch'}[key]
    for key in ('model', 'geometry'):
        if key in context and context[key] != metadata.get(key):
            yield f'{key}_identity_mismatch'
    rotor_ids = record.get('rotor_ids', (record.get('actuator_state') or {}).get('rotor_ids'))
    if 'rotor_order' in context and context['rotor_order'] != rotor_ids:
        yield 'rotor_order_mismatch'
    schema = fixture['schema'].split('.')[0]
    if schema == 'state':
        yield from state_errors(record)
    elif schema == 'dynamics' and 'rotors' in record:
        if not ordered_rotors(record):
            yield 'rotor_order_mismatch'
    elif schema == 'geometry':
        yield from geometry_errors(record)
    elif schema == 'control':
        yield from control_errors(record, context)
    elif schema == 'evaluation' and 'Q_complete' in record:
        yield from evaluation_errors(record)
    elif schema == 'feasibility':
        if 'validation_status' in record:
            yield from feasibility_errors(record)
        elif 'initial_full_state' in record:
            state_meta = record['initial_full_state']['state']['metadata']
            if state_meta.get('model') != record['versions']['physics_model']:
                yield 'model_identity_mismatch'
            if state_meta.get('geometry') != record['versions']['geometry']:
                yield 'geometry_identity_mismatch'
            for key in FRAME_CONTEXT_KEYS:
                if state_meta.get(key) != metadata.get(key):
                    yield {'frame_id': 'frame_identity_mismatch', 'clock_id': 'clock_identity_mismatch',
                           'reset_epoch': 'reset_epoch_mismatch'}[key]


class SemanticTests(unittest.TestCase):
    def test_valid_fixtures_preserve_declared_semantics(self):
        for path, fixture in load_fixtures('valid'):
            with self.subTest(fixture=path.name):
                self.assertFalse(structural_errors(fixture))
                self.assertEqual(set(semantic_errors(fixture)), set())

    def test_semantic_negatives_are_well_formed_and_fail_for_the_declared_reason(self):
        count = 0
        for path, fixture in load_fixtures('invalid'):
            expected = fixture['expect'].get('semantic')
            if expected is None:
                continue
            count += 1
            with self.subTest(fixture=path.name):
                self.assertFalse(structural_errors(fixture))
                self.assertIn(expected, set(semantic_errors(fixture)))
        self.assertGreater(count, 0)

    def test_four_term_precedence_exhaustively(self):
        for terms in product(VERDICT_ORDER, repeat=4):
            expected = ('FAIL' if 'FAIL' in terms else 'BLOCKED' if 'BLOCKED' in terms else
                        'NOT_RUN' if 'NOT_RUN' in terms else 'PASS')
            with self.subTest(terms=terms):
                self.assertEqual(required_verdict(terms), expected)

    def test_prediction_ancestry_cannot_be_relabelled_measured(self):
        fixture = next(fixture for path, fixture in load_fixtures('valid') if path.name == 'control-ctbr.json')
        changed = deepcopy(fixture)
        changed['record']['metadata']['input_kind'] = 'REAL_MEASURED'
        changed['record']['metadata']['lineage'] = [{'input_kind': 'PREDICTED', 'source': 'synthetic-forecast'}]
        self.assertFalse(structural_errors(changed))
        self.assertEqual(set(semantic_errors(changed)), {'prediction_not_measured'})

    def test_command_context_cannot_silently_change_clock_or_epoch(self):
        fixture = next(fixture for path, fixture in load_fixtures('valid') if path.name == 'control-ctbr.json')
        changed = {**fixture, 'context': {'clock_id': 'synthetic-unregistered-clock', 'reset_epoch': 99}}
        self.assertEqual(set(semantic_errors(changed)), {'clock_identity_mismatch', 'reset_epoch_mismatch'})
