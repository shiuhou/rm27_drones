"""Exercise the proposed JSON records without importing RM27 runtime code."""
import unittest
from urllib.parse import urljoin

from jsonschema import Draft202012Validator
from jsonschema.exceptions import RefResolutionError

from rm27_exchange.validation import (
    ROOT, BUNDLE, BASE, DIALECT, NAMES, LocalResolver, read_json, schemas, validator_for,
)


def load_fixtures(kind):
    return [(path, read_json(path))
            for path in sorted((BUNDLE / 'examples' / kind).glob('*.json'))]


def schema_references(value):
    if isinstance(value, dict):
        if '$ref' in value:
            yield value['$ref']
        for child in value.values():
            yield from schema_references(child)
    elif isinstance(value, list):
        for child in value:
            yield from schema_references(child)


def validation_errors(errors):
    for error in errors:
        yield error
        yield from validation_errors(error.context)


def structural_errors(fixture):
    return list(validator_for(fixture['schema']).iter_errors(fixture['record']))


def fixture_label(errors):
    return '; '.join(f'{error.json_path}: {error.message}' for error in errors)


class SchemaTests(unittest.TestCase):
    def test_all_schemas_compile_and_point_to_frozen_drafts(self):
        self.assertEqual(set(schemas()), {BASE + f'{name}.schema.json' for name in NAMES})
        for name in NAMES:
            uri = BASE + f'{name}.schema.json'
            document = 'README.md' if name == 'common' else f'{name.upper()}.md'
            with self.subTest(schema=name):
                schema = schemas()[uri]
                Draft202012Validator.check_schema(schema)
                self.assertEqual(schema['$schema'], DIALECT)
                self.assertEqual(schema['$id'], uri)
                self.assertEqual(schema['x-rm27'], {
                    'version': '0.1.0-draft', 'status': 'PROPOSED',
                    'document': f'.agents/docs/contracts/{document}',
                })
                baseline = (ROOT / schema['x-rm27']['document']).read_text(encoding='utf-8')
                self.assertIn('0.1.0', baseline)
                self.assertIn('PROPOSED', baseline)
                self.assertIn('Phase 1.1' if name == 'common' else 'PHASE 1.1 REVIEWED DRAFT', baseline)

    def test_all_references_resolve_inside_bundle(self):
        for uri, schema in schemas().items():
            for ref in schema_references(schema):
                with self.subTest(schema=uri, ref=ref):
                    name, _, fragment = urljoin(uri, ref).partition('#')
                    target = schemas()[name]
                    if fragment:
                        self.assertTrue(fragment.startswith('/'))
                        for part in fragment[1:].split('/'):
                            target = target[part.replace('~1', '/').replace('~0', '~')]
                    self.assertIsInstance(target, (dict, bool))

    def test_unregistered_references_are_rejected_without_retrieval(self):
        entry = {'$schema': DIALECT, '$ref': BASE + 'not-in-bundle.schema.json'}
        validator = Draft202012Validator(entry, resolver=LocalResolver.from_schema(entry, store=schemas()))
        with self.assertRaisesRegex(RefResolutionError, 'Unregistered offline schema'):
            validator.validate({})

    def test_each_contract_has_positive_and_negative_examples(self):
        for kind in ('valid', 'invalid'):
            self.assertEqual({fixture['schema'].split('.')[0] for _, fixture in load_fixtures(kind)},
                             set(NAMES) - {'common'})

    def test_valid_and_semantic_invalid_examples_are_structurally_valid(self):
        for kind in ('valid', 'invalid'):
            for path, fixture in load_fixtures(kind):
                with self.subTest(fixture=path.name):
                    self.assertIs(fixture['synthetic'], True)
                    self.assertIsInstance(fixture['record'], dict)
                    if kind == 'valid':
                        self.assertNotIn('expect', fixture)
                    else:
                        self.assertIn('expect', fixture)
                        if 'semantic' not in fixture['expect']:
                            continue
                    errors = structural_errors(fixture)
                    self.assertFalse(errors, fixture_label(errors))

    def test_structural_invalid_examples_fail_for_the_declared_reason(self):
        count = 0
        for path, fixture in load_fixtures('invalid'):
            expected = fixture['expect']
            if 'semantic' in expected:
                continue
            count += 1
            with self.subTest(fixture=path.name):
                errors = list(validation_errors(structural_errors(fixture)))
                self.assertTrue(any(error.validator == expected['keyword']
                                    and list(error.absolute_path) == expected['path']
                                    for error in errors), fixture_label(errors))
        self.assertGreater(count, 0)

    def test_optional_metadata_and_versioned_extension_are_not_core_fields(self):
        validate = validator_for('common.schema.json#/$defs/RecordMetadata')
        metadata = dict(contract_id='rm27.state', contract_version='0.1.0-draft',
                        run_id='synthetic-run', record_id='synthetic-record', source='fixture',
                        sequence=0, input_kind='SYNTHETIC', lineage=[], clock_id=None,
                        frame_id=None, reset_epoch=None, availability={})
        validate.validate(metadata)
        self.assertFalse(validate.is_valid({**metadata, 'undeclared': True}))
        self.assertFalse(validate.is_valid({**metadata, 'extensions': {'synthetic.test': {'data': {}}}}))
        validate.validate({**metadata, 'extensions': {'synthetic.test': {'version': '1', 'data': {}}}})
        for key in ('contract_version', 'input_kind', 'reset_epoch'):
            with self.subTest(required=key):
                self.assertFalse(validate.is_valid({k: v for k, v in metadata.items() if k != key}))

    def test_boundary_frames_and_motor_input_units_do_not_convert_implicitly(self):
        frames = validator_for('common.schema.json#/$defs/FrameConvention')
        canonical = {'world': 'NED', 'body': 'FRD', 'quaternion': 'HAMILTON_WXYZ'}
        frames.validate(canonical)
        self.assertFalse(frames.is_valid({**canonical, 'world': 'NWU', 'body': 'FLU'}))
        self.assertFalse(frames.is_valid({**canonical, 'quaternion': 'XYZW'}))
        motor = validator_for('control.schema.json#/$defs/MotorInput')
        mapping = {'id': 'synthetic-map', 'version': 'fixture', 'sha256': 'a' * 64}
        force = {'kind': 'FORCE_TARGET_N', 'units': 'N', 'values': [0.5], 'mapping': mapping}
        motor.validate(force)
        self.assertFalse(motor.is_valid({**force, 'kind': 'NORMALIZED_THROTTLE'}))
        motor.validate({**force, 'kind': 'NORMALIZED_THROTTLE', 'units': '1'})

    def test_unavailable_value_requires_reason_and_complete_coverage_has_no_gaps(self):
        availability = validator_for('common.schema.json#/$defs/Availability')
        availability.validate({'status': 'UNKNOWN', 'reason': 'not recorded in fixture'})
        self.assertFalse(availability.is_valid({'status': 'UNKNOWN', 'reason': None}))
        coverage = validator_for('common.schema.json#/$defs/Coverage')
        self.assertFalse(coverage.is_valid({'complete': True, 'missing_intervals': [
            {'start_s': 1, 'end_s': 2},
        ]}))
