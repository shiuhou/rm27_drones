"""Production draft validation on both supported local jsonschema APIs."""
import unittest

from jsonschema.exceptions import ValidationError

from rm27_exchange.validation import (
    BUNDLE, loads_json, read_json, require_json_value, validate_record, validator_for,
)


class ProductionValidationTests(unittest.TestCase):
    def test_existing_positive_fixtures_validate_through_production_api(self):
        # Includes nested state/common/control refs and unevaluatedProperties;
        # this catches using a legacy resolver with modern jsonschema.
        for path in sorted((BUNDLE / 'examples' / 'valid').glob('*.json')):
            with self.subTest(fixture=path.name):
                fixture = read_json(path)
                validate_record(fixture['schema'], fixture['record'])

    def test_json_rejects_duplicate_members_and_nonfinite_numbers(self):
        for text in ('{"sequence":1,"sequence":2}', '{"nested":{"a":0,"a":1}}',
                     '[NaN]', '[-Infinity]', '[1e999]'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                loads_json(text)
        self.assertEqual(loads_json('{"sequence":0,"time":1.25}'),
                         {'sequence': 0, 'time': 1.25})

    def test_in_memory_values_cannot_evade_json_restrictions(self):
        for value in (float('nan'), {'values': [float('inf')]}, {0: 'coerced key'}, (1, 2)):
            with self.subTest(value=repr(value)), self.assertRaises(ValueError):
                require_json_value(value)
        with self.assertRaises(ValidationError):
            validate_record('common.schema.json#/$defs/Identity', {})

    def test_external_or_unknown_schema_references_are_not_accepted(self):
        for reference in ('https://example.org/state.schema.json', 'file:///tmp/state.json',
                          '../state.schema.json', 'unknown.schema.json', None):
            with self.subTest(reference=reference), self.assertRaises(ValueError):
                validator_for(reference)


if __name__ == '__main__':
    unittest.main()
