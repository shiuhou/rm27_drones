"""Strict JSON and local-only validation against the frozen draft bundle.

Schema validation establishes structure only. Frame/clock registration,
freshness, provenance and consumer admission remain the adapter's responsibility.
No reference is retrieved from a network or filesystem outside this bundle.
"""
from functools import lru_cache
import json
import math
from pathlib import Path
import re

from jsonschema import Draft202012Validator, RefResolver

try:
    from referencing import Registry, Resource
except ImportError:  # jsonschema 4.10 in the documented system interpreter
    Registry = Resource = None


ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / 'contracts/v0.1.0'
BASE = 'https://rm27.example/contracts/v0.1.0/'
DIALECT = 'https://json-schema.org/draft/2020-12/schema'
NAMES = ('common', 'state', 'dynamics', 'geometry', 'control', 'evaluation', 'feasibility')


def loads_json(text, *, label='<JSON>'):
    """Decode JSON, rejecting duplicate members and non-finite numbers."""
    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'{label}: duplicate JSON member {key!r}')
            result[key] = value
        return result

    def finite_number(token):
        value = float(token)
        if not math.isfinite(value):
            raise ValueError(f'{label}: non-finite number {token}')
        return value

    return json.loads(text, object_pairs_hook=object_pairs,
                      parse_float=finite_number, parse_constant=finite_number)


def read_json(path):
    path = Path(path)
    return loads_json(path.read_text(encoding='utf-8'), label=str(path))


def require_json_value(value, *, path='$'):
    """Reject values that cannot be represented faithfully as finite JSON.

    In-memory adapter inputs need the same numeric restrictions as JSONL input;
    Python's encoder would otherwise silently coerce non-string object keys.
    """
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f'{path}: non-finite JSON number')
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            require_json_value(child, path=f'{path}[{index}]')
        return
    if isinstance(value, dict):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ValueError(f'{path}: JSON object member must be a string')
            require_json_value(child, path=f'{path}.{key}')
        return
    raise ValueError(f'{path}: unsupported JSON value {type(value).__name__}')


@lru_cache(maxsize=1)
def schemas():
    return {BASE + path.name: read_json(path)
            for path in sorted((BUNDLE / 'schemas').glob('*.schema.json'))}


class LocalResolver(RefResolver):
    # Keep the installed jsonschema 4.10 API used by Phase 2. Unknown references,
    # including file/HTTP URIs, must fail without attempting retrieval.
    def resolve_remote(self, uri):
        raise ValueError(f'Unregistered offline schema: {uri}')


def validator_for(reference):
    if not isinstance(reference, str):
        raise ValueError(f'Invalid bundle schema reference: {reference!r}')
    name = reference.split('#', 1)[0]
    if BASE + name not in schemas() or not re.fullmatch(
        r'[a-z]+\.schema\.json(?:#/\$defs/[A-Za-z0-9]+)?', reference
    ):
        raise ValueError(f'Invalid bundle schema reference: {reference}')
    entry = {'$schema': DIALECT, '$ref': BASE + reference}
    if Registry is not None:
        # Modern jsonschema resolves unevaluatedProperties through referencing,
        # independently of its deprecated RefResolver. Register complete source
        # documents so their internal fragments retain the correct base URI.
        # Registry's default retriever always refuses unknown resources.
        registry = Registry().with_resources(
            (uri, Resource.from_contents(schema)) for uri, schema in schemas().items()
        )
        return Draft202012Validator(entry, registry=registry)
    return Draft202012Validator(entry, resolver=LocalResolver.from_schema(entry, store=schemas()))


def validate_record(reference, record):
    """Validate an existing draft record, without implying runtime admission."""
    require_json_value(record)
    validator_for(reference).validate(record)
