"""Hash-verified replay of the recorded Navigation → Handover evidence set.

This catalog deliberately returns native records, not admissible Shared State.
Missing registration and actuator evidence stay missing. Readable recordings do
not establish a handover distribution, state validity or physical feasibility.
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import sys

from rm27_exchange.offline_contracts import Blocked, verify_sha256
from rm27_exchange.validation import loads_json, read_json, require_json_value


DEFAULT_MANIFEST = 'configs/handover_scenarios_v1.json'
_SOURCE_FIELDS = ('source_run_id', 'source_family_id', 'source_path', 'source_sha256')
_REGISTRATION_FIELDS = ('world_to_contract', 'body_origin_to_com', 'clock_mapping',
                        'estimator_reset_epoch', 'actuator_profile')
_PAYLOAD_FIELDS = {'simulation_s', 'phase', 'telemetry', 'physical_state', 'controls'}
_TELEMETRY_FIELDS = {'local_position', 'local_velocity', 'local_time', 'quaternion',
                     'rates', 'attitude_time', 'estimator_flags', 'source_timing'}


def _path(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise Blocked('scenario paths must be root-relative')
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise Blocked('scenario path escapes the supplied root')
    return path


def _read(path):
    try:
        record = read_json(path)
        require_json_value(record)
    except (OSError, ValueError) as exc:
        raise Blocked(f'invalid or unavailable scenario JSON: {path}') from exc
    if not isinstance(record, dict):
        raise Blocked('scenario document must be an object')
    return record


def _identifier(value):
    if not isinstance(value, str) or not value.strip():
        raise Blocked('missing scenario identity')
    return value


def _digest(value):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9a-f]{64}', value):
        raise Blocked('invalid scenario SHA-256 identity')


def _is_projection(projected, native):
    if isinstance(projected, dict):
        return (isinstance(native, dict) and all(
            key in native and _is_projection(value, native[key])
            for key, value in projected.items()))
    return type(projected) is type(native) and projected == native


def _verify_original(root, fixture):
    source = fixture['source']
    trace_path = _path(root, source['source_path'])
    verify_sha256(trace_path, source['source_sha256'])
    verify_sha256(_path(root, source.get('summary_path')), source.get('summary_sha256'))
    wanted = {row['source_line']: row['payload'] for row in fixture['rows']}
    seen = set()
    with trace_path.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if line_number in wanted:
                try:
                    native = loads_json(line, label=f'{trace_path}:{line_number}')
                except ValueError as exc:
                    raise Blocked('invalid original navigation row') from exc
                if not _is_projection(wanted[line_number], native):
                    raise Blocked(f'fixture differs from original row {line_number}')
                seen.add(line_number)
            if len(seen) == len(wanted):
                break
    if seen != wanted.keys():
        raise Blocked('original navigation source lines are missing')


def load_scenario_set(root, manifest_path=DEFAULT_MANIFEST, *, verify_originals=False):
    """Load v1 evidence with family isolation and optional raw-source checking.

    Returns ``manifest``, ``fixtures``, ``records``, ``dataset_readiness``,
    ``reasons`` and ``originals_verified``. ``records`` retain raw payloads,
    lineage, split, hashes and unresolved registration. No Shared State or model
    qualification is inferred from the native telemetry.
    """
    root = Path(root).resolve()
    manifest = _read(_path(root, manifest_path))
    if manifest.get('version') != '1':
        raise Blocked('unsupported scenario set version')
    if manifest.get('dataset_readiness') != 'BLOCKED':
        raise Blocked('v1 native evidence does not establish dataset readiness')
    reasons = manifest.get('reasons')
    if not isinstance(reasons, list) or not reasons or any(
            not isinstance(reason, str) or not reason.strip() for reason in reasons):
        raise Blocked('BLOCKED dataset must retain its missing-evidence reasons')
    sources = manifest.get('sources')
    if not isinstance(sources, list) or not sources:
        raise Blocked('scenario sources are missing')

    families, traces, ids, source_lines = {}, {}, set(), set()
    fixtures, records = [], []
    for source in sources:
        if not isinstance(source, dict) or source.get('input_kind') != 'RECORDED':
            raise Blocked('native scenario source must remain RECORDED')
        for field in _SOURCE_FIELDS:
            _identifier(source.get(field))
        _path(root, source['source_path'])
        _digest(source['source_sha256'])
        split = source.get('split')
        if split not in ('train', 'dev', 'held-out'):
            raise Blocked('invalid source-family split')
        family = source['source_family_id']
        if family in families and families[family] != split:
            raise Blocked('source-family split leakage')
        families[family] = split
        for trace in ((field, source[field]) for field in
                      ('source_run_id', 'source_path', 'source_sha256')):
            if trace in traces and traces[trace] != family:
                raise Blocked('one source trajectory cannot use multiple family identities')
            traces[trace] = family
        lineage = source.get('lineage')
        if not isinstance(lineage, list) or not lineage or any(
                not isinstance(link, dict) or link.get('input_kind') not in
                ('GROUND_TRUTH', 'ESTIMATED', 'RECORDED', 'SYNTHETIC', 'PREDICTED')
                or not isinstance(link.get('source'), str) or not link['source'].strip()
                for link in lineage):
            raise Blocked('recorded scenario source lineage is missing or invalid')
        fixture_path = _path(root, source.get('fixture_path'))
        verify_sha256(fixture_path, source.get('fixture_sha256'))
        fixture = _read(fixture_path)
        if fixture.get('version') != '1' or fixture.get('purpose') != \
                'RECORDED_NAVIGATION_PROJECTION_NOT_ADMISSIBLE_RUNTIME_STATE':
            raise Blocked('native fixture purpose or version mismatch')
        identity = fixture.get('source')
        if not isinstance(identity, dict) or any(
                source[field] != identity.get(field) for field in _SOURCE_FIELDS):
            raise Blocked('manifest/fixture source identity mismatch')
        _path(root, identity.get('summary_path'))
        _digest(identity.get('summary_sha256'))
        registration = fixture.get('registration')
        if not isinstance(registration, dict) or any(
                field not in registration for field in _REGISTRATION_FIELDS):
            raise Blocked('fixture must explicitly retain missing registration evidence')
        rows = fixture.get('rows')
        if not isinstance(rows, list) or not rows:
            raise Blocked('native scenario rows are missing')
        last_line, last_time = 0, -math.inf
        for row in rows:
            if not isinstance(row, dict) or row.get('input_kind') != 'RECORDED':
                raise Blocked('native scenario row must remain RECORDED')
            scenario_id = _identifier(row.get('scenario_id'))
            line = row.get('source_line')
            if type(line) is not int or line <= last_line:
                raise Blocked('source lines must be positive and strictly ordered')
            if scenario_id in ids or (source['source_sha256'], line) in source_lines:
                raise Blocked('duplicate scenario or source row')
            ids.add(scenario_id)
            source_lines.add((source['source_sha256'], line))
            payload = row.get('payload')
            if (not isinstance(payload, dict) or set(payload) != _PAYLOAD_FIELDS
                    or not isinstance(payload.get('telemetry'), dict)
                    or set(payload['telemetry']) != _TELEMETRY_FIELDS):
                raise Blocked('incomplete native navigation projection')
            time = payload['simulation_s']
            if type(time) not in (int, float) or not math.isfinite(time) or time <= last_time:
                raise Blocked('simulation timestamps must be finite and strictly ordered')
            last_line, last_time = line, time
            records.append({
                'scenario_id': scenario_id, 'source_line': line,
                'input_kind': 'RECORDED', 'split': split,
                'source': deepcopy(identity), 'lineage': deepcopy(lineage),
                'registration': deepcopy(registration), 'payload': deepcopy(payload),
                'dataset_readiness': 'BLOCKED',
            })
        if verify_originals:
            _verify_original(root, fixture)
        fixtures.append(deepcopy(fixture))

    return {'manifest': deepcopy(manifest), 'fixtures': fixtures, 'records': records,
            'dataset_readiness': 'BLOCKED', 'reasons': deepcopy(reasons),
            'originals_verified': bool(verify_originals)}


def replay_records(root, manifest_path=DEFAULT_MANIFEST, *, verify_originals=False):
    """Yield independent native rows; no state reset, conversion or interpolation."""
    yield from load_scenario_set(root, manifest_path, verify_originals=verify_originals)['records']


def replay_snapshots(root, manifest_path=DEFAULT_MANIFEST, *, verify_originals=False):
    """Archive each unresolved native row in the common diagnostic Snapshot.

    ``state=None`` is deliberate: registrations are unavailable, so native
    telemetry and separate truth cannot be promoted to Shared EstimatedState.
    """
    from .handover import build_snapshot
    for record in replay_records(root, manifest_path, verify_originals=verify_originals):
        source = record['source']
        yield build_snapshot(snapshot_id=record['scenario_id'], source={
            'run_id': source['source_run_id'], 'trajectory_family_id': source['source_family_id'],
            'input_kind': 'RECORDED', 'artifact': {
                'id': source['source_path'], 'version': 'original-recording',
                'sha256': source['source_sha256'],
            }, 'record_locator': f"line:{record['source_line']}"}, native_record=record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--manifest', default=DEFAULT_MANIFEST)
    parser.add_argument('--verify-originals', action='store_true')
    parser.add_argument('--snapshots', action='store_true', help='emit common diagnostic Snapshots')
    args = parser.parse_args()
    try:
        dataset = load_scenario_set(args.root, args.manifest, verify_originals=args.verify_originals)
    except Blocked as exc:
        parser.exit(1, f'BLOCKED: {exc}\n')
    print('Dataset readiness BLOCKED: ' + '; '.join(dataset['reasons']), file=sys.stderr)
    records = (replay_snapshots(args.root, args.manifest) if args.snapshots else dataset['records'])
    for record in records:
        print(json.dumps(record, allow_nan=False))


if __name__ == '__main__':
    main()
