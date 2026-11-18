"""Recorded sensor-estimator output to the existing G0 Snapshot.

This adapter never runs a plant/estimator or substitutes the recording's sibling
truth for an estimate. Registrations are receiver-owned offline configuration.
The source's integer-nanosecond convention is explicit; raw floats and validity
flags remain available unchanged in the versioned evidence extension.
"""
from copy import deepcopy
import gzip
import math
from pathlib import Path

from .handover import HandoverRegistry, build_snapshot, digest
from .offline_contracts import (
    Blocked, NativeContext, _state_record, verify_local_asm2, verify_sha256,
)
from .validation import loads_json, read_json

EXTENSION = 'rm27.g0.recorded_navigation'
FIXTURE = 'tests/contracts/fixtures/handover_navigation_estimated.json'
_CHANNELS = {
    'position': ('position_W_m', 'position_m', 'local_navigation'),
    'velocity': ('velocity_W_m_s', 'velocity_m_s', 'local_navigation'),
    'attitude': ('q_WB_wxyz', 'rotation_body_to_local', 'body_FLU_to_local_navigation'),
    'body_rate': ('body_rates_B_rad_s', 'angular_velocity_body_rad_s', 'body_FLU'),
}

# This rebuild profile describes one existing recording, not arbitrary files
# selected by a caller. Bind the portable copies to their original source roles.
_EVIDENCE_ROLES = {
    'alignment': ('alignment.py', 'components/rm27_non_rl/rm27_non_rl/asm2_flight_alignment.py'),
    'config': ('config.json', 'components/rm27_non_rl/artifacts/asm2_handover/dev04_enclosure/config.json'),
    'estimator': ('estimator.py', 'components/rm27_non_rl/rm27_non_rl/asm2_sensor_estimator.py'),
    'navigation': ('navigation.py', 'components/rm27_non_rl/.agents/runs/asm2_handover/moving_handover.py'),
    'plant': ('plant.py', 'components/rm27_sim/rm27_sim/simulation/asm2.py'),
    'sensor_generation': ('recording.py', 'components/rm27_sim/rm27_sim/experiments/asm2_sensor_recording.py'),
    'scene': ('scene_provenance.json', 'components/rm27_non_rl/artifacts/asm2_handover/moving_handover_02/scene_provenance.json'),
    'sensors': ('sensors.jsonl.gz', 'components/rm27_non_rl/artifacts/asm2_handover/moving_handover_02/sensors.jsonl.gz'),
    'summary': ('summary.json', 'components/rm27_non_rl/artifacts/asm2_handover/moving_handover_02/summary.json'),
    'trace': ('trace.jsonl.gz', 'components/rm27_non_rl/artifacts/asm2_handover/moving_handover_02/trace.jsonl.gz'),
}


def _bind_recorded_evidence(fixture, summary, path):
    """Check receiver configuration against verified recording provenance.

    This detects contradictory source registrations; it does not authenticate an
    untrusted receiver registry. Historical absolute paths are compared by their
    recorded project-relative suffix, never opened on the current machine.
    """
    files = fixture['evidence']['files']
    if not isinstance(files, list) or not files:
        raise Blocked('missing recorded source evidence files')
    by_original, archive_paths = {}, set()
    for item in files:
        original = item['original_path']
        if original in by_original or item['path'] in archive_paths:
            raise Blocked('duplicate recorded source evidence')
        by_original[original] = item
        archive_paths.add(item['path'])
        verify_sha256(path(item['path']), item['sha256'])
    roles = {}
    for role, (basename, original) in _EVIDENCE_ROLES.items():
        item = by_original.get(original)
        if item is None or Path(item['path']).name != basename:
            raise Blocked(f'missing/misidentified recorded {role} evidence')
        roles[role] = item

    def matches(actual, expected, role):
        if actual != expected:
            raise Blocked(f'recorded {role} identity mismatch')

    inputs = summary['inputs_sha256']
    for role in ('alignment', 'config', 'estimator', 'navigation', 'plant', 'sensor_generation'):
        original = roles[role]['original_path']
        recorded = [value for name, value in inputs.items()
                    if name == original or name.endswith('/' + original)]
        if len(recorded) != 1:
            raise Blocked(f'missing/ambiguous recorded {role} summary provenance')
        matches(roles[role]['sha256'], recorded[0], role)
    # The source scene was copied from dev04; its archived bytes must retain the
    # same provenance, despite the recording's different output directory.
    scene_original = 'components/rm27_non_rl/artifacts/asm2_handover/dev04_enclosure/scene_provenance.json'
    recorded_scene = [value for name, value in inputs.items()
                      if name == scene_original or name.endswith('/' + scene_original)]
    if len(recorded_scene) != 1:
        raise Blocked('missing/ambiguous recorded scene summary provenance')
    matches(roles['scene']['sha256'], recorded_scene[0], 'scene provenance')
    source = fixture['source']
    for role, source_key, hash_key in (
            ('trace', 'source_path', 'source_sha256'),
            ('summary', 'summary_path', 'summary_sha256')):
        matches(roles[role]['path'], source[source_key], role + ' path')
        matches(roles[role]['sha256'], source[hash_key], role + ' source')
    for role in ('trace', 'sensors'):
        matches(roles[role]['sha256'], summary['output_sha256'][Path(roles[role]['path']).name], role + ' output')
    profile = fixture['registration']['recorded_simulation']
    matches(profile['registration']['id'], 'g0-moving-handover-02-recorded-simulation', 'registration')
    matches(profile['registration']['sha256'], source['summary_sha256'], 'registration summary')
    matches(profile['source_run_id'], source['source_run_id'], 'run')
    matches(profile['source_artifact']['id'], source['source_path'], 'source artifact path')
    matches(profile['source_artifact']['sha256'], source['source_sha256'], 'source artifact')
    for role in ('estimator', 'sensor_generation'):
        matches(profile['roles'][role]['identity']['sha256'], roles[role]['sha256'], role + ' role')
    expected_input = {'role': 'sensor_generation', 'artifact': {
        'id': roles['sensors']['path'], 'sha256': roles['sensors']['sha256'],
        'version': profile['source_artifact']['version'],
    }}
    matches(profile['runtime_inputs'], [expected_input], 'runtime inputs')
    scene = read_json(path(roles['scene']['path']))
    matches(profile['roles']['truth']['identity']['sha256'], summary['scene_sha256'], 'truth scene')
    matches(scene['compiled_sha256'], summary['scene_sha256'], 'compiled scene')
    matches(fixture['registration']['identity']['config_hash'], roles['config']['sha256'], 'configuration')


def _ns_seconds(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise Blocked('missing/nonfinite native time')
    # Matches the recorded estimator time_ns(), NOT a tolerance on freshness.
    return round(value * 1_000_000_000) / 1_000_000_000


def snapshot_from_row(payload, source, registry, *, source_line, snapshot_id):
    """Lossless evidence plus converted estimate, with no invented capabilities.

    Raw command values exist, but original command lease/expiry and generation
    metadata were not recorded. They therefore stay diagnostic raw history,
    rather than fabricated AppliedActuatorCommand records. Hidden activation is
    explicitly truth and never fills EstimatedState actuator channels.
    """
    if registry.mode != 'RECORDED_SIM_ESTIMATED' or not registry.recorded_simulation:
        raise Blocked('recorded simulation registration required')
    evidence = registry.recorded_simulation
    estimate = payload.get('estimate', {})
    epoch = estimate.get('generation')
    if (type(epoch) is not int or epoch != registry.epoch or estimate.get('epoch_id') != epoch
            or estimate.get('source') != evidence['roles']['sensor_generation']['source']
            or estimate.get('alignment') != 'IMU_EXTERNAL_VELOCITY_FLIGHT_ALIGNMENT'
            or estimate.get('world_alignment_observed') is not True
            or estimate.get('alignment_evidence_id') != 'MEASURED_INERTIAL_VELOCITY_CONSISTENCY'
            or estimate.get('faults') != []):
        raise Blocked('recording lacks valid sensor-only initialization/epoch evidence')
    stamp = _ns_seconds(estimate.get('state_time_s'))
    if any(_ns_seconds(estimate.get(key)) != stamp for key in ('timestamp_s', 'time_s')) \
            or _ns_seconds(payload.get('time_s')) != stamp:
        raise Blocked('represented recording times disagree')
    frame, clock = registry.frame, registry.clock
    context = NativeContext(
        source['source_run_id'], snapshot_id, source_line, frame.source_id, clock.source_id,
        epoch, 'Recorded estimator generation/epoch_id and per-channel epoch_id; '
        'integer-nanosecond source clock, no inferred reboot mapping', 'ESTIMATED',
        source_record_id=f'{source["source_path"]}:line:{source_line}',
    )
    values, measurements = {}, {}
    raw_channels = estimate.get('channels', {})
    for name, (field, value_key, native_frame) in _CHANNELS.items():
        channel = raw_channels.get(name, {})
        if (channel.get('valid') is not True or channel.get('reasons') != []
                or channel.get('epoch_id') != epoch or channel.get('measurement_epoch_id') != epoch
                or channel.get('frame') != native_frame
                or channel.get('source') != evidence['roles']['sensor_generation']['source']):
            raise Blocked(f'invalid/missing recorded {name} provenance/validity')
        acquired = _ns_seconds(channel.get('acquisition_s'))
        delivered = _ns_seconds(channel.get('delivery_s'))
        represented = _ns_seconds(channel.get('represented_s'))
        if not 0 <= acquired <= delivered <= stamp or represented != stamp:
            raise Blocked(f'invalid recorded {name} times')
        if value_key not in estimate or estimate[value_key] is None:
            raise Blocked(f'missing recorded {name} value')
        values[field] = estimate[value_key]
        measurements[field] = (acquired, registry.max_age_s)
    now = clock.map(stamp, source_id=clock.source_id, epoch=epoch)
    state = _state_record(
        context, source=evidence['roles']['estimator']['source'], lineage_kind='ESTIMATED',
        stamp=stamp, values=values, measurements=measurements, frame=frame, clock=clock,
        now_s=now, max_age_s=registry.max_age_s, identity=registry.identity,
    )
    for validity in state['validity'].values():
        # Observed sensor-only alignment and channel-valid flags above; this is
        # interface initialization, never calibrated heading confidence.
        validity['initialized'] = True
    roles = evidence['roles']
    state['metadata']['lineage'] = [
        {'input_kind': 'RECORDED', 'source': source['source_path'], 'record_id': f'line:{source_line}'},
        {'input_kind': 'GROUND_TRUTH', 'source': roles['truth']['source'],
         'transform': roles['sensor_generation']['identity']['id']},
        {'input_kind': 'SYNTHETIC', 'source': roles['sensor_generation']['source']},
        {'input_kind': 'ESTIMATED', 'source': roles['estimator']['source']},
        {'input_kind': 'ESTIMATED', 'source': roles['estimator']['source'],
         'transform': frame.registration_id},
        {'input_kind': 'ESTIMATED', 'source': roles['estimator']['source'],
         'transform': clock.registration_id},
    ]
    state['uncertainty'] = None
    state['metadata']['availability']['uncertainty'] = {
        'status': 'UNKNOWN', 'reason': estimate.get('covariance_reason', 'Not recorded'),
    }
    state['metadata']['extensions'] = {EXTENSION: {'version': '1', 'data': {
        'scope': 'RECORDED_SIMULATION_INTERFACE_ONLY',
        'original_estimate': deepcopy(estimate),
        'source_time_s': payload['time_s'],
        'time_representation': 'ROUND_SECONDS_TO_NEAREST_NS_FROM_RECORDED_ESTIMATOR',
        'raw_control_history': {'input_kind': 'RECORDED', 'records': [{
            key: deepcopy(payload[key]) for key in
            ('time_s', 'phase', 'blend_fraction', 'blended_ctbr_target',
             'applied_motor_command_N', 'limited') if key in payload
        }], 'ctbr_order': ['p_FLU_rad_s', 'q_FLU_rad_s', 'r_FLU_rad_s', 'T_N'],
            'timing_semantics': 'Same-tick issued command; activation sampled before command application. '
                                'Not a reconstructed last-applied command or authority lease.',
            'admission': 'DIAGNOSTIC_ONLY_MISSING_ORIGINAL_LEASE_AND_EXPIRY'},
        'diagnostic_actuator_truth': {
            'input_kind': 'GROUND_TRUTH',
            'motor_activation_N': deepcopy(payload.get('motor_activation_N')),
            'use': 'OFFLINE_EVIDENCE_ONLY_NOT_ESTIMATED_ACTUATOR_STATE',
        },
    }}}
    return build_snapshot(
        snapshot_id=snapshot_id, source={
            'run_id': source['source_run_id'], 'trajectory_family_id': source['source_family_id'],
            'input_kind': 'RECORDED', 'artifact': deepcopy(evidence['source_artifact']),
            'record_locator': f'line:{source_line}',
        }, state={'schema': 'state.schema.json#/$defs/EstimatedState', 'record': state},
        identity=registry.identity, registration={'frame': frame.registration_id,
                                                'clock': clock.registration_id},
        control_history={'availability': {'status': 'UNKNOWN', 'reason':
            'Raw commands retained in evidence extension; original lease/expiry unavailable'}, 'records': []},
    )


def rebuild_recorded_fixture(root, fixture_path=FIXTURE):
    """Rebuild all admitted snapshots from immutable raw bytes without simulation.

    Source evidence is required even when the portable projection is present.
    Historical source snapshots are read as bytes, never imported or executed.
    """
    root = Path(root).resolve()
    def path(relative):
        result = (root / relative).resolve()
        if Path(relative).is_absolute() or not result.is_relative_to(root):
            raise Blocked('recording path must stay inside root')
        return result
    fixture = read_json(path(fixture_path))
    source = fixture['source']
    for field, hash_field in (('source_path', 'source_sha256'), ('summary_path', 'summary_sha256')):
        verify_sha256(path(source[field]), source[hash_field])
    registry_dict = fixture['registration']
    summary = read_json(path(source['summary_path']))
    _bind_recorded_evidence(fixture, summary, path)
    actual = verify_local_asm2(root)
    if any(registry_dict['identity'].get(key) != value for key, value in actual.items()):
        raise Blocked('canonical aircraft identity mismatch')
    if (summary['checks']['no_contact_safety'] != 'PASS' or summary['first_contact_s'] is not None
            or summary['contact_planner_executed'] is not False
            or summary['truth_used_for_commands'] is not False
            or summary['output_sha256']['trace.jsonl.gz'] != source['source_sha256']):
        raise Blocked('not the registered noncontact sensor-only navigation recording')
    registry_dict['recorded_simulation']['snapshot_sha256'] = {}
    registry = HandoverRegistry.from_dict(registry_dict)
    wanted = {row['source_line']: row for row in fixture['rows']}
    with gzip.open(path(source['source_path']), 'rt') as stream:
        for line, text in enumerate(stream, 1):
            if line not in wanted:
                continue
            row = wanted.pop(line)
            row['payload'] = loads_json(text)
            row['snapshot'] = snapshot_from_row(row['payload'], source, registry,
                source_line=line, snapshot_id=row['scenario_id'])
            row['replay_now_s'] = row['snapshot']['state']['record']['state_time_s']
            registry_dict['recorded_simulation']['snapshot_sha256'][row['scenario_id']] = digest(row['snapshot'])
    if wanted:
        raise Blocked('navigation source row missing')
    return fixture


def main():
    """Rebuild and verify the existing fixture; no plant or estimator execution."""
    import argparse
    import json
    from .handover import admit_snapshot
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--fixture', default=FIXTURE)
    args = parser.parse_args()
    try:
        rebuilt = rebuild_recorded_fixture(args.root, args.fixture)
        if rebuilt != read_json(args.root / args.fixture):
            raise Blocked('fixture differs from raw-source reconstruction')
        registry = HandoverRegistry.from_dict(rebuilt['registration'])
        results = []
        for row in rebuilt['rows']:
            admit_snapshot(row['snapshot'], registry, now_s=row['replay_now_s'])
            results.append({'snapshot_id': row['scenario_id'], 'status': 'ADMITTED',
                            'sha256': digest(row['snapshot'])})
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(1, f'BLOCKED: {exc}\n')
    print(json.dumps({'profile': registry.mode, 'snapshots': results,
                      'dataset_readiness': 'BLOCKED',
                      'scope': 'RECORDED_SIMULATION_INTERFACE_ONLY'}, indent=2))


if __name__ == '__main__':
    main()
