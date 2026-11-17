"""Construct actual local consumer data types, without invoking policy or plant.

Admission stays in the Exchange interpreter. A constructor-only child uses each
consumer's existing environment, which need not contain the schema validator.
Its output alone is explicitly unadmitted and cannot establish conformance.
"""
from copy import deepcopy
from dataclasses import asdict
from hashlib import sha256
import importlib.metadata
import inspect
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
INTERPRETERS = {
    'rl': ROOT / 'components/rm27_RL/.venv/bin/python',
    'non_rl': Path('/home/shiuhou/venvs/mujoco/bin/python'),
}


def _type_identity(cls):
    path = Path(inspect.getfile(cls)).resolve()
    return {'type': f'{cls.__module__}.{cls.__qualname__}', 'path': str(path),
            'sha256': sha256(path.read_bytes()).hexdigest()}


def _json_values(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _json_values(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_values(child) for child in value]
    return value


def _construct_native(payload, consumer):
    """Unadmitted constructor diagnostic; callable only after caller admission.

    This child intentionally has no dependency on schemas or Exchange admission.
    It neither claims PASS nor grants control authority for its untrusted input.
    """
    import numpy as np
    if consumer == 'rl':
        from rm27_contact_physics.model import VehicleEstimate
        native = VehicleEstimate(
            **{key: np.asarray(value, dtype=float) for key, value in payload['values'].items()},
            timestamp=payload['stamp'], source='G0_OFFLINE_SYNTHETIC_RECONSTRUCTION')
        native_type = _type_identity(VehicleEstimate)
        native_values = _json_values(asdict(native))
        native_status = {
            'policy_observation': {'status': 'BLOCKED', 'reason':
                'No target geometry, previous CTBR or episode horizon is fabricated'},
        }
    elif consumer == 'non_rl':
        from rm27_non_rl.asm2_estimation_integrity import Source, StateSample
        samples = []
        for row in payload['samples']:
            sample = StateSample(**{**row, 'value': np.asarray(row['value'], dtype=float),
                                    'source': Source.RECONSTRUCTED})
            samples.append(_json_values(asdict(sample)))
        native_type = _type_identity(StateSample)
        native_values = samples
        native_status = {
            'integrity_trust': {'status': 'BLOCKED', 'reason':
                'Reconstructed synthetic samples have no sensor or propagation approvals'},
            'original_sensor_fault_flags': {'status': 'UNKNOWN', 'reason':
                'Native clipped/gap defaults describe this diagnostic projection only'},
            'delivery_provenance': 'OFFLINE_CONSUMER_RECEIPT_NOT_ORIGINAL_SENSOR_DELIVERY',
        }
    else:
        raise ValueError('unknown native consumer')
    versions = {}
    for name in ('numpy', 'scipy', 'mujoco', 'rm27-contact-physics', 'rm27-contact-rl'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
    return {
        'scope': 'UNADMITTED_NATIVE_CONSTRUCTOR_ONLY', 'consumer': consumer,
        'python': sys.executable, 'versions': versions,
        'native_type': native_type, 'native_values': native_values,
        'native_capability': native_status,
    }


def project_native(snapshot, registry, consumer, now_s):
    """Admit, invert reviewed registrations, then construct installed data types.

    The authoritative shared snapshot stays with the caller; its digest and
    original metadata bind this lossy native view to the unchanged evidence.
    Native worlds use the registered source world, native consumer bodies FLU.
    Runtime/predicted estimates remain blocked because these native projections
    cannot fully express their provenance. This is synthetic conformance only.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation
    from .handover import admit_snapshot, digest, encode
    from .offline_contracts import Blocked
    from .validation import loads_json

    if consumer not in INTERPRETERS:
        raise Blocked('unknown native consumer')
    state = admit_snapshot(snapshot, registry, now_s=now_s)
    if registry.mode != 'SYNTHETIC_TEST' or state['metadata']['input_kind'] != 'ESTIMATED':
        raise Blocked('native type conformance supports synthetic estimates only; '
                      'runtime/predicted provenance is not representable by these native views')
    clock, frame = registry.clock, registry.frame
    unmap = lambda value: (value - clock.offset_s) / clock.scale
    stamp = unmap(state['state_time_s'])
    world_rotation, shift, _ = frame.transforms(
        stamp, source_id=frame.source_id, clock_id=clock.source_id, epoch=registry.epoch)
    # FLU is the consumer convention even if the original producer used FRD.
    body_flu_frd = np.diag([1., -1., -1.])
    q = state['q_WB_wxyz']
    rotation_ned_frd = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    values = {
        'position': world_rotation.T @ (np.asarray(state['position_W_m']) - shift),
        'velocity': world_rotation.T @ np.asarray(state['velocity_W_m_s']),
        'rotation': world_rotation.T @ rotation_ned_frd @ body_flu_frd,
        'omega_body': body_flu_frd @ np.asarray(state['body_rates_B_rad_s']),
    }
    channels = (
        ('position', 'position', 'position_W_m', 'world'),
        ('velocity', 'velocity', 'velocity_W_m_s', 'world'),
        ('attitude', 'rotation', 'q_WB_wxyz', 'body_FLU_to_world'),
        ('body_rate', 'omega_body', 'body_rates_B_rad_s', 'body_FLU'),
    )
    samples = []
    for quantity, value_key, channel, native_frame in channels:
        acquisition = unmap(state['measurement_times_s'][channel]['time_s'])
        samples.append(dict(
            quantity=quantity, value=values[value_key], acquisition_s=acquisition,
            delivery_s=unmap(now_s), state_time_s=stamp, epoch_id=registry.epoch,
            frame=native_frame, max_age_s=registry.max_age_s / clock.scale,
            valid=True, propagated=acquisition != stamp,
        ))
    payload = _json_values({'values': values, 'stamp': stamp, 'samples': samples})
    python = INTERPRETERS[consumer]
    if not python.is_file():
        raise Blocked(f'{consumer} native interpreter unavailable: {python}')
    environment = {key: value for key, value in os.environ.items()
                   if key not in ('PYTHONPATH', 'PYTHONHOME')}
    environment.update(PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1')
    try:
        child = subprocess.run(
            [str(python), '-m', 'rm27_exchange.handover_consumers',
             '--construct-only', '--consumer', consumer], cwd=ROOT, env=environment,
            input=encode(payload) + '\n', text=True, capture_output=True, timeout=30, check=True,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        reason = getattr(exc, 'stderr', None) or str(exc)
        raise Blocked(f'{consumer} native constructor unavailable: {reason}') from exc
    lines = child.stdout.splitlines()
    if len(lines) != 1:
        raise Blocked('native constructor returned invalid JSONL framing')
    result = loads_json(lines[0])
    if (result.get('scope') != 'UNADMITTED_NATIVE_CONSTRUCTOR_ONLY'
            or result.get('consumer') != consumer or 'native_type' not in result):
        raise Blocked(f'native constructor did not construct expected type: {result}')
    result.update(
        status='PASS', scope='SYNTHETIC_OFFLINE_NATIVE_TYPE_CONSTRUCTION_ONLY',
        native_frame={'world_id': frame.source_id, 'body': 'FLU', 'origin': 'COM',
                      'clock_id': clock.source_id},
        snapshot_sha256=digest(snapshot), source_metadata=deepcopy(state['metadata']),
        unrepresented_fields={key: 'PRESERVED_IN_SHARED_SNAPSHOT' for key in
                              ('actuator_state', 'control_history', 'uncertainty', 'target_geometry')},
    )
    return result


def main():
    """JSONL entry; unadmitted constructor mode never returns conformance PASS."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--consumer', choices=('rl', 'non_rl'), required=True)
    parser.add_argument('--construct-only', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    for line in sys.stdin:
        try:
            if args.construct_only:
                result = _construct_native(json.loads(line), args.consumer)
            else:
                from .handover import HandoverRegistry
                from .validation import loads_json
                message = loads_json(line)
                result = project_native(message['snapshot'], HandoverRegistry.from_dict(message['registry']),
                                        args.consumer, message['now_s'])
        except (ValueError, KeyError, TypeError, ImportError) as exc:
            result = {'status': 'BLOCKED', 'reason': str(exc), 'consumer': args.consumer}
        print(json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
