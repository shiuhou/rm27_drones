"""Offline identity, ego-state and target-free CTBR contract records.

No simulator, controller, policy, allocator, transport or device is invoked.
Registrations are supplied by the caller from reviewed evidence; the adapter
checks applicability, not the physical correctness of a calibration.
"""

from copy import deepcopy
from dataclasses import dataclass
from hashlib import file_digest
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation


VERSION = '0.1.0-draft'
FRAMES = {'world': 'NED', 'body': 'FRD', 'quaternion': 'HAMILTON_WXYZ'}
FLU_TO_FRD = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
_IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
_ASSETS = (
    ('data/shared_aircraft/aircraft.json', 'c818545af9cc3c96af7491366464428c40f5b305cddd7696ed19d5e54b4d0621'),
    ('data/shared_aircraft/v1.0.0/models/asm2_isaac_proxy.xml', '2356bc3f8372fe14002521a82d4a9533867c688f8ed3d13cf172c74e17905ae4'),
    ('data/shared_aircraft/v1.0.0/models/plant.json', 'f4a5c52e8c06de893d3dc99b5544ba4408f03eed812bee52b595ee52db860b08'),
)


class Blocked(ValueError):
    """Missing, invalid or mismatched evidence prevents offline conversion."""


def verify_sha256(path, expected):
    """Verify file bytes against a separately supplied digest."""
    if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-f]{64}', expected):
        raise Blocked('invalid expected SHA-256')
    path = Path(path)
    try:
        with path.open('rb') as stream:
            actual = file_digest(stream, 'sha256').hexdigest()
    except OSError as exc:
        raise Blocked(f'artifact unavailable: {path}') from exc
    if actual != expected:
        raise Blocked(f'SHA-256 mismatch: {path}')
    return actual


def verify_local_asm2(root):
    """Verify D1's three local files; this does not compile or compare models."""
    catalog, model, plant = (verify_sha256(Path(root) / path, sha) for path, sha in _ASSETS)
    return {
        'catalog_sha256': catalog,
        'model': {'id': 'ASM2', 'version': '1.0.0', 'sha256': model},
        'geometry': {'id': 'ASM2-local-native-3522', 'version': '0.1.0', 'sha256': model},
        'plant_profile': {'id': 'ASM2-plant-metadata', 'version': '1.0.0', 'sha256': plant},
    }


def require_same_identity(expected, observed):
    """Require exact manifests, including model/profile and configuration hashes."""
    if not expected or expected != observed:
        raise Blocked('identity mismatch or missing registration')


def _state_identity(expected, observed):
    require_same_identity(expected, observed)
    for key in ('model', 'geometry', 'plant_profile'):
        if key not in observed:
            raise Blocked(f'missing {key} identity')


def _array(value, shape):
    try:
        result = np.asarray(value)
        if result.shape != shape or result.dtype.kind not in 'fiu':
            raise ValueError('wrong shape or type')
        result = result.astype(float)
        if not np.isfinite(result).all():
            raise ValueError('non-finite')
        return result
    except (TypeError, ValueError, OverflowError) as exc:
        raise Blocked('invalid numeric value') from exc


def _number(value):
    return float(_array(value, ()))


def _rotation(value):
    matrix = _array(value, (3, 3))
    if not (np.allclose(matrix @ matrix.T, _IDENTITY, rtol=0, atol=1e-8)
            and abs(np.linalg.det(matrix) - 1) <= 1e-8):
        raise Blocked('expected a proper rotation')
    return matrix


def _id(value):
    if not isinstance(value, str) or not value.strip():
        raise Blocked('missing identifier')
    return value


def _epoch(value):
    if type(value) is not int or value < 0:
        raise Blocked('unknown or invalid epoch/sequence')
    return value


def _interval_contains(interval, stamp):
    start, end = _array(interval, (2,))
    if not start <= stamp <= end:
        raise Blocked('timestamp outside registration validity')


def _fresh(stamp, now, max_age, uncertainty_s=0):
    age = _number(now) - _number(stamp)
    if not 0 <= uncertainty_s <= age or not age + uncertainty_s <= _number(max_age):
        raise Blocked('stale or future data')
    return age


def _body_rotation(body):
    if body == 'FLU':
        return np.asarray(FLU_TO_FRD)
    if body == 'FRD':
        return np.asarray(_IDENTITY)
    raise Blocked('unregistered body convention')


@dataclass(frozen=True)
class ClockRegistration:
    registration_id: str
    source_id: str
    destination_id: str
    epoch: int
    source_valid_s: tuple[float, float]
    scale: float
    offset_s: float
    uncertainty_s: float

    def map(self, stamp, *, source_id, epoch):
        _id(self.registration_id)
        _id(self.destination_id)
        if _id(source_id) != self.source_id or _epoch(epoch) != _epoch(self.epoch):
            raise Blocked('clock or epoch registration mismatch')
        stamp = _number(stamp)
        _interval_contains(self.source_valid_s, stamp)
        scale = _number(self.scale)
        if scale <= 0 or _number(self.uncertainty_s) < 0:
            raise Blocked('invalid clock scale or uncertainty')
        return _number(stamp * scale + _number(self.offset_s))


@dataclass(frozen=True)
class FrameRegistration:
    registration_id: str
    source_id: str
    destination_id: str
    source_clock_id: str
    epoch: int
    source_valid_s: tuple[float, float]
    rotation_ned_native: tuple
    translation_ned_m: tuple
    source_body: str
    body_origin: str

    def transforms(self, stamp, *, source_id, clock_id, epoch):
        _id(self.registration_id)
        _id(self.destination_id)
        if (_id(source_id) != self.source_id or _id(clock_id) != self.source_clock_id
                or _epoch(epoch) != _epoch(self.epoch) or self.body_origin != 'COM'):
            raise Blocked('frame, clock, epoch or body origin registration mismatch')
        _interval_contains(self.source_valid_s, stamp)
        return (_rotation(self.rotation_ned_native), _array(self.translation_ned_m, (3,)),
                _body_rotation(self.source_body))


@dataclass(frozen=True)
class NativeContext:
    run_id: str
    record_id: str
    sequence: int
    frame_id: str
    clock_id: str
    epoch: int
    epoch_provenance: str
    input_kind: str
    source_record_id: str | None = None
    synthetic_fixture_id: str | None = None


_UNKNOWN = 'No qualified value was supplied to this offline adapter'
_FIELDS = ('position_W_m', 'velocity_W_m_s', 'q_WB_wxyz', 'body_rates_B_rad_s')


def _metadata(context, *, contract, source, frame, clock, lineage, identity):
    result = dict(
        contract_id=contract, contract_version=VERSION, run_id=_id(context.run_id),
        record_id=_id(context.record_id), source=_id(source), sequence=_epoch(context.sequence),
        input_kind=context.input_kind, lineage=lineage, clock_id=clock, frame_id=frame,
        reset_epoch=_epoch(context.epoch), availability={},
    )
    for key in ('model', 'geometry', 'config_hash'):
        if key in identity:
            result[key] = deepcopy(identity[key])
    return result


def _state_record(context, *, source, lineage_kind, stamp, values, measurements,
                  frame, clock, now_s, max_age_s, identity):
    mapped_time = clock.map(stamp, source_id=context.clock_id, epoch=context.epoch)
    age = _fresh(mapped_time, now_s, max_age_s, clock.uncertainty_s)
    world, shift, body = frame.transforms(
        _number(stamp), source_id=context.frame_id, clock_id=context.clock_id, epoch=context.epoch,
    )
    origin = {'input_kind': lineage_kind, 'source': source}
    if context.source_record_id is not None:
        origin['record_id'] = _id(context.source_record_id)
    lineage = []
    if context.synthetic_fixture_id is not None:
        lineage.append({'input_kind': 'SYNTHETIC', 'source': _id(context.synthetic_fixture_id)})
    lineage.extend(dict(origin, transform=item)
                   for item in (frame.registration_id, clock.registration_id))
    metadata = _metadata(
        context, contract='rm27.state', source=source, frame=frame.destination_id,
        clock=clock.destination_id, identity=identity,
        lineage=lineage,
    )
    result = dict(metadata=metadata, frames=FRAMES.copy(), state_time_s=mapped_time,
                  validity={}, data_age_s={'state': age}, epoch_provenance=_id(context.epoch_provenance))
    for field in (*_FIELDS, 'actuator_state', 'uncertainty'):
        if values.get(field) is None:
            metadata['availability'][field] = {'status': 'UNKNOWN', 'reason': _UNKNOWN}
            continue
        value = values[field]
        if field == 'q_WB_wxyz':
            x, y, z, w = Rotation.from_matrix(world @ _rotation(value) @ body.T).as_quat()
            result[field] = [float(w), float(x), float(y), float(z)]
        else:
            vec = _array(value, (3,))
            transformed = body @ vec if field == 'body_rates_B_rad_s' else world @ vec
            if field == 'position_W_m':
                transformed = transformed + shift
            result[field] = _array(transformed, (3,)).tolist()
        result['validity'][field] = dict(
            availability={'status': 'AVAILABLE', 'reason': None},
            initialized=None, aligned=True, finite=True, current=True,
        )
    if measurements:
        result['measurement_times_s'] = {}
        for key, (acquisition, channel_limit) in measurements.items():
            time = clock.map(acquisition, source_id=context.clock_id, epoch=context.epoch)
            if time > mapped_time:
                raise Blocked('measurement after represented state')
            result['data_age_s'][key] = _fresh(
                time, now_s, min(_number(channel_limit), _number(max_age_s)), clock.uncertainty_s,
            )
            result['measurement_times_s'][key] = {'time_s': time, 'clock_id': clock.destination_id}
    return result


def adapt_sim_truth(sample, context, *, frame, clock, now_s, max_age_s, current_epoch,
                    expected_identity, observed_identity):
    """Project ego-only fields of a native Sim VehicleEstimate truth snapshot."""
    if context is None or frame is None or clock is None:
        raise Blocked('missing frame, clock or epoch registration')
    _state_identity(expected_identity, observed_identity)
    if _epoch(context.epoch) != _epoch(current_epoch):
        raise Blocked('superseded state epoch')
    if (context.input_kind != 'GROUND_TRUTH'
            or sample.source not in {'MUJOCO_TRUTH_TEACHER', 'MUJOCO_ORACLE'}):
        raise Blocked('Sim truth requires native truth lineage and model identity')
    if frame.source_body != 'FLU':
        raise Blocked('Sim source expects FLU')
    values = dict(zip(_FIELDS, (sample.position, sample.velocity, sample.rotation, sample.omega_body)))
    return _state_record(context, source=sample.source, lineage_kind='GROUND_TRUTH',
                         stamp=sample.timestamp, values=values, measurements={}, frame=frame,
                         clock=clock, now_s=now_s, max_age_s=max_age_s, identity=observed_identity)


def adapt_non_rl_sample(sample, context, *, frame, clock, now_s, max_age_s, current_epoch,
                        expected_identity, observed_identity):
    """Convert a single time-stamped ego-only VIO StateSample channel."""
    channels = dict(zip(('position', 'velocity', 'attitude', 'body_rate'), _FIELDS))
    expected_frame = {'position': 'world', 'velocity': 'world',
                      'attitude': 'body_FLU_to_world', 'body_rate': 'body_FLU'}
    if context is None or frame is None or clock is None:
        raise Blocked('missing frame, clock or epoch registration')
    _state_identity(expected_identity, observed_identity)
    if _epoch(context.epoch) != _epoch(current_epoch):
        raise Blocked('superseded state epoch')
    if (context.input_kind != 'ESTIMATED' or sample.source != 'IMPLEMENTED_VIO'
            or sample.quantity not in channels or sample.propagated or sample.value is None
            or not sample.valid or sample.clipped or sample.gap):
        raise Blocked('no supported, valid VIO estimate with known lineage')
    if (sample.frame != expected_frame[sample.quantity] or frame.source_body != 'FLU'
            or _epoch(sample.epoch_id) != context.epoch):
        raise Blocked('sample frame or epoch mismatch')
    if not 0 <= _number(sample.acquisition_s) == _number(sample.state_time_s) <= _number(sample.delivery_s):
        raise Blocked('non-propagated state requires acquisition time and ordered delivery')
    delivered = clock.map(sample.delivery_s, source_id=context.clock_id, epoch=context.epoch)
    _fresh(delivered, now_s, max_age_s, clock.uncertainty_s)
    field = channels[sample.quantity]
    return _state_record(context, source='IMPLEMENTED_VIO', lineage_kind='ESTIMATED',
                         stamp=sample.state_time_s, values={field: sample.value},
                         measurements={field: (sample.acquisition_s, sample.max_age_s)},
                         frame=frame, clock=clock, now_s=now_s, max_age_s=max_age_s,
                         identity=observed_identity)


def adapt_physical_ctbr(values, *, representation, source_body, envelope,
                        source_state, active_authority, current_epoch, now_s, max_state_age_s):
    """Validate a synthetic, target-free CTBR fixture in the exchange clock.

    `envelope` supplies the existing DirectCTBR record fields except thrust and
    rates. `values` is ordered [T_N, p_rad_s, q_rad_s, r_rad_s]. The return is
    a test record; it does not acquire a lease or allocate a single motor.
    """
    if representation != 'PHYSICAL_CTBR_SI':
        raise Blocked('only explicit physical CTBR in N and rad/s is supported')
    metadata = envelope['metadata']
    state_meta = source_state['metadata']
    authority = envelope['authority']
    timing = envelope['timing']
    if (envelope.get('record_type') != 'DirectCTBR' or envelope.get('target_id') is not None
            or metadata.get('input_kind') != 'SYNTHETIC'
            or not any(item.get('input_kind') == 'SYNTHETIC' for item in state_meta.get('lineage', []))):
        raise Blocked('only synthetic target-free CTBR fixtures are supported')
    if (envelope.get('frames') != FRAMES or source_state.get('frames') != FRAMES
            or metadata.get('contract_version') != VERSION or state_meta.get('contract_version') != VERSION
            or metadata.get('contract_id') != 'rm27.control' or state_meta.get('contract_id') != 'rm27.state'):
        raise Blocked('state or command exchange convention mismatch')
    for field in ('run_id', 'clock_id', 'frame_id'):
        if _id(metadata.get(field)) != _id(state_meta.get(field)):
            raise Blocked(f'state/command {field} mismatch')
    if (_epoch(metadata.get('reset_epoch')) != _epoch(current_epoch)
            or _epoch(state_meta.get('reset_epoch')) != current_epoch):
        raise Blocked('superseded state or command epoch')
    if _id(envelope.get('source_state_id')) != _id(state_meta.get('record_id')):
        raise Blocked('source state ID mismatch')
    require_same_identity(metadata.get('model'), state_meta.get('model'))
    for key in ('geometry', 'config_hash'):
        if key in metadata or key in state_meta:
            require_same_identity(metadata.get(key), state_meta.get(key))
    if (authority != active_authority or authority.get('allowed_layer') != 'DIRECT_CTBR'
            or authority.get('handoff_state') != 'ACTIVE'
            or _id(metadata.get('source')) != _id(authority.get('owner_id'))):
        raise Blocked('command authority conflict')

    now = _number(now_s)
    state_time = _number(source_state['state_time_s'])
    generated, apply_at, start, end = (
        _number(timing[key]) for key in ('generated_time_s', 'apply_time_s', 'valid_from_s', 'expires_at_s')
    )
    if not (state_time == _number(timing['source_state_time_s'])
            and state_time <= generated <= now <= apply_at and generated <= start <= apply_at < end):
        raise Blocked('invalid command timestamps or expired application window')
    if max(now, apply_at) >= _number(authority['lease_expires_at_s']):
        raise Blocked('control lease expired')
    _fresh(state_time, apply_at, max_state_age_s)
    # The represented state's timestamp need not be its informative acquisition.
    acquisition_times = []
    for measurement in source_state.get('measurement_times_s', {}).values():
        acquisition = _number(measurement['time_s'])
        if measurement['clock_id'] != metadata['clock_id'] or acquisition > state_time:
            raise Blocked('unmapped or future informative measurement')
        acquisition_times.append(acquisition)
    if not acquisition_times and state_meta.get('input_kind') != 'GROUND_TRUTH':
        raise Blocked('informative measurement age unknown')
    age = _fresh(min(acquisition_times, default=state_time), apply_at, max_state_age_s)
    thrust, *rates = _array(values, (4,))
    if thrust < 0:
        raise Blocked('negative collective thrust')
    record = deepcopy(envelope)
    record['target_id'] = None
    record['collective_thrust_N'] = float(thrust)
    record['body_rate_sp_B_rad_s'] = (_body_rotation(source_body) @ rates).tolist()
    record['timing']['measurement_age_s'] = age
    return record
