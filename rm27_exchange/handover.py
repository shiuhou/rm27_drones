"""G0: lossless handover envelopes and a local, non-actuating handshake.

Payloads are the existing draft State/Control/Geometry records. This module
neither propagates a state nor resets a plant, controller or actuator. Admission
is an interface check, never a claim of navigation or physical feasibility.
"""
from copy import deepcopy
from dataclasses import dataclass, field
from hashlib import sha256
import json
import math

from jsonschema.exceptions import ValidationError

from .offline_contracts import (
    Blocked, ClockRegistration, FrameRegistration, FRAMES, require_same_identity,
)
from .validation import loads_json, require_json_value, validate_record as _schema_validate

FORMAT = 'rm27.handover/1'
STATE_REFS = {f'state.schema.json#/$defs/{name}'
              for name in ('GroundTruthState', 'EstimatedState')}
CONTROL_REFS = {f'control.schema.json#/$defs/{name}' for name in (
    'ReferenceTrajectory', 'DirectCTBR', 'AppliedActuatorCommand',
)}
CHANNELS = ('position_W_m', 'velocity_W_m_s', 'q_WB_wxyz', 'body_rates_B_rad_s')


def validate_record(reference, record):
    try:
        _schema_validate(reference, record)
    except (ValueError, ValidationError) as exc:
        raise Blocked(f'invalid contract record: {exc}') from exc


def _require(condition, reason):
    if not condition:
        raise Blocked(reason)


def _id(value):
    _require(isinstance(value, str) and bool(value.strip()), 'missing identifier')
    return value


def _number(value):
    _require(type(value) in (int, float) and math.isfinite(value), 'invalid finite time/number')
    return value


def _keys(value, required, optional=()):
    _require(isinstance(value, dict) and set(required) <= value.keys()
             and value.keys() <= set(required) | set(optional), 'invalid handover envelope fields')


def encode(value):
    """Stable JSON serialization without rounding or filling missing values."""
    require_json_value(value)
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return sha256(encode(value).encode('utf-8')).hexdigest()


def _record(wrapper, allowed):
    _keys(wrapper, ('schema', 'record'))
    _require(wrapper['schema'] in allowed, 'unsupported payload schema')
    try:
        validate_record(wrapper['schema'], wrapper['record'])
    except ValueError as exc:
        raise Blocked(f'invalid contract record: {exc}') from exc
    return wrapper['record']


def validate_snapshot(snapshot):
    """Structure and intrinsic semantics; insufficient snapshots remain readable.

    Do not confuse this check with ``admit_snapshot``. In particular a recorded
    native row without known frame/clock mappings can be preserved losslessly.
    """
    require_json_value(snapshot)
    _keys(snapshot, ('format', 'snapshot_id', 'source', 'state', 'identity',
                     'registration', 'control_history', 'target_geometry', 'native_record'))
    _require(snapshot['format'] == FORMAT, 'unsupported handover format')
    _id(snapshot['snapshot_id'])
    source = snapshot['source']
    _keys(source, ('run_id', 'trajectory_family_id', 'input_kind', 'artifact', 'record_locator'))
    for name in ('run_id', 'trajectory_family_id', 'record_locator'):
        _id(source[name])
    _require(source['input_kind'] in {'REAL_MEASURED', 'RECORDED', 'SYNTHETIC'},
             'acquisition kind must not erase state lineage')
    validate_record('common.schema.json#/$defs/Identity', source['artifact'])
    history = snapshot['control_history']
    _keys(history, ('availability', 'records'))
    validate_record('common.schema.json#/$defs/Availability', history['availability'])
    _require(isinstance(history['records'], list), 'history must be a list')
    _require(not history['records'] or history['availability']['status'] == 'AVAILABLE',
             'history availability contradicts supplied records')
    for wrapper in history['records']:
        _record(wrapper, CONTROL_REFS)
    if snapshot['state'] is not None:
        state = _record(snapshot['state'], STATE_REFS)
        _require(state['metadata']['run_id'] == source['run_id'], 'source run mismatch')
        q = state.get('q_WB_wxyz')
        if q is not None:
            _require(abs(math.hypot(*q) - 1) <= 1e-8, 'quaternion must be unit WXYZ')
        actuator = state.get('actuator_state')
        if actuator:
            for name in ('rotor_speed_rad_s', 'rotor_thrust_N'):
                values = actuator.get(name)
                _require(values is None or len(values) == len(actuator['rotor_ids']),
                         'actuator vector/order mismatch')
            applied = actuator.get('motor_command_applied')
            if applied:
                _require(applied['rotor_ids'] == actuator['rotor_ids']
                         and len(applied['input']['values']) == len(actuator['rotor_ids']),
                         'applied command rotor/order mismatch')
    else:
        _require(snapshot['native_record'] is not None, 'missing both state and native evidence')
    identity = snapshot['identity']
    if identity is not None:
        _keys(identity, ('model', 'geometry', 'plant_profile', 'config_hash'), ('catalog_sha256',))
        for name in ('model', 'geometry', 'plant_profile'):
            validate_record('common.schema.json#/$defs/Identity', identity[name])
        validate_record('common.schema.json#/$defs/Sha256', identity['config_hash'])
    if snapshot['registration'] is not None:
        _keys(snapshot['registration'], ('frame', 'clock'))
        for value in snapshot['registration'].values():
            _id(value)
    target = snapshot['target_geometry']
    if target is not None:
        _record(target, {'geometry.schema.json#/$defs/TargetGeometry'})
    return snapshot


def build_snapshot(*, snapshot_id, source, state=None, identity=None, registration=None,
                   control_history=None, target_geometry=None, native_record=None):
    """Copy existing records; never synthesize missing physical channels."""
    result = deepcopy(dict(format=FORMAT, snapshot_id=snapshot_id, source=source,
                           state=state, identity=identity, registration=registration,
                           control_history=control_history if control_history is not None else {
                               'availability': {'status': 'UNKNOWN', 'reason': 'Not recorded'},
                               'records': [],
                           }, target_geometry=target_geometry, native_record=native_record))
    return validate_snapshot(result)


def decode_snapshot(text):
    return validate_snapshot(loads_json(text))


@dataclass(frozen=True)
class HandoverRegistry:
    """Receiver-owned registrations, not trust granted by the sending envelope.

    SYNTHETIC_TEST is an explicitly separate test profile. It cannot admit a
    truth snapshot either, and its PASS has no autonomous-flight meaning.
    """
    identity: dict
    frame: FrameRegistration
    clock: ClockRegistration
    epoch: int
    max_age_s: float
    mode: str = 'RUNTIME_ESTIMATED'
    targets: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value):
        return cls(identity=deepcopy(value['identity']), frame=FrameRegistration(**value['frame']),
                   clock=ClockRegistration(**value['clock']), epoch=value['epoch'],
                   max_age_s=value['max_age_s'], mode=value.get('mode', 'RUNTIME_ESTIMATED'),
                   targets=deepcopy(value.get('targets', {})))


def _fresh(time, now, registry):
    age = _number(now) - _number(time)
    error = _number(registry.clock.uncertainty_s)
    _require(0 <= error <= age and age + error <= _number(registry.max_age_s),
             'stale or future informative state')


def _scope(record, state, identity):
    sm = state['metadata']
    metadata = record['metadata']
    for name in ('run_id', 'frame_id', 'clock_id', 'reset_epoch'):
        _require(metadata[name] == sm[name], f'{name} mismatch')
    for name in ('model', 'geometry', 'config_hash'):
        require_same_identity(identity[name], metadata.get(name))


def _lineage_kinds(metadata):
    return {metadata['input_kind']} | {item['input_kind'] for item in metadata['lineage']}


def _consumer_lineage(metadata, registry):
    _require(bool(metadata['lineage']), 'missing source lineage')
    if registry.mode == 'RUNTIME_ESTIMATED':
        _require(not (_lineage_kinds(metadata) & {'GROUND_TRUTH', 'SYNTHETIC'}),
                 'runtime record has oracle or synthetic ancestry')


def admit_snapshot(snapshot, registry, *, now_s):
    """Admit only fully qualified interface channels under a registered profile."""
    validate_snapshot(snapshot)
    _require(registry.mode in {'RUNTIME_ESTIMATED', 'SYNTHETIC_TEST'}, 'unknown admission mode')
    _require(type(registry.epoch) is int and registry.epoch >= 0, 'unknown receiver epoch')
    _require(registry.max_age_s > 0, 'invalid freshness policy')
    _require(snapshot['state'] is not None and snapshot['native_record'] is None,
             'native recording lacks registered shared state; BLOCKED')
    state = snapshot['state']['record']
    metadata = state['metadata']
    kinds = _lineage_kinds(metadata)
    _require(snapshot['state']['schema'].endswith('/EstimatedState')
             and metadata['input_kind'] in {'ESTIMATED', 'PREDICTED'}
             and 'GROUND_TRUTH' not in kinds, 'ground truth is isolated from runtime handover')
    _require(bool(metadata['lineage']), 'missing source lineage')
    acquisition = snapshot['source']['input_kind']
    if registry.mode == 'RUNTIME_ESTIMATED':
        _require('SYNTHETIC' not in kinds and acquisition != 'SYNTHETIC',
                 'synthetic fixture is not runtime estimated evidence')
    else:
        _require('SYNTHETIC' in kinds and acquisition == 'SYNTHETIC',
                 'synthetic profile requires explicit synthetic ancestry')
    if acquisition == 'RECORDED':
        _require('RECORDED' in kinds, 'recorded storage lineage missing')
    require_same_identity(registry.identity, snapshot['identity'])
    _scope(state, state, registry.identity)
    frame, clock = registry.frame, registry.clock
    _require(snapshot['registration'] == {'frame': frame.registration_id, 'clock': clock.registration_id},
             'unknown frame/clock registration')
    _require(metadata['reset_epoch'] == registry.epoch == frame.epoch == clock.epoch,
             'state/registration epoch mismatch')
    _require(state['frames'] == FRAMES and metadata['frame_id'] == frame.destination_id
             and metadata['clock_id'] == clock.destination_id, 'unregistered destination frame/clock')
    stamp = _number(state['state_time_s'])
    _require(_number(clock.scale) > 0, 'invalid clock registration scale')
    native_stamp = (stamp - _number(clock.offset_s)) / clock.scale
    clock.map(native_stamp, source_id=clock.source_id, epoch=registry.epoch)
    frame.transforms(native_stamp, source_id=frame.source_id,
                     clock_id=clock.source_id, epoch=registry.epoch)
    _require(frame.source_clock_id == clock.source_id, 'registration source clock mismatch')
    transforms = {item.get('transform') for item in metadata['lineage']}
    _require({frame.registration_id, clock.registration_id} <= transforms,
             'state lineage does not identify registered transforms')
    _fresh(stamp, now_s, registry)
    for channel in CHANNELS:
        _require(channel in state, f'missing {channel}')
        validity = state['validity'].get(channel, {})
        _require(validity.get('availability', {}).get('status') == 'AVAILABLE'
                 and all(validity.get(name) is True for name in ('initialized', 'aligned', 'finite', 'current')),
                 f'invalid or unknown {channel} validity')
        measurement = state.get('measurement_times_s', {}).get(channel)
        _require(measurement is not None and measurement['clock_id'] == metadata['clock_id'],
                 f'unknown informative acquisition clock/time for {channel}')
        _require(measurement['time_s'] <= stamp, 'measurement after represented state')
        acquisition_native = (measurement['time_s'] - clock.offset_s) / clock.scale
        clock.map(acquisition_native, source_id=clock.source_id, epoch=registry.epoch)
        frame.transforms(acquisition_native, source_id=frame.source_id,
                         clock_id=clock.source_id, epoch=registry.epoch)
        _fresh(measurement['time_s'], now_s, registry)
    for time_key, clock_key in (('published_time_s', 'published_clock_id'),
                                ('received_time_s', 'received_clock_id')):
        if time_key in state:
            _require(state[clock_key] == metadata['clock_id']
                     and stamp <= state[time_key] <= now_s, 'unmapped or invalid publication/receipt')
    target = snapshot['target_geometry']
    relative = state.get('target_relative')
    if relative:
        _require(target is not None and relative['target_id'] == target['record']['target_id'],
                 'target-relative channel lacks matching registered geometry')
    if target:
        record = target['record']
        require_same_identity(registry.targets.get(record['target_id']), record)
        _require(record['registration_epoch'] == registry.epoch
                 and record['target_frame_WT']['parent_frame_id'] == metadata['frame_id'],
                 'target registration epoch/frame mismatch')
    history = snapshot['control_history']['records']
    last_time, sequences = -math.inf, {}
    for wrapper in history:
        record = wrapper['record']
        _scope(record, state, registry.identity)
        _require(record['frames'] == state['frames'], 'history frame convention mismatch')
        time = record['timing']['apply_time_s']
        seq = record['metadata']['sequence']
        history_key = (record['metadata']['run_id'], record['metadata']['source'],
                       record['metadata']['reset_epoch'])
        _require(last_time <= time <= stamp and seq > sequences.get(history_key, -1),
                 'history time/sequence order mismatch')
        _consumer_lineage(record['metadata'], registry)
        last_time, sequences[history_key] = time, seq
        if 'rotor_ids' in record:
            for key in ('motor_command_requested', 'motor_command_applied'):
                if key in record:
                    _require(len(record[key]['values']) == len(record['rotor_ids']),
                             'history motor vector mismatch')
    actuator = state.get('actuator_state', {})
    applied = actuator.get('motor_command_applied')
    if applied:
        _require(applied['clock_id'] == metadata['clock_id'] and applied['apply_time_s'] <= stamp,
                 'actuator history clock/time mismatch')
        matching = [w['record'] for w in history if w['record'].get('command_id') == applied['command_id']]
        _require(len(matching) == 1, 'last applied command missing or ambiguous in history')
        cmd = matching[0]
        _require(cmd['motor_command_applied'] == applied['input']
                 and cmd['rotor_ids'] == actuator['rotor_ids']
                 and cmd['timing']['apply_time_s'] == applied['apply_time_s'],
                 'last applied actuator command/history mismatch')
    return deepcopy(state)


def assert_continuity(before, after):
    """ACK echoes the same boundary, not a reinitialized/evolved plant state.

    No state propagation is performed. A different boundary requires a new
    request. Quaternion sign is physically immaterial; all other evidence,
    timestamps, actuator values and command histories are preserved exactly.
    """
    validate_snapshot(before)
    validate_snapshot(after)
    left, right = deepcopy(before), deepcopy(after)
    if left['state'] and right['state']:
        q1, q2 = (item['state']['record'].get('q_WB_wxyz') for item in (left, right))
        if q1 and q2 and all(abs(a + b) <= 1e-12 for a, b in zip(q1, q2)):
            right['state']['record']['q_WB_wxyz'] = q1
    _require(left == right, 'handover boundary/history discontinuity')


class HandoverProtocol:
    """Single-owner offline lease transfer; never emits a motor command.

    ACCEPT/ADJUST reserve a transfer only. ACK commits it for the registered
    effective time. TIMEOUT/ABORT retain the unexpired predecessor; epoch reset
    revokes all ownership and requires a new registered protocol instance.
    """
    def __init__(self, registry, authority, *, parent_reference_id=None):
        validate_record('control.schema.json#/$defs/AuthorityLease', authority)
        _require(authority['handoff_state'] == 'ACTIVE', 'initial authority not active')
        self.registry = deepcopy(registry)
        self.active = deepcopy(authority)
        self.parent_reference_id = parent_reference_id
        self.pending = None
        self.boundary = None
        self.accepted_reference = None
        self.last_time = -math.inf
        self.sequences = {}
        self.command_sequences = {}
        self.request_ids = set()
        self.invalidated = False

    def _advance(self, now_s):
        now = _number(now_s)
        _require(now >= self.last_time, 'receiver clock moved backwards')
        self.last_time = now
        status = None
        if self.pending:
            p = self.pending
            if not p['acked'] and now >= p['request']['ack_deadline_s']:
                self.pending = None
                status = 'TIMEOUT'
            elif p['acked'] and now >= p['effective']:
                self.active = deepcopy(p['successor'])
                self.active['handoff_state'] = 'ACTIVE'
                self.boundary = deepcopy(p['request']['snapshot'])
                self.parent_reference_id = p['reference']['record']['reference_id']
                self.accepted_reference = deepcopy(p['reference']['record'])
                self.accepted_reference['authority'] = deepcopy(self.active)
                self.pending = None
                status = 'ACK'
        if self.active and now >= self.active['lease_expires_at_s']:
            self.active = None
        return status

    def tick(self, *, now_s):
        return {'status': self._advance(now_s) or 'UNCHANGED', 'authority': deepcopy(self.active)}

    def invalidate_epoch(self, epoch):
        _require(type(epoch) is int and epoch > self.registry.epoch, 'non-increasing reset epoch')
        self.active = self.pending = self.boundary = self.accepted_reference = None
        self.invalidated = True
        return {'status': 'ABORT', 'reason': 'epoch reset; new registrations required', 'authority': None}

    def _reference(self, wrapper, snapshot, successor, effective, now_s):
        record = _record(wrapper, {'control.schema.json#/$defs/ReferenceTrajectory'})
        state = snapshot['state']['record']
        _scope(record, state, self.registry.identity)
        _consumer_lineage(record['metadata'], self.registry)
        _require(record['frames'] == state['frames'], 'reference convention mismatch')
        _require(record['authority'] == successor and record['metadata']['source'] == successor['owner_id'],
                 'reference authority mismatch')
        _require(record['source_state_id'] == state['metadata']['record_id']
                 and record['parent_id'] == self.parent_reference_id, 'parent state/reference mismatch')
        target = snapshot['target_geometry']
        target_id = target['record']['target_id'] if target else None
        _require(record['target_id'] == target_id, 'unregistered reference target')
        self._timing(record, state, now_s=now_s, apply_at=effective)
        _require(record['timing']['apply_time_s'] == effective,
                 'reference application must match registered transfer boundary')
        knots = record['knots']
        times = [k['time_s'] for k in knots]
        _require(times == sorted(set(times)) and times[0] <= effective <= times[-1],
                 'reference knot window/order mismatch')
        for knot in knots:
            q = knot.get('q_ref_WB_wxyz')
            _require(q is None or abs(math.hypot(*q) - 1) <= 1e-8, 'reference quaternion norm')
        return record

    def _timing(self, record, state, *, now_s, apply_at):
        timing = record['timing']
        _require(timing['source_state_time_s'] == state['state_time_s'], 'parent state time mismatch')
        _require(state['state_time_s'] <= timing['generated_time_s'] <= now_s
                 and timing['generated_time_s'] <= timing['valid_from_s'] <= timing['apply_time_s']
                 and timing['apply_time_s'] <= apply_at < timing['expires_at_s']
                 and now_s < timing['expires_at_s'], 'expired plan or invalid command timing')
        _require(apply_at < record['authority']['lease_expires_at_s'], 'command lease expired')
        for measurement in state['measurement_times_s'].values():
            _fresh(measurement['time_s'], max(now_s, apply_at), self.registry)

    def request(self, request, *, now_s):
        self._advance(now_s)
        _require(not self.invalidated and self.active is not None, 'no registered active authority')
        _require(self.pending is None, 'another handover is pending')
        _keys(request, ('request_id', 'snapshot', 'reference', 'authority', 'ack_deadline_s'))
        request_id = _id(request['request_id'])
        _require(request_id not in self.request_ids, 'duplicate request')
        state = admit_snapshot(request['snapshot'], self.registry, now_s=now_s)
        sm = state['metadata']
        key = (sm['run_id'], sm['source'], sm['reset_epoch'])
        _require(sm['sequence'] > self.sequences.get(key, -1), 'duplicate or out-of-order state sequence')
        auth = _record(request['authority'], {'control.schema.json#/$defs/ControlAuthority'})
        _scope(auth, state, self.registry.identity)
        _consumer_lineage(auth['metadata'], self.registry)
        _require(auth['authority'] == self.active and auth['metadata']['source'] == self.active['owner_id'],
                 'invalid request authority')
        handoff = auth.get('handoff', {})
        _require(handoff.get('predecessor') == self.active
                 and handoff.get('acknowledged_successor') is False, 'invalid handoff predecessor/ack')
        successor = handoff['successor']
        effective = handoff['effective_apply_time_s']
        _require(successor['vehicle_id'] == self.active['vehicle_id']
                 and successor['lease_id'] != self.active['lease_id']
                 and successor['owner_id'] != self.active['owner_id']
                 and successor['generation'] > self.active['generation']
                 and successor['handoff_state'] == 'PENDING'
                 and successor['allowed_layer'] == 'REFERENCE_TRAJECTORY', 'invalid successor lease')
        _require(now_s < request['ack_deadline_s'] <= effective
                 and effective < min(self.active['lease_expires_at_s'], successor['lease_expires_at_s']),
                 'invalid transfer/ack deadline or expired lease')
        self._reference(request['reference'], request['snapshot'], successor, effective, now_s)
        self.request_ids.add(request_id)
        self.sequences[key] = sm['sequence']
        self.pending = {'request': deepcopy(request), 'reference': deepcopy(request['reference']),
                        'successor': deepcopy(successor), 'effective': effective,
                        'decision': None, 'acked': False}
        return {'status': 'REQUEST', 'request_id': request_id, 'authority': deepcopy(self.active)}

    def decide(self, request_id, decision, *, now_s, reference=None):
        self._advance(now_s)
        p = self.pending
        _require(p is not None and p['request']['request_id'] == request_id
                 and p['decision'] is None, 'unknown or already decided request')
        _require(decision in {'ACCEPT', 'ADJUST', 'ABORT'}, 'invalid decision')
        if decision == 'ABORT':
            self.pending = None
        else:
            admit_snapshot(p['request']['snapshot'], self.registry, now_s=now_s)
            if decision == 'ADJUST':
                _require(reference is not None, 'ADJUST requires revised reference')
                self._reference(reference, p['request']['snapshot'], p['successor'], p['effective'], now_s)
                p['reference'] = deepcopy(reference)
            else:
                _require(reference is None, 'ACCEPT does not replace reference')
                self._reference(p['reference'], p['request']['snapshot'], p['successor'], p['effective'], now_s)
            p['decision'] = decision
        return {'status': decision, 'request_id': request_id, 'authority': deepcopy(self.active)}

    def acknowledge(self, request_id, snapshot, reference_id, successor, *, now_s):
        self._advance(now_s)
        p = self.pending
        _require(p is not None and p['request']['request_id'] == request_id
                 and p['decision'] in {'ACCEPT', 'ADJUST'} and not p['acked'], 'no accepted pending request')
        _require(successor == p['successor'] and reference_id == p['reference']['record']['reference_id'],
                 'ACK authority/reference mismatch')
        admit_snapshot(snapshot, self.registry, now_s=now_s)
        assert_continuity(p['request']['snapshot'], snapshot)
        self._reference(p['reference'], snapshot, p['successor'], p['effective'], now_s)
        p['acked'] = True
        self._advance(now_s)
        return {'status': 'ACK', 'request_id': request_id, 'effective_time_s': p['effective'],
                'authority': deepcopy(self.active)}

    def validate_command(self, wrapper, *, now_s):
        self._advance(now_s)
        _require(self.active is not None and self.boundary is not None and not self.invalidated,
                 'no acknowledged handover boundary/authority')
        record = _record(wrapper, {'control.schema.json#/$defs/ReferenceTrajectory'})
        state = admit_snapshot(self.boundary, self.registry, now_s=now_s)
        _scope(record, state, self.registry.identity)
        _require(record['authority'] == self.active and record['metadata']['source'] == self.active['owner_id'],
                 'command authority conflict')
        _require(record['reference_id'] == self.parent_reference_id
                 and record['source_state_id'] == state['metadata']['record_id'], 'command parent identity mismatch')
        _require(record['frames'] == state['frames'], 'command frame mismatch')
        expected, observed = deepcopy(self.accepted_reference), deepcopy(record)
        # A repeated delivery may have a new transport record/sequence, but a
        # reference ID cannot silently acquire different geometry or motion.
        for value in (expected, observed):
            for name in ('sequence', 'record_id'):
                value['metadata'].pop(name)
        _require(expected == observed, 'accepted reference content changed')
        self._timing(record, state, now_s=now_s, apply_at=now_s)
        sequence = record['metadata']['sequence']
        key = self.active['lease_id']
        _require(sequence > self.command_sequences.get(key, -1), 'duplicate/out-of-order command')
        self.command_sequences[key] = sequence
        return {'status': 'PASS', 'scope': 'OFFLINE_COMMAND_ADMISSION_ONLY', 'record': deepcopy(record)}


def main():
    """JSONL worker: same snapshot reader for either consumer; no policy imports."""
    import argparse
    from pathlib import Path
    import sys
    from .validation import read_json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True,
                        help='receiver-owned registry and initial offline authority JSON')
    parser.add_argument('--consumer', choices=('rl', 'non_rl'), required=True)
    args = parser.parse_args()
    fixture = read_json(args.fixture)
    registry = HandoverRegistry.from_dict(fixture['registry'])
    protocol = HandoverProtocol(registry, fixture['initial_authority'],
                                parent_reference_id=fixture['parent_reference_id'])
    print(encode({'module': str(Path(__file__).resolve()), 'consumer': args.consumer,
                  'mode': registry.mode, 'scope': 'G0_OFFLINE_COMMON_READER'}), file=sys.stderr, flush=True)
    for line in sys.stdin:
        try:
            message = loads_json(line)
            operation = message['operation']
            now = message.get('now_s')
            if operation in {'read', 'inspect', 'native-read'}:
                snapshot = decode_snapshot(encode(message['snapshot']))
                if operation == 'native-read':
                    from .handover_consumers import project_native
                    result = project_native(snapshot, registry, args.consumer, now)
                    result['snapshot'] = snapshot
                elif operation == 'inspect':
                    try:
                        admit_snapshot(snapshot, registry, now_s=now)
                        readiness = {'status': 'PASS', 'scope': 'INTERFACE_ONLY'}
                    except Blocked as exc:
                        readiness = {'status': 'BLOCKED', 'reason': str(exc)}
                    result = {'status': 'PASS', 'snapshot': snapshot,
                              'admission': readiness, 'snapshot_sha256': digest(snapshot)}
                else:
                    state = admit_snapshot(snapshot, registry, now_s=now)
                    result = {'status': 'PASS', 'snapshot': snapshot, 'state': state,
                              'snapshot_sha256': digest(snapshot)}
            elif operation == 'request':
                result = protocol.request(message['request'], now_s=now)
            elif operation == 'decide':
                result = protocol.decide(message['request_id'], message['decision'], now_s=now,
                                         reference=message.get('reference'))
            elif operation == 'ack':
                result = protocol.acknowledge(message['request_id'], message['snapshot'],
                                              message['reference_id'], message['successor'], now_s=now)
            elif operation == 'tick':
                result = protocol.tick(now_s=now)
            elif operation == 'command':
                result = protocol.validate_command(message['command'], now_s=now)
            elif operation == 'reset':
                result = protocol.invalidate_epoch(message['epoch'])
            else:
                raise Blocked('unknown worker operation')
            result['consumer'] = args.consumer
        except (ValueError, KeyError, TypeError) as exc:
            result = {'status': 'BLOCKED', 'reason': str(exc), 'consumer': args.consumer}
        print(encode(result), flush=True)


if __name__ == '__main__':
    main()
