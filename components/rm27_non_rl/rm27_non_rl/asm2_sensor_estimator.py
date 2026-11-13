"""Optional sensor-only complementary estimator; frozen M3 prediction is reused.

All packets are synthetic measurements, never real-sensor qualification. The
navigation frame has a local yaw gauge. External world p/v requires an explicit
registration assumption; that assumption is not observed world alignment.
Delayed measurements cause bounded acquisition-time replay. On each interval the
PREVIOUS delivered gyro and accelerometer samples are held (ZOH); a new sample
is never integrated backwards. No covariance or statistical integrity is invented.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from typing import Mapping

import mujoco
import numpy as np

from .m3_estimator import EstimatorConfig, LocalizationEstimator
from m3_sensors import IMUSample

SYNTHETIC = 'TRUTH_GENERATED_SYNTHETIC'
_NS = 1_000_000_000


def time_ns(value: float) -> int:
    """One explicit nearest-nanosecond representation, not a clock tolerance."""
    if isinstance(value, bool) or not isinstance(value, (int, float, np.floating)):
        raise ValueError('time must be a finite nonnegative scalar')
    if not math.isfinite(value) or value < 0:
        raise ValueError('time must be finite and nonnegative')
    return int(round(float(value) * _NS))


def _vector(value, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite three-vector') from exc
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must be a finite three-vector')
    return tuple(float(x) for x in result)


@dataclass(frozen=True)
class SensorPacket:
    kind: str
    acquisition_s: float
    delivery_s: float
    value: object
    source: str = SYNTHETIC
    valid: bool = True
    clipped: bool = False

    def __post_init__(self):
        acquired, delivered = time_ns(self.acquisition_s), time_ns(self.delivery_s)
        if delivered < acquired:
            raise ValueError('delivery precedes acquisition')
        if self.source != SYNTHETIC:
            raise ValueError('only explicitly synthetic measurement packets are accepted')
        if type(self.valid) is not bool or type(self.clipped) is not bool:
            raise ValueError('valid and clipped must be booleans')
        if self.kind in ('gyro', 'accel'):
            object.__setattr__(self, 'value', _vector(self.value, self.kind))
        elif self.kind == 'external':
            if not isinstance(self.value, Mapping) or set(self.value) not in (
                    {'position_m', 'velocity_m_s'}, {'position', 'velocity'}):
                raise ValueError('external value requires only position and velocity vectors')
            # A tuple prevents mutation of already-ingested packet contents.
            object.__setattr__(self, 'value', (
                _vector(self.value.get('position_m', self.value.get('position')), 'position_m'),
                _vector(self.value.get('velocity_m_s', self.value.get('velocity')), 'velocity_m_s')))
        elif self.kind == 'impact':
            if not isinstance(self.value, Mapping) or set(self.value) not in (
                    {'lower_s', 'upper_s'}, {'lower_s', 'upper_s', 'event_id'}):
                raise ValueError('impact value requires only lower_s and upper_s')
            if 'event_id' in self.value and (not isinstance(self.value['event_id'], str) or not self.value['event_id']):
                raise ValueError('event_id must be a nonempty string')
            lower, upper = time_ns(self.value['lower_s']), time_ns(self.value['upper_s'])
            if not lower <= upper <= acquired:
                raise ValueError('impact interval must precede or end at acquisition')
            object.__setattr__(self, 'value', (lower, upper))
        else:
            raise ValueError('unknown packet kind')

    @classmethod
    def from_mapping(cls, record):
        if not isinstance(record, Mapping):
            raise ValueError('packet must be a mapping')
        allowed = {'kind', 'acquisition_s', 'delivery_s', 'value', 'source', 'valid', 'clipped'}
        if set(record) - allowed:
            raise ValueError('unknown packet fields are forbidden')
        required = {'kind', 'acquisition_s', 'delivery_s', 'value', 'source'}
        if required - set(record):
            raise ValueError('missing required packet fields')
        return cls(**record)


@dataclass(frozen=True)
class SensorEstimatorConfig:
    imu_gap_s: float = .010
    external_max_age_s: float = .05
    history_s: float = .1
    max_packets: int = 20000
    external_frame_registration: str | None = None
    # Existing M3 gains: all three axes, explicitly external p/v, not optical flow.
    alpha_position: float = EstimatorConfig().alpha_tof
    alpha_velocity: float = EstimatorConfig().alpha_flow

    def __post_init__(self):
        for name in ('imu_gap_s', 'external_max_age_s', 'history_s'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not isinstance(self.max_packets, int) or self.max_packets < 4:
            raise ValueError('max_packets must be at least four')
        if self.history_s < self.imu_gap_s:
            raise ValueError('history must cover the IMU gap interval')
        if self.alpha_position != .5 or self.alpha_velocity != .35:
            raise ValueError('this experiment preserves existing M3 complementary gains')
        if self.external_frame_registration is not None and (
                not isinstance(self.external_frame_registration, str)
                or not self.external_frame_registration.strip()):
            raise ValueError('frame registration must name an explicit assumption')


@dataclass
class SensorEstimate:
    timestamp_s: float
    position_m: np.ndarray
    velocity_m_s: np.ndarray
    rotation_body_to_local: np.ndarray
    angular_velocity_body_rad_s: np.ndarray
    epoch_id: int
    channels: dict
    alignment: str
    frame_registration: str | None
    faults: tuple[str, ...]
    covariance: object = None
    covariance_reason: str = 'M3_COMPLEMENTARY_FILTER_HAS_NO_COVARIANCE'
    source: str = SYNTHETIC
    world_attitude_trusted: bool = False
    propagation_method: str = 'PREVIOUS_SAMPLE_ZERO_ORDER_HOLD_M3_PREDICT'

    def to_dict(self):
        out = asdict(self)
        for name in ('position_m', 'velocity_m_s', 'rotation_body_to_local',
                     'angular_velocity_body_rad_s'):
            out[name] = getattr(self, name).tolist()
        return out


@dataclass
class _Context:
    core: LocalizationEstimator
    time: int
    gyro: SensorPacket | None = None
    accel: SensorPacket | None = None
    external: SensorPacket | None = None
    epoch: int = 0
    barrier: int = -1
    alignment: str = 'NOT_OBSERVED_LOCAL_YAW_GAUGE'
    faults: tuple[str, ...] = ()
    external_epoch: int | None = None
    alignment_evidence_id: str | None = None
    alignment_time: int | None = None


class SensorOnlyEstimator:
    """Ingest delivered measurements and publish passive local-frame estimates.

    ``ingest(packet, now_s=...)`` accepts SensorPacket or a strict mapping and
    returns {accepted, reason}. ``estimate(now_s)`` returns a serializable dict.
    Calls use a nondecreasing runtime clock. History bounds reject late packets
    explicitly; rejected packets remain in ``rejections``. The original M3
    implementation and its tests are untouched.
    """
    def __init__(self, config: SensorEstimatorConfig | None = None):
        self.config = config or SensorEstimatorConfig()
        self._base = _Context(LocalizationEstimator(EstimatorConfig()), 0)
        self._events = []
        self._checkpoints = {}
        self._dirty = 0
        self._now = 0
        self._pruned = False
        self._keys = set()
        self._latest_barrier = -1
        self._latched_faults = set()
        self._last_reinit = -1
        self.rejections = []
        self.accepted_packets = 0
        self.replayed_event_groups = 0

    def _clock(self, now_s):
        now = time_ns(now_s)
        if now < self._now:
            raise ValueError('runtime clock must be nondecreasing')
        return now

    def _reject(self, packet, reason):
        self.rejections.append({'kind': packet.kind, 'acquisition_s': packet.acquisition_s,
                                'delivery_s': packet.delivery_s, 'reason': reason})
        return {'accepted': False, 'reason': reason}

    def ingest(self, packet: SensorPacket | Mapping, *, now_s: float | None = None):
        if not isinstance(packet, SensorPacket):
            packet = SensorPacket.from_mapping(packet)
        now = self._clock(max(self._now / _NS, packet.delivery_s) if now_s is None else now_s)
        acq, delivery = time_ns(packet.acquisition_s), time_ns(packet.delivery_s)
        if delivery > now:
            return self._reject(packet, 'NOT_DELIVERED')
        self._now = now
        event_time = packet.value[1] if packet.kind == 'impact' else acq
        if self._pruned and event_time <= self._base.time:
            return self._reject(packet, 'OUTSIDE_RETAINED_HISTORY')
        if now - event_time > time_ns(self.config.history_s):
            return self._reject(packet, 'OUTSIDE_RETAINED_HISTORY')
        if packet.kind == 'external' and acq <= self._latest_barrier:
            return self._reject(packet, 'PRE_IMPACT_OR_REINITIALIZATION_EXTERNAL')
        key = (event_time, packet.kind)
        if key in self._keys:
            previous = next(e[2] for e in self._events if e[0] == event_time
                            and isinstance(e[2], SensorPacket) and e[2].kind == packet.kind)
            return self._reject(packet, 'DUPLICATE' if previous == packet else 'CONFLICTING_SAMPLE')
        if len(self._events) >= self.config.max_packets:
            return self._reject(packet, 'HISTORY_PACKET_LIMIT')
        priority = {'impact': 0, 'gyro': 1, 'accel': 2, 'external': 3}[packet.kind]
        self._events.append((event_time, priority, packet))
        self._events.sort(key=lambda event: event[:2])
        self._keys.add(key)
        self._dirty = event_time if self._dirty is None else min(self._dirty, event_time)
        if packet.kind == 'impact' and packet.valid and not packet.clipped:
            self._latest_barrier = max(self._latest_barrier, packet.value[1])
        self.accepted_packets += 1
        return {'accepted': True, 'reason': 'ACCEPTED_FOR_ACQUISITION_TIME_REPLAY'}

    def reinitialize_stationary(self, *, now_s: float, mean_specific_force_body,
                               stationary_status: bool, evidence_id: str,
                               mean_gyro_body_rad_s=None, source: str = SYNTHETIC):
        """Explicit measured-gravity reinit; yaw remains a local gauge.

        The caller must have independently established the stationary status.
        This is not a detector, a truth attitude input, or world registration.
        No default truth-aligned airborne initialization is available.
        """
        now = self._clock(now_s)
        force = _vector(mean_specific_force_body, 'mean_specific_force_body')
        if source != SYNTHETIC or stationary_status is not True or not isinstance(evidence_id, str) or not evidence_id:
            raise ValueError('explicit synthetic stationary-status evidence is required')
        if abs(np.linalg.norm(force) - 9.81) > .5:
            raise ValueError('stationary gravity norm is inconsistent')
        if mean_gyro_body_rad_s is not None and np.linalg.norm(_vector(mean_gyro_body_rad_s, 'gyro mean')) > .05:
            raise ValueError('stationary gyro evidence is inconsistent')
        if now <= self._last_reinit:
            raise ValueError('reinitialization must advance time')
        self._now = now
        self._latest_barrier = max(self._latest_barrier, now)
        self._last_reinit = now
        # Internal evidence event has no external packet entry point.
        self._events.append((now, 4, ('stationary', force, evidence_id)))
        self._events.sort(key=lambda event: event[:2])
        self._dirty = now if self._dirty is None else min(self._dirty, now)
        self._latched_faults.clear()

    def initialize_flight_history(self, records, *, now_s):
        from .asm2_flight_alignment import solve_alignment

        now = self._clock(now_s)
        result = solve_alignment(records, now_s=now_s)
        if result is None:
            return None
        acquired = time_ns(result.acquisition_s)
        if acquired <= self._base.time or acquired <= self._latest_barrier:
            return None
        self._now = now
        self._events.append((acquired, 4, ('flight_alignment', result)))
        self._events.sort(key=lambda event: event[:2])
        self._dirty = acquired if self._dirty is None else min(self._dirty, acquired)
        return dict(acquisition_s=result.acquisition_s, delivery_s=result.delivery_s,
                    residual_m_s=result.residual_m_s, excitation_ratio=result.excitation_ratio,
                    intervals=result.intervals, covariance=None,
                    integrity='SENSOR_CONSISTENCY_NOT_CALIBRATED_CONFIDENCE')

    @staticmethod
    def _fault(context, reason):
        context.faults = tuple(sorted(set(context.faults) | {reason}))

    def _advance(self, context, target):
        if target == context.time:
            return
        if context.gyro is None:
            if (context.alignment_time is not None
                    and target - context.alignment_time > time_ns(self.config.imu_gap_s)):
                self._fault(context, 'GYRO_GAP_LATCHED')
            context.time = target
            context.core._last_imu_time = target / _NS
            return
        gap = time_ns(self.config.imu_gap_s)
        gyro_deadline = time_ns(context.gyro.acquisition_s) + gap
        if target > gyro_deadline:
            self._fault(context, 'GYRO_GAP_LATCHED')
        accel_deadline = (-1 if context.accel is None else
                          time_ns(context.accel.acquisition_s) + gap)
        if context.accel is not None and target > accel_deadline:
            self._fault(context, 'ACCEL_GAP_LATCHED')
        end = min(target, gyro_deadline)
        if context.gyro.valid and not context.gyro.clipped and end > context.time:
            # Split when accelerometer freshness expires. Reuse M3's exact
            # quaternion integrator in both segments; when accel is unavailable,
            # its mandatory force argument is a computational placeholder ONLY:
            # discard all resulting translation and mark translation unavailable.
            cut = min(end, max(context.time, accel_deadline))
            for segment_end in sorted({cut, end}):
                if segment_end <= context.time:
                    continue
                translation = (context.accel is not None and context.accel.valid
                               and not context.accel.clipped
                               and segment_end <= accel_deadline)
                force = np.zeros(3) if context.accel is None else np.asarray(context.accel.value)
                position, velocity = context.core.position.copy(), context.core.velocity.copy()
                context.core.predict(IMUSample(segment_end / _NS, np.asarray(context.gyro.value), force))
                if not translation:
                    context.core.position, context.core.velocity = position, velocity
                context.time = segment_end
        # Unknown gyro intervals are skipped; new samples never fill them backwards.
        context.time = target
        context.core._last_imu_time = target / _NS

    def _apply(self, context, event):
        _, _, packet = event
        if isinstance(packet, tuple):
            if packet[0] == 'flight_alignment':
                result = packet[1]
                quaternion = np.empty(4)
                mujoco.mju_mat2Quat(quaternion, result.rotation.flatten())
                context.core.quaternion_wxyz = quaternion
                context.core.position = np.asarray(result.external.value[0]).copy()
                context.core.velocity = np.asarray(result.external.value[1]).copy()
                context.external = result.external
                context.external_epoch = context.epoch
                context.alignment = 'IMU_EXTERNAL_VELOCITY_FLIGHT_ALIGNMENT'
                context.alignment_evidence_id = 'MEASURED_INERTIAL_VELOCITY_CONSISTENCY'
                context.alignment_time = context.time
                return
            _, force, evidence_id = packet
            fx, fy, fz = force
            roll = math.atan2(fy, fz)
            pitch = math.atan2(-fx, math.hypot(fy, fz))
            flat = np.empty(9)
            mujoco.mju_quat2Mat(flat, context.core.quaternion_wxyz)
            rotation = flat.reshape(3, 3)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            context.core.quaternion_wxyz = np.array([
                cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])
            context.alignment = 'MEASURED_GRAVITY_LOCAL_YAW_GAUGE_NOT_WORLD_HEADING'
            context.alignment_evidence_id = evidence_id
            context.alignment_time = context.time
            context.faults = ()
            context.epoch += 1
            context.barrier = max(context.barrier, context.time)
            return
        if packet.kind == 'impact':
            if packet.valid and not packet.clipped:
                context.epoch += 1
                context.barrier = max(context.barrier, packet.value[1])
            return
        if not packet.valid or packet.clipped:
            if packet.kind in ('gyro', 'accel'):
                setattr(context, packet.kind, packet)
                self._fault(context, packet.kind.upper() + ('_CLIPPED_LATCHED' if packet.clipped else '_INVALID_LATCHED'))
            return
        if packet.kind == 'gyro':
            context.gyro = packet
            context.core.angular_velocity_body = np.array(packet.value)
        elif packet.kind == 'accel':
            context.accel = packet
        elif (packet.kind == 'external' and context.time > context.barrier
              and self.config.external_frame_registration is not None):
            position, velocity = packet.value
            context.core.position += self.config.alpha_position * (np.asarray(position) - context.core.position)
            context.core.velocity += self.config.alpha_velocity * (np.asarray(velocity) - context.core.velocity)
            context.external = packet
            context.external_epoch = context.epoch
        if context.gyro is not None:
            # Only anchors the time; propagation itself is in _advance.
            context.core._last_imu_time = context.time / _NS

    def _recompute(self, now):
        if self._dirty is None:
            context = deepcopy(self._checkpoints[max(self._checkpoints)] if self._checkpoints else self._base)
            return context
        prior = [t for t in self._checkpoints if t < self._dirty]
        context = deepcopy(self._checkpoints[max(prior)] if prior else self._base)
        self._checkpoints = {t: c for t, c in self._checkpoints.items() if t < self._dirty}
        pending = [event for event in self._events if context.time <= event[0] <= now]
        # Base/checkpoints describe a fully applied group; initial t=0 is special.
        if prior or self._pruned:
            pending = [event for event in pending if event[0] > context.time]
        index = 0
        while index < len(pending):
            timestamp = pending[index][0]
            self._advance(context, timestamp)
            while index < len(pending) and pending[index][0] == timestamp:
                self._apply(context, pending[index])
                index += 1
            self._checkpoints[timestamp] = deepcopy(context)
            self.replayed_event_groups += 1
        self._dirty = None
        return context

    def estimate(self, now_s: float) -> dict:
        now = self._clock(now_s)
        self._now = now
        context = self._recompute(now)
        self._advance(context, now)
        self._latched_faults.update(context.faults)
        faults = tuple(sorted(self._latched_faults))
        aligned = context.alignment != 'NOT_OBSERVED_LOCAL_YAW_GAUGE'
        attitude_reasons = [reason for reason in faults if reason.startswith('GYRO_')]
        if not aligned:
            attitude_reasons.append('INITIAL_ALIGNMENT_NOT_OBSERVED')
        rate_reasons = []
        if context.gyro is None:
            rate_reasons.append('GYRO_MISSING')
        elif not context.gyro.valid or context.gyro.clipped:
            rate_reasons.append('GYRO_SAMPLE_INVALID_OR_CLIPPED')
        elif now - time_ns(context.gyro.acquisition_s) > time_ns(self.config.imu_gap_s):
            rate_reasons.append('GYRO_STALE')
        attitude_reasons += rate_reasons
        attitude_valid = not attitude_reasons
        translation_reasons = []
        if self.config.external_frame_registration is None:
            translation_reasons.append('EXTERNAL_FRAME_REGISTRATION_NOT_DECLARED')
        if context.external is None:
            translation_reasons.append('EXTERNAL_MEASUREMENT_MISSING')
        elif context.external_epoch != context.epoch:
            translation_reasons.append('POST_EVENT_EXTERNAL_REACQUISITION_REQUIRED')
        elif now - time_ns(context.external.acquisition_s) > time_ns(self.config.external_max_age_s):
            translation_reasons.append('EXTERNAL_MEASUREMENT_STALE')
        if not attitude_valid:
            translation_reasons.append('ATTITUDE_NOT_VALID_FOR_INERTIAL_PROPAGATION')
        translation_reasons += [reason for reason in faults if reason.startswith('ACCEL_')]
        if context.accel is None:
            translation_reasons.append('ACCEL_MISSING_FOR_INERTIAL_PROPAGATION')
        translation_valid = not translation_reasons
        flat = np.empty(9)
        mujoco.mju_quat2Mat(flat, context.core.quaternion_wxyz)

        def channel(packet, valid, reasons, frame):
            return {'acquisition_s': None if packet is None else packet.acquisition_s,
                    'delivery_s': None if packet is None else packet.delivery_s,
                    'represented_s': now / _NS, 'epoch_id': context.epoch,
                    'measurement_epoch_id': context.external_epoch if frame == 'local_navigation' else context.epoch,
                    'valid': valid, 'reasons': tuple(reasons), 'frame': frame,
                    'source': SYNTHETIC, 'trusted': False}

        channels = {
            'attitude': channel(context.gyro, attitude_valid, attitude_reasons, 'body_FLU_to_local_navigation'),
            'body_rate': channel(context.gyro, not rate_reasons, rate_reasons, 'body_FLU'),
            'position': channel(context.external, translation_valid, translation_reasons, 'local_navigation'),
            'velocity': channel(context.external, translation_valid, translation_reasons, 'local_navigation'),
        }
        channels['body_rate']['represented_s'] = (None if context.gyro is None
                                                    else context.gyro.acquisition_s)
        # Body rate is a raw sample channel, including flagged invalid samples.
        # Pair its value with that exact sample's clock, not the propagated core's
        # previous valid rate when a newer clipped/invalid packet was received.
        body_rate = (context.core.angular_velocity_body.copy() if context.gyro is None
                     else np.asarray(context.gyro.value))
        out = SensorEstimate(now/_NS, context.core.position.copy(), context.core.velocity.copy(),
                             flat.reshape(3, 3), body_rate,
                             context.epoch, channels, context.alignment,
                             self.config.external_frame_registration, faults)
        # Preserve a fully applied old checkpoint for bounded rewinds.
        cutoff = now - time_ns(self.config.history_s)
        old = [t for t in self._checkpoints if t <= cutoff]
        if old:
            boundary = max(old)
            self._base = deepcopy(self._checkpoints[boundary])
            self._pruned = True
            self._checkpoints = {t: c for t, c in self._checkpoints.items() if t > boundary}
            self._events = [event for event in self._events if event[0] > boundary]
            self._keys = {(event[0], event[2].kind) for event in self._events if isinstance(event[2], SensorPacket)}
        result = out.to_dict()
        result.update(time_s=now/_NS, state_time_s=now/_NS, generation=context.epoch,
                      world_alignment_observed=context.alignment == 'IMU_EXTERNAL_VELOCITY_FLIGHT_ALIGNMENT',
                      alignment_evidence_id=context.alignment_evidence_id,
                      history_start_s=self._base.time/_NS,
                      accepted_packets=self.accepted_packets,
                      rejected_packets=len(self.rejections))
        return result


# Short public name used by the independent offline runner.
SensorEstimator = SensorOnlyEstimator
