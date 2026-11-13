"""Optional, estimate-only capability supervisor for offline R2.2 validation.

This module never estimates a missing state or emits flight commands. Independent
integrity reports are supplied by the caller; an identifier is not a calibration
algorithm. Synthetic inputs remain diagnostic even when marked valid.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite

import numpy as np

from rm27_non_rl.asm2_recovery_initialization import EPS as REFERENCE_EPS


class Source(str, Enum):
    GROUND_TRUTH = 'GROUND_TRUTH'
    SYNTHETIC = 'TRUTH_GENERATED_SYNTHETIC'
    RECONSTRUCTED = 'RECONSTRUCTED_SYNTHETIC'
    REAL_SENSOR = 'REAL_SENSOR'
    VIO = 'IMPLEMENTED_VIO'
    MODEL = 'DECLARED_MODEL'


class EstimatorMode(str, Enum):
    NORMAL = 'NORMAL_ESTIMATION'
    SUSPECTED = 'IMPACT_SUSPECTED'
    TRANSITION = 'POST_IMPACT_TRANSITION'
    ATTITUDE_ONLY = 'ATTITUDE_ONLY'
    REACQUIRE = 'TRANSLATION_REACQUISITION'
    VALIDATION = 'ESTIMATE_VALIDATION'
    TRUSTED = 'TRUSTED_ESTIMATION'


class RecoveryMode(str, Enum):
    RESPONSE = 'CONTACT_RESPONSE'
    SEPARATION = 'SEPARATION'
    ATTITUDE = 'ATTITUDE_STABILIZATION'
    VERTICAL = 'VERTICAL_RECOVERY'
    BRAKING = 'VELOCITY_BRAKING'
    HOLD = 'POSITION_HOLD'


# Frames are deliberately explicit; a body vector cannot silently become world.
_SCHEMA = {
    'attitude': ((3, 3), 'body_FLU_to_world'),
    'body_rate': ((3,), 'body_FLU'),
    'gravity_body': ((3,), 'body_FLU'),
    'position': ((3,), 'world'),
    'velocity': ((3,), 'world'),
    'surface_normal': ((3,), 'world'),
    'vertical_velocity': ((), 'world_up'),
    'position_target': ((3,), 'world'),
    **{name: ((), 'predicate') for name in
       ('safe_contact_response', 'separated', 'attitude_stable',
        'vertical_clearance', 'vertical_stable', 'velocity_stable')},
}
_TRANSLATION = {'position', 'velocity', 'vertical_velocity', 'position_target'}
_ALIGNMENT = {'attitude', 'gravity_body'}
_PREDICATES = {name for name, (_, frame) in _SCHEMA.items() if frame == 'predicate'}


@dataclass(frozen=True)
class StateSample:
    quantity: str
    value: object
    acquisition_s: float
    delivery_s: float
    state_time_s: float
    epoch_id: int
    frame: str
    source: Source
    max_age_s: float | None
    valid: bool = True
    propagated: bool = False
    clipped: bool = False
    gap: bool = False
    integrity_id: str | None = None
    continuity_id: str | None = None
    alignment_id: str | None = None
    propagation_method: str | None = None
    propagation_integrity_id: str | None = None


@dataclass(frozen=True)
class PropagationApproval:
    """Caller-reviewed propagation report for an exact acquisition/state interval.

    This scope is separate from measurement integrity. Its presence asserts an
    external review, not an implemented estimator or invented confidence value.
    """
    evidence_id: str
    method: str
    quantity: str
    frame: str
    epoch_id: int
    source: Source
    acquisition_s: float
    state_time_s: float


@dataclass(frozen=True)
class ImpactEvent:
    event_id: str
    lower_s: float
    upper_s: float
    delivery_s: float
    source: Source
    confirmed: bool = False
    integrity_id: str | None = None


@dataclass(frozen=True)
class EventApproval:
    """Independent contact-confirmation report, scoped to immutable event clocks."""
    evidence_id: str
    event_id: str
    lower_s: float
    upper_s: float
    delivery_s: float
    source: Source


@dataclass(frozen=True)
class Capability:
    usable: bool
    trusted: bool
    reasons: tuple[str, ...]
    age_s: float | None
    propagated: bool
    integrity_id: str | None
    measurement_trusted: bool
    propagation_trusted: bool


@dataclass(frozen=True)
class Decision:
    epoch_id: int
    estimator_mode: EstimatorMode
    concurrent_estimator_modes: tuple[EstimatorMode, ...]
    recovery_modes: tuple[RecoveryMode, ...]
    selected_recovery_mode: RecoveryMode | None
    brief_normal_release_eligible: bool
    capabilities: dict[str, Capability]
    blocked_by: tuple[str, ...]
    transition: tuple[str | None, str]
    # An eligible architecture stage does not implement its control algorithm.
    emits_commands: bool = False


class CapabilitySupervisor:
    """Two coordinated mode axes, with explicit event/reset epochs.

    Approved integrity identifiers must refer to independently reviewed reports
    scoped to the channel, frame, epoch and post-impact conditions. This caller
    boundary is intentional: the supervisor cannot validate an external report.
    No durations, gains or empirical tolerances are selected here.
    """

    def __init__(self, *, approved_integrity=frozenset(), approved_propagation=frozenset(),
                 approved_events=frozenset()):
        # Keys bind evidence to quantity/frame/epoch, preventing reuse by name.
        self.approved_integrity = frozenset(approved_integrity)
        self.approved_propagation = frozenset(approved_propagation)
        self.approved_events = frozenset(approved_events)
        self.epoch_id = 0
        self.event = None
        self.cutoff_s = None
        self.alignment_reset_required = False
        self.failure = None
        self._clock = None
        self._mode = None
        self._event_ids = set()
        self.reset_reason = None

    def _time(self, now):
        if (not isfinite(now) or now < 0
                or self._clock is not None and now < self._clock):
            raise ValueError('Nonfinite or nonmonotonic supervisor time')
        self._clock = now

    def impact(self, event, *, now):
        self._time(now)
        if (not event.event_id or not isinstance(event.source, Source)
                or event.source in (Source.GROUND_TRUTH, Source.MODEL)
                or not all(isfinite(x) for x in (event.lower_s, event.upper_s, event.delivery_s))
                or not 0 <= event.lower_s <= event.upper_s <= event.delivery_s <= now):
            raise ValueError('Invalid or privileged impact evidence')
        if self.event is not None and event.event_id == self.event.event_id:
            if ((event.lower_s, event.upper_s, event.delivery_s, event.source)
                    != (self.event.lower_s, self.event.upper_s,
                        self.event.delivery_s, self.event.source)
                    or self.event.confirmed and not event.confirmed):
                raise ValueError('Repeated event cannot rewrite its clock or confirmation')
            self.event = event
            return
        if self.cutoff_s is not None and event.upper_s < self.cutoff_s:
            raise ValueError('Cannot insert a historical event into a later reset epoch')
        if event.event_id in self._event_ids:
            raise ValueError('Historical event cannot reopen an epoch')
        self._event_ids.add(event.event_id)
        self.event = event
        self.epoch_id += 1
        self.cutoff_s = event.upper_s

    def reset(self, *, now, reason):
        """Explicit discontinuity/gap reset; does not silently realign attitude."""
        self._time(now)
        if not reason:
            raise ValueError('Reset requires a reason')
        self.epoch_id += 1
        self.cutoff_s = now
        self.alignment_reset_required = True
        self.reset_reason = reason

    def stop(self, *, now, reason):
        """Preserve an existing safety failure across subsequent transitions."""
        self._time(now)
        if not reason:
            raise ValueError('Stop requires a reason')
        self.failure = self.failure or reason

    def _approved(self, sample, evidence_id):
        return (evidence_id is not None and
                (evidence_id, sample.quantity, sample.frame, self.epoch_id, sample.source)
                in self.approved_integrity)

    def _event_trusted(self):
        event = self.event
        return (event is not None and event.confirmed
                and event.source in (Source.REAL_SENSOR, Source.VIO)
                and bool(event.integrity_id)
                and EventApproval(event.integrity_id, event.event_id, event.lower_s,
                                  event.upper_s, event.delivery_s, event.source)
                in self.approved_events)

    def _propagation_approved(self, sample):
        if not sample.propagation_method or not sample.propagation_integrity_id:
            return False
        scope = PropagationApproval(
            sample.propagation_integrity_id, sample.propagation_method,
            sample.quantity, sample.frame, self.epoch_id, sample.source,
            sample.acquisition_s, sample.state_time_s)
        return scope in self.approved_propagation

    def _capability(self, sample, now):
        reasons = []
        shape, frame = _SCHEMA[sample.quantity]
        value = np.asarray(sample.value, dtype=float)
        if value.shape != shape or not np.isfinite(value).all():
            reasons.append('INVALID_VALUE')
        elif sample.quantity == 'attitude':
            if (not np.allclose(value.T @ value, np.eye(3), atol=1e-6, rtol=0.)
                    or not np.isclose(np.linalg.det(value), 1., atol=1e-6, rtol=0.)):
                reasons.append('INVALID_ROTATION')
        elif sample.quantity in ('surface_normal', 'gravity_body'):
            if not np.isclose(np.linalg.norm(value), 1., atol=1e-6, rtol=0.):
                reasons.append('INVALID_UNIT_DIRECTION')
        elif sample.quantity in _PREDICATES and value not in (0., 1.):
            reasons.append('INVALID_PREDICATE')
        clock = (sample.acquisition_s, sample.delivery_s, sample.state_time_s, sample.max_age_s)
        if (not all(x is not None and isfinite(x) for x in clock) or sample.max_age_s <= 0
                or not 0 <= sample.acquisition_s <= sample.delivery_s <= now
                or not sample.acquisition_s <= sample.state_time_s <= now):
            reasons.append('INVALID_OR_UNDELIVERED_CLOCK')
        age = now - sample.acquisition_s
        if (not isfinite(age) or age < 0 or sample.max_age_s is None
                or age > sample.max_age_s):
            reasons.append('STALE_OR_FUTURE_MEASUREMENT')
        if not sample.propagated and sample.state_time_s != sample.acquisition_s:
            reasons.append('PROPAGATION_NOT_DECLARED')
        if sample.frame != frame:
            reasons.append('FRAME_MISMATCH')
        if not isinstance(sample.source, Source) or sample.source == Source.GROUND_TRUTH:
            reasons.append('PRIVILEGED_OR_UNKNOWN_SOURCE')
        if sample.epoch_id != self.epoch_id:
            reasons.append('ESTIMATOR_EPOCH_MISMATCH')
        if not sample.valid or sample.clipped or sample.gap:
            reasons.append('CHANNEL_FAULT')
        if self.cutoff_s is not None and sample.acquisition_s <= self.cutoff_s:
            # Translation may never be rescued by relabelling its generation.
            if (sample.quantity in _TRANSLATION or sample.quantity not in _ALIGNMENT
                    or not self._approved(sample, sample.continuity_id)):
                reasons.append('PRE_EVENT_OR_AMBIGUOUS_ACQUISITION')
        if (self.alignment_reset_required and sample.quantity in _ALIGNMENT
                and not self._approved(sample, sample.alignment_id)):
            reasons.append('ALIGNMENT_REACQUISITION_REQUIRED')
        usable = not reasons
        measurement_trusted = (usable and sample.source in (Source.REAL_SENSOR, Source.VIO)
                               and self._approved(sample, sample.integrity_id))
        requires_propagation = sample.state_time_s != sample.acquisition_s
        propagation_trusted = (not requires_propagation or
                               measurement_trusted and self._propagation_approved(sample))
        trusted = measurement_trusted and propagation_trusted
        if usable and not measurement_trusted:
            reasons.append('INTEGRITY_UNSUPPORTED')
        if usable and requires_propagation and not propagation_trusted:
            reasons.append('PROPAGATION_INTEGRITY_UNSUPPORTED')
        return Capability(usable, trusted, tuple(reasons), age if isfinite(age) else None,
                          sample.propagated, sample.integrity_id,
                          measurement_trusted, propagation_trusted)

    def evaluate(self, *, now, samples, actuator_available=False, simulation_execution=False):
        self._time(now)
        if len({s.quantity for s in samples}) != len(samples):
            raise ValueError('Duplicate channel quantities')
        if any(s.quantity not in _SCHEMA for s in samples):
            raise ValueError('Unknown state quantity')
        caps = {s.quantity: self._capability(s, now) for s in samples}
        usable = lambda *names: all(n in caps and caps[n].usable for n in names)
        trusted = lambda *names: all(n in caps and caps[n].trusted for n in names)
        values = {s.quantity: np.asarray(s.value) for s in samples}
        predicate = lambda name: trusted(name) and bool(values[name])
        rotation_ok = usable('attitude', 'body_rate')
        translation_count = sum(usable(n) for n in ('position', 'velocity'))
        full = rotation_ok and translation_count == 2
        current = lambda *names: (trusted(*names) and all(
            abs(next(s.state_time_s for s in samples if s.quantity == name) - now) <= 1e-9
            for name in names))
        full_trusted = current('attitude', 'body_rate', 'position', 'velocity')
        if self.event is not None and not self._event_trusted():
            mode = EstimatorMode.SUSPECTED
        elif full_trusted:
            mode = EstimatorMode.TRUSTED
        elif self.event is None and self.cutoff_s is None and full:
            mode = EstimatorMode.NORMAL
        elif full:
            mode = EstimatorMode.VALIDATION
        elif rotation_ok and translation_count == 0:
            mode = EstimatorMode.ATTITUDE_ONLY
        elif rotation_ok:
            mode = EstimatorMode.REACQUIRE
        else:
            mode = EstimatorMode.TRANSITION
        concurrent = {mode}
        if rotation_ok and not full_trusted:
            concurrent.add(EstimatorMode.REACQUIRE)
        if full and not full_trusted:
            concurrent.add(EstimatorMode.VALIDATION)
        if rotation_ok:
            concurrent.add(EstimatorMode.ATTITUDE_ONLY)
        if simulation_execution:
            eligible = lambda *names: all(
                name in caps and caps[name].usable
                and next(sample.source for sample in samples if sample.quantity == name) == Source.SYNTHETIC
                for name in names)
            current = lambda *names: eligible(*names) and all(
                abs(next(sample.state_time_s for sample in samples if sample.quantity == name)-now) <= 1e-9
                for name in names)
            predicate = lambda name: eligible(name) and bool(values[name])
        else:
            eligible = trusted
        event_eligible = self._event_trusted() or (
            simulation_execution and self.event is not None and self.event.source == Source.SYNTHETIC)
        full_eligible = current('attitude', 'body_rate', 'position', 'velocity')
        enabled = set()
        if self.failure is None and actuator_available:
            if event_eligible:
                if predicate('safe_contact_response'):
                    enabled.add(RecoveryMode.RESPONSE)
                if current('attitude', 'body_rate', 'velocity') and eligible('surface_normal'):
                    enabled.add(RecoveryMode.SEPARATION)
            if current('body_rate') and (current('gravity_body') or current('attitude')):
                enabled.add(RecoveryMode.ATTITUDE)
            if (current('attitude', 'body_rate', 'vertical_velocity')
                    and predicate('vertical_clearance')):
                enabled.add(RecoveryMode.VERTICAL)
            if current('attitude', 'body_rate', 'velocity') and predicate('attitude_stable'):
                enabled.add(RecoveryMode.BRAKING)
            if (full_eligible and current('position_target')
                    and predicate('attitude_stable') and predicate('velocity_stable')
                    and predicate('vertical_clearance')):
                enabled.add(RecoveryMode.HOLD)
        # Priority expresses current capability, not compulsory stage sequencing.
        priority = (RecoveryMode.HOLD, RecoveryMode.BRAKING, RecoveryMode.VERTICAL,
                    RecoveryMode.SEPARATION, RecoveryMode.ATTITUDE, RecoveryMode.RESPONSE)
        selected = next((stage for stage in priority if stage in enabled), None)
        missing = [name for name in ('attitude', 'body_rate', 'position', 'velocity', 'surface_normal')
                   if not trusted(name)]
        blocked = tuple(['UNTRUSTED_OR_MISSING:' + name for name in missing]
                        + ([] if actuator_available else ['ACTUATOR_UNAVAILABLE'])
                        + ([] if self.failure is None else ['LATCHED_SAFETY_FAILURE:' + self.failure]))
        reference_normal_ok = (eligible('surface_normal') and
                               np.linalg.norm(values['surface_normal'][:2]) >= REFERENCE_EPS)
        bnr = (self.failure is None and actuator_available and full_eligible
               and reference_normal_ok and event_eligible)
        if simulation_execution:
            blocked += ('SIMULATION_ONLY_UNCALIBRATED_EXECUTION',)
        if self.event is not None and not self._event_trusted():
            blocked += ('EVENT_INTEGRITY_UNSUPPORTED',)
        if trusted('surface_normal') and not reference_normal_ok:
            blocked += ('BNR_REFERENCE_REQUIRES_HORIZONTAL_NORMAL_COMPONENT',)
        if trusted('attitude', 'body_rate', 'position', 'velocity') and not full_trusted:
            blocked += ('STATE_EPOCHS_NOT_ALIGNED_TO_CONTROL_TIME',)
        previous = None if self._mode is None else self._mode.value
        self._mode = mode
        return Decision(self.epoch_id, mode, tuple(sorted(concurrent, key=lambda x: x.value)),
                        tuple(stage for stage in priority if stage in enabled), selected,
                        bnr, caps, blocked, (previous, mode.value))
