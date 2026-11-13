"""Event-aware initialization for the independent low-energy recovery fixture.

This optional reference retains the existing recovery feedback, normal-release
acceleration, target schedule and time scales. It does not estimate velocity or
read physics truth. Acquisition, delivery and reference adoption have distinct
clocks. Existing recovery entry points retain their original behavior.
"""
from __future__ import annotations

import numpy as np

from rm27_non_rl.asm2_contact_control import GRAVITY

EPS = 1e-9


class RecoveryInitializationError(ValueError):
    """The reference cannot be supported by the supplied timestamped inputs."""

    def __init__(self, reason):
        self.reason = reason
        super().__init__(reason)


def _smoothstep(elapsed, duration):
    fraction = np.clip(elapsed / duration, 0., 1.)
    return float(fraction * fraction * (3. - 2. * fraction))


class EventAwareRecovery:
    """Use bounded normal release until an applicable velocity sample arrives.

    ``state.velocity`` must be the delivered external velocity sample, as in
    RecoverySensors. The estimate query time is not its acquisition time. A
    sample acquired inside the event interval has ambiguous applicability.
    ``event_reference_s`` retains the existing retreat target schedule. Only
    the velocity-initialization clock starts at the actual adoption time.
    """

    def __init__(self, normal, *, event_lower_s, event_upper_s,
                 event_reference_s, hold_s, blend_s, translation_max_age_s,
                 event_source='DECLARED_EVENT_INTERVAL'):
        normal = np.asarray(normal, dtype=float)
        numbers = [event_lower_s, event_upper_s, event_reference_s,
                   hold_s, blend_s, translation_max_age_s]
        if (not np.isfinite(np.asarray(numbers, dtype=float)).all() or event_lower_s < 0.
                or not event_lower_s <= event_reference_s <= event_upper_s
                or hold_s < 0. or blend_s <= 0. or translation_max_age_s <= 0.):
            raise ValueError('Invalid event interval or inherited recovery timing')
        if (normal.shape != (3,) or not np.isfinite(normal).all()
                or not np.isclose(np.linalg.norm(normal), 1., atol=1e-8, rtol=0.)
                or np.linalg.norm(normal[:2]) < EPS):
            raise ValueError('Recovery requires a unit normal with a horizontal component')
        self.normal = normal.copy()
        self.event_lower_s = float(event_lower_s)
        self.event_upper_s = float(event_upper_s)
        self.event_reference_s = float(event_reference_s)
        self.event_source = event_source
        self.hold_s = float(hold_s)
        self.blend_s = float(blend_s)
        self.translation_max_age_s = float(translation_max_age_s)
        self.provisional_deadline_s = self.event_upper_s + self.translation_max_age_s
        self._release_acceleration = normal * (GRAVITY @ normal + 1.2) - GRAVITY
        self._retreat = np.r_[normal[:2], 0.]
        self._retreat /= np.linalg.norm(self._retreat)
        self._anchor = None
        self._adopted_at = self._anchor_measurement_at = self._anchor_delivery_at = None
        self._blend_start = None
        self._last_observation_at = None
        self._last_sample = None
        self._transitions = []

    def _validate_state(self, state):
        if (not np.isfinite(state.timestamp) or state.timestamp < self.event_upper_s - EPS
                or not all(np.isfinite(value).all() for value in
                           (state.position, state.velocity, state.rotation, state.omega_body))):
            raise RecoveryInitializationError('INVALID_STATE_OR_EVENT_TIME')

    def observe(self, state, *, measurement_time_s, delivery_time_s,
                translation_valid=True, source='EXTERNAL_TRANSLATION'):
        """Adopt the first strictly post-event delivered velocity, without backdating.

        A fresh post-event sample at the provisional deadline is accepted before
        evaluating timeout. Subsequent samples continue through normal feedback;
        they do not repeatedly restart the finite initialization envelope.
        """
        self._validate_state(state)
        now = float(state.timestamp)
        if self._last_observation_at is not None and now < self._last_observation_at - EPS:
            raise RecoveryInitializationError('NONMONOTONIC_CONTROL_TIME')
        if not translation_valid:
            raise RecoveryInitializationError('TRANSLATION_UNAVAILABLE')
        if (measurement_time_s is None or delivery_time_s is None
                or not np.isfinite([measurement_time_s, delivery_time_s]).all()):
            raise RecoveryInitializationError('TRANSLATION_TIMESTAMPS_UNAVAILABLE')
        acquired, delivered = float(measurement_time_s), float(delivery_time_s)
        if not -EPS <= acquired <= delivered + EPS or delivered > now + EPS:
            raise RecoveryInitializationError('TRANSLATION_NOT_YET_DELIVERED_OR_INVALID_CLOCK')
        age = now - acquired
        if not -EPS <= age <= self.translation_max_age_s + EPS:
            raise RecoveryInitializationError('TRANSLATION_STALE_OR_FUTURE')
        if not isinstance(source, str) or not source:
            raise RecoveryInitializationError('TRANSLATION_SOURCE_UNAVAILABLE')
        if acquired < self.event_lower_s - EPS:
            applicability = 'PRE_EVENT'
        elif acquired <= self.event_upper_s + EPS:
            applicability = 'AMBIGUOUS_EVENT_INTERVAL'
        else:
            applicability = 'POST_EVENT'
        self._last_observation_at = now
        self._last_sample = dict(acquisition_s=acquired, delivery_s=delivered,
                                 age_s=max(0., age), age_valid=True, source=source,
                                 event_applicability=applicability)
        if self._anchor is None and now > self.provisional_deadline_s + EPS:
            raise RecoveryInitializationError('POST_EVENT_TRANSLATION_UNAVAILABLE')
        if self._anchor is None and applicability == 'POST_EVENT':
            self._anchor = np.asarray(state.velocity, dtype=float).copy()
            self._anchor_measurement_at, self._anchor_delivery_at = acquired, delivered
            self._adopted_at = now
            self._blend_start = max(self.event_reference_s + self.hold_s, now)
            self._transitions.append(dict(
                adopted_at_s=now, sample_acquisition_s=acquired, sample_delivery_s=delivered,
                source=source, velocity_m_s=self._anchor.tolist(),
                interpretation='Delayed post-event measured velocity adopted now; no claim of propagation to now',
            ))
        if self._anchor is None and now >= self.provisional_deadline_s - EPS:
            raise RecoveryInitializationError('POST_EVENT_TRANSLATION_UNAVAILABLE')
        return self.status(now)

    def reference(self, state):
        """Return the original four-part reference without changing controller gains."""
        self._validate_state(state)
        now = float(state.timestamp)
        if self._anchor is None:
            if now >= self.provisional_deadline_s - EPS:
                raise RecoveryInitializationError('POST_EVENT_TRANSLATION_UNAVAILABLE')
            # Equal p/v cancel translation feedback. This is an explicit model
            # release command, not a claim that either velocity is post-event.
            return (state.position.copy(), state.velocity.copy(),
                    self._release_acceleration.copy(), np.zeros(3))
        if now < self._adopted_at - EPS:
            raise RecoveryInitializationError('BACKDATED_REFERENCE_QUERY')
        event_age = max(0., now - self.event_reference_s)
        # Preserve the frozen recovery_reference retreat schedule. These are
        # its existing target values and durations, not new fitted parameters.
        target = .30 * self._retreat * np.clip((1.20 - event_age) / .45, 0., 1.)
        target[2] = .08 * np.clip((.85 - event_age) / .35, 0., 1.)
        initialization = _smoothstep(now - self._adopted_at, .30)
        base_velocity = self._anchor + initialization * (target - self._anchor)
        blend = _smoothstep(now - self._blend_start, self.blend_s)
        return (state.position.copy(),
                state.velocity + blend * (base_velocity - state.velocity),
                (1. - blend) * self._release_acceleration,
                np.zeros(3))

    def status(self, now):
        """Expose event applicability separately from ordinary data age."""
        sample = None if self._last_sample is None else dict(self._last_sample)
        if sample is not None:
            sample['age_s'] = float(now - sample['acquisition_s'])
            sample['age_valid'] = -EPS <= sample['age_s'] <= self.translation_max_age_s + EPS
        return dict(
            phase='MODEL_NORMAL_RELEASE' if self._anchor is None else 'POST_EVENT_INITIALIZED',
            event_interval_s=[self.event_lower_s, self.event_upper_s],
            event_reference_s=self.event_reference_s, event_source=self.event_source,
            provisional_deadline_s=self.provisional_deadline_s,
            provisional_model='Existing normal-release acceleration; translation feedback cancelled',
            latest_translation=sample,
            anchor_velocity_m_s=None if self._anchor is None else self._anchor.tolist(),
            anchor_acquisition_s=self._anchor_measurement_at,
            anchor_delivery_s=self._anchor_delivery_at,
            anchor_adoption_s=self._adopted_at, blend_start_s=self._blend_start,
            anchor_role='Reference value from delayed measurement, not a current-state velocity estimate',
            velocity_initialization_end_s=None if self._adopted_at is None else self._adopted_at + .30,
            transitions=list(self._transitions),
        )
