"""Estimate-only reference planning and CTBR feedback for the ASM2 experiment."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace

from math import factorial

import numpy as np
from scipy.signal import lfilter
from scipy.spatial.transform import Rotation, Slerp

from rm27_contact_physics.control import Septic, oriented_frame
from rm27_contact_physics.model import TargetState, VehicleEstimate, unit
from rm27_contact_physics.teacher_interface import RateThrust

GRAVITY = np.array([0., 0., 9.81])


@dataclass(frozen=True)
class Observation:
    vehicle: VehicleEstimate
    target: TargetState

    def predict_vehicle(self, when, max_age):
        dt = when - self.vehicle.timestamp
        if not 0 <= dt <= max_age:
            return None
        return replace(
            self.vehicle, position=self.vehicle.position + dt * self.vehicle.velocity,
            rotation=self.vehicle.rotation @ Rotation.from_rotvec(dt * self.vehicle.omega_body).as_matrix(),
            timestamp=when,
        )

    def predict(self, when, max_age):
        state = self.predict_vehicle(when, max_age)
        if state is None or not self.target.valid(when, max_age):
            return None
        return Observation(state, self.target)


class ObservationSource:
    """Explicit truth baseline or a sample-and-hold sensor surrogate with latency."""

    def __init__(self, case, config):
        self.case, self.config = case, config
        self.rng = np.random.default_rng(case['seed'])
        self.next_capture = 0
        self.queue = deque()
        self.latest = None
        self.latest_target = None

    def update(self, state, position, normal):
        now = state.timestamp
        if self.case['source'] == 'oracle':
            target = TargetState(position.copy(), normal.copy(), np.zeros((3, 3)), now, source='MUJOCO_ORACLE')
            return Observation(state, target)
        config = self.config
        if now + 1e-9 >= self.next_capture * config['period_s']:
            self.next_capture += 1
            outage = self.case.get('outage_s')
            publish_target = outage is None or not outage[0] <= now < outage[1]
            self.queue.append((now + config['latency_s'], self._sample(state, position, normal), publish_target))
        while self.queue and self.queue[0][0] <= now + 1e-9:
            _, sampled, publish_target = self.queue.popleft()
            if publish_target:
                self.latest_target = sampled.target
            if self.latest_target is not None:
                self.latest = Observation(sampled.vehicle, self.latest_target)
        return self.latest

    def _sample(self, state, position, normal):
        config, rng = self.config, self.rng

        def jitter(name, key):
            return config[name] + rng.normal(0., config[key], 3)

        rotation_error = Rotation.from_rotvec(np.deg2rad(jitter('attitude_bias_deg', 'attitude_noise_deg')))
        normal_error = Rotation.from_rotvec(np.deg2rad(jitter('target_normal_bias_deg', 'target_normal_noise_deg')))
        vehicle = VehicleEstimate(
            state.position + jitter('position_bias_m', 'position_noise_m'),
            state.velocity + jitter('velocity_bias_m_s', 'velocity_noise_m_s'),
            state.rotation @ rotation_error.as_matrix(),
            state.omega_body + rng.normal(0., config['rate_noise_rad_s'], 3),
            state.timestamp, 'SYNTHETIC_DELAYED',
        )
        target = TargetState(
            position + jitter('target_position_bias_m', 'target_position_noise_m'),
            normal_error.apply(normal), np.eye(3) * config['target_position_noise_m'] ** 2,
            state.timestamp, source='SYNTHETIC_DELAYED',
        )
        return Observation(vehicle, target)


def _blend_reference(old, new, fraction, duration):
    # Septic smoothstep has zero first through third derivatives at both joins.
    s = fraction
    w = s**4 * (35. - s*(84. - s*(70. - 20.*s)))
    dw = 140.*s**3*(1.-s)**3 / duration
    ddw = 420.*s**2*(1.-s)**2*(1.-2.*s) / duration**2
    dddw = 840.*s*(1.-s)*(1.-5.*s+5.*s*s) / duration**3
    p, v, a, j = new - old
    return old + np.asarray((w*p, w*v+dw*p, w*a+2*dw*v+ddw*p,
                             w*j+3*dw*a+3*ddw*v+dddw*p))


_SEPTIC_DERIVATIVES = tuple(
    np.asarray([factorial(k) / factorial(k-order) for k in range(order, 8)])
    for order in range(4)
)


@dataclass(frozen=True)
class Reference:
    trajectory: Septic
    start: float
    previous: Reference | None = None
    blend: float = 0.
    aligned: float = 0.
    contact_preview: dict | None = None
    blend_delay_s: float = 0.
    forward_segments: tuple | None = None
    forward_axis: np.ndarray | None = None
    vertical_segments: tuple | None = None

    def forward_values(self, times, segments=None):
        elapsed = np.asarray(times)-self.start
        first, second = self.forward_segments if segments is None else segments
        values = np.empty((4, len(elapsed)))
        for selected, segment, offset in ((elapsed <= first.duration, first, 0.),
                                          (elapsed > first.duration, second, first.duration)):
            t = np.clip((elapsed[selected]-offset)/segment.duration, 0., 1.)
            values[:, selected] = np.asarray([
                ((t[:, None] ** np.arange(8-order)*weights) @ segment.c[order:]).ravel()
                / segment.duration**order for order, weights in enumerate(_SEPTIC_DERIVATIVES)])
        tail = np.maximum(elapsed-first.duration-second.duration, 0.)
        values[0] += tail*values[1]+.5*tail**2*values[2]
        values[1] += tail*values[2]
        values[3, tail > 0.] = 0.
        return values

    @property
    def end(self):
        return self.start + self.trajectory.duration + self.aligned

    def at(self, now):
        if self.forward_segments is not None:
            return tuple(self.samples(np.asarray([now]))[:, 0])
        p, v, a, jerk = self.trajectory.sample(now - self.start)
        dt = now - self.start - self.trajectory.duration
        if dt > 0:
            return p + dt*v + .5*dt*dt*a, v + dt*a, a, np.zeros(3)
        if self.previous is None or now >= self.start + self.blend_delay_s + self.blend:
            return p, v, a, jerk
        if now <= self.start + self.blend_delay_s:
            return self.previous.at(now)
        return tuple(_blend_reference(np.asarray(self.previous.at(now)), np.asarray((p, v, a, jerk)),
                                      (now-self.start-self.blend_delay_s)/self.blend, self.blend))

    def samples(self, times):
        """Evaluate the same p/v/a/jerk contract for a planner time grid."""
        elapsed = np.asarray(times) - self.start
        duration = self.trajectory.duration
        t = np.clip(elapsed/duration, 0., 1.)
        values = np.asarray([
            (t[:, None] ** np.arange(8-order) * weights) @ self.trajectory.c[order:] / duration**order
            for order, weights in enumerate(_SEPTIC_DERIVATIVES)
        ])
        tail = np.maximum(elapsed-duration, 0.)[:, None]
        values[0] += tail*values[1] + .5*tail**2*values[2]
        values[1] += tail*values[2]
        values[3, elapsed > duration] = 0.
        if self.forward_segments is not None:
            longitudinal = self.forward_values(times)
            values += (longitudinal-values @ self.forward_axis)[:, :, None]*self.forward_axis
        if self.vertical_segments is not None:
            values[:, :, 2] = self.forward_values(times, self.vertical_segments)
        if self.previous is not None:
            blending = elapsed < self.blend + self.blend_delay_s
            if np.any(blending):
                s = np.clip((elapsed[blending, None]-self.blend_delay_s) / self.blend, 0., 1.)
                values[:, blending] = _blend_reference(
                    self.previous.samples(times[blending]), values[:, blending], s, self.blend)
        return values


@dataclass(frozen=True)
class ContactGeometry:
    """Static compiled support points in the same body frame as the estimate."""
    pads: np.ndarray
    airframe: np.ndarray
    halfwidth: np.ndarray

    def terminal(self, target, config):
        n = unit(target.normal)
        frame = oriented_frame(n, [1., 0., 0.])
        slope = -frame[:, 0]
        planning = config['contact_planning']
        aligned = planning.get('surface_alignment')
        if aligned is not None:
            return self.centered_terminal(target, config, frame, slope, aligned)
        rotation = frame @ Rotation.from_euler('y', planning['rear_pad_bias_deg'], degrees=True).as_matrix()
        offsets = self.pads @ rotation.T
        support = offsets @ n
        low = support.min()
        anchor = offsets[support <= low + .0001].mean(axis=0)
        anchor += (low - anchor @ n)*n
        position = (target.position - anchor + planning['slope_offset_m']*slope
                    - config['contact_reference_depth_m']*n)
        velocity = planning['slope_speed_m_s']*slope - config['contact_reference_speed_m_s']*n
        # Use thrust direction for attitude with zero normal acceleration.
        z = rotation[:, 2]
        acceleration = z * (GRAVITY @ n) / (z @ n) - GRAVITY
        return position, velocity, acceleration, frame

    def centered_terminal(self, target, config, frame, slope, aligned):
        """Specify the contact state and propagate through a small plane crossing."""
        n = frame[:, 2]
        depth = config['contact_reference_depth_m']
        closing = config['contact_reference_speed_m_s']
        normal_accel = aligned.get('normal_accel_m_s2', 0.)
        remaining_speed_squared = closing**2 - 2.*normal_accel*depth
        if depth < 0. or closing <= 0. or remaining_speed_squared <= 0.:
            raise ValueError('Centered reference requires a closing plane crossing')
        after_contact = 2.*depth / (closing + np.sqrt(remaining_speed_squared))
        # At the plane crossing, the local +Z axis passes through the surface
        # center and thrust determines attitude via a + g = thrust/mass * n.
        center = (target.position - self.pads[:, 2].min()*n
                  + aligned.get('center_slope_offset_m', 0.)*slope)
        velocity = config['contact_planning']['slope_speed_m_s']*slope - closing*n
        acceleration = (GRAVITY@n + normal_accel)*n - GRAVITY
        return (center + after_contact*velocity + .5*after_contact**2*acceleration,
                velocity + after_contact*acceleration, acceleration, frame)

    def surface_window(self, target, frame, times, pos, vel, rotation, rate, config, first):
        """Check the complete coplanar native contact footprint before arrival."""
        planning = config['contact_planning']['surface_alignment']
        lead = planning['full_surface_lead_s']
        sample = slice(int(np.searchsorted(times, times[first]-lead, side='right'))-1, first+1)
        # Bottom vertices span all of the convex native floor faces. Position
        # and velocity are affine over each face, so extrema occur at vertices.
        pads = self.pads[self.pads[:, 2] <= self.pads[:, 2].min()+1e-6]
        arm = pads @ rotation[sample].transpose(0, 2, 1)
        points = pos[sample, None, :] - target.position + arm
        velocity = vel[sample, None, :] + np.cross(rate[sample, None, :], arm)
        n = frame[:, 2]
        gap = points @ n
        span = np.ptp(gap, axis=1).max()
        closing = -(velocity @ n)
        tangent = np.linalg.norm(velocity @ frame[:, :2], axis=-1)
        if (span > planning['maximum_gap_span_m']
                or np.any(np.abs(points @ frame[:, :2]) > self.halfwidth)
                or closing.min() < 0. or closing.max() > config['contact_max_normal_speed_m_s']
                or tangent.max() > config['contact_max_tangent_speed_m_s']):
            return None
        axis = rotation[first, :, 2]
        origin = pos[first] - target.position
        axis_crossing = origin - (origin@n)/(axis@n)*axis
        return dict(full_surface_window_s=float(times[first]-times[sample.start]),
                    full_surface_maximum_gap_span_m=float(span),
                    full_surface_maximum_tangent_speed_m_s=float(tangent.max()),
                    full_surface_peak_angular_rate_rad_s=float(np.linalg.norm(rate[sample], axis=-1).max()),
                    projected_axis_offset_m=float(np.linalg.norm(axis_crossing)))

    def preview(self, target, frame, times, pos, vel, rotation, rate, config, *, release=None):
        n = frame[:, 2]
        axes = frame[:, :2]
        offset = pos - target.position
        plane_distance = offset @ n
        local_normals = rotation.transpose(0, 2, 1) @ n
        pad_gaps = local_normals @ self.pads.T + plane_distance[:, None]
        lowest_pad = pad_gaps.min(axis=1)
        # Require the desired pad to lead the full aircraft at the plane.
        if np.any((local_normals @ self.airframe.T + plane_distance[:, None]).min(axis=1)
                  <= np.minimum(lowest_pad, 0.)):
            return None
        hit = np.flatnonzero(lowest_pad <= 0.)
        if not len(hit):
            return None
        first = hit[0]
        near = np.flatnonzero((lowest_pad <= config['contact_planning']['contact_lead_clearance_m'])
                              & (np.arange(len(times)) <= first))
        support_mask = pad_gaps[near] <= lowest_pad[near, None] + .00015
        indices, points = np.nonzero(support_mask)
        indices = near[indices]
        lever = np.einsum('nij,nj->ni', rotation[indices], self.pads[points])
        position = offset[indices] + lever
        velocity = vel[indices] + np.cross(rate[indices], lever)
        closing = -(velocity @ n)
        tangent = np.linalg.norm(velocity @ axes, axis=1)
        attitude = np.degrees(np.arccos(np.clip((np.einsum('ij,nij->n', frame, rotation[near])-1.)/2., -1., 1.)))
        if (np.any(np.abs(position @ axes) > self.halfwidth)
                or closing.min() < 0. or closing.max() > config['contact_max_normal_speed_m_s']
                or tangent.max() > config['contact_max_tangent_speed_m_s']
                or attitude.max() > config['contact_attitude_error_deg']):
            return None
        k = int(np.argmin(pad_gaps[first]))
        lever = rotation[first] @ self.pads[k]
        contact_position = pos[first] + lever
        contact_velocity = vel[first] + np.cross(rate[first], lever)
        preview = dict(time_s=float(times[first]), position_m=contact_position.tolist(),
                       velocity_m_s=contact_velocity.tolist(), normal_speed_m_s=float(-contact_velocity @ n),
                       tangent_speed_m_s=float(np.linalg.norm(contact_velocity @ axes)),
                       earliest_window_s=float(times[near[0]]), maximum_window_tangent_m_s=float(tangent.max()),
                       maximum_window_attitude_deg=float(attitude.max()))
        if 'footprint_reserve' in config:
            from .asm2_recoverability import footprint_reserve
            reserve = footprint_reserve(self, target, frame, pos[first], vel[first],
                                        rotation[first], rate[first], config['footprint_reserve'])
            if not reserve['accepted']:
                return None
            preview['footprint_reserve'] = reserve
        aligned = config['contact_planning'].get('surface_alignment')
        if aligned is not None and 'full_surface_lead_s' in aligned:
            if times[first]-times[0] < aligned['full_surface_lead_s']:
                return None
            surface = self.surface_window(target, frame, times, pos, vel, rotation, rate, config, first)
            if surface is None:
                return None
            preview.update(surface)
        if release is not None:
            # The qualification check above uses every nearby support vertex.
            # For ranking, sample the extremal pitch and roll lever arms at
            # each time. This bounds the online work even on coplanar pads.
            axes_body = np.array([[1., 0., 0.], [-1., 0., 0.], [0., 1., 0.], [0., -1., 0.]])
            arms = np.cross(self.pads[None, :, :], local_normals[near, None, :])
            arm_extrema = np.argmax(np.where(support_mask[:, :, None], arms @ axes_body.T, -np.inf), axis=1)
            release_times = np.repeat(near, len(axes_body))
            preview.update(self.release_preview(local_normals[release_times], vel[release_times] @ n,
                                                rotation[release_times], rate[release_times],
                                                arm_extrema.ravel(), release,
                                                horizon_s=config.get('release_preview_horizon_s')))
        return preview

    def release_preview(self, normal_body, normal_speed, rotation, rate, points, tracker, *, horizon_s=None):
        """Rank possible touchdown states with a short plane clearance projection.

        Compare constant-twist coast and an instantaneous normal-only plastic
        impulse. These are projections, without compliant contact, friction,
        or neighboring surfaces.
        """
        # For a contact arm r, (r × n) · omega is the rotational normal speed.
        omega = (rotation.transpose(0, 2, 1) @ rate[:, :, None]).squeeze(-1)
        arms = np.cross(self.pads[points], normal_body)
        closing = np.maximum(0., -normal_speed - np.sum(arms*omega, axis=1))
        inv_inertia_arm = arms / tracker.inertia
        impulse = closing / (1./tracker.mass + np.sum(arms*inv_inertia_arm, axis=1))
        # Translate the predicted pose to the pad plane before forecasting its
        # relative shell gap. This is a ranker, never contact qualification.
        pad_depth = np.sum(self.pads[points]*normal_body, axis=1)[:, None]
        before = normal_body @ self.airframe.T - pad_depth
        horizon = tracker.dt if horizon_s is None else horizon_s
        if not np.isfinite(horizon) or horizon <= 0.:
            raise ValueError('Release projection horizon must be positive')
        rotor_hold = (normal_body + horizon*np.cross(normal_body, omega)) @ self.airframe.T
        rotor_hold += horizon*normal_speed[:, None] - pad_depth
        post_omega = omega + impulse[:, None]*inv_inertia_arm
        post = (normal_body + horizon*np.cross(normal_body, post_omega)) @ self.airframe.T
        post += horizon*(normal_speed+impulse/tracker.mass)[:, None] - pad_depth
        return dict(release_projection_horizon_s=float(horizon), minimum_shell_gap_at_touch_m=float(before.min()),
                    minimum_shell_gap_coast_m=float(rotor_hold.min()),
                    minimum_shell_gap_after_normal_impulse_m=float(post.min()),
                    maximum_pad_closing_m_s=float(closing.max()),
                    maximum_rate_after_normal_impulse_rad_s=float(np.linalg.norm(post_omega, axis=1).max()))


def nominal_start(target, config):
    slope = oriented_frame(target.normal, [1., 0., 0.])[:, 0]
    return (
        target.position + config['handover_standoff_m'] * target.normal + config['handover_slope_offset_m'] * slope,
        -config['handover_speed_m_s'] * target.normal + config['handover_slope_speed_m_s'] * slope,
    )


def plan_reference(obs, config, geometry, tracker, previous=None, *, activation=None, diagnostics=None):
    """Search feasible approach profiles, optionally ranking their release state."""
    profiles = config.get('release_profiles')
    if not profiles:
        return _plan_profile(obs, config, geometry, tracker, previous, activation, diagnostics=diagnostics)
    candidates = []
    for profile in profiles:
        planning = config['contact_planning']
        parameters = {
            **config,
            'contact_reference_speed_m_s': profile['normal_speed_m_s'],
            'contact_reference_depth_m': profile['depth_m'],
            'contact_alignment_s': profile.get('alignment_s', config['contact_alignment_s']),
            'plan_durations_s': profile.get('durations_s', config['plan_durations_s']),
            'contact_planning': {
                **planning,
                'slope_offset_m': profile.get('slope_offset_m', planning['slope_offset_m']),
                'slope_speed_m_s': profile.get('slope_speed_m_s', planning['slope_speed_m_s']),
                'rear_pad_bias_deg': profile.get('pitch_bias_deg', planning['rear_pad_bias_deg']),
                **({'surface_alignment': profile['surface_alignment']}
                   if 'surface_alignment' in profile else {}),
            },
        }
        reference = _plan_profile(obs, parameters, geometry, tracker, previous, activation,
                                  release=None if 'footprint_reserve' in parameters else tracker,
                                  diagnostics=diagnostics, profile_id=profile['id'])
        if reference is not None:
            preview = {**reference.contact_preview, 'release_profile': profile['id']}
            candidates.append(replace(reference, contact_preview=preview))
    if not candidates:
        return None

    def clearance(candidate):
        preview = candidate.contact_preview
        if 'footprint_reserve' in preview:
            return preview['footprint_reserve']['minimum_reserve_m']
        return min(preview['minimum_shell_gap_coast_m'], preview['minimum_shell_gap_after_normal_impulse_m'])

    return max(candidates, key=clearance)


def coordinate_braking(candidate, start, end, normal, config):
    """Join longitudinal braking and vertical state with the same polynomial clock."""
    p0, v0, a0, j0 = start
    braking = config['forward_braking_s']
    first_duration = candidate.trajectory.duration-braking
    if first_duration <= 0. or np.linalg.norm(a0)+np.linalg.norm(j0) > 1e-8:
        raise ValueError('FORWARD_BRAKING_BOUNDARY')
    axis = -unit(np.r_[normal[:2], 0.])
    start_x, initial_speed = float(p0@axis), float(v0@axis)
    end_x, end_speed, end_acceleration, end_jerk = [float(value@axis) for value in end]
    middle_speed = (2*(end_x-start_x)-initial_speed*first_duration
        -end_speed*braking+end_acceleration*braking**2/5)/candidate.trajectory.duration
    middle_x = start_x+.5*(initial_speed+middle_speed)*first_duration
    middle = np.asarray([middle_x, middle_speed, 0., 0.])+np.asarray(
        config.get('braking_forward_waypoint', [0., 0., 0., 0.]))
    boundary = lambda values: tuple(np.asarray([value]) for value in values)
    candidate = replace(candidate, forward_axis=axis, forward_segments=(
        Septic(boundary((start_x, initial_speed, 0., 0.)),
               boundary(middle), first_duration),
        Septic(boundary(middle),
               boundary((end_x, end_speed, end_acceleration, end_jerk)), braking)))
    vertical = config.get('braking_vertical_waypoint')
    if vertical is not None:
        mid = np.asarray(candidate.trajectory.sample(first_duration))[:, 2]+np.asarray(vertical)
        candidate = replace(candidate, vertical_segments=(
            Septic(boundary([value[2] for value in start]), boundary(mid), first_duration),
            Septic(boundary(mid), boundary([value[2] for value in end]), braking)))
    return candidate


def _plan_profile(obs, config, geometry, tracker, previous, activation, *, release=None,
                  diagnostics=None, profile_id='default'):
    state, target = obs.vehicle, obs.target
    if activation is None and config.get('check_initial_rate_demand', False):
        activation = tracker.expected_activation
    zeros = np.zeros(3)
    p0, v0 = (nominal_start(target, config)
              if previous is None and not config.get('initialize_reference_from_estimate', False)
              else (state.position, state.velocity))
    a0, j0 = (zeros, zeros) if previous is None else previous.at(state.timestamp)[2:]
    endpoint, end_velocity, end_accel, frame = geometry.terminal(target, config)
    alignment = config['contact_alignment_s']
    end = (endpoint - alignment*end_velocity + .5*alignment**2*end_accel,
           end_velocity - alignment*end_accel, end_accel, zeros)
    durations = config['plan_durations_s']
    blends = (0.,)
    if previous is not None:
        remaining = previous.end - state.timestamp
        durations = ([remaining + offset for offset in (0., .10, .20, .30)]
                     if previous.contact_preview is not None else durations)
        blends = config['plan_blend_s']
    for duration in durations:
        for blend in blends:
            attempt = dict(profile=profile_id, duration_s=float(duration), blend_s=float(blend),
                           rejection='BLEND_WINDOW', feasible=False)
            if diagnostics is not None:
                diagnostics.append(attempt)
            if duration-alignment <= blend or (previous is not None and previous.contact_preview is not None and remaining-previous.aligned < blend):
                continue
            candidate = Reference(Septic((p0, v0, a0, j0), end, duration-alignment),
                                  state.timestamp, previous, blend, alignment,
                                  blend_delay_s=tracker.dt if previous is not None and config.get('command_continuous_splice') else 0.)
            braking = config.get('forward_braking_s')
            if braking is not None:
                try:
                    candidate = coordinate_braking(candidate, (p0, v0, a0, j0), end, target.normal, config)
                except ValueError:
                    attempt['rejection'] = 'FORWARD_BRAKING_BOUNDARY'
                    continue
                axis = candidate.forward_axis
            ticks = int(np.ceil(duration / tracker.dt))
            times = state.timestamp + np.linspace(0., duration, ticks+1)
            pos, vel, acc, jerk = candidate.samples(times)
            if braking is not None and (np.max(acc@axis) > .01 or
                    np.max(vel@axis-np.minimum.accumulate(vel@axis)) > .001):
                attempt.update(maximum_forward_acceleration_m_s2=float(np.max(acc@axis)),
                               intermediate_forward_speed_m_s=float(candidate.forward_segments[1].c[1, 0]
                                   / candidate.forward_segments[1].duration))
                attempt['rejection'] = 'FORWARD_REACCELERATION'
                continue
            if config.get('guidance'):
                lower, upper = np.asarray(config['flight_box_relative_m'])+target.position
                if np.any(pos < lower) or np.any(pos > upper):
                    attempt['rejection'] = 'FLIGHT_ENVELOPE'
                    continue
            force = acc + GRAVITY
            tilt = np.degrees(np.arctan2(np.linalg.norm(force[:, :2], axis=1), force[:, 2]))
            if (np.max(np.linalg.norm(vel, axis=1)) > .85 * config['maximum_speed_m_s']
                    or np.max(tilt) > config['maximum_tilt_deg']-5. or np.min(force[:, 2]) <= 3.):
                attempt['rejection'] = 'SPEED_TILT_OR_VERTICAL_FORCE'
                continue
            rotation, rate = _reference_attitude_and_rate(acc, jerk)
            demand = tracker.reference_demand(times, force, rotation, rate, state, activation)
            if demand is None:
                attempt['rejection'] = 'RATE_OR_MOTOR_AUTHORITY'
                continue
            if previous is None and config.get('check_initial_rate_demand', False):
                # Use the unchanged tracker on a copy: include attitude correction
                # and its first motor-lag step, not only nominal angular feedforward.
                from copy import deepcopy
                probe = deepcopy(tracker)
                if activation is not None:
                    probe.expected_activation = np.asarray(activation).copy()
                _, _, saturated = probe.command(state, candidate, state.timestamp)
                if saturated:
                    attempt['rejection'] = 'INITIAL_TRACKER_SATURATION'
                    continue
            preview = geometry.preview(target, frame, times, pos, vel, rotation, rate, config, release=release)
            if preview is not None:
                attempt.update(rejection=None, feasible=True)
                return replace(candidate, contact_preview={**preview, **demand})
            attempt['rejection'] = 'CONTACT_GEOMETRY_OR_RESERVE'
    return None


def _requested_force(state, ref):
    pos, vel, acc, _ = ref
    requested = acc + 6.*(pos-state.position) + 4.*(vel-state.velocity) + GRAVITY
    requested[2] = np.clip(requested[2], 4., 16.)
    horizontal = np.linalg.norm(requested[:2])
    if horizontal > requested[2] * .8:
        requested[:2] *= requested[2] * .8 / horizontal
    return requested


def feedback(state, ref, mass, max_collective, config):
    """Rate-proxy feedback retained for separation and recovery."""
    requested = _requested_force(state, ref)
    jerk = ref[3]
    desired = oriented_frame(requested, [1., 0., 0.])
    b3 = desired[:, 2]
    relative = state.rotation.T @ desired
    error = Rotation.from_matrix(relative).as_rotvec()
    ff = state.rotation.T @ np.cross(b3, jerk / np.linalg.norm(requested))
    max_ff = config['controller_jerk_feedforward_limit_rad_s']
    raw_rates = 7. * error + np.clip(ff, -max_ff, max_ff)
    rate_limit = config['controller_rate_limit_rad_s']
    thrust = np.clip(mass * requested @ state.rotation[:, 2], 0., max_collective)
    return RateThrust(np.clip(raw_rates, -rate_limit, rate_limit), float(thrust)), bool(
        np.any(np.abs(raw_rates) > rate_limit)
    )


def _reference_attitude_and_rate(acceleration, jerk):
    """Differentiate the thrust-aligned frame in world axes, optionally batched."""
    force = acceleration + GRAVITY
    length = np.linalg.norm(force, axis=-1, keepdims=True)
    z = force / length
    y_raw = np.cross(z, [1., 0., 0.])
    y_length = np.linalg.norm(y_raw, axis=-1, keepdims=True)
    y = y_raw / y_length
    x = np.cross(y, z)
    z_dot = (jerk - z * np.sum(z*jerk, axis=-1, keepdims=True)) / length
    y_raw_dot = np.cross(z_dot, [1., 0., 0.])
    y_dot = (y_raw_dot - y*np.sum(y*y_raw_dot, axis=-1, keepdims=True)) / y_length
    x_dot = np.cross(y_dot, z) + np.cross(y, z_dot)
    rate = .5 * (np.cross(x, x_dot) + np.cross(y, y_dot) + np.cross(z, z_dot))
    return np.stack((x, y, z), axis=-1), rate


def _reference_angular_velocity(acceleration, jerk):
    return _reference_attitude_and_rate(acceleration, jerk)[1]


def _limit_norm(vector, limit):
    norm = np.linalg.norm(vector)
    return vector * (limit / norm) if norm > limit else vector


class ApproachTracker:
    """CTBR attitude tracking with rate acceleration feedforward and bounded motor lead."""

    def __init__(self, metadata, config):
        self.mass = metadata['mass_kg']
        self.maximum = metadata['max_rotor_thrust_N']
        self.inertia = np.diag(metadata['inertia_body_kg_m2']).copy()
        self.inverse_mixer = np.linalg.inv(metadata['mixer_collective_and_body_torque_per_rotor_N'])
        self.rate_gain = metadata['rate_gain_per_s']
        self.dt = config['control_dt_s']
        self.rate_limit = .95 * config['maximum_rate_rad_s']
        self.correction_limit = config['controller_rate_limit_rad_s']
        self.reference_rate = np.zeros(3)
        self.reference_rate_step = self.dt * self.rate_gain * self.rate_limit
        tau = np.asarray(metadata['motor_time_constant_s'])
        self.activation_decay = np.exp(-self.dt / tau)
        # Invert the predicted motor state over three control ticks, avoiding
        # the much larger command jump of a one-tick inverse.
        self.motor_horizon = 3. * self.dt
        self.inverse_horizon_weight = 1. / -np.expm1(-self.motor_horizon / tau)
        self.expected_activation = np.asarray(metadata['hover_motor_thrust_N']).copy()

    def activation_at(self, lead, held_command):
        """Forecast from command history, using the command held at request time."""
        decay = self.activation_decay ** (lead / self.dt)
        return decay*self.expected_activation + (1.-decay)*held_command

    def reference_demand(self, times, force, rotation, rate, state, activation):
        """Bound angular and nominal rotor demand under the existing tracker model."""
        world_alpha = np.diff(rate, axis=0) / np.diff(times)[:, None]
        peak_rate = np.linalg.norm(rate, axis=1).max()
        peak_alpha = np.linalg.norm(world_alpha, axis=1).max()
        if peak_rate > self.rate_limit or peak_alpha > self.rate_gain*self.rate_limit:
            return None
        body_from_world = rotation[:-1].transpose(0, 2, 1)
        omega = (body_from_world @ rate[:-1, :, None]).squeeze(-1)
        alpha = (body_from_world @ world_alpha[:, :, None]).squeeze(-1)
        thrust = self.mass * np.linalg.norm(force[:-1], axis=1)
        torque = alpha*self.inertia + np.cross(omega, omega*self.inertia)
        if activation is not None:
            # Reserve the first tick for correcting measured rate mismatch.
            actual = state.omega_body
            desired = state.rotation.T @ rate[0]
            angular_acceleration = state.rotation.T @ world_alpha[0] + self.rate_gain*(desired-actual)
            torque[0] = angular_acceleration*self.inertia + np.cross(actual, actual*self.inertia)
            thrust[0] = self.mass * (force[0] @ state.rotation[:, 2])
        desired = np.column_stack((thrust, torque)) @ self.inverse_mixer.T
        last = self.expected_activation if activation is None else activation
        # In the unsaturated controller, the lead command and first-order
        # activation update reduce to a linear filter on desired rotor forces.
        gains = np.broadcast_to((1.-self.activation_decay)*self.inverse_horizon_weight, last.shape)
        after = np.column_stack([
            lfilter([gain], [1., gain-1.], desired[:, i], zi=[(1.-gain)*last[i]])[0]
            for i, gain in enumerate(gains)
        ])
        prior = np.vstack((last, after[:-1]))
        command = prior + self.inverse_horizon_weight * (desired-prior)
        if command.min() < 0. or command.max() > self.maximum:
            return None
        return dict(peak_reference_rate_rad_s=float(peak_rate), peak_reference_alpha_rad_s2=float(peak_alpha),
                    minimum_predicted_motor_command_N=float(command.min()),
                    maximum_predicted_motor_command_N=float(command.max()))

    def command(self, state, reference, now):
        ref = reference.at(now)
        desired_force = _requested_force(state, ref)
        desired = oriented_frame(desired_force, [1., 0., 0.])
        attitude_error = Rotation.from_matrix(state.rotation.T @ desired).as_rotvec()
        # Bound the response even when the commanded reference leaves its
        # predicted angular-rate envelope.
        requested_rate = _limit_norm(_reference_angular_velocity(*ref[2:]), self.rate_limit)
        self.reference_rate += _limit_norm(requested_rate - self.reference_rate, self.reference_rate_step)
        world_rate = self.reference_rate
        requested_future = _limit_norm(
            _reference_angular_velocity(*reference.at(now + self.dt)[2:]), self.rate_limit)
        future_rate = world_rate + _limit_norm(requested_future - world_rate, self.reference_rate_step)
        nominal_rate = state.rotation.T @ world_rate
        raw_correction = 7. * attitude_error
        rate_correction = _limit_norm(raw_correction, self.correction_limit)
        raw_rate = nominal_rate + rate_correction
        target_rate = _limit_norm(raw_rate, self.rate_limit)
        self.rate_limited = bool(np.any(raw_correction != rate_correction) or np.any(raw_rate != target_rate))
        omega = state.omega_body
        alpha_ff = state.rotation.T @ ((future_rate - world_rate) / self.dt) - np.cross(omega, nominal_rate)
        # Attenuate feedforward if the reference rate cannot fit the active
        # rate limit. The physical envelope still checks the actual rate.
        alpha = self.rate_gain * (target_rate - omega) + alpha_ff
        predicted_rate = _limit_norm(omega + self.motor_horizon * alpha, self.rate_limit)
        alpha = (predicted_rate - omega) / self.motor_horizon
        thrust = float(np.clip(self.mass * (desired_force @ state.rotation[:, 2]), 0., 4. * self.maximum))
        torque = self.inertia * alpha + np.cross(omega, self.inertia * omega)
        force = self.inverse_mixer @ np.r_[thrust, torque]
        command, clipped = self.motor_lead(force)
        return RateThrust(target_rate, thrust), command, clipped

    def motor_lead(self, force):
        raw = self.expected_activation + self.inverse_horizon_weight * (force - self.expected_activation)
        command = np.clip(raw, 0., self.maximum)
        self.expected_activation += (1. - self.activation_decay) * (command - self.expected_activation)
        return command, bool(np.any(raw != command))

    def recovery_motor_command(self, state, action):
        omega = state.omega_body
        torque = self.inertia*self.rate_gain*(action.body_rates_flu_rad_s-omega)+np.cross(omega, self.inertia*omega)
        desired = self.inverse_mixer @ np.r_[action.collective_force_N, torque]
        return self.motor_lead(desired)


def recovery_reference(state, normal, since, initial_velocity):
    """Use velocity first, without a post-contact position anchor."""
    retreat = np.r_[normal[:2], 0.]
    retreat /= np.linalg.norm(retreat)
    desired_velocity = .30 * retreat * np.clip((1.20 - since) / .45, 0., 1.)
    desired_velocity[2] = .08 * np.clip((.85 - since) / .35, 0., 1.)
    fraction = np.clip(since / .30, 0., 1.)
    weight = fraction**2 * (3. - 2.*fraction)
    desired_velocity = initial_velocity + weight*(desired_velocity-initial_velocity)
    return state.position, desired_velocity, np.zeros(3), np.zeros(3)


def scheduled_recovery_reference(state, normal, elapsed, initial_velocity, normal_accel_m_s2=1.2):
    position, velocity, acceleration, jerk = recovery_reference(state, normal, elapsed, initial_velocity)
    fraction = np.clip((elapsed-.025)/.065, 0., 1.)
    weight = fraction**2*(3.-2.*fraction)
    normal_acceleration = normal*(9.81*normal[2]+normal_accel_m_s2)-[0., 0., 9.81]
    return (position, state.velocity+weight*(velocity-state.velocity),
            (1.-weight)*normal_acceleration+weight*acceleration, jerk)


def leveling_clearance(state, center, normal, geometry, destination=None):
    """Conservative fixed-COM rotation clearance from the registered plane."""
    upright = (oriented_frame([0., 0., 1.], state.rotation[:, 0])
               if destination is None else destination)
    rotations = Rotation.from_matrix(np.asarray([state.rotation, upright]))
    arc = (rotations[0].inv()*rotations[1]).magnitude()
    samples = Slerp([0., 1.], rotations)(np.linspace(0., 1., 17)).as_matrix()
    vertices = np.concatenate((geometry.airframe, geometry.pads))
    radius = np.max(np.linalg.norm(vertices, axis=1))
    offsets = np.einsum('ij,tkj->tik', vertices, samples)
    gap = np.min((offsets+state.position-center)@normal)-radius*arc/32.
    return float(gap)


def clearance_limited_recovery(state, center, normal, geometry, elapsed, initial_velocity,
                               previous_weight, dt, required_gap, normal_accel=1.2):
    release = scheduled_recovery_reference(state, normal, .025, initial_velocity, normal_accel)
    recovery = recovery_reference(state, normal, elapsed, initial_velocity)
    maximum = min(1., previous_weight+dt/.065)
    selected, selected_gap = 0., None
    desired = release
    for weight in np.linspace(maximum, 0., 11):
        candidate = tuple(a+weight*(b-a) for a, b in zip(release, recovery))
        destination = oriented_frame(_requested_force(state, candidate), [1., 0., 0.])
        gap = leveling_clearance(state, center, normal, geometry, destination)
        if gap >= required_gap and state.velocity@normal >= 0.:
            selected, selected_gap, desired = float(weight), gap, candidate
            break
    return desired, selected, dict(blend_weight=selected, requested_maximum_weight=maximum,
        estimated_rotation_gap_m=selected_gap, required_gap_m=required_gap,
        admissible_rotation=selected_gap is not None, normal_release_fallback=selected_gap is None,
        source='SYNTHETIC_ESTIMATE_FIXED_COM_ARC_NOT_FUTURE_CLEARANCE_PROOF')
