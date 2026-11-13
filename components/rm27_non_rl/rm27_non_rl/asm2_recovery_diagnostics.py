"""Offline event classification and recovery-command timing diagnostics.

These helpers analyze saved sensor streams. They neither change a detector
configuration nor send commands to a simulator or an aircraft.
"""
from __future__ import annotations

import math

import numpy as np

from rm27_contact_physics.recovery import ImpactDetector


def evaluate(stream, candidate):
    # Match the frozen 142 g fixture and its 2.5 ms event transport.
    detector = ImpactDetector(**candidate)
    for row in stream['samples']:
        detector.update(row[0], row[4:7], stream['held_thrust_N'], .142)
        if detector.detected_at is not None:
            break
    acquired = detector.detected_at
    first = stream['first_contact_s']
    false = acquired is not None and (first is None or acquired < first - 1e-9)
    detected = (first is not None and acquired is not None and not false
                and acquired <= stream['deadline_s'] + 1e-9)
    return dict(
        id=stream['id'], kind=stream['kind'], split=stream['split'],
        acquired_s=acquired,
        delivery_s=None if acquired is None else acquired + .0025,
        false_alarm=bool(false), detected=bool(detected),
        miss=bool(first is not None and not detected),
        latency_s=None if not detected else acquired - first,
        delivery_latency_s=None if not detected else acquired + .0025 - first,
        peak_residual_g=float(stream['samples'][:, 7].max()),
    )


def next_control_tick(timestamp, period_s=.005, time_tolerance_s=.000001):
    """Match native-trial 5ms grid eligibility using its dt/100 tolerance.

The caller must distinguish an unavailable channel from a timestamp. Returning
this tick only claims temporal eligibility, without estimator/actuator validity.
 """
    if timestamp is None:
        return None
    if (not np.isfinite(timestamp) or timestamp < 0
            or not np.isfinite([period_s, time_tolerance_s]).all()
            or period_s <= 0 or not 0 <= time_tolerance_s < period_s):
        raise ValueError('Invalid executable-command clock input')
    return max(1, math.ceil((timestamp - time_tolerance_s) / period_s)) * period_s


def command_specific_force(sample_times_s, command_times_s, collective_commands_N, *,
                           initial_collective_N, mass_kg, motor_tau_s,
                           accel_config, physics_dt_s=.0001):
    """Predict filtered COM specific force using commands, never actuator truth.

    Total thrust follows the registered identical first-order motor model.
    World acceleration R e_z T/m - g becomes body specific force e_z T/m
    after adding gravity and applying R.T. Thus attitude cancels exactly for
    this zero-drag COM model. No claim covers displaced IMUs, thrust-axis
    uncertainty, aerodynamics or motor-model mismatch.

    The initial model state is declared at the first sample. Commands earlier
    than that sample are rejected because startup history cannot be inferred.
    Sample intervals must span at least one native physics interval.
    Samples correspond to endpoint times of the existing synthetic instrument.
    Expected thrust is averaged over the preceding native interval, matching
    its velocity-difference construction, then passed through the existing
    internal and PX4 filters without adding instrument noise.
    """
    from rm27_contact_physics.px4_rate import LowPass2p

    times = np.asarray(sample_times_s, dtype=float)
    updates = np.asarray(command_times_s, dtype=float)
    commands = np.asarray(collective_commands_N, dtype=float)
    scalars = [initial_collective_N, mass_kg, motor_tau_s, physics_dt_s]
    if (times.ndim != 1 or updates.ndim != 1 or commands.shape != updates.shape
            or not len(times) or not np.isfinite(times).all()
            or not np.isfinite(updates).all() or not np.isfinite(commands).all()
            or not np.isfinite(scalars).all() or np.any(np.diff(times) <= 0)
            or np.any(np.diff(times) < physics_dt_s - 1e-12)
            or np.any(np.diff(updates) <= 0) or times[0] < 0
            or np.any(updates < times[0]) or min(scalars[1:]) <= 0
            or initial_collective_N < 0 or np.any(commands < 0)):
        raise ValueError('Invalid timestamped collective command model')
    accel = accel_config['accel']
    internal = LowPass2p(accel['odr_hz'], accel['internal_cutoff_hz'])
    px4 = LowPass2p(accel['odr_hz'], accel_config['px4_accel_cutoff_hz'])
    steady = np.array([0., 0., initial_collective_N / mass_kg])
    internal.reset(steady)
    px4.reset(steady)
    initial_clock = times[0]
    clock = initial_clock
    activation = float(initial_collective_N)
    held = float(initial_collective_N)
    index = 0
    output = []
    for now in times:
        # The scalar analytic solution is exact between recorded command times.
        # A query at t uses the force averaged on (t-dt,t], not a future command.
        left = max(initial_clock, now - physics_dt_s)
        probes = [left, now]
        interval_average = activation
        for probe_index, endpoint in enumerate(probes):
            integral = 0.
            interval_start = clock
            while index < len(updates) and updates[index] <= endpoint + 1e-12:
                change = max(clock, float(updates[index]))
                duration = change - clock
                decay = np.exp(-duration / motor_tau_s)
                integral += held * duration + (activation - held) * motor_tau_s * (1. - decay)
                activation = held + (activation - held) * decay
                clock = change
                held = float(commands[index])
                index += 1
            duration = endpoint - clock
            decay = np.exp(-duration / motor_tau_s)
            integral += held * duration + (activation - held) * motor_tau_s * (1. - decay)
            activation = held + (activation - held) * decay
            clock = endpoint
            if probe_index == 1:
                interval_average = (activation if endpoint == interval_start else
                                    integral / (endpoint - interval_start))
        force = np.array([0., 0., interval_average / mass_kg])
        register = internal.apply(force)
        filtered = px4.apply(register)
        output.append([*force, *register, *filtered])
    return np.asarray(output)
