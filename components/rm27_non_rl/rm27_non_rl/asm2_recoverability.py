"""Finite-footprint screening and offline native planning-model forecasts.

The forecast has its own plant, initialized ONLY from an estimated state and
command-derived motor activation. Its perfect predicted-state feedback and
predicted contact event are optimistic model assumptions, not sensor evidence.
It is deliberately not called inside the 30 ms online solver deadline.
"""
from dataclasses import replace
from pathlib import Path
import time

import mujoco
from .geometry import _compiled_contact_geometry
import numpy as np
from scipy.spatial.transform import Rotation


def footprint_reserve(geometry, target, frame, position, velocity, rotation, world_rate, options):
    """Screen all pad vertices under a declared short coast/stress envelope.

    Continuous extrema of each quadratic coordinate are checked, including
    stationary points. Rotation is constant-twist to first order; its truncation
    is bounded using the lever length. This is NOT a friction/contact prediction
    or a certificate for surrounding geometry; those require native rollouts.
    """
    horizon = float(options['horizon_s'])
    angle = np.deg2rad(float(options['attitude_stress_deg']))
    p_error = float(options['position_stress_m'])
    v_error = float(options.get('velocity_stress_m_s', 0.))
    if not np.all(np.isfinite([horizon, angle, p_error, v_error])) or min(horizon, angle, p_error, v_error) < 0. or angle > np.pi:
        raise ValueError('Invalid footprint stress assumption')
    arms = geometry.pads @ rotation.T
    coordinates = (position-target.position+arms) @ frame[:, :2]
    speed = (velocity+np.cross(world_rate, arms)) @ frame[:, :2]
    # Gravity tangent remains even when thrust is normal to the plate.
    acceleration = np.array([0., 0., -9.81]) @ frame[:, :2]
    endpoint = coordinates + horizon*speed + .5*horizon**2*acceleration
    extrema = np.maximum(np.abs(coordinates), np.abs(endpoint))
    for axis in range(2):
        if abs(acceleration[axis]) > 1e-12:
            at = np.clip(-speed[:, axis]/acceleration[axis], 0., horizon)
            extrema[:, axis] = np.maximum(extrema[:, axis], np.abs(
                coordinates[:, axis]+at*speed[:, axis]+.5*at**2*acceleration[axis]))
    radius = np.linalg.norm(geometry.pads, axis=1)
    stress = p_error + 2.*radius*np.sin(angle/2.) + horizon*v_error
    linearization = .5*radius*(np.linalg.norm(world_rate)*horizon)**2
    margin = geometry.halfwidth - extrema - (stress+linearization)[:, None]
    return dict(accepted=bool(np.min(margin) >= 0.), minimum_reserve_m=float(np.min(margin)),
                axis_reserve_m=margin.min(axis=0).tolist(), **options,
                confidence='ASSUMED_STRESS_NOT_CALIBRATED',
                model='TANGENTIAL_GRAVITY_COAST; no contact/friction or finite-body guarantee')


def native_forecast(model_path, metadata, mapping, config, observation, reference, tracker,
                    held_command, output, *, response_latency_s=.09, post_contact_s=.25,
                    wall_limit_s=45.):
    """Bounded *offline* forecast, independent of any live simulator state.

    Full native integration/actuator history is retained from estimated handover
    across first contact. There is no reset at contact. It can reject a candidate
    but cannot qualify sensor-driven recovery or real-time execution.
    """
    from copy import deepcopy
    from rm27_sim.simulation import asm2 as native
    from .asm2_contact_control import feedback, scheduled_recovery_reference
    from .asm2_recovery_initialization import EventAwareRecovery
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    import shutil
    sources = [Path(__file__), Path(__file__).with_name('asm2_contact_control.py'),
               Path(native.__file__), Path(__file__).with_name('geometry.py'),
               Path(__file__).with_name('asm2_recovery_initialization.py')]
    (output/'sources').mkdir()
    for path in sources:
        shutil.copy2(path, output/'sources'/path.name)
    native.write_json(output/'provenance.json', {str(p):native.digest(p) for p in sources})
    native.write_json(output/'config.json',config)
    (output/'model.mjb').symlink_to(Path(model_path).resolve())
    if observation.vehicle.source == 'MUJOCO_ORACLE':
        raise ValueError('Forecast requires an explicitly estimated handover')
    model = mujoco.MjModel.from_binary_path(str(model_path))
    plant = native.Plant(model, metadata, {})
    state = observation.vehicle
    plant.reset(state.position, state.velocity)
    plant.data.time = state.timestamp
    plant.data.qpos[plant.qadr+3:plant.qadr+7] = Rotation.from_matrix(state.rotation).as_quat(scalar_first=True)
    plant.data.qvel[plant.dadr+3:plant.dadr+6] = state.omega_body
    plant.data.act[plant.activation] = tracker.expected_activation
    plant.data.ctrl[plant.motors] = held_command
    mujoco.mj_forward(model, plant.data)
    tracker = deepcopy(tracker)
    scorer = native.ContactScorer(plant, mapping, metadata, config)
    first = forbidden = recovery = None
    release_velocity = None
    release_command_s = None
    scheduled_release_s = (reference.contact_preview[config.get('scheduled_release_reference', 'time_s')]
                           + config.get('scheduled_release_offset_s', 0.)
                           if config.get('scheduled_separation') else None)
    motor_lead_s = (float(np.mean(metadata['motor_time_constant_s']))
                    if config.get('separation_motor_lead') else 0.)
    records, states = [], [plant.snapshot()]
    commands = [plant.data.ctrl[plant.motors].copy()]
    start = time.perf_counter()
    failure = None
    limits = np.array(config['flight_box_relative_m'])+scorer.center
    maximum_time = min(state.timestamp+4., reference.end+post_contact_s)
    saturated = 0
    while plant.data.time < maximum_time:
        if time.perf_counter()-start > wall_limit_s:
            failure = 'FORECAST_WALL_LIMIT'
            break
        predicted = replace(plant.state(), source='PLANNING_MODEL_PREDICTION')
        if scheduled_release_s is not None and predicted.timestamp+1e-9 >= scheduled_release_s-motor_lead_s:
            if release_velocity is None:
                release_velocity = predicted.velocity.copy()
                release_command_s = predicted.timestamp
            desired = scheduled_recovery_reference(predicted, scorer.normal,
                max(0., predicted.timestamp-scheduled_release_s), release_velocity)
            action, _ = feedback(predicted, desired, metadata['mass_kg'], 4*metadata['max_rotor_thrust_N'], config)
            saturated += plant.command(action, predicted.omega_body)
        elif scheduled_release_s is None and first is not None and predicted.timestamp >= first['time_s']+response_latency_s:
            if recovery is None:
                event_time = first['time_s']+response_latency_s
                recovery = EventAwareRecovery(scorer.normal, event_lower_s=first['time_s'],
                    event_upper_s=event_time, event_reference_s=event_time,
                    hold_s=.025, blend_s=.065, translation_max_age_s=.05,
                    event_source='PREDICTED_MODEL_CONTACT_NOT_A_MEASUREMENT')
            recovery.observe(predicted, measurement_time_s=predicted.timestamp,
                             delivery_time_s=predicted.timestamp, source='PLANNING_MODEL_PREDICTION')
            action, _ = feedback(predicted, recovery.reference(predicted), metadata['mass_kg'],
                                 4*metadata['max_rotor_thrust_N'], config)
            saturated += plant.command(action, predicted.omega_body)
        else:
            action, motors, clipped = tracker.command(predicted, reference, predicted.timestamp)
            plant.data.ctrl[plant.motors] = motors
            saturated += clipped
        for _ in range(round(config['control_dt_s']/model.opt.timestep)):
            before = plant.state()
            mujoco.mj_step(model, plant.data)
            contacts = scorer.active(plant.data, before)
            if contacts:
                entry = dict(time_s=float(plant.data.time), records=contacts)
                records.append(entry)
                if first is None:
                    first = {**entry, 'qualified': all(not c['rejected'] for c in contacts)}
                    maximum_time = min(maximum_time, first['time_s']+post_contact_s)
                if any('WRONG_AIRCRAFT_GEOM' in c['rejected'] or 'WRONG_SURFACE' in c['rejected'] for c in contacts):
                    forbidden = forbidden or entry
            current = plant.state()
            if (np.linalg.norm(current.velocity) > config['maximum_speed_m_s']
                or np.linalg.norm(current.omega_body) > config['maximum_rate_rad_s']
                or current.rotation[2,2] < np.cos(np.deg2rad(config['maximum_tilt_deg']))
                or np.any(current.position < limits[0]) or np.any(current.position > limits[1])):
                failure = 'FLIGHT_ENVELOPE'
            if forbidden is not None or failure:
                break
        states.append(plant.snapshot()); commands.append(plant.data.ctrl[plant.motors].copy())
        if forbidden is not None or failure:
            break
    result = dict(first_contact=first, forbidden=forbidden, failure=failure,
        duration_s=float(plant.data.time-state.timestamp), wall_s=time.perf_counter()-start,
        saturated_updates=int(saturated),
        response_latency_assumption_s=response_latency_s if scheduled_release_s is None else None,
        estimated_initial_state=state.source, initial_activation_source='COMMAND_MODEL_NOT_TELEMETRY',
        controller_state_source='COPIED_APPROACH_TRACKER',
        recovery_state_source='OPTIMISTIC_PLANNING_MODEL; NOT_SENSOR_RECOVERY',
        recovery_branch='REFERENCE_SCHEDULE' if scheduled_release_s is not None else 'PREDICTED_CONTACT_EVENT',
        scheduled_release_s=scheduled_release_s, first_release_command_s=release_command_s,
        clean_prefix=first is not None and first['qualified'] and forbidden is None and failure is None,
        model_sha256=native.digest(model_path), post_contact_horizon_s=post_contact_s)
    native.write_json(output/'summary.json', result)
    native.write_json(output/'contacts.json', records)
    np.savez_compressed(output/'forecast.npz', integration=np.asarray(states), motors=np.asarray(commands))
    return result
