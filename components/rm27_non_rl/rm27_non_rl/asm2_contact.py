"""Offline ASM2 controlled-contact and recovery comparison on native field geometry."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from rm27_sim.integration.field import current_field, expanded_root
from rm27_sim.integration.field_subset import select_subset
from rm27_sim.paths import ROOT
from rm27_sim.simulation.asm2 import compose_scene, Plant, ContactScorer, load_scene
from .geometry import _compiled_contact_geometry
from rm27_sim.experiments.contact_metrics import point_approach_speed

DEFAULT_CONFIG = ROOT / 'configs/asm2_contact_recovery.json'
STATE = mujoco.mjtState.mjSTATE_INTEGRATION


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def _bounded_plan(observation, config, geometry, previous, tracker, activation):
    from .asm2_contact_control import plan_reference

    started = time.monotonic()
    result = plan_reference(observation, config, geometry, tracker, previous, activation=activation)
    finished = time.monotonic()
    return result, finished, finished-started


def simulate(model, metadata, mapping, config, case, method, output, scene_hash, *, recovery_seed=None):
    from concurrent.futures import ThreadPoolExecutor

    from .asm2_contact_control import (
        ApproachTracker, ObservationSource, feedback, nominal_start, plan_reference, recovery_reference,
    )
    from rm27_contact_physics.model import TargetState

    started = time.monotonic()
    trial_model = copy.copy(model)
    plant = Plant(trial_model, metadata, case)
    trial_model.body_mass[plant.body] *= plant.mass_scale
    trial_model.body_inertia[plant.body] *= plant.mass_scale
    trial_model.actuator_dynprm[plant.motors, 0] *= plant.tau_scale
    mujoco.mj_setConst(trial_model, plant.data)
    scorer = ContactScorer(plant, mapping, metadata, config)
    target = TargetState(scorer.center, scorer.normal, np.zeros((3, 3)), 0.)
    start_p, start_v = nominal_start(target, config)
    start_p += case.get('position_offset_m', [0., 0., 0.])
    start_v += case.get('velocity_offset_m_s', [0., 0., 0.])
    model, data = trial_model, plant.data
    if recovery_seed is None:
        plant.reset(start_p, start_v)
        if any(plant.body in model.geom_bodyid[np.asarray(contact.geom)] for contact in data.contact):
            raise ValueError('Handover overlaps field geometry')
    else:
        if method != 'recovery' or case['source'] != 'oracle':
            raise ValueError('Detached recovery requires an oracle recovery case')
        saved, event = recovery_seed
        mujoco.mj_setState(model, data, saved, STATE)
        if not event['records'] or not event['time_s'] <= data.time <= event['time_s'] + config['control_dt_s'] + 1e-9:
            raise ValueError('Recovery seed must follow a recorded physical contact within one control tick')
        mujoco.mj_forward(model, data)
    approach_tracker = ApproachTracker(metadata, config)
    geometry = _compiled_contact_geometry(plant, scorer) if recovery_seed is None else None
    source = ObservationSource(case, config['synthetic'])
    initial_mass = model.body_mass[plant.body]
    control_dt, physics_dt = config['control_dt_s'], model.opt.timestep
    substeps = round(control_dt / physics_dt)
    if not np.isclose(control_dt, substeps * physics_dt):
        raise ValueError('Control clock must be a multiple of the physics clock')
    phase = 'WAIT_ESTIMATE'
    failure = None
    reference = None
    initial_reference = None
    normal = None
    first_contact = None
    last_contact_time = None
    secondary_contact = False
    separated_at = None
    recovery_started = None
    recovery_velocity = None
    stable_since = None
    abort_reason = None
    if recovery_seed is not None:
        phase, normal = 'RECOVERY', scorer.normal.copy()
        first_contact = event
        recovery_started = event['time_s']
        last_contact_time = float(data.time)
    pending = None
    last_future = None
    next_plan = 0.
    replans = []
    trajectory, phases, commands = [plant.snapshot()], [phase], [np.zeros(4)]
    raw_command = commands[-1]
    rates_clipped = motors_clipped = updates = 0
    peaks = dict(speed_m_s=0., tilt_deg=0., rate_rad_s=0.)
    min_position, max_position = np.asarray(config['flight_box_relative_m']) + scorer.center
    control_wall = []
    recovered = False
    with ThreadPoolExecutor(max_workers=1) as pool:
        for tick in range(round((config['episode_limit_s'] - data.time)/control_dt)):
            if time.monotonic() - started > config['episode_wall_limit_s']:
                failure = 'EPISODE_WALL_BUDGET'
                break
            tick_started = time.monotonic()
            truth = plant.state()
            now = truth.timestamp
            observed = source.update(truth, scorer.center, scorer.normal)
            measured = None if observed is None else observed.predict_vehicle(now, config['max_observation_age_s'])
            if measured is None and (phase != 'WAIT_ESTIMATE' or now > config['max_observation_age_s']):
                failure = 'VEHICLE_ESTIMATE_UNAVAILABLE'
                break
            if measured is not None and phase == 'WAIT_ESTIMATE':
                if observed.target.valid(now, config['max_observation_age_s']):
                    normal = observed.target.normal.copy()
                    reference = plan_reference(
                        observed.predict(now, config['max_observation_age_s']),
                        config, geometry, approach_tracker,
                    )
                    if reference is None:
                        failure = 'INITIAL_REFERENCE_INFEASIBLE'
                        break
                    initial_reference = reference.contact_preview
                    phase = 'APPROACH'
            if phase == 'APPROACH':
                if not observed.target.valid(now, config['max_observation_age_s']):
                    phase, recovery_started, abort_reason = 'ABORT', now, 'TARGET_ESTIMATE_STALE'
                elif now > reference.end + .075:
                    phase, recovery_started, abort_reason = 'ABORT', now, 'CONTACT_WINDOW_ELAPSED'
                else:
                    normal = observed.target.normal.copy()

            if pending is not None and now + 1e-9 >= pending[0]:
                apply_time, deadline, future, requested_at = pending
                entry = dict(request_s=requested_at, application_s=apply_time)
                if not future.done():
                    future.cancel()
                    entry['status'] = 'MISSED_APPLICATION'
                else:
                    try:
                        candidate, finished, latency = future.result()
                    except (ValueError, np.linalg.LinAlgError) as error:
                        entry.update(status='SOLVER_ERROR', error=str(error))
                    else:
                        entry['solve_wall_s'] = latency
                        if finished > deadline:
                            entry['status'] = 'WALL_DEADLINE'
                        elif phase != 'APPROACH':
                            entry['status'] = 'PHASE_CHANGED'
                        elif candidate is None:
                            entry['status'] = 'NO_FEASIBLE_REFERENCE'
                        else:
                            delta = np.linalg.norm(candidate.at(apply_time+.25)[0] -
                                                   reference.at(apply_time+.25)[0])
                            entry.update(status='APPLIED', reference_change_m=float(delta),
                                         new_contact_reference_s=candidate.end,
                                         contact_preview=candidate.contact_preview,
                                         blend_s=candidate.blend)
                            reference = candidate
                replans.append(entry)
                pending = None

            if (phase == 'APPROACH' and method == 'online' and pending is None
                    and (last_future is None or last_future.done())
                    and now + 1e-9 >= max(next_plan, reference.start + reference.blend)
                    and reference.end-now >= config['plan_apply_lead_s']+config['terminal_commit_s']):
                next_plan = now + config['plan_period_s']
                apply_time = now + config['plan_apply_lead_s']
                predicted = observed.predict(apply_time,
                                             config['max_observation_age_s']+config['plan_apply_lead_s'])
                if predicted is not None:
                    deadline = time.monotonic() + config['plan_wall_deadline_s']
                    activation = approach_tracker.activation_at(
                        config['plan_apply_lead_s'], data.ctrl[plant.motors])
                    future = pool.submit(_bounded_plan, predicted, config, geometry, reference,
                                         approach_tracker, activation)
                    last_future = future
                    pending = (apply_time, deadline, future, now)

            if phase == 'APPROACH':
                action, motor_command, saturated = approach_tracker.command(measured, reference, now)
                data.ctrl[plant.motors] = motor_command
                rates_clipped += approach_tracker.rate_limited
                motors_clipped += saturated
                updates += 1
                raw_command = np.r_[action.collective_force_N, action.body_rates_flu_rad_s]
            elif phase in ('RECOVERY', 'ABORT'):
                if recovery_velocity is None:
                    recovery_velocity = measured.velocity.copy()
                desired = recovery_reference(measured, normal, now-recovery_started, recovery_velocity)
                action, rate_clipped = feedback(
                    measured, desired, plant.nominal_mass, 4*plant.max_motor, config)
                motors_clipped += plant.command(action, measured.omega_body)
                rates_clipped += rate_clipped
                updates += 1
                raw_command = np.r_[action.collective_force_N, action.body_rates_flu_rad_s]
            control_wall.append(time.monotonic()-tick_started)

            before = truth
            for _ in range(substeps):
                mujoco.mj_step(model, data)
                if data.ncon:
                    contacts = scorer.active(data, before)
                    if contacts:
                        if first_contact is None:
                            first_contact = dict(
                                interval_start_s=before.timestamp, time_s=float(data.time),
                                records=contacts, qualified=not any(x['rejected'] for x in contacts),
                                pre_contact_height_m=float(before.position[2]), phase=phase,
                            )
                            if phase != 'APPROACH':
                                first_contact['qualified'] = False
                            phase = 'RECOVERY'
                            recovery_started = float(data.time)
                        elif float(data.time) - last_contact_time > .03 or any(
                            'WRONG_AIRCRAFT_GEOM' in event['rejected'] or 'WRONG_SURFACE' in event['rejected']
                            for event in contacts
                        ):
                            secondary_contact = True
                        last_contact_time = float(data.time)
                before = plant.state()
                if not np.isfinite(np.r_[data.qpos, data.qvel, data.act, data.ctrl]).all():
                    failure = 'NONFINITE_PLANT'
                    break
                if any(data.warning[k].number for k in (
                    mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                    mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADCTRL)):
                    failure = 'PHYSICS_WARNING'
                    break
                speed = float(np.linalg.norm(before.velocity))
                tilt = float(np.degrees(np.arccos(np.clip(before.rotation[2, 2], -1., 1.))))
                rate = float(np.linalg.norm(before.omega_body))
                peaks['speed_m_s'] = max(peaks['speed_m_s'], speed)
                peaks['tilt_deg'] = max(peaks['tilt_deg'], tilt)
                peaks['rate_rad_s'] = max(peaks['rate_rad_s'], rate)
                if (speed > config['maximum_speed_m_s'] or tilt > config['maximum_tilt_deg']
                        or rate > config['maximum_rate_rad_s']
                        or np.any(before.position < min_position) or np.any(before.position > max_position)):
                    failure = 'FLIGHT_ENVELOPE'
                    if first_contact and first_contact['time_s'] == before.timestamp:
                        first_contact['qualified'] = False
                    break
            trajectory.append(plant.snapshot())
            phases.append(phase)
            commands.append(raw_command.copy())
            if failure:
                break
            if phase in ('RECOVERY', 'ABORT'):
                gap = (before.position - scorer.center) @ scorer.normal - scorer.pad_height
                if separated_at is None and gap >= config['separation_m'] and (
                        last_contact_time is None or before.timestamp-last_contact_time >= .03):
                    separated_at = before.timestamp
                height_ok = first_contact is None or (
                    before.position[2] >= first_contact['pre_contact_height_m']-config['recovery_altitude_loss_m'])
                stable = (separated_at is not None and not secondary_contact and height_ok
                          and tilt <= config['recovery_tilt_deg'] and rate <= config['recovery_rate_rad_s']
                          and speed <= config['recovery_speed_m_s'])
                if stable:
                    if stable_since is None:
                        stable_since = before.timestamp
                    if before.timestamp-stable_since >= config['recovery_hold_s']-1e-9:
                        recovered = stable_since-recovery_started <= config['recovery_deadline_s']
                        break
                else:
                    stable_since = None
                if stable_since is None and before.timestamp-recovery_started > config['recovery_deadline_s']:
                    failure = 'RECOVERY_DEADLINE'
                    break
        if pending is not None:
            pending[2].cancel()
            replans.append(dict(request_s=pending[3], application_s=pending[0], status='EPISODE_ENDED'))
    if not recovered and failure is None:
        failure = 'EPISODE_TIMEOUT'
    qualified = bool(recovery_seed is None and first_contact and first_contact['qualified'])
    output.mkdir()
    np.savez_compressed(output/'trajectory.npz', integration=np.asarray(trajectory),
                        phases=np.asarray(phases), ctbr=np.asarray(commands))
    if case.get('mass_scale', 1.) != 1. or case.get('motor_tau_scale', 1.) != 1.:
        from rm27_sim.replay.recording import save_overlay
        save_overlay(model, output, output.parent/'scene.mjb', scene_hash)
    else:
        # Keep a single immutable model on disk for all nominal recordings.
        (output/'model.mjb').symlink_to('../scene.mjb')
    latencies = [entry['solve_wall_s'] for entry in replans if 'solve_wall_s' in entry]
    result = dict(
        case=case, method=method, qualified_contact=qualified, first_contact=first_contact,
        initial_reference_contact_preview=initial_reference,
        resumed_from_contact=recovery_seed is not None,
        separated=separated_at is not None, separated_at_s=separated_at, recovered=recovered,
        joint_success=qualified and recovered, safe_abort=abort_reason is not None and recovered and first_contact is None,
        abort_reason=abort_reason, failure=failure, secondary_contact=secondary_contact,
        duration_s=float(data.time), wall_s=time.monotonic()-started, peaks=peaks,
        motor_saturation_fraction=motors_clipped/max(updates, 1), rate_clip_fraction=rates_clipped/max(updates, 1),
        replans=replans, solver_p95_wall_s=float(np.quantile(latencies, .95)) if latencies else None,
        control_p95_wall_s=float(np.quantile(control_wall, .95)) if control_wall else None,
        mass_kg=float(initial_mass),
        trajectory_sha256=digest(output/'trajectory.npz'),
        contact_detector='MUJOCO_COMPRESSIVE_SOLVER_EVENT_ORACLE',
    )
    write_json(output/'summary.json', result)
    return result


def main():
    import platform

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiled-scene', type=Path, help='Verified output of an earlier composition')
    parser.add_argument('--full-field', action='store_true')
    parser.add_argument('--case', action='append', help='Select a declared case; run both methods')
    parser.add_argument('--prepare-scene', action='store_true', help='Compose and hash inputs without flying')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    cases = [case for case in config['cases'] if not args.case or case['id'] in args.case]
    if not cases or (args.case and {c['id'] for c in cases} != set(args.case)):
        parser.error('Select declared case IDs')
    if len(cases) > 4 or config['episode_limit_s'] > 8.:
        parser.error('The registered budget is four matched cases and eight simulated seconds each')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    model, metadata, mapping, source = load_scene(config, output, args.compiled_scene, args.full_field)
    if source['field_sha256']['collision_geoms.xml'] != config['front_plate_backing_source_sha256']:
        raise ValueError('Backing plate registration does not match the field collision source')
    if args.prepare_scene:
        print(f'Compiled {model.ngeom} geometries in {source["compile_wall_s"]:.3f} s', flush=True)
        return 0
    rows = []
    for case in cases:
        for method in ('fixed', 'online'):
            result = simulate(model, metadata, mapping, config, case, method,
                              output/f'{case["id"]}_{method}', source['compiled_sha256'])
            rows.append(result)
            print(json.dumps({k: result[k] for k in
                              ('case', 'method', 'qualified_contact', 'recovered', 'safe_abort',
                               'failure', 'abort_reason', 'duration_s', 'replans')}), flush=True)
    changed_replan = any(
        row['case']['id'] == 'changed_handover' and entry.get('reference_change_m', 0.) >= .01
        for row in rows for entry in row['replans']
    )
    evaluation = {
        'all_declared_cases_run': len(cases) == len(config['cases']),
        'nominal_both_joint_success': len([r for r in rows if r['case']['id']=='nominal' and r['joint_success']]) == 2,
        'meaningful_changed_handover_replan': changed_replan,
        'target_outage_both_safe_abort': len([r for r in rows if r['case']['id']=='synthetic_outage' and r['safe_abort']]) == 2,
    }
    source_hashes = dict(scene=source['compiled_sha256'], config=digest(args.config),
                         runner=digest(__file__), controller=digest(Path(__file__).with_name('asm2_contact_control.py')))
    from rm27_contact_physics import control, model as estimate_types, teacher_interface
    source_hashes.update({module.__name__:digest(module.__file__) for module in (control, estimate_types, teacher_interface)})
    summary = dict(
        passed=all(evaluation.values()), evaluation=evaluation, cases=rows,
        inputs=source_hashes, config=config, full_field=source['full_field'],
        mujoco_version=mujoco.__version__, numpy_version=np.__version__, host=platform.node(),
        low_level='ASM2 motor lead and angular-acceleration tracking during approach; shared CTBR proxy during recovery',
        vision_closed_loop='NOT_RUN', estimator_recovery='NOT_RUN (synthetic vehicle samples remain available)',
        contact_detector='oracle contact from MuJoCo solver; never a planned time', wall_s=time.monotonic()-started,
    )
    write_json(output/'summary.json', summary)
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
