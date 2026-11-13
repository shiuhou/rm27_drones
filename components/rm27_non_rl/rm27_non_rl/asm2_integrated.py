"""Continuous native contact experiment with sensor-only runtime inputs."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import gzip
import heapq
import json
from pathlib import Path
import shutil
import time

import mujoco
from .geometry import _compiled_contact_geometry
import numpy as np
from scipy.spatial.transform import Rotation

from rm27_contact_physics.model import TargetState, VehicleEstimate
from rm27_contact_physics.control import Septic
from rm27_contact_physics.teacher_interface import RateThrust
from rm27_sim.simulation import asm2 as native
from rm27_sim.simulation import clearance as scene_clearance
from rm27_sim.experiments import asm2_sensor_recording as sensor_recording
from .asm2_contact_control import ApproachTracker, Observation, Reference, feedback, nominal_start, plan_reference, scheduled_recovery_reference, leveling_clearance, clearance_limited_recovery
from .asm2_estimation_integrity import CapabilitySupervisor, ImpactEvent, Source, StateSample
from .asm2_recovery_initialization import EventAwareRecovery, RecoveryInitializationError
from .asm2_sensor_estimator import SensorEstimator, SensorEstimatorConfig, time_ns
from rm27_sim.experiments.asm2_sensor_recording import SensorRecorder
from .asm2_guidance import AutonomousGuidance, PendingPlan, SceneClearance, continuous
from .paths import ROOT


def reference_record(reference):
    """Exact adopted polynomial; previous references are logged at their adoption."""
    return dict(start_s=reference.start, end_s=reference.end,
                polynomial_coefficients=reference.trajectory.c.tolist(),
                polynomial_duration_s=reference.trajectory.duration,
                aligned_s=reference.aligned, blend_s=reference.blend, blend_delay_s=reference.blend_delay_s,
                previous_start_s=None if reference.previous is None else reference.previous.start,
                forward_segments=None if reference.forward_segments is None else
                    [dict(duration_s=s.duration, coefficients=s.c.tolist()) for s in reference.forward_segments],
                forward_axis=None if reference.forward_axis is None else reference.forward_axis.tolist(),
                vertical_segments=None if reference.vertical_segments is None else
                    [dict(duration_s=s.duration, coefficients=s.c.tolist()) for s in reference.vertical_segments],
                preview=reference.contact_preview)


def gate_samples(estimate, normal, target_position, config):
    now = estimate['time_s']
    samples = []
    world = estimate.get('world_alignment_observed', False)
    for name, field, frame in (
        ('attitude', 'rotation_body_to_local', 'body_FLU_to_world'),
        ('body_rate', 'angular_velocity_body_rad_s', 'body_FLU'),
        ('position', 'position_m', 'world'), ('velocity', 'velocity_m_s', 'world'),
    ):
        channel = estimate['channels'][name]
        if channel['acquisition_s'] is None:
            continue
        acquired = time_ns(channel['acquisition_s'])/1e9
        delivered = time_ns(channel['delivery_s'])/1e9
        samples.append(StateSample(name, estimate[field], acquired, delivered, now,
            estimate['epoch_id'], frame, Source.SYNTHETIC,
            .05 if name in ('position', 'velocity') else .01,
            valid=channel['valid'] and (world or name == 'body_rate'), propagated=now != acquired))
    rotation = np.asarray(estimate['rotation_body_to_local'])
    velocity = np.asarray(estimate['velocity_m_s'])
    valid = all(channel['valid'] for channel in estimate['channels'].values()) and world
    derived = [
        ('surface_normal', normal, 'world'),
        ('position_target', target_position, 'world'),
        ('attitude_stable', rotation[2, 2] >= np.cos(np.deg2rad(config['recovery_tilt_deg'])), 'predicate'),
        ('velocity_stable', np.linalg.norm(velocity) <= config['recovery_speed_m_s'], 'predicate'),
        ('vertical_clearance', (np.asarray(estimate['position_m'])-target_position) @ normal > .08, 'predicate'),
        ('vertical_velocity', velocity[2], 'world_up'),
    ]
    for name, value, frame in derived:
        samples.append(StateSample(name, value, now, now, now, estimate['epoch_id'], frame,
                                   Source.SYNTHETIC, .01, valid=valid))
    return samples


class FlightRuntime:
    def __init__(self, metadata, mapping, config, *, method='online', initialize=True, clearance=None):
        self.config, self.metadata, self.method = config, metadata, method
        self.normal = np.asarray(mapping['armor_normal'], dtype=float)
        self.normal /= np.linalg.norm(self.normal)
        self.center = np.asarray(mapping['armor_surface_center_m'])
        self.estimator = SensorEstimator(SensorEstimatorConfig(
            external_frame_registration='SENSOR_FLIGHT_ALIGNMENT_REQUIRED'))
        self.supervisor = CapabilitySupervisor()
        self.tracker = ApproachTracker(metadata, config)
        self.initialize = initialize
        self.records, self.pending = [], []
        self.alignment = None
        self.event = None
        self.latest_external = None
        self.reference = None
        self.recovery = None
        self.phase = 'ALIGNMENT_FLIGHT'
        self.replans = []
        self.next_plan = 0.
        self.hold_position = None
        self.first_recovery_command = None
        self.first_plan = None
        self.pending_reference = None
        self.handover = None
        self.last_diagnostic = None
        self.scheduled_release_since = None
        self.release_initial_velocity = None
        self.trigger_kind = None
        self.release_effective_s = None
        self.leveling_since = None
        self.recovery_blend_weight = 0.
        self.previous_recovery_acceleration = None
        self.guidance = AutonomousGuidance(config, clearance) if config.get("guidance") else None
        self.pending_contract = None

    def receive(self, packets):
        for packet in packets:
            self.records.append(packet)
            heapq.heappush(self.pending, (time_ns(packet['delivery_s']), len(self.records), packet))

    def update(self, now, geometry):
        now = time_ns(now)/1e9
        while self.pending and self.pending[0][0] <= time_ns(now):
            _, _, packet = heapq.heappop(self.pending)
            disposition = self.estimator.ingest(packet, now_s=now)
            if not disposition['accepted'] or not packet['valid'] or packet['clipped']:
                continue
            if packet['kind'] == 'external':
                self.latest_external = packet
            elif packet['kind'] == 'impact' and self.event is None:
                self.event = packet
                event = packet['value']
                self.supervisor.impact(ImpactEvent('sensor-contact', event['lower_s'], event['upper_s'],
                    packet['delivery_s'], Source.SYNTHETIC), now=now)
                self.recovery = EventAwareRecovery(self.normal, event_lower_s=event['lower_s'],
                    event_upper_s=event['upper_s'], event_reference_s=event['upper_s'],
                    hold_s=.025, blend_s=.065, translation_max_age_s=.05,
                    event_source='SYNTHETIC_IMU_1G')
        if self.initialize and self.alignment is None and now >= 1.2 and self.event is None:
            self.alignment = self.estimator.initialize_flight_history(self.records, now_s=now)
        estimate = self.estimator.estimate(now)
        measured = VehicleEstimate(np.asarray(estimate['position_m']), np.asarray(estimate['velocity_m_s']),
            np.asarray(estimate['rotation_body_to_local']), np.asarray(estimate['angular_velocity_body_rad_s']),
            now, 'SENSOR_ESTIMATED_SYNTHETIC')
        decision = self.supervisor.evaluate(now=now, samples=gate_samples(estimate, self.normal, self.center, self.config),
                                            actuator_available=True, simulation_execution=True)
        diagnostic = dict(estimate=estimate, supervisor=asdict(decision), alignment=self.alignment)
        self.last_diagnostic = diagnostic
        valid_attitude = estimate['channels']['attitude']['valid'] and estimate['world_alignment_observed']
        valid_translation = all(estimate['channels'][name]['valid'] for name in ('position', 'velocity'))
        if not estimate['channels']['body_rate']['valid']:
            raise RuntimeError('BODY_RATE_ESTIMATE_UNAVAILABLE')
        if self.phase == 'ALIGNMENT_FLIGHT' and self.alignment is None:
            if now > 1.6:
                raise RuntimeError('SENSOR_ALIGNMENT_UNOBSERVABLE')
            rates = [1.*np.cos(2*np.pi*now/1.2), .12*np.sin(2*np.pi*now/1.2), 0.]
            return RateThrust(np.asarray(rates), self.metadata['mass_kg']*9.81), None, diagnostic
        if not valid_attitude:
            raise RuntimeError('ATTITUDE_ESTIMATE_UNAVAILABLE')
        release_clock = self.config.get('scheduled_release_reference', 'time_s')
        if release_clock not in ('time_s', 'earliest_window_s'):
            raise ValueError('Unknown reference release clock')
        release_offset = self.config.get('scheduled_release_offset_s', 0.)
        if not np.isfinite(release_offset) or release_offset < 0.:
            raise ValueError('Invalid scheduled release offset')
        if (self.config.get('scheduled_separation', False) and self.reference is not None
                and self.scheduled_release_since is None and self.event is None
                and now >= self.reference.contact_preview[release_clock]+release_offset-
                (float(np.mean(self.metadata['motor_time_constant_s']))
                 if self.config.get('separation_motor_lead', False) else 0.)):
            self.scheduled_release_since = now
            self.release_effective_s = self.reference.contact_preview[release_clock]+release_offset
            self.release_initial_velocity = measured.velocity.copy() if valid_translation else None
            self.trigger_kind = 'REFERENCE_SCHEDULE_NOT_CONTACT_DETECTION'
        if self.scheduled_release_since is not None:
            if self.first_recovery_command is None:
                self.first_recovery_command = now
            elapsed = max(0., now-self.release_effective_s)
            clearance_gate = self.config.get('leveling_clearance_m')
            incremental = self.config.get('incremental_clearance_recovery', False)
            if clearance_gate is not None and not incremental:
                if not valid_translation or geometry is None:
                    raise RuntimeError('LEVELING_CLEARANCE_STATE_UNAVAILABLE')
                gap = leveling_clearance(measured, self.center, self.normal, geometry)
                ready = gap >= clearance_gate and measured.velocity@self.normal >= 0.
                if self.leveling_since is None and ready and elapsed >= .025:
                    self.leveling_since = now
                elapsed = (min(elapsed, .025) if self.leveling_since is None
                           else .025+now-self.leveling_since)
                diagnostic['leveling_gate'] = dict(estimated_rotation_gap_m=gap,
                    required_gap_m=clearance_gate, ready=bool(ready),
                    transition_start_s=self.leveling_since, recovery_reference_elapsed_s=elapsed,
                    source='SYNTHETIC_ESTIMATE_STATIC_GEOMETRY_NOT_CALIBRATED')
            if valid_translation:
                if self.release_initial_velocity is None:
                    self.release_initial_velocity = measured.velocity.copy()
                desired = scheduled_recovery_reference(measured, self.normal, elapsed, self.release_initial_velocity,
                    self.config.get('separation_normal_accel_m_s2', 1.2))
                if incremental:
                    if geometry is None or clearance_gate is None:
                        raise RuntimeError('LEVELING_CLEARANCE_STATE_UNAVAILABLE')
                    desired, self.recovery_blend_weight, diagnostic['incremental_leveling'] = clearance_limited_recovery(
                        measured, self.center, self.normal, geometry, elapsed, self.release_initial_velocity,
                        self.recovery_blend_weight, self.config['control_dt_s'], clearance_gate,
                        self.config.get('separation_normal_accel_m_s2', 1.2))
                self.phase = 'SCHEDULED_SEPARATION_UNCONFIRMED' if self.event is None else 'SEPARATION'
                if elapsed >= .09 and (not incremental or self.recovery_blend_weight >= 1.):
                    self.phase = 'ATTITUDE_STABILIZATION'
                    if measured.rotation[2, 2] > np.cos(np.deg2rad(10)):
                        self.phase = 'VELOCITY_RECOVERY'
                if elapsed > 1.2 and np.linalg.norm(measured.velocity) < .15:
                    if self.hold_position is None:
                        self.hold_position = measured.position.copy()
                    if any(mode.value == 'POSITION_HOLD' for mode in decision.recovery_modes):
                        self.phase = 'POSITION_HOLD'
                        desired = (self.hold_position, np.zeros(3), np.zeros(3), np.zeros(3))
            else:
                self.phase = 'ATTITUDE_ONLY_TRANSLATION_UNAVAILABLE'
                desired = (measured.position, measured.velocity, np.zeros(3), np.zeros(3))
            if self.config.get('recovery_reference_feedforward', False):
                jerk = (np.zeros(3) if self.previous_recovery_acceleration is None else
                        (desired[2]-self.previous_recovery_acceleration)/self.config['control_dt_s'])
                self.previous_recovery_acceleration = desired[2].copy()
                desired = (*desired[:3], jerk)
                diagnostic['recovery_reference_derivative'] = dict(jerk_m_s3=jerk.tolist(),
                    method='CAUSAL_BACKWARD_DIFFERENCE_REFERENCE_ACCELERATION',
                    feedforward_limit_rad_s=self.config['controller_jerk_feedforward_limit_rad_s'])
            action, _ = feedback(measured, desired, self.metadata['mass_kg'],
                                  4*self.metadata['max_rotor_thrust_N'], self.config)
            if self.config.get('recovery_motor_lead', False):
                motors, clipped = self.tracker.recovery_motor_command(measured, action)
                diagnostic['recovery_motor_lead'] = dict(clipped=clipped,
                    model_horizon_s=self.tracker.motor_horizon, source='COMMAND_HISTORY_MODEL')
                return action, motors, diagnostic
            return action, None, diagnostic
        if self.event is not None:
            self.trigger_kind = 'SYNTHETIC_IMU_EVENT'
            if self.first_recovery_command is None:
                self.first_recovery_command = now
            if valid_translation and decision.brief_normal_release_eligible:
                external = self.latest_external
                anchor_state = replace(measured, velocity=np.asarray(external['value']['velocity_m_s']))
                self.recovery.observe(anchor_state, measurement_time_s=external['acquisition_s'],
                    delivery_time_s=external['delivery_s'], source=external['source'])
                desired = self.recovery.reference(measured)
                self.phase = 'SEPARATION'
                elapsed = now-self.event['value']['upper_s']
                if elapsed >= .09:
                    self.phase = 'ATTITUDE_STABILIZATION'
                if elapsed >= .09 and measured.rotation[2, 2] > np.cos(np.deg2rad(10)):
                    self.phase = 'VELOCITY_RECOVERY'
                if self.phase == 'VELOCITY_RECOVERY' and np.linalg.norm(measured.velocity) < .15 and elapsed > 1.2:
                    if self.hold_position is None:
                        self.hold_position = measured.position.copy()
                    self.phase = 'POSITION_HOLD'
                    desired = (self.hold_position, np.zeros(3), np.zeros(3), np.zeros(3))
            else:
                if now-self.event['value']['upper_s'] > .05:
                    raise RuntimeError('POST_IMPACT_TRANSLATION_UNAVAILABLE')
                self.phase = 'MODEL_NORMAL_RELEASE_WAIT_TRANSLATION'
                desired = (measured.position, measured.velocity,
                           self.normal*(9.81*self.normal[2]+1.2)-[0., 0., 9.81], np.zeros(3))
            action, _ = feedback(measured, desired, self.metadata['mass_kg'],
                                  4*self.metadata['max_rotor_thrust_N'], self.config)
            return action, None, diagnostic
        if not valid_translation:
            raise RuntimeError('PRE_CONTACT_TRANSLATION_UNAVAILABLE')
        target = TargetState(self.center, self.normal, np.zeros((3, 3)), now, source='REGISTERED_STATIC_MAP')
        observed = Observation(measured, target)
        if self.guidance is not None and self.reference is None:
            previous_guide = self.guidance.current
            guided, self.phase = self.guidance.update(observed, estimate, self.tracker, geometry)
            diagnostic['guidance'] = dict(kind=self.guidance.kind, identity=self.guidance.identity,
                                         event_count=len(self.guidance.events))
            if previous_guide is not guided:
                diagnostic['adopted_reference'] = dict(kind=self.guidance.kind, **reference_record(guided))
            if self.guidance.terminal:
                self.reference = guided
                self.first_plan = now
                self.next_plan = now+self.config['plan_period_s']
                diagnostic['adopted_reference'] = dict(kind='AUTONOMOUS_HANDOVER', **reference_record(guided))
            else:
                action, motors, _ = self.tracker.command(measured, guided, now)
                self.guidance.last_command = motors.copy()
                return action, motors, diagnostic
        elif self.guidance is None:
            if self.handover is None:
                goal_position, goal_velocity = nominal_start(target, self.config)
                zeros = np.zeros(3)
                self.handover = Reference(Septic((measured.position, measured.velocity, zeros, zeros),
                    (goal_position, goal_velocity, zeros, zeros), 1.5), now)
            if now < self.handover.end:
                self.phase = 'HANDOVER'
                action, motors, _ = self.tracker.command(measured, self.handover, now)
                return action, motors, diagnostic
            if self.reference is None:
                self.reference = plan_reference(observed, self.config, geometry, self.tracker)
                if self.reference is None:
                    raise RuntimeError('INITIAL_REFERENCE_INFEASIBLE')
                self.first_plan = now
                diagnostic['adopted_reference'] = dict(kind='INITIAL', **reference_record(self.reference))
                self.next_plan = now+.2
        if self.pending_reference is not None and now+(1e-9 if self.guidance is not None else 0.) >= self.pending_reference.start:
            reason = None
            if self.pending_contract is not None:
                reason = self.pending_contract.invalid_reason(now=now,parent=self.reference.start,
                    epoch=estimate['epoch_id'],valid=valid_attitude and valid_translation)
            if reason is None:
                self.reference = self.pending_reference
                self.replans[-1]['status'] = 'APPLIED'
                diagnostic['adopted_reference'] = dict(kind='REPLACEMENT', **reference_record(self.reference))
            else:
                self.replans[-1].update(status='REJECTED',reason=reason)
            self.pending_reference = self.pending_contract = None
        if (self.method == 'online' and now >= self.next_plan
                and self.pending_reference is None
                and now >= self.reference.start+self.reference.blend
                and self.reference.end-now > self.config['terminal_commit_s']+.06):
            begun = time.perf_counter()
            apply_time = now+self.config['plan_apply_lead_s']
            predicted = observed.predict(apply_time, self.config['max_observation_age_s']+.06)
            attempts = []
            candidate = plan_reference(predicted, self.config, geometry, self.tracker, self.reference,
                                       activation=(self.tracker.activation_at(apply_time-now,self.guidance.last_command)
                        if self.guidance is not None and self.guidance.last_command is not None
                        else self.tracker.expected_activation), diagnostics=attempts)
            rejection = 'NO_FEASIBLE_REFERENCE' if candidate is None else None
            if candidate is not None and self.guidance is not None:
                ok, rejection = self.guidance.clearance.check(candidate,apply_time,begun+self.config['plan_wall_deadline_s'])
                if ok and not continuous(self.reference,candidate,apply_time,self.tracker,predicted.vehicle):
                    ok, rejection = False, 'DISCONTINUOUS_SPLICE'
                if not ok:
                    candidate = None
            latency = time.perf_counter()-begun
            accepted = candidate is not None and latency <= self.config['plan_wall_deadline_s']
            self.replans.append(dict(time_s=now, application_s=apply_time, solve_wall_s=latency,
                                     status='PENDING' if accepted else 'REJECTED',
                                     reason=None if accepted else ('SOLVER_DEADLINE' if latency > self.config['plan_wall_deadline_s'] else rejection),
                                     terminal_attempts=attempts,
                                     preview=None if candidate is None else candidate.contact_preview))
            if accepted:
                self.pending_reference = candidate
                if self.guidance is not None:
                    self.pending_contract = PendingPlan(candidate,'TERMINAL',len(self.replans),self.reference.start,
                        estimate['epoch_id'],now,min(c['acquisition_s'] for c in estimate['channels'].values()),
                        now,apply_time+self.config['control_dt_s'])
                    self.replans[-1].update(epoch=estimate['epoch_id'],parent_reference_s=self.reference.start,
                        acquisition_s=self.pending_contract.acquisition,state_valid_at_s=now)
            self.next_plan = now+.2
        if now > self.reference.end+.075:
            raise RuntimeError('CONTACT_EVENT_NOT_OBSERVED')
        self.phase = 'APPROACH'
        action, motors, _ = self.tracker.command(measured, self.reference, now)
        if self.guidance is not None:
            self.guidance.last_command = motors.copy()
        return action, motors, diagnostic


class SpeedEnvelopeObservation:
    def __init__(self, config):
        self.limit = config['maximum_speed_m_s']
        self.diagnostic = config.get('speed_diagnostic')
        self.first_violation = None
        self.peak = 0.
        if self.diagnostic is not None:
            cap = self.diagnostic['hard_stop_m_s']
            if not np.isfinite(cap) or cap <= self.limit:
                raise ValueError('Diagnostic hard stop must exceed the unchanged qualification limit')

    def observe(self, timestamp, velocity):
        speed = float(np.linalg.norm(velocity))
        self.peak = max(self.peak, speed)
        if speed <= self.limit:
            return False
        if self.first_violation is None:
            self.first_violation = dict(time_s=timestamp, speed_m_s=speed,
                                       velocity_m_s=np.asarray(velocity).tolist())
        return self.diagnostic is None or speed > self.diagnostic['hard_stop_m_s']

    def qualify_recovery(self, observed):
        return bool(observed and self.first_violation is None)


def run(output, *, case, method='online', initialize=True, config_path=None):
    run_started = time.perf_counter()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    source_directory = output/'runtime_sources'
    source_directory.mkdir()
    source_paths = [Path(__file__), Path(__file__).with_name('asm2_flight_alignment.py'),
                   Path(__file__).with_name('asm2_sensor_estimator.py'),
                   Path(__file__).with_name('asm2_contact_control.py'),
                   Path(__file__).with_name('asm2_recoverability.py'),
                   Path(__file__).with_name('asm2_guidance.py'),
                   Path(__file__).with_name('asm2_estimation_integrity.py'),
                   Path(__file__).with_name('asm2_recovery_initialization.py'),
                   Path(__file__).with_name('m3_estimator.py'),
                   Path(__file__).with_name('geometry.py'),
                   Path(sensor_recording.__file__), Path(native.__file__),
                   Path(scene_clearance.__file__)]
    source_hashes = {str(path):native.digest(path) for path in source_paths}
    for path in source_paths:
        shutil.copy2(path, source_directory/path.name)
    baseline = ROOT/'configs/asm2_integrated.json'
    config = json.loads(Path(config_path or baseline).read_text())
    config.update(initialize_reference_from_estimate=True)
    speed_observation = SpeedEnvelopeObservation(config)
    model, metadata, mapping, provenance = native.load_scene(config, output,
        native.ROOT/'artifacts/asm2_contact_recovery/scene_base_01', False)
    plant = native.Plant(model, metadata, case)
    model.body_mass[plant.body] *= case.get('mass_scale', 1.)
    model.body_inertia[plant.body] *= case.get('mass_scale', 1.)
    model.actuator_dynprm[plant.motors, 0] *= case.get('motor_tau_scale', 1.)
    mujoco.mj_setConst(model, plant.data)
    scorer = native.ContactScorer(plant, mapping, metadata, config)
    start, _ = nominal_start(TargetState(scorer.center, scorer.normal, np.zeros((3, 3)), 0.), config)
    start = start+np.array([0., .2, 0.])
    plant.reset(start+case.get('position_offset_m', np.zeros(3)), np.asarray(case.get('velocity_offset_m_s', np.zeros(3))))
    plant.data.qpos[plant.qadr+3:plant.qadr+7] = Rotation.from_euler('xyz', case.get('attitude_deg', [0., 0., 0.]), degrees=True).as_quat(scalar_first=True)
    plant.data.qvel[plant.dadr+3:plant.dadr+6] = case.get('body_rate_rad_s', [0., 0., 0.])
    mujoco.mj_forward(model, plant.data)
    sensors_config = json.loads((native.ROOT/'configs/asm2_recovery_validation.json').read_text())
    sensors = sensors_config['sensors']
    sensors['imu'] = json.loads((native.ROOT/sensors_config['imu_config']).read_text())
    sensors['translation_latency_s'] = case.get('translation_latency_s', sensors['translation_latency_s'])
    recorder = SensorRecorder(sensors, case['seed'], start_s=0., native_dt_s=model.opt.timestep,
                              held_thrust_N=metadata['mass_kg']*9.81, mass_kg=metadata['mass_kg'])
    clearance = (SceneClearance(output/'scene.mjb',
        [model.geom(g).name for g in scorer.patch_ids], [model.geom(g).name for g in scorer.plate_ids])
        if config.get('guidance') else None)
    runtime = FlightRuntime(metadata, mapping, config, method=method, initialize=initialize, clearance=clearance)
    geometry = _compiled_contact_geometry(plant, scorer)
    truth = plant.state()
    recorder.observe(0, truth.position, truth.velocity, truth.rotation, truth.omega_body)
    packet_index = 0
    native_tick = 0
    started = time.perf_counter()
    trace, contacts, states, commands = [], [], [plant.snapshot()], [np.zeros(4)]
    phases = ['ALIGNMENT_FLIGHT']
    runtime_wall_s = native_wall_s = 0.
    first, forbidden, separation, stable_since = None, None, None, None
    last_contact = None
    recovered, secondary, failure = False, False, None
    safe_abort, abort_stable_since = False, None
    envelope_stop = None
    maximum, minimum = np.asarray(config['flight_box_relative_m'])[1]+scorer.center, np.asarray(config['flight_box_relative_m'])[0]+scorer.center
    for _ in range(round(config['episode_limit_s']/config['control_dt_s'])):
        now = time_ns(plant.data.time)/1e9
        if time.perf_counter()-started > config['episode_wall_limit_s']:
            failure = 'EPISODE_WALL_BUDGET'
            break
        runtime.receive(recorder.packets[packet_index:])
        packet_index = len(recorder.packets)
        try:
            update_started = time.perf_counter()
            action, motors, diagnostic = runtime.update(now, geometry)
            runtime_wall_s += time.perf_counter()-update_started
        except (RuntimeError, RecoveryInitializationError) as error:
            failure = str(error)
            break
        if motors is None:
            plant.command(action, np.asarray(diagnostic['estimate']['angular_velocity_body_rad_s']))
            runtime.tracker.expected_activation += (1.-runtime.tracker.activation_decay)*(
                plant.data.ctrl[plant.motors]-runtime.tracker.expected_activation)
        else:
            plant.data.ctrl[plant.motors] = motors
        truth = plant.state()
        trace.append(dict(time_s=now, phase=runtime.phase, **diagnostic,
            predicted_motor_activation=runtime.tracker.expected_activation.tolist(),
            tracker_reference_rate=runtime.tracker.reference_rate.tolist(),
            truth_offline=dict(position=truth.position.tolist(), velocity=truth.velocity.tolist(),
                rotation=truth.rotation.tolist(), omega=truth.omega_body.tolist()),
            issued_motors=plant.data.ctrl[plant.motors].tolist(), actual_motors=plant.data.act[plant.activation].tolist()))
        physics_started = time.perf_counter()
        for _ in range(round(config['control_dt_s']/model.opt.timestep)):
            before = plant.state()
            mujoco.mj_step(model, plant.data)
            native_tick += 1
            truth = plant.state()
            recorder.observe(native_tick, truth.position, truth.velocity, truth.rotation, truth.omega_body,
                             before_velocity=before.velocity, before_rotation=before.rotation)
            records = scorer.active(plant.data, before) if plant.data.ncon else []
            if records:
                entry = dict(time_s=float(plant.data.time), interval_start_s=before.timestamp, records=records)
                contacts.append(entry)
                if first is None:
                    first = dict(**entry, qualified=(runtime.phase == 'APPROACH' and
                                 not any(record['rejected'] for record in records)),
                                 pre_contact_height_m=float(before.position[2]))
                elif last_contact is not None and truth.timestamp-last_contact > .03:
                    secondary = True
                if any('WRONG_AIRCRAFT_GEOM' in record['rejected'] or 'WRONG_SURFACE' in record['rejected'] for record in records):
                    forbidden = forbidden or entry
                    secondary = True
                last_contact = truth.timestamp
            speed = float(np.linalg.norm(truth.velocity))
            tilt = float(np.rad2deg(np.arccos(np.clip(truth.rotation[2, 2], -1., 1.))))
            rate = float(np.linalg.norm(truth.omega_body))
            speed_stop = speed_observation.observe(truth.timestamp, truth.velocity)
            if (not np.isfinite(np.r_[plant.data.qpos, plant.data.qvel, plant.data.act, plant.data.ctrl]).all()
                    or speed_stop or tilt > config['maximum_tilt_deg']
                    or rate > config['maximum_rate_rad_s'] or np.any(truth.position > maximum)
                    or np.any(truth.position < minimum)):
                failure = 'FLIGHT_ENVELOPE'
                envelope_stop = dict(time_s=truth.timestamp, speed_m_s=speed, tilt_deg=tilt,
                    rate_rad_s=rate, position_m=truth.position.tolist(), speed_stop=speed_stop,
                    tilt_stop=tilt > config['maximum_tilt_deg'],
                    rate_stop=rate > config['maximum_rate_rad_s'],
                    position_stop=bool(np.any(truth.position > maximum) or np.any(truth.position < minimum)),
                    finite=bool(np.isfinite(np.r_[plant.data.qpos, plant.data.qvel,
                                                plant.data.act, plant.data.ctrl]).all()))
                if first and first['time_s'] == truth.timestamp:
                    first['qualified'] = False
                break
            if any(plant.data.warning[index].number for index in (
                    mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                    mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADCTRL)):
                failure = 'PHYSICS_WARNING'
                break
        native_wall_s += time.perf_counter()-physics_started
        states.append(plant.snapshot())
        phases.append(runtime.phase)
        commands.append(np.r_[action.collective_force_N, action.body_rates_flu_rad_s])
        if failure:
            break
        if runtime.phase == 'GUIDANCE_EXIT' and not contacts:
            stable = (tilt <= config['recovery_tilt_deg'] and rate <= config['recovery_rate_rad_s']
                      and speed <= config['recovery_speed_m_s'])
            abort_stable_since = (truth.timestamp if abort_stable_since is None else abort_stable_since) if stable else None
            if abort_stable_since is not None and truth.timestamp-abort_stable_since >= config['recovery_hold_s']:
                safe_abort = True
                break
        if first:
            gap = float((truth.position-scorer.center) @ scorer.normal-scorer.pad_height)
            if separation is None and gap >= config['separation_m'] and truth.timestamp-last_contact >= .03:
                separation = truth.timestamp
            stable = (separation is not None and not secondary and tilt <= config['recovery_tilt_deg']
                      and rate <= config['recovery_rate_rad_s'] and speed <= config['recovery_speed_m_s']
                      and truth.position[2] >= first['pre_contact_height_m']-config['recovery_altitude_loss_m'])
            stable_since = (truth.timestamp if stable_since is None else stable_since) if stable else None
            if stable_since is not None and truth.timestamp-stable_since >= config['recovery_hold_s']:
                recovered = stable_since-first['time_s'] <= config['recovery_deadline_s']
                break
            if stable_since is None and truth.timestamp-first['time_s'] > config['recovery_deadline_s']:
                failure = 'RECOVERY_DEADLINE'
                break
    np.savez_compressed(output/'trajectory.npz', integration=np.asarray(states), commands=np.asarray(commands),
                        phases=np.asarray(phases))
    if case.get('mass_scale', 1.) != 1. or case.get('motor_tau_scale', 1.) != 1.:
        mujoco.mj_saveModel(model, str(output/'model.mjb'))
    else:
        (output/'model.mjb').symlink_to('scene.mjb')
    for name, records in [('trace', trace), ('sensors', recorder.packets)]:
        with gzip.open(output/f'{name}.jsonl.gz', 'wt') as stream:
            for record in records:
                stream.write(json.dumps(record, default=lambda value: value.value)+'\n')
    native.write_json(output/'contacts.json', contacts)
    observed_recovery = recovered
    recovered = speed_observation.qualify_recovery(recovered)
    diagnostic_stop = failure or ('RECOVERY_OBSERVED' if observed_recovery else
                                 'SAFE_ABORT_OBSERVED' if safe_abort else 'EPISODE_TIMEOUT')
    if speed_observation.first_violation is not None:
        failure = 'FLIGHT_ENVELOPE'
        safe_abort = False
    result = dict(case=case, method=method, alignment=runtime.alignment, first_contact=first,
        recovery_trigger_kind=runtime.trigger_kind, scheduled_release_since_s=runtime.scheduled_release_since,
        qualified_contact=bool(first and first['qualified']), forbidden=forbidden, secondary_contact=secondary,
        separated=separation is not None, separated_at_s=separation, stable_since_s=stable_since,
        recovered=recovered, joint_success=bool(first and first['qualified'] and recovered),
        final_contact_preview=None if runtime.reference is None else runtime.reference.contact_preview,
        safe_abort=safe_abort,
        guidance_events=[] if runtime.guidance is None else runtime.guidance.events,
        failure=failure or (None if recovered or safe_abort else 'EPISODE_TIMEOUT'), duration_s=float(plant.data.time),
        wall_s=time.perf_counter()-started, event=runtime.event, first_recovery_command_s=runtime.first_recovery_command,
        runtime_update_wall_s=runtime_wall_s, native_sensor_scoring_wall_s=native_wall_s,
        total_wall_s_including_setup=time.perf_counter()-run_started,
        first_plan_s=runtime.first_plan, replans=runtime.replans, source='SYNTHETIC_SENSOR_CLOSED_LOOP',
        confidence='UNCALIBRATED_COMPLEMENTARY_ESTIMATOR', target_source='REGISTERED_STATIC_MAP',
        scene_sha256=provenance['compiled_sha256'],
        assumptions=dict(initial_preflight_across_offset_m=.2, flight_alignment_at_s=1.2,
                         calibration_rate_peak_rad_s=[1., .12, 0.], calibration_period_s=1.2,
                         handover_duration_s=1.5, covariance=None, runtime_ground_truth=False,
                         synthetic_capability_execution=True, real_sensor_qualification=False))
    if speed_observation.diagnostic is not None:
        result['speed_diagnostic'] = dict(
            qualification_limit_m_s=speed_observation.limit,
            hard_stop_m_s=speed_observation.diagnostic['hard_stop_m_s'],
            first_violation=speed_observation.first_violation,
            peak_native_speed_m_s=speed_observation.peak,
            recovery_observed=observed_recovery, stop_reason=diagnostic_stop,
            envelope_stop=envelope_stop,
            formal_pass=speed_observation.first_violation is None and result['joint_success'])
    native.write_json(output/'summary.json', result)
    native.write_json(output/'config.json', config)
    native.write_json(output/'provenance.json', source_hashes)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', type=Path)
    parser.add_argument('--method', choices=['fixed', 'online'], default='online')
    parser.add_argument('--no-initialize', action='store_true')
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    case = (json.loads(args.case.read_text()) if args.case else
            json.loads((args.config or ROOT/'configs/asm2_integrated.json').read_text())['cases'][0])
    print(json.dumps(run(args.output, case=case, method=args.method,
                         initialize=not args.no_initialize, config_path=args.config), indent=2))


if __name__ == '__main__':
    main()
