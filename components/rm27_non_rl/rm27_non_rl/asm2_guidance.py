"""Bounded sensor-state guidance; independent static-scene collision queries.

Geometry queries are discrete model predictions, not continuous clearance proofs.
The evaluator never receives the live plant, contacts, or actuator state.
"""
from copy import deepcopy
from dataclasses import dataclass
import time

import numpy as np

from rm27_contact_physics.control import Septic
from rm27_sim.simulation.clearance import SceneClearance
from .asm2_contact_control import (GRAVITY, Reference, _reference_attitude_and_rate,
                                    plan_reference)


@dataclass
class PendingPlan:
    reference: Reference
    kind: str
    identity: int
    parent: int
    epoch: int
    requested: float
    acquisition: float
    valid_at: float
    expires: float

    def invalid_reason(self, *, now, parent, epoch, valid):
        if self.epoch != epoch:
            return 'ESTIMATOR_EPOCH_CHANGED'
        if self.parent != parent:
            return 'PARENT_REFERENCE_CHANGED'
        if now > self.expires+1e-9:
            return 'REFERENCE_EXPIRED'
        if not valid:
            return 'ESTIMATE_UNAVAILABLE'
        return None


def continuous(old, new, when, tracker, state):
    """Same tracker state must produce the same command at a splice."""
    if not np.allclose(old.at(when), new.at(when), atol=1e-8, rtol=0):
        return False
    a, b = deepcopy(tracker), deepcopy(tracker)
    ca, ma, _ = a.command(state, old, when)
    cb, mb, _ = b.command(state, new, when)
    return (np.allclose(ma, mb, atol=1e-5, rtol=0)
            and np.allclose(ca.body_rates_flu_rad_s, cb.body_rates_flu_rad_s, atol=1e-5, rtol=0))


def adjustment(observation, tracker, config, previous, *, kind, duration):
    """State-relative braking/projection, never a return to nominal_start."""
    s, target = observation.vehicle, observation.target
    n = target.normal / np.linalg.norm(target.normal)
    across = np.cross([0., 0., 1.], n); across /= np.linalg.norm(across)
    slope = np.cross(n, across)
    frame = np.column_stack((n, across, slope))
    relative = frame.T @ (s.position-target.position)
    # Nearest point in a region, preserving all coordinates already inside.
    bounds = np.asarray(config['guidance']['region_bounds_m'])
    goal = np.clip(relative, bounds[0], bounds[1])
    if kind == 'BRAKE':
        endpoint = s.position + .5*duration*s.velocity
    else:
        endpoint = target.position + frame @ goal
    velocity = np.zeros(3)
    a,j = previous.at(s.timestamp)[2:]
    return Reference(Septic((s.position, s.velocity, a, j),
                           (endpoint, velocity, np.zeros(3), np.zeros(3)), duration),
                     s.timestamp, previous, config['plan_blend_s'][0], blend_delay_s=tracker.dt)


def adjustment_feasible(obs, ref, tracker, config, activation):
    times = np.linspace(ref.start, ref.end, int(np.ceil(ref.trajectory.duration/tracker.dt))+1)
    p,v,a,j = ref.samples(times)
    lo,hi = np.asarray(config['flight_box_relative_m'])+obs.target.position
    force = a+GRAVITY
    if (np.any(p < lo) or np.any(p > hi) or np.max(np.linalg.norm(v,axis=1)) > .85*config['maximum_speed_m_s']
        or np.any(force[:,2] <= 3.) or np.max(np.degrees(np.arctan2(np.linalg.norm(force[:,:2],axis=1),force[:,2]))) > config['maximum_tilt_deg']-5.):
        return False
    r,w = _reference_attitude_and_rate(a,j)
    return tracker.reference_demand(times,force,r,w,obs.vehicle,activation) is not None


class AutonomousGuidance:
    def __init__(self, config, clearance):
        self.config, self.clearance = config, clearance
        self.current = self.pending = None
        self.identity = 0
        self.serial = 0
        self.next_query = 0.
        self.events = []
        self.kind = 'INITIALIZE'
        self.started = None
        self.terminal = False
        self.last_command = None

    def update(self, observation, estimate, tracker, geometry):
        now = observation.vehicle.timestamp
        epoch = estimate['epoch_id']
        valid = estimate['world_alignment_observed'] and all(c['valid'] for c in estimate['channels'].values())
        if not valid:
            self.pending = None
            raise RuntimeError('GUIDANCE_ESTIMATE_UNAVAILABLE')
        if self.current is None:
            self.started = now
            s = observation.vehicle
            acceleration = s.rotation[:,2]*(np.sum(tracker.expected_activation)/tracker.mass)-GRAVITY
            jerk = s.rotation @ np.cross(s.omega_body,[0.,0.,np.sum(tracker.expected_activation)/tracker.mass])
            duration = self.config['guidance']['durations_s'][-1]
            self.current = Reference(Septic((s.position,s.velocity,acceleration,jerk),
                (s.position+.5*duration*s.velocity,np.zeros(3),np.zeros(3),np.zeros(3)),duration),now)
            # This bootstrap is a proposal only. Never issue it without validation.
            self.kind = 'UNVALIDATED_BOOTSTRAP'
        if self.pending is not None and now+1e-9 >= self.pending.reference.start:
            pending = self.pending
            reason = pending.invalid_reason(now=now,parent=self.identity,epoch=epoch,valid=valid)
            if reason is None:
                self.current, self.identity, self.kind = pending.reference,pending.identity,pending.kind
                self.terminal = self.kind == 'TERMINAL'
            self.events.append(dict(time_s=now,status='APPLIED' if reason is None else 'REJECTED',
                reason=reason,identity=pending.identity,parent=pending.parent,kind=pending.kind,epoch=epoch,
                position_m=observation.vehicle.position.tolist(), velocity_m_s=observation.vehicle.velocity.tolist(),
                rotation_body_to_world=observation.vehicle.rotation.tolist(),
                body_rate_rad_s=observation.vehicle.omega_body.tolist(),
                predicted_motor_activation_N=tracker.expected_activation.tolist(),
                target_position_m=observation.target.position.tolist(), target_normal=observation.target.normal.tolist()))
            self.pending = None
        if self.terminal:
            return self.current, 'APPROACH'
        if now-self.started >= self.config['guidance']['decision_horizon_s']:
            self.kind = 'EXIT' if self.kind in ('BRAKE','REPOSITION') and now >= self.current.end else self.kind
        if now >= self.next_query and self.pending is None and self.kind != 'EXIT':
            started = time.perf_counter(); deadline = started+self.config['plan_wall_deadline_s']
            cpu_started = time.process_time()
            apply = now+self.config['plan_apply_lead_s']
            predicted = observation.predict(apply,self.config['max_observation_age_s']+self.config['plan_apply_lead_s'])
            if predicted is None:
                raise RuntimeError('GUIDANCE_STATE_EXPIRED')
            activation = tracker.activation_at(apply-now, self.last_command if self.last_command is not None else tracker.expected_activation)
            attempts = []
            search_started = time.perf_counter()
            candidate = (plan_reference(predicted,self.config,geometry,tracker,self.current,activation=activation,
                                        diagnostics=attempts)
                         if self.config['guidance'].get('allow_terminal',True) else None)
            search_wall = time.perf_counter()-search_started
            geometry_wall = 0.
            reason = ('NO_FEASIBLE_TERMINAL' if self.config['guidance'].get('allow_terminal',True)
                      else 'CONTACT_DISABLED_FOR_APPROACH_VALIDATION')
            kind = 'TERMINAL'
            if candidate is not None:
                geometry_started = time.perf_counter()
                ok,reason = self.clearance.check(candidate,apply,deadline)
                geometry_wall += time.perf_counter()-geometry_started
                if not ok: candidate=None
            # Do not repeatedly replace a still-executable adjustment.
            if candidate is None and (self.kind == 'UNVALIDATED_BOOTSTRAP' or now >= self.current.end):
                for kind in ('REPOSITION','BRAKE'):
                    for duration in self.config['guidance']['durations_s']:
                        ref=adjustment(predicted,tracker,self.config,self.current,kind=kind,duration=duration)
                        if time.perf_counter() >= deadline: break
                        if not adjustment_feasible(predicted,ref,tracker,self.config,activation): continue
                        geometry_started = time.perf_counter()
                        ok,reason = self.clearance.check(ref,apply,deadline)
                        geometry_wall += time.perf_counter()-geometry_started
                        if ok: candidate=ref; break
                    if candidate is not None or time.perf_counter() >= deadline: break
            if candidate is not None and time.perf_counter() <= deadline:
                if not continuous(self.current,candidate,apply,tracker,predicted.vehicle):
                    candidate=None;reason='DISCONTINUOUS_SPLICE'
            elapsed=time.perf_counter()-started
            if elapsed > self.config['plan_wall_deadline_s']:
                candidate=None;reason='SOLVER_DEADLINE'
            self.serial += 1
            channels=estimate['channels']; acquired=min(c['acquisition_s'] for c in channels.values() if c['acquisition_s'] is not None)
            record=dict(time_s=now,status='PENDING' if candidate is not None else 'REJECTED',reason=reason if candidate is None else None,
                kind=kind,identity=self.serial,parent=self.identity,epoch=epoch,acquisition_s=acquired,
                state_valid_at_s=now,application_s=apply,solve_wall_s=elapsed,
                search_wall_s=search_wall, geometry_wall_s=geometry_wall,
                solve_process_s=time.process_time()-cpu_started,
                terminal_attempts=attempts, candidate_count=len(attempts))
            self.events.append(record)
            if candidate is not None:
                self.pending=PendingPlan(candidate,kind,self.serial,self.identity,epoch,now,acquired,now,apply+self.config['control_dt_s'])
            self.next_query=now+self.config['plan_period_s']
        if self.kind == 'UNVALIDATED_BOOTSTRAP':
            # Certify the actual bridge executed during the application lead.
            deadline=time.perf_counter()+self.config['plan_wall_deadline_s']
            if not adjustment_feasible(observation,self.current,tracker,self.config,tracker.expected_activation):
                raise RuntimeError('NO_VALIDATED_INITIAL_BRIDGE')
            ok,reason=self.clearance.check(self.current,now,deadline,
                end_at=self.pending.reference.start if self.pending is not None else None)
            if not ok: raise RuntimeError(reason or 'NO_VALIDATED_INITIAL_BRIDGE')
            self.kind='BRAKE'
        return self.current, 'GUIDANCE_'+self.kind
