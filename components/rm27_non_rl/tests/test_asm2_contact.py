"""Focused contracts for the non-RL ASM2 controller and the existing viewer."""
import json

import mujoco
import numpy as np

from rm27_contact_physics.model import TargetState, VehicleEstimate, unit
from rm27_contact_physics.control import motor_mixer
from rm27_sim.experiments.asm2_contact import DEFAULT_CONFIG
from rm27_non_rl.asm2_contact_control import (
    ApproachTracker, ContactGeometry, Observation, ObservationSource, nominal_start, plan_reference, recovery_reference,
)
from rm27_sim.replay.viewing import recorded_states


def test_piecewise_forward_reference_matches_scalar_sampling_and_join():
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    boundary = lambda p, v: tuple(np.asarray([x]) for x in (p, v, 0., 0.))
    first = Septic(boundary(0., .4), boundary(.15, .2), .5)
    second = Septic(boundary(.15, .2), boundary(.2, 0.), .5)
    zeros = np.zeros(3)
    base = Septic((zeros, zeros, zeros, zeros), (zeros, zeros, zeros, zeros), 1.)
    reference = Reference(base, 2., forward_segments=(first, second), forward_axis=np.array([1., 0., 0.]))
    times = np.linspace(2., 3.1, 221)
    values = reference.samples(times)
    np.testing.assert_allclose(values, np.array([reference.at(t) for t in times]).transpose(1, 0, 2), atol=1e-12)
    np.testing.assert_allclose(first.sample(.5), second.sample(0.), atol=1e-10)
    assert np.max(np.diff(values[1, :, 0])) < 1e-10
    assert np.isfinite(values).all()


def test_leveling_checks_complete_body_rotation_not_elapsed_time():
    from rm27_non_rl.asm2_contact_control import leveling_clearance
    geometry = ContactGeometry(np.array([[0., 0., -.02]]),
        np.array([[-.08, 0., -.02], [.08, 0., -.02]]), np.array([.045, .05]))
    normal = unit([-.46, 0., .89])
    near = sample(100., .025*normal, np.zeros(3))
    far = sample(0., .20*normal, np.zeros(3))
    assert leveling_clearance(near, np.zeros(3), normal, geometry) < .005
    assert leveling_clearance(far, np.zeros(3), normal, geometry) > .005


def test_incremental_recovery_only_adopts_clear_rotation_and_limits_blend_step():
    from rm27_non_rl.asm2_contact_control import clearance_limited_recovery, leveling_clearance, _requested_force
    from rm27_contact_physics.control import oriented_frame
    geometry = ContactGeometry(np.array([[0., 0., -.02]]),
        np.array([[-.08, 0., -.02], [.08, 0., -.02]]), np.array([.045, .05]))
    normal = unit([-.46, 0., .89])
    near = sample(.1, .025*normal, .2*normal)
    far = sample(.1, .20*normal, .2*normal)
    _, weight, diagnostic = clearance_limited_recovery(
        near, np.zeros(3), normal, geometry, .1, near.velocity, 0., .005, .005, 3.)
    assert weight == 0. and diagnostic['normal_release_fallback']
    desired, weight, diagnostic = clearance_limited_recovery(
        far, np.zeros(3), normal, geometry, .1, far.velocity, 0., .005, .005, 3.)
    assert 0. < weight <= .005/.065
    destination = oriented_frame(_requested_force(far, desired), [1., 0., 0.])
    assert leveling_clearance(far, np.zeros(3), normal, geometry, destination) >= .005
    assert diagnostic['admissible_rotation'] and np.isfinite(desired).all()


def test_coordinated_vertical_waypoint_preserves_boundary_and_segment_continuity():
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference, coordinate_braking
    start = (np.array([0., 0., 1.]), np.array([.4, 0., 0.]), np.zeros(3), np.zeros(3))
    end = (np.array([.3, 0., .9]), np.array([.1, 0., -.05]), np.array([-1., 0., -.1]), np.zeros(3))
    base = Reference(Septic(start, end, 1.), .8, aligned=.03)
    config = dict(forward_braking_s=.3, braking_vertical_waypoint=[.003, .02, -.1, .2])
    reference = coordinate_braking(base, start, end, np.array([-1., 0., 0.]), config)
    np.testing.assert_allclose(reference.at(.8), start, atol=1e-10)
    np.testing.assert_allclose(reference.at(1.8), end, atol=1e-9)
    first, second = reference.vertical_segments
    np.testing.assert_allclose(first.sample(first.duration), second.sample(0.), atol=1e-9)
    times = np.linspace(.8, 1.83, 104)
    np.testing.assert_allclose(reference.samples(times),
        np.asarray([reference.at(t) for t in times]).transpose(1, 0, 2), atol=1e-10)
    assert not np.allclose(reference.samples(times)[0, :, 2], base.samples(times)[0, :, 2])


def sample(timestamp, position, velocity, source='MUJOCO_ORACLE'):
    return VehicleEstimate(np.asarray(position), np.asarray(velocity), np.eye(3), np.zeros(3),
                           timestamp, source)


def test_recovery_motor_lead_preserves_hover_and_reports_real_limits():
    from rm27_contact_physics.teacher_interface import RateThrust
    config = json.loads(DEFAULT_CONFIG.read_text())
    tracker = ApproachTracker(dict(mass_kg=.142, max_rotor_thrust_N=1.104,
        inertia_body_kg_m2=np.eye(3)*.0001,
        mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046, .013),
        rate_gain_per_s=17., motor_time_constant_s=.04, hover_motor_thrust_N=[.348255]*4), config)
    state = sample(0., np.zeros(3), np.zeros(3), 'SYNTHETIC_ESTIMATE')
    motors, clipped = tracker.recovery_motor_command(state, RateThrust(np.zeros(3), .142*9.81))
    np.testing.assert_allclose(motors, [.348255]*4)
    assert not clipped
    before = tracker.expected_activation.copy()
    motors, clipped = tracker.recovery_motor_command(state, RateThrust(np.array([3.2, 0., 0.]), 4.416))
    assert clipped and motors.min() >= 0. and motors.max() <= 1.104
    assert np.linalg.norm(tracker.expected_activation-before) < np.linalg.norm(motors-before)


def test_estimate_latency_and_target_loss_do_not_restore_truth():
    config = json.loads(DEFAULT_CONFIG.read_text())
    case = next(case for case in config['cases'] if case['id'] == 'synthetic_outage')
    source = ObservationSource(case, config['synthetic'])
    normal = unit([-.46, 0., .89])
    for i in range(67):
        t = i * config['control_dt_s']
        observation = source.update(sample(t, [1., 0., 1.], [0., 0., 0.]), np.zeros(3), normal)
        if t < config['synthetic']['latency_s']:
            assert observation is None
        if .03 <= t < .16:
            assert observation.vehicle.timestamp <= t-config['synthetic']['latency_s']+1e-9
            assert observation.vehicle.source == 'SYNTHETIC_DELAYED'
            assert not np.array_equal(observation.vehicle.position, [1., 0., 1.])
    assert observation.predict(t, config['max_observation_age_s']) is None
    assert observation.predict_vehicle(t, config['max_observation_age_s']).source == 'SYNTHETIC_DELAYED'


def test_changed_handover_changes_reference_with_continuous_splice():
    config = json.loads(DEFAULT_CONFIG.read_text())
    target = TargetState(np.zeros(3), unit([-.4617, 0., .887]), np.zeros((3, 3)), 0.)
    p, v = nominal_start(target, config)
    initial = Observation(sample(0., p, v), target)
    geometry = ContactGeometry(
        np.array([[-.0275, y, -.01986] for y in (-.04, .04)]),
        np.array([[-.0275, y, -.01981] for y in (-.04, .04)]),
        np.array([.045, .05]),
    )
    tracker = ApproachTracker(dict(
        mass_kg=.142, max_rotor_thrust_N=1.104, inertia_body_kg_m2=np.eye(3)*.0001,
        mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046, .013), rate_gain_per_s=17.,
        motor_time_constant_s=.04, hover_motor_thrust_N=[.348255]*4,
    ), config)
    old = plan_reference(initial, config, geometry, tracker)
    assert old is not None
    measured_start = Observation(sample(0., initial.vehicle.position+[0., .002, 0.],
                                        initial.vehicle.velocity+[.002, 0., 0.]), target)
    from_estimate = plan_reference(measured_start, {**config, 'initialize_reference_from_estimate': True},
                                   geometry, tracker)
    assert from_estimate is not None
    np.testing.assert_allclose(from_estimate.at(0.)[0], measured_start.vehicle.position)
    np.testing.assert_allclose(from_estimate.at(0.)[1], measured_start.vehicle.velocity)
    t = config['plan_apply_lead_s']
    p, v, _, _ = old.at(t)
    case = next(case for case in config['cases'] if case['id'] == 'changed_handover')
    changed = Observation(sample(t, p + case['position_offset_m'], v + case['velocity_offset_m_s']), target)
    proposed = plan_reference(changed, config, geometry, tracker, old,
                              activation=tracker.activation_at(t, tracker.expected_activation))
    assert proposed is not None
    np.testing.assert_allclose(proposed.at(t+1e-10), old.at(t), atol=1e-5)
    end_blend = t + proposed.blend
    np.testing.assert_allclose(proposed.at(end_blend-1e-10), proposed.at(end_blend+1e-10), atol=1e-5)
    assert np.linalg.norm(proposed.at(t+.25)[0]-old.at(t+.25)[0]) > .01
    times = np.array([t, t+.125, end_blend, proposed.end])
    np.testing.assert_allclose(proposed.samples(times).transpose(1, 0, 2),
                               [proposed.at(when) for when in times], atol=1e-11)
    assert proposed.contact_preview['maximum_window_tangent_m_s'] <= config['contact_max_tangent_speed_m_s']
    # A fresh handover also needs authority to correct its measured body rate.
    from dataclasses import replace
    high_rate = replace(initial, vehicle=replace(initial.vehicle, omega_body=np.array([20., 0., 0.])))
    assert plan_reference(high_rate, {**config, 'check_initial_rate_demand': True}, geometry, tracker) is None


def test_reference_preview_includes_material_point_rotation_and_airframe_order():
    config = json.loads(DEFAULT_CONFIG.read_text())
    target = TargetState(np.zeros(3), np.array([0., 0., 1.]), np.zeros((3, 3)), 0.)
    times = np.array([0., .05])
    rotation = np.broadcast_to(np.eye(3), (2, 3, 3))
    pos = np.array([[0., 0., .024], [0., 0., .019]])
    vel = np.array([[0., 0., -.1], [0., 0., -.1]])
    pad = np.array([[.02, 0., -.02]])
    clean = ContactGeometry(pad, np.zeros((1, 3)), np.array([.045, .05]))
    assert clean.preview(target, np.eye(3), times, pos, vel, rotation, np.zeros((2, 3)), config)
    # Translation passes on its own. Angular velocity adds 0.28 m/s tangent.
    rotating = np.broadcast_to([0., 14., 0.], (2, 3))
    assert clean.preview(target, np.eye(3), times, pos, vel, rotation, rotating, config) is None
    obstructed = ContactGeometry(pad, np.array([[0., 0., -.025]]), clean.halfwidth)
    assert obstructed.preview(target, np.eye(3), times, pos, vel, rotation, np.zeros((2, 3)), config) is None


def test_finite_footprint_reserve_checks_rotation_gravity_and_uncertainty():
    from rm27_non_rl.asm2_recoverability import footprint_reserve
    geometry = ContactGeometry(np.array([[.02, .04, -.02], [-.02, -.04, -.02]]),
                               np.zeros((1, 3)), np.array([.045, .05]))
    target = TargetState(np.zeros(3), np.array([0., 0., 1.]), np.zeros((3, 3)), 0.)
    def check(**options):
        return footprint_reserve(geometry, target, np.eye(3), np.array([0., 0., .02]),
            np.zeros(3), np.eye(3), np.array([0., 0., 2.]),
            dict(horizon_s=.1, attitude_stress_deg=0., position_stress_m=0., **options))
    assert check()['accepted']
    # Pure yaw sweeps the lateral vertex past an edge without COM translation.
    fast = footprint_reserve(geometry, target, np.eye(3), np.array([0., 0., .02]),
        np.zeros(3), np.eye(3), np.array([0., 0., 8.]),
        dict(horizon_s=.1, attitude_stress_deg=0., position_stress_m=0.))
    assert not fast['accepted']
    strict = footprint_reserve(geometry, target, np.eye(3), np.array([0., 0., .02]),
        np.zeros(3), np.eye(3), np.zeros(3),
        dict(horizon_s=0., attitude_stress_deg=7., position_stress_m=.005))
    assert not strict['accepted']
    assert strict['confidence'] == 'ASSUMED_STRESS_NOT_CALIBRATED'


def test_center_offset_changes_contact_location_without_changing_contact_velocity():
    from copy import deepcopy
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['contact_planning']['surface_alignment'] = {}
    geometry = ContactGeometry(np.array([[.02, .04, -.02], [-.02, -.04, -.02]]),
                               np.zeros((1, 3)), np.array([.045, .05]))
    target = TargetState(np.zeros(3), unit([-.46, 0., .89]), np.zeros((3,3)), 0.)
    baseline = geometry.terminal(target, config)
    candidate = deepcopy(config)
    candidate['contact_planning']['surface_alignment']['center_slope_offset_m'] = -.008
    modified = geometry.terminal(target, candidate)
    np.testing.assert_allclose(modified[0]-baseline[0], .008*baseline[3][:,0], atol=1e-12)
    for old, new in zip(baseline[1:], modified[1:]):
        np.testing.assert_array_equal(old, new)


def test_footprint_invalid_assumptions_are_not_silently_accepted():
    import pytest
    from rm27_non_rl.asm2_recoverability import footprint_reserve
    g=ContactGeometry(np.array([[0.,0.,-.02]]),np.zeros((1,3)),np.array([.045,.05]))
    target=TargetState(np.zeros(3),np.array([0.,0.,1.]),np.zeros((3,3)),0.)
    for value in [-.1, float('nan'), float('inf')]:
        with pytest.raises(ValueError,match='Invalid footprint'):
            footprint_reserve(g,target,np.eye(3),np.array([0.,0.,.02]),np.zeros(3),np.eye(3),np.zeros(3),
                dict(horizon_s=value,attitude_stress_deg=0.,position_stress_m=0.))


def test_centered_terminal_reaches_the_surface_with_aligned_thrust():
    from rm27_non_rl.asm2_contact_control import GRAVITY

    config = json.loads(DEFAULT_CONFIG.read_text())
    config['contact_planning']['surface_alignment'] = {'normal_accel_m_s2': .5}
    config['contact_reference_speed_m_s'] = .075
    elapsed = .01
    config['contact_reference_depth_m'] = .075*elapsed - .25*elapsed**2
    target = TargetState(np.array([1., 2., 3.]), unit([-.46, 0., .89]), np.zeros((3, 3)), 0.)
    floor = np.array([[x, y, -.02] for x in (-.03, .03) for y in (-.04, .04)])
    geometry = ContactGeometry(floor, np.zeros((1, 3)), np.array([.045, .05]))
    endpoint, velocity, acceleration, frame = geometry.terminal(target, config)
    at_contact = endpoint - elapsed*velocity + .5*elapsed**2*acceleration
    np.testing.assert_allclose(at_contact-.02*frame[:, 2], target.position, atol=1e-12)
    np.testing.assert_allclose((at_contact+floor@frame.T-target.position)@target.normal, 0., atol=1e-12)
    np.testing.assert_allclose(unit(acceleration+GRAVITY), target.normal, atol=1e-12)
    np.testing.assert_allclose((velocity-elapsed*acceleration)@target.normal, -.075, atol=1e-12)


def test_full_surface_window_rejects_an_early_raised_contact_edge():
    from scipy.spatial.transform import Rotation

    config = json.loads(DEFAULT_CONFIG.read_text())
    target = TargetState(np.zeros(3), np.array([0., 0., 1.]), np.zeros((3, 3)), 0.)
    floor = np.array([[x, y, -.02] for x in (-.03, .03) for y in (-.04, .04)])
    geometry = ContactGeometry(floor, np.zeros((1, 3)), np.array([.045, .05]))
    times = np.array([0., .02, .04])
    pos = np.array([[0., 0., .024], [0., 0., .022], [0., 0., .0199]])
    velocity = np.broadcast_to([0., 0., -.1], (3, 3))
    rates = np.zeros((3, 3))
    rotation = Rotation.from_euler('y', [[0.], [.3], [0.]], degrees=True).as_matrix()
    assert geometry.preview(target, np.eye(3), times, pos, velocity, rotation, rates, config)
    config['contact_planning']['surface_alignment'] = {
        'full_surface_lead_s': .025, 'maximum_gap_span_m': .00005,
    }
    assert geometry.preview(target, np.eye(3), times, pos, velocity, rotation, rates, config) is None
    aligned = np.broadcast_to(np.eye(3), (3, 3, 3))
    result = geometry.preview(target, np.eye(3), times, pos, velocity, aligned, rates, config)
    assert result['full_surface_maximum_gap_span_m'] == 0.


def test_release_preview_resolves_shell_clearance_from_contact_arm():
    from types import SimpleNamespace

    geometry = ContactGeometry(
        np.array([[.03, 0., 0.]]), np.array([[-.03, 0., .00005]]), np.array([.045, .05]),
    )
    tracker = SimpleNamespace(mass=.142, inertia=np.full(3, .0001), dt=.005)
    normal = np.array([[0., 0., 1.]])
    rotation = np.eye(3)[None]
    points = np.array([0])
    stopped = np.zeros((1, 3))
    fast = geometry.release_preview(normal, np.array([-.08]), rotation, stopped, points, tracker)
    slow = geometry.release_preview(normal, np.array([-.01]), rotation, stopped, points, tracker)
    np.testing.assert_allclose(fast['minimum_shell_gap_at_touch_m'], .00005, atol=1e-12)
    np.testing.assert_allclose(fast['minimum_shell_gap_coast_m'], -.00035, atol=1e-12)
    assert slow['minimum_shell_gap_coast_m'] > fast['minimum_shell_gap_coast_m']
    # A normal impulse at the forward pad rotates the rear shell inward.
    assert fast['maximum_rate_after_normal_impulse_rad_s'] > 0.
    assert fast['minimum_shell_gap_after_normal_impulse_m'] < slow['minimum_shell_gap_after_normal_impulse_m']
    delayed = geometry.release_preview(normal, np.array([-.08]), rotation, stopped, points, tracker,
                                        horizon_s=.035)
    assert delayed['release_projection_horizon_s'] == .035
    assert delayed['minimum_shell_gap_coast_m'] < fast['minimum_shell_gap_coast_m']


def test_recovery_starts_from_estimated_velocity_and_damps_to_zero():
    estimate = sample(1., [2., 0., 1.], [.17, -.06, .04], 'SYNTHETIC_DELAYED')
    n = unit([-.46, 0., .89])
    _, beginning, *_ = recovery_reference(estimate, n, 0., estimate.velocity)
    _, retreat, *_ = recovery_reference(estimate, n, .5, estimate.velocity)
    _, stop, *_ = recovery_reference(estimate, n, 1.3, estimate.velocity)
    np.testing.assert_array_equal(beginning, estimate.velocity)
    assert retreat @ n > 0
    np.testing.assert_array_equal(stop, np.zeros(3))


def test_thrust_frame_angular_feedforward_matches_rotation_derivative():
    from scipy.spatial.transform import Rotation
    from rm27_non_rl.asm2_contact_control import GRAVITY, _reference_angular_velocity, oriented_frame

    acceleration = np.array([-2.1, 1.7, .5])
    jerk = np.array([6.2, -3.9, 5.4])
    epsilon = 1e-6
    left = oriented_frame(GRAVITY + acceleration - epsilon * jerk, [1., 0., 0.])
    right = oriented_frame(GRAVITY + acceleration + epsilon * jerk, [1., 0., 0.])
    derivative = Rotation.from_matrix(right @ left.T).as_rotvec() / (2. * epsilon)
    np.testing.assert_allclose(_reference_angular_velocity(acceleration, jerk), derivative, atol=1e-9, rtol=0)


def test_full_state_recording_preserves_extra_joint_and_actuator(tmp_path):
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
        <body><joint name="extra" type="slide"/><geom size=".01" mass="1"/></body>
        <body name="uav" pos="0 0 1"><freejoint/><geom size=".01" mass="1"/></body>
        </worldbody><actuator>
        <general joint="extra" dyntype="filter" dynprm=".04"/>
        </actuator></mujoco>
    """)
    original = mujoco.MjData(model)
    original.qpos[0] = .012
    original.qvel[0] = -.017
    original.act[0] = .18
    original.ctrl[0] = .27
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    rows = np.empty((2, mujoco.mj_stateSize(model, state_type)))
    for index, row in enumerate(rows):
        original.time = .005 * index
        mujoco.mj_getState(model, original, row, state_type)
    np.savez(tmp_path/'trajectory.npz', integration=rows, phases=['APPROACH', 'RECOVERY'])
    restored = mujoco.MjData(model)
    timeline, phases, restore = recorded_states(tmp_path, model, restored)
    restore(timeline[-1])
    assert phases[-1] == 'RECOVERY'
    np.testing.assert_array_equal(restored.qpos, original.qpos)
    np.testing.assert_array_equal(restored.qvel, original.qvel)
    np.testing.assert_array_equal(restored.act, original.act)
    np.testing.assert_array_equal(restored.ctrl, original.ctrl)
    assert restored.time == original.time


def test_scorer_reads_real_contact_and_rejects_unregistered_surface():
    from types import SimpleNamespace
    from rm27_sim.experiments.asm2_contact import ContactScorer

    model = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
          <asset><mesh name="pad" vertex="
             -.01 -.01 -.01   .01 -.01 -.01   -.01 .01 -.01   .01 .01 -.01
             -.01 -.01 .01    .01 -.01 .01    -.01 .01 .01    .01 .01 .01"/></asset>
          <worldbody>
            <geom name="rmuc2027_surface_36293" type="box" pos="0 0 -.01" size=".10 .10 .01"/>
            <body name="uav" pos="0 0 .011">
              <freejoint/>
              <geom name="pad" type="mesh" mesh="pad" mass=".142"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    data.qvel[2] = -.1
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['front_plate_backing_indices'] = [36293]
    mapping = dict(armor_surface_center_m=[0., 0., 0.], armor_normal=[0., 0., 1.],
                   slope_axis=[1., 0., 0.], across_axis=[0., 1., 0.],
                   contact_region_halfsize_slope_across_m=[.045, .05], registered_contact_geoms=[])
    plant = SimpleNamespace(model=model, body=model.body('uav').id)
    scorer = ContactScorer(plant, mapping, {'contact_patches': [{'name': 'pad'}]}, config)
    for _ in range(25):
        before = sample(float(data.time), data.qpos[:3].copy(), data.qvel[:3].copy())
        mujoco.mj_step(model, data)
        contacts = scorer.active(data, before)
        if contacts:
            break
    assert contacts and all(not row['rejected'] and row['force_N'] > 0 for row in contacts)
    scorer.plate_ids.clear()
    rejected = scorer.active(data, before)
    assert all(row['rejected'] == ['WRONG_SURFACE'] for row in rejected)


def test_guidance_result_rejects_epoch_parent_expiry_and_invalid_state():
    from rm27_non_rl.asm2_guidance import PendingPlan
    pending = PendingPlan(None, 'TERMINAL', 2, 1, 3, .1, .08, .1, .165)
    args = dict(now=.16, parent=1, epoch=3, valid=True)
    assert pending.invalid_reason(**args) is None
    for change, reason in [({'epoch':4},'ESTIMATOR_EPOCH_CHANGED'),
                           ({'parent':2},'PARENT_REFERENCE_CHANGED'),
                           ({'now':.171},'REFERENCE_EXPIRED'),
                           ({'valid':False},'ESTIMATE_UNAVAILABLE')]:
        assert pending.invalid_reason(**(args|change)) == reason


def test_guidance_splice_preserves_tracker_lookahead_and_vector_sampling():
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    from rm27_non_rl.asm2_guidance import continuous
    config = json.loads(DEFAULT_CONFIG.read_text())
    z = np.zeros(3)
    old = Reference(Septic((z,z,z,z), (np.array([.2,0,0]),z,z,z),1.),0.)
    t=.2
    new = Reference(Septic(old.at(t),(np.array([.3,.1,0]),z,z,z),1.),t,
                    previous=old,blend=.5,blend_delay_s=.005)
    tracker = ApproachTracker(dict(mass_kg=.142,max_rotor_thrust_N=1.104,
        inertia_body_kg_m2=np.eye(3)*.0001,
        mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046,.013),rate_gain_per_s=17.,
        motor_time_constant_s=.04,hover_motor_thrust_N=[.348255]*4),config)
    state=sample(t,old.at(t)[0],old.at(t)[1])
    assert continuous(old,new,t,tracker,state)
    times=np.array([t,t+.002,t+.005,t+.006,t+.2,t+.505,t+.6])
    np.testing.assert_allclose(new.samples(times).transpose(1,0,2),[new.at(ti) for ti in times],atol=1e-11)
    np.testing.assert_allclose(old.at(t+.005),new.at(t+.005),atol=1e-11)
    nested_time = .4
    nested = Reference(Septic(new.at(nested_time), (np.array([.25, .05, 0.]), z, z, z), 1.),
                       nested_time, previous=new, blend=.5, blend_delay_s=.005)
    nested_state = sample(nested_time, new.at(nested_time)[0], new.at(nested_time)[1])
    assert continuous(new, nested, nested_time, tracker, nested_state)
    nested_times = np.linspace(nested_time, nested_time+.8, 40)
    np.testing.assert_allclose(nested.samples(nested_times).transpose(1, 0, 2),
                               [nested.at(timestamp) for timestamp in nested_times], atol=1e-10)


def test_adjustment_projects_region_without_unique_handover_point():
    from types import SimpleNamespace
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    from rm27_non_rl.asm2_guidance import adjustment
    config={'guidance':{'region_bounds_m':[[.1,-.02,-.2],[.3,.02,.2]]},'plan_blend_s':[.5]}
    z=np.zeros(3);target=TargetState(z,np.array([1.,0,0]),np.zeros((3,3)),0.)
    ends=[]
    for p in [[.15,.01,.05],[.25,-.01,-.05]]:
        obs=Observation(sample(0.,p,z),target)
        old=Reference(Septic((p,z,z,z),(p,z,z,z),1.),0.)
        ref=adjustment(obs,SimpleNamespace(dt=.005),config,old,kind='REPOSITION',duration=1.5)
        np.testing.assert_allclose(ref.at(ref.end)[0],p,atol=1e-12)
        ends.append(ref.at(ref.end)[0])
    assert np.linalg.norm(ends[0]-ends[1]) > .1


def test_scene_query_detects_forbidden_body_using_independent_model(tmp_path):
    import time
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    from rm27_non_rl.asm2_guidance import SceneClearance
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="wall" type="box" size=".05 .5 .5" pos=".5 0 .5"/>
      <body name="uav"><freejoint name="asm2_free"/>
      <geom name="shell" type="sphere" size=".1" mass=".142"/></body>
      </worldbody></mujoco>''')
    path=tmp_path/'model.mjb';mujoco.mj_saveModel(model,str(path))
    check=SceneClearance(path,[],[]);z=np.zeros(3)
    for endpoint,expected in [([.1,0,.5],True),([.5,0,.5],False)]:
        ref=Reference(Septic(([0,0,.5],z,z,z),(endpoint,z,z,z),1.),0.)
        assert check.check(ref,0.,time.perf_counter()+1.)[0] is expected
    assert check.check(ref,0.,time.perf_counter()-1.) == (False,'SOLVER_DEADLINE')


def test_terminal_can_start_after_a_completed_guidance_hold():
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    config=json.loads(DEFAULT_CONFIG.read_text())
    config.update(initialize_reference_from_estimate=True,command_continuous_splice=True)
    target=TargetState(np.zeros(3),unit([-.4617,0.,.887]),np.zeros((3,3)),2.)
    p,v=nominal_start(target,config);z=np.zeros(3)
    guide=Reference(Septic((p,z,z,z),(p,z,z,z),1.),0.)
    obs=Observation(sample(2.,p,z),target)
    geometry=ContactGeometry(np.array([[-.0275,y,-.01986] for y in (-.04,.04)]),
        np.array([[-.0275,y,-.01981] for y in (-.04,.04)]),np.array([.045,.05]))
    tracker=ApproachTracker(dict(mass_kg=.142,max_rotor_thrust_N=1.104,
        inertia_body_kg_m2=np.eye(3)*.0001,mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046,.013),
        rate_gain_per_s=17.,motor_time_constant_s=.04,hover_motor_thrust_N=[.348255]*4),config)
    ref=plan_reference(obs,config,geometry,tracker,previous=guide)
    assert ref is not None
    np.testing.assert_allclose(ref.at(2.),guide.at(2.),atol=1e-12)


def test_scene_query_encloses_mesh_parts_and_preserves_finite_obstacle(tmp_path):
    from scipy.spatial.transform import Rotation
    from rm27_sim.simulation.clearance import SceneClearance
    source = mujoco.MjModel.from_xml_string('''<mujoco><asset>
      <mesh name="part" vertex="-.02 -.02 -.02 .02 -.02 -.02 0 .02 -.02 0 0 .02"/>
      </asset><worldbody><geom name="plate" type="box" size=".03 .03 .01" pos="0 0 .2"/>
      <body name="uav"><freejoint name="asm2_free"/>
      <geom name="left" type="mesh" mesh="part" pos="-.04 0 0"/>
      <geom name="right" type="mesh" mesh="part" pos=".04 0 0"/>
      </body></worldbody></mujoco>''')
    path = tmp_path/'native.mjb'
    mujoco.mj_saveModel(source, str(path))
    checker = SceneClearance(path, [], ['plate'])
    assert checker.enclosure['query_shell']['maximum_outside_m'] < 1e-12
    original = mujoco.MjData(source)
    native_hits = 0
    for height in (.15, .19, .2, .21, .25):
        for roll in (-.3, 0., .3):
            original.qpos[:] = [0., 0., height, *Rotation.from_euler('x', roll).as_quat(scalar_first=True)]
            checker.data.qpos[:] = original.qpos
            for model, data in ((source, original), (checker.model, checker.data)):
                mujoco.mj_kinematics(model, data)
                mujoco.mj_collision(model, data)
            native_collision = any(contact.dist <= 0. for contact in original.contact)
            query_collision = any(contact.dist <= 0. for contact in checker.data.contact)
            native_hits += native_collision
            assert not native_collision or query_collision
    assert native_hits > 0


def test_query_deadline_and_patch_permission_do_not_remove_forbidden_shell(tmp_path):
    import time
    from rm27_contact_physics.control import Septic
    from rm27_non_rl.asm2_contact_control import Reference
    from rm27_sim.simulation.clearance import SceneClearance
    source = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="plate" type="box" size=".2 .2 .01"/>
      <body name="uav"><freejoint name="asm2_free"/>
      <geom name="patch" type="sphere" size=".01" pos="0 0 -.02"/>
      <geom name="shell" type="sphere" size=".02" pos=".03 0 0"/>
      </body></worldbody></mujoco>''')
    path = tmp_path/'native.mjb'
    mujoco.mj_saveModel(source, str(path))
    checker = SceneClearance(path, ['patch'], ['plate'])
    zeros = np.zeros(3)
    for height, expected in ((.035, True), (.015, False)):
        boundary = ([0., 0., height], zeros, zeros, zeros)
        reference = Reference(Septic(boundary, boundary, .1), 0.,
                              contact_preview={'earliest_window_s': 0.})
        assert checker.check(reference, 0., time.perf_counter()+1.)[0] == expected
        assert checker.check(reference, 0., time.perf_counter()-1.) == (False, 'SOLVER_DEADLINE')
