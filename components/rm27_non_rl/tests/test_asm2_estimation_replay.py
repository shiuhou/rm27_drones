"""End-to-end sensor-only replay tests; handmade packets, not flight evidence."""
import gzip
import json
from pathlib import Path
import numpy as np
import pytest
from rm27_non_rl.asm2_estimation_replay import run, runtime_packet, read_lines


def test_speed_diagnostic_preserves_fail_after_stabilization():
    from rm27_non_rl.asm2_integrated import SpeedEnvelopeObservation
    original = SpeedEnvelopeObservation({'maximum_speed_m_s': .95})
    diagnostic = SpeedEnvelopeObservation({'maximum_speed_m_s': .95,
                                          'speed_diagnostic': {'hard_stop_m_s': 2.}})
    assert not original.observe(1., [.8, 0., 0.])
    assert original.observe(2., [-.951, 0., 0.])
    assert not diagnostic.observe(2., [-.951, 0., 0.])
    assert not diagnostic.observe(3., [.2, 0., 0.])
    assert diagnostic.first_violation == original.first_violation
    assert not diagnostic.qualify_recovery(True)
    assert diagnostic.observe(4., [2.01, 0., 0.])
    assert diagnostic.first_violation['time_s'] == 2.
    with pytest.raises(ValueError):
        SpeedEnvelopeObservation({'maximum_speed_m_s': .95,
                                  'speed_diagnostic': {'hard_stop_m_s': .9}})


def test_integrated_scheduled_release_is_not_a_contact_detection(monkeypatch):
    from types import SimpleNamespace
    from rm27_contact_physics.control import motor_mixer
    from rm27_sim.experiments.asm2_contact import DEFAULT_CONFIG
    from rm27_non_rl.asm2_integrated import FlightRuntime

    config = json.loads(DEFAULT_CONFIG.read_text())
    config['scheduled_separation'] = True
    metadata = dict(mass_kg=.142, max_rotor_thrust_N=1.104,
                    inertia_body_kg_m2=np.eye(3)*.0001,
                    mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046, .013),
                    rate_gain_per_s=17., motor_time_constant_s=.04, hover_motor_thrust_N=[.348255]*4)
    runtime = FlightRuntime(metadata, {'armor_normal':[-.6, 0., .8],
                                      'armor_surface_center_m':[0., 0., 0.]}, config)
    runtime.alignment = {'evidence':'fabricated software fixture'}
    runtime.reference = SimpleNamespace(contact_preview={'time_s':.2})
    estimate = dict(time_s=.2, epoch_id=0, world_alignment_observed=True,
                    position_m=[.1, 0., .3], velocity_m_s=[.1, 0., -.03],
                    rotation_body_to_local=np.eye(3).tolist(), angular_velocity_body_rad_s=[0., 0., 0.],
                    channels={name:dict(acquisition_s=.2, delivery_s=.2, valid=True)
                              for name in ('position','velocity','attitude','body_rate')})
    monkeypatch.setattr(runtime.estimator, 'estimate', lambda now:estimate)
    action, _, diagnostic = runtime.update(.2, None)
    assert runtime.event is None
    assert runtime.trigger_kind == 'REFERENCE_SCHEDULE_NOT_CONTACT_DETECTION'
    assert runtime.first_recovery_command == .2
    assert np.isfinite(action.body_rates_flu_rad_s).all()
    assert diagnostic['supervisor']['capabilities']
    assert all(not value['trusted'] for value in diagnostic['supervisor']['capabilities'].values())
    from rm27_non_rl.asm2_contact_control import ContactGeometry
    config['leveling_clearance_m'] = .005
    geometry = ContactGeometry(np.array([[0., 0., -.02]]),
        np.array([[-.08, 0., -.02], [.08, 0., -.02]]), np.array([.045, .05]))
    estimate['position_m'] = [0., 0., 0.]
    _, _, diagnostic = runtime.update(.4, geometry)
    assert runtime.leveling_since is None
    assert diagnostic['leveling_gate']['recovery_reference_elapsed_s'] == .025
    estimate['position_m'] = [-.12, 0., .16]
    estimate['velocity_m_s'] = [-.1, 0., .1]
    _, _, diagnostic = runtime.update(.405, geometry)
    assert runtime.leveling_since == .405
    assert diagnostic['leveling_gate']['recovery_reference_elapsed_s'] == pytest.approx(.025)
    config.update(incremental_clearance_recovery=True, recovery_motor_lead=True,
                  recovery_reference_feedforward=True)
    _, motors, diagnostic = runtime.update(.410, geometry)
    assert np.isfinite(motors).all() and (motors >= 0).all() and (motors <= metadata['max_rotor_thrust_N']).all()
    np.testing.assert_array_equal(diagnostic['recovery_reference_derivative']['jerk_m_s3'], np.zeros(3))
    _, _, diagnostic = runtime.update(.415, geometry)
    assert np.linalg.norm(diagnostic['recovery_reference_derivative']['jerk_m_s3']) > 0.
    assert diagnostic['incremental_leveling']['admissible_rotation']
    estimate['channels']['attitude']['valid'] = False
    estimate['ground_truth'] = {'rotation':np.eye(3).tolist()}
    with pytest.raises(RuntimeError, match='ATTITUDE_ESTIMATE_UNAVAILABLE'):
        runtime.update(.420, None)


def test_autonomous_replanning_waits_for_parent_blend_and_respects_commit(monkeypatch):
    from types import SimpleNamespace
    from rm27_contact_physics.control import motor_mixer, Septic
    from rm27_non_rl import asm2_integrated
    from rm27_non_rl.asm2_contact_control import Reference
    from rm27_sim.experiments.asm2_contact import DEFAULT_CONFIG
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['guidance'] = {'enabled_fixture': True}
    metadata = dict(mass_kg=.142, max_rotor_thrust_N=1.104,
        inertia_body_kg_m2=np.eye(3)*.0001, mixer_collective_and_body_torque_per_rotor_N=motor_mixer(.046, .013),
        rate_gain_per_s=17., motor_time_constant_s=.04, hover_motor_thrust_N=[.348255]*4)
    runtime = asm2_integrated.FlightRuntime(metadata,
        {'armor_normal': [0., 0., 1.], 'armor_surface_center_m': [0., 0., 0.]}, config,
        clearance=SimpleNamespace(check=lambda *args: (True, None)))
    runtime.alignment = {'evidence': 'fabricated software fixture'}
    zeros = np.zeros(3)
    boundary = (np.array([.1, 0., .3]), zeros, zeros, zeros)
    hold = Reference(Septic(boundary, boundary, 2.), -.1)
    runtime.reference = Reference(Septic(boundary, boundary, 1.1), 0., previous=hold,
        blend=.5, blend_delay_s=.005, contact_preview={'time_s': 1.1})
    runtime.guidance.last_command = np.array(metadata['hover_motor_thrust_N'])
    def estimate(now):
        return dict(time_s=now, epoch_id=0, world_alignment_observed=True,
            position_m=boundary[0].tolist(), velocity_m_s=zeros.tolist(),
            rotation_body_to_local=np.eye(3).tolist(), angular_velocity_body_rad_s=zeros.tolist(),
            channels={name: dict(acquisition_s=now, delivery_s=now, valid=True)
                      for name in ('position', 'velocity', 'attitude', 'body_rate')})
    monkeypatch.setattr(runtime.estimator, 'estimate', estimate)
    queries = []
    def candidate(observation, config, geometry, tracker, previous, **kwargs):
        queries.append(observation.vehicle.timestamp)
        return Reference(Septic(boundary, boundary, 1.1), observation.vehicle.timestamp,
            previous=previous, blend=.5, blend_delay_s=.005, contact_preview={'time_s': 1.36})
    monkeypatch.setattr(asm2_integrated, 'plan_reference', candidate)
    runtime.update(.2, None)
    assert queries == []
    assert runtime.pending_reference is None
    runtime.update(.5, None)
    assert queries == [.56]
    assert runtime.pending_reference is not None
    runtime.update(.56, None)
    assert runtime.reference.start == .56
    assert runtime.replans[-1]['status'] == 'APPLIED'
    runtime.next_plan = 0.
    runtime.update(runtime.reference.end-.2, None)
    assert queries == [.56]


def packet(kind, t, value, delay=0.):
    return dict(kind=kind, acquisition_s=t, delivery_s=t+delay, value=value,
                source='TRUTH_GENERATED_SYNTHETIC', valid=True, clipped=False)


def test_offline_namespaces_cannot_enter_runtime_projection():
    item = packet('gyro', 0., [0., 0., 0.])
    poisoned = dict(item, ground_truth={'rotation': np.eye(3).tolist()}, phase='POST_CONTACT')
    assert runtime_packet(item) == runtime_packet(poisoned)


def test_sensor_only_replay_outputs_full_state_with_explicit_unobserved_alignment(tmp_path):
    packets = []
    for k in range(21):
        t = k*.001
        packets.extend([packet('gyro', t, [0., 0., 0.]), packet('accel', t, [0., 0., 9.81])])
    packets.append(packet('external', 0., {'position_m': [1., 0., 1.], 'velocity_m_s': [0., 0., 0.]}, .015))
    sensor_file = tmp_path/'sensors.jsonl.gz'
    with gzip.open(sensor_file, 'wt') as stream:
        for item in reversed(packets):
            stream.write(json.dumps(item)+'\n')
    output = tmp_path/'replay'
    result = run(sensor_file, output)
    rows = read_lines(output/'estimates.jsonl.gz')
    assert result['emits_commands'] is False
    assert result['recovery_modes_eligible'] == 0
    assert result['covariance'] is None
    assert rows[-1]['estimate']['position_m'][0] > 0.1  # Real measurement correction.
    assert rows[0]['estimate']['position_m'][0] == 0.  # No future measurement use.
    assert np.asarray(rows[-1]['estimate']['rotation_body_to_local']).shape == (3, 3)
    assert not rows[-1]['estimate']['channels']['attitude']['valid']
    assert 'INITIAL_ALIGNMENT_NOT_OBSERVED' in rows[-1]['estimate']['channels']['attitude']['reasons']
    provenance = json.loads((output/'provenance.json').read_text())
    assert provenance['truth_input'] is False
    assert provenance['runtime_input_only'] == str(sensor_file)


def test_imu_only_cannot_copy_external_measurements(tmp_path):
    packets = []
    for t in (0., .005, .010, .015, .020):
        packets.extend([packet('gyro', t, [0., 0., 0.]),packet('accel', t, [0., 0., 9.81])])
    packets.append(packet('external', 0., {'position_m': [10., 10., 10.], 'velocity_m_s': [5., 5., 5.]}))
    path = tmp_path/'sensors.jsonl.gz'
    with gzip.open(path, 'wt') as stream:
        for item in packets:
            stream.write(json.dumps(item)+'\n')
    out = tmp_path/'imu_only'
    run(path, out, mode='imu_only')
    estimate = read_lines(out/'estimates.jsonl.gz')[-1]['estimate']
    assert np.allclose(estimate['position_m'], 0.)
    assert not estimate['channels']['velocity']['valid']


def test_invalid_impact_cannot_open_supervisor_epoch(tmp_path):
    packets=[]
    for t in (0.,.005,.010):
        packets.extend([packet('gyro',t,[0.,0.,0.]),packet('accel',t,[0.,0.,9.81])])
    invalid=packet('impact',.005,{'lower_s':0.,'upper_s':.005})
    invalid['valid']=False
    packets.append(invalid)
    sensor=tmp_path/'sensors.jsonl.gz'
    with gzip.open(sensor,'wt') as stream:
        for p in packets:stream.write(json.dumps(p)+'\n')
    out=tmp_path/'replay'
    run(sensor,out)
    for row in read_lines(out/'estimates.jsonl.gz'):
        assert row['estimate']['epoch_id']==0
        assert row['passive_supervisor']['epoch_id']==0


def test_passive_adapter_uses_declared_integer_clock_without_mutating_raw_times():
    from rm27_non_rl.asm2_estimation_replay import supervisor_samples
    from rm27_non_rl.asm2_estimation_integrity import CapabilitySupervisor
    raw=.30000000000000004
    missing={'acquisition_s':None,'delivery_s':None}
    estimate={'epoch_id':0,'channels':{n:dict(missing) for n in ('attitude','position','velocity')},
              'angular_velocity_body_rad_s':[0.,0.,0.]}
    estimate['channels']['body_rate']={'acquisition_s':raw,'delivery_s':raw,'represented_s':raw,
                                     'valid':True,'frame':'body_FLU'}
    samples=supervisor_samples(estimate,{'imu_max_age_s':.01,'external_max_age_s':.05})
    result=CapabilitySupervisor().evaluate(now=.3,samples=samples)
    assert result.capabilities['body_rate'].usable
    assert not result.capabilities['body_rate'].trusted
    assert estimate['channels']['body_rate']['acquisition_s']==raw
