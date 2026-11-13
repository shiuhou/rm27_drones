"""Focused checks for the independent low-energy recovery experiment."""
import json
from dataclasses import replace

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27_contact_physics.model import VehicleEstimate
from rm27_sim.experiments.asm2_contact import DEFAULT_CONFIG, STATE, ContactScorer, Plant
from rm27_sim.experiments.asm2_recovery_fixture import initialize, load_fixture
from rm27_sim.experiments.asm2_recovery_sensors import RecoverySensors


@pytest.fixture(scope='module')
def bench(tmp_path_factory):
    return load_fixture(tmp_path_factory.mktemp('asm2-fixture'))


def test_fixture_keeps_native_aircraft_and_only_one_external_contact_geometry(bench):
    model, metadata, mapping, _ = bench
    plant = Plant(model, metadata, {})
    config = {**json.loads(DEFAULT_CONFIG.read_text()), 'front_plate_backing_indices': []}
    scorer = ContactScorer(plant, mapping, metadata, config)
    active = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
    external = active & (model.geom_bodyid != plant.body)
    aircraft = active & (model.geom_bodyid == plant.body)
    panel = model.geom('fixture_panel').id

    assert model.body_mass[plant.body] == pytest.approx(.142)
    np.testing.assert_allclose(np.sort(model.body_inertia[plant.body]),
                               np.linalg.eigvalsh(metadata['inertia_body_kg_m2']), rtol=1e-5)
    assert model.nu == model.na == 4
    np.testing.assert_allclose(model.actuator_dynprm[plant.motors, 0], metadata['motor_time_constant_s'])
    assert model.opt.timestep == pytest.approx(.0001)
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert np.count_nonzero(aircraft) == 3522
    assert np.flatnonzero(external).tolist() == [panel]
    assert model.geom_type[panel] == mujoco.mjtGeom.mjGEOM_BOX
    assert scorer.plate_ids == {panel}
    assert len(scorer.patch_ids) == 15


def test_fixture_contact_is_generated_in_native_steps_and_replays(bench):
    model, metadata, mapping, _ = bench
    config = {**json.loads(DEFAULT_CONFIG.read_text()), 'front_plate_backing_indices': []}
    case = dict(gap_m=.002, inward_speed_m_s=.13, tangential_speed_m_s=0.,
                tilt_offset_deg=0., roll_offset_deg=0., omega_body_rad_s=[0., 0., 0.],
                hover_fraction=.9)
    plant = Plant(model, metadata, case)
    scorer = ContactScorer(plant, mapping, metadata, config)
    initialize(plant, scorer, case)
    data = plant.data
    assert data.ncon == 0
    np.testing.assert_allclose(data.act[plant.activation], .9 * metadata['hover_motor_thrust_N'][0])
    np.testing.assert_array_equal(data.ctrl[plant.motors], data.act[plant.activation])

    def first_compressive_contact():
        for _ in range(350):
            before = plant.state()
            mujoco.mj_step(model, data)
            contacts = scorer.active(data, before)
            if contacts:
                return before.timestamp, contacts, plant.snapshot()
        pytest.fail('Separated fixture never reached positive-force native contact')

    start = plant.snapshot()
    t_before, contacts, endpoint = first_compressive_contact()
    assert data.time > 0.
    assert data.time - t_before == pytest.approx(model.opt.timestep)
    assert all(row['surface'] == 'fixture_panel' and row['force_N'] > 0. for row in contacts)
    assert np.isfinite(endpoint).all()
    assert not np.array_equal(start, endpoint)
    np.testing.assert_array_equal(data.ctrl[plant.motors], data.act[plant.activation])

    mujoco.mj_setState(model, data, start, STATE)
    mujoco.mj_forward(model, data)
    replay_start, replay_contacts, replay_endpoint = first_compressive_contact()
    assert replay_start == t_before and replay_contacts == contacts
    np.testing.assert_array_equal(replay_endpoint, endpoint)


def sensor_config(**overrides):
    path = DEFAULT_CONFIG.parent / 'impact_recovery/imu_bmi088_px4.json'
    imu = json.loads(path.read_text())
    imu['accel'].update(noise_density_ug_rthz=0., vibration_std_m_s2=0.)
    imu['gyro'].update(noise_density_dps_rthz=0.)
    config = dict(imu=imu, attitude_initial_noise_deg=0., gyro_bias_std_rad_s=0.,
                  position_noise_m=0., velocity_noise_m_s=0.)
    config.update(overrides)
    return config


def motion(time, position=0., velocity=0.):
    return VehicleEstimate(np.array([position, 0., 1.]), np.array([velocity, 0., 0.]),
                           np.eye(3), np.zeros(3), time, 'TEST_TRUTH')


def feed_sensors(sensors, initial, end, acceleration):
    previous = initial
    steps = round((end - initial.timestamp) / .0001)
    for index in range(1, steps + 1):
        time = initial.timestamp + index * .0001
        dv = acceleration(time) * .0001
        state = motion(time, previous.position[0] + (previous.velocity[0] + .5*dv)*.0001,
                       previous.velocity[0] + dv)
        sensors.update(previous, state)
        yield state
        previous = state


def test_translation_uses_timestamped_samples_and_never_live_truth():
    config = sensor_config(translation_period_s=.02, translation_latency_s=.015,
                           translation_max_age_s=.05)
    start = motion(0., velocity=.1)
    sensors = RecoverySensors(config, {'sensor_mode': 'aided'}, 3, start)
    for truth in feed_sensors(sensors, start, .02, lambda _t: 4.):
        if truth.timestamp < .015:
            estimate, status = sensors.estimate(truth.timestamp)
            assert estimate is None
            assert status['attitude_valid'] and not status['translation_valid']
    estimate, status = sensors.estimate(truth.timestamp)
    assert status['attitude_valid'] and status['translation_valid']
    assert status['translation_timestamp_s'] == 0.
    assert status['translation_age_s'] == pytest.approx(.02)
    assert estimate.timestamp == truth.timestamp
    assert estimate.source == 'SYNTHETIC_GYRO_AND_DELAYED_EXTERNAL_TRANSLATION'
    np.testing.assert_allclose(estimate.position, start.position + .02 * start.velocity)
    np.testing.assert_allclose(estimate.velocity, start.velocity)
    assert np.linalg.norm(estimate.position - truth.position) > .0005


@pytest.mark.parametrize(('case', 'validity'), [
    ({'sensor_mode': 'imu_only'}, (True, False)),
    ({'translation_outage_s': [.018, .06]}, (True, False)),
    ({'gyro_outage_s': [.018, .06]}, (False, True)),
])
def test_attitude_and_translation_validity_are_independent(case, validity):
    config = sensor_config(translation_period_s=.01, translation_latency_s=.005,
                           translation_max_age_s=.012, gyro_max_age_s=.005)
    start = motion(0.)
    sensors = RecoverySensors(config, case, 9, start)
    for truth in feed_sensors(sensors, start, .031, lambda _t: 0.):
        estimate, status = sensors.estimate(truth.timestamp)
    assert (status['attitude_valid'], status['translation_valid']) == validity
    assert estimate is None


def test_aided_attitude_still_uses_gyro_instead_of_live_pose():
    config = sensor_config(translation_latency_s=0.)
    start = motion(0.)
    sensors = RecoverySensors(config, {}, 8, start)
    actual_rotation = Rotation.from_euler('z', .4).as_matrix()
    previous = start
    for index in range(1, 61):
        state = replace(start, timestamp=index * .0001, rotation=actual_rotation)
        sensors.update(previous, state)
        previous = state
    estimate, status = sensors.estimate(previous.timestamp)
    assert status['attitude_valid'] and status['translation_valid']
    np.testing.assert_allclose(estimate.rotation, np.eye(3), atol=1e-12)
    assert not np.array_equal(estimate.rotation, actual_rotation)


def test_gyro_return_does_not_silently_realign_attitude_after_long_gap():
    config = sensor_config(translation_latency_s=0., gyro_max_age_s=.005)
    start = motion(0.)
    sensors = RecoverySensors(config, {'gyro_outage_s': [.01, .03]}, 5, start)
    for truth in feed_sensors(sensors, start, .04, lambda _t: 0.):
        estimate, status = sensors.estimate(truth.timestamp)
    assert status['attitude_age_s'] <= .005
    assert status['attitude_reason'] == 'GYRO_GAP_LOST_ALIGNMENT'
    assert status['translation_valid'] and not status['attitude_valid']
    assert estimate is None


def test_imu_detection_requires_samples_and_obeys_event_delivery_clock():
    config = sensor_config(detector_threshold_g=.5, detector_persistence=2,
                           event_latency_s=.0075)
    start = motion(0.)
    sensors = RecoverySensors(config, {'sensor_mode': 'imu_only'}, 11, start)
    assert sensors.detected_at(0.) is None

    first_delivery = None
    for truth in feed_sensors(sensors, start, .06,
                              lambda t: 40. if .020 < t <= .030 else 0.):
        acquired = sensors.detected_at(truth.timestamp)
        if acquired is not None:
            first_delivery = truth.timestamp
            break

    assert first_delivery is not None, 'Synthetic pulse should trigger the assumed filtered sensor'
    assert acquired > .02
    assert first_delivery - acquired == pytest.approx(.0075, abs=.0001)
    estimate, status = sensors.estimate(first_delivery)
    assert estimate is None and status['attitude_valid'] and not status['translation_valid']
    assert status['translation_reason'] == 'NO_EXTERNAL_TRANSLATION'
    assert status['translation_timestamp_s'] is None
    assert status['translation_age_s'] is None


def test_detector_diagnostics_separate_physical_label_and_delivery_latency():
    from rm27_non_rl.asm2_recovery_diagnostics import evaluate
    # An acquisition inside the deadline remains a detection even if event
    # delivery crosses it. Transport is reported independently.
    samples = np.zeros((2, 10))
    samples[:, 0] = [.099, .109]
    samples[:, 6] = 9.81
    samples[1, 4] = 9.81
    stream = dict(id='pulse', kind='contact', split='holdout', samples=samples,
                  first_contact_s=.1, deadline_s=.11, held_thrust_N=.142 * 9.81)
    result = evaluate(stream, dict(threshold_g=.5, persistence=1))
    assert result['detected'] and not result['false_alarm']
    assert result['latency_s'] == pytest.approx(.009)
    assert result['delivery_latency_s'] == pytest.approx(.0115)
    assert result['delivery_s'] > stream['deadline_s']
    # The same signal before actual contact is a false alarm and missed contact,
    # independent of the later recovery outcome.
    stream['first_contact_s'] = .12
    result = evaluate(stream, dict(threshold_g=.5, persistence=1))
    assert result['false_alarm'] and result['miss'] and not result['detected']


def test_observability_executable_tick_matches_trial_tolerance():
    from rm27_non_rl.asm2_recovery_diagnostics import next_control_tick as next_tick
    assert next_tick(None) is None
    assert next_tick(.03500000000000001) == pytest.approx(.035)
    assert next_tick(.0350009) == pytest.approx(.035)
    assert next_tick(.0350011) == pytest.approx(.040)
    assert next_tick(.0375) == pytest.approx(.040)
    # Episode loop cannot send a command at initialization (step zero).
    assert next_tick(0.) == pytest.approx(.005)
    with pytest.raises(ValueError):
        next_tick(float('nan'))
    with pytest.raises(ValueError):
        next_tick(.01, period_s=0.)


def initialization_reference(**overrides):
    from rm27_non_rl.asm2_recovery_initialization import EventAwareRecovery
    normal = np.array([-np.sin(np.deg2rad(18.)), 0., np.cos(np.deg2rad(18.))])
    arguments = dict(event_lower_s=.0149, event_upper_s=.0151, event_reference_s=.0151,
                     hold_s=.025, blend_s=.065, translation_max_age_s=.050)
    arguments.update(overrides)
    return EventAwareRecovery(normal, **arguments)


def test_initialization_data_age_does_not_make_pre_event_velocity_applicable():
    recovery = initialization_reference()
    state = motion(.025, velocity=.12)
    status = recovery.observe(state, measurement_time_s=0., delivery_time_s=.015,
                              source='SYNTHETIC_EXTERNAL_TRANSLATION')
    assert status['latest_translation']['age_valid']
    assert status['latest_translation']['age_s'] == pytest.approx(.025)
    assert status['latest_translation']['event_applicability'] == 'PRE_EVENT'
    assert status['phase'] == 'MODEL_NORMAL_RELEASE'
    assert status['anchor_velocity_m_s'] is None
    assert status['anchor_acquisition_s'] is None
    assert not status['transitions']
    # This state is already extrapolated to the query time, but applicability
    # still uses the external measurement's original acquisition timestamp.
    reference = recovery.reference(state)
    np.testing.assert_array_equal(reference[0], state.position)
    np.testing.assert_array_equal(reference[1], state.velocity)


@pytest.mark.parametrize('sample_time, classification, adopts', [
    (.0148, 'PRE_EVENT', False),
    (.0149, 'AMBIGUOUS_EVENT_INTERVAL', False),
    (.0150, 'AMBIGUOUS_EVENT_INTERVAL', False),
    (.0151, 'AMBIGUOUS_EVENT_INTERVAL', False),
    (.0152, 'POST_EVENT', True),
])
def test_initialization_uses_event_upper_bound_for_uncertain_contact(sample_time,
                                                                     classification, adopts):
    recovery = initialization_reference()
    state = motion(.035, velocity=-.07)
    status = recovery.observe(state, measurement_time_s=sample_time, delivery_time_s=.030)
    assert status['latest_translation']['age_valid']
    assert status['latest_translation']['event_applicability'] == classification
    assert (status['anchor_adoption_s'] is not None) == adopts
    if adopts:
        assert status['anchor_acquisition_s'] == sample_time
        assert status['anchor_delivery_s'] == .030
        assert status['anchor_adoption_s'] == state.timestamp
        assert status['anchor_adoption_s'] != status['anchor_acquisition_s']
        np.testing.assert_array_equal(status['anchor_velocity_m_s'], state.velocity)


def test_initialization_missing_translation_never_creates_a_zero_measurement():
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    recovery = initialization_reference()
    state = motion(.025, velocity=.17)
    with pytest.raises(RecoveryInitializationError, match='TRANSLATION_UNAVAILABLE'):
        recovery.observe(state, measurement_time_s=None, delivery_time_s=None,
                         translation_valid=False)
    status = recovery.status(state.timestamp)
    assert status['latest_translation'] is None
    assert status['anchor_velocity_m_s'] is None
    assert status['phase'] == 'MODEL_NORMAL_RELEASE'
    assert 'translation feedback cancelled' in status['provisional_model']
    # Explicit model release can cancel feedback using the supplied state's v.
    # That reference equality does not turn v into an accepted measurement.
    np.testing.assert_array_equal(recovery.reference(state)[1], state.velocity)
    assert recovery.status(state.timestamp)['anchor_velocity_m_s'] is None
    assert not recovery.status(state.timestamp)['transitions']


def test_initialization_adoption_is_continuous_at_identical_state_and_time():
    from rm27_contact_physics.control import oriented_frame
    from rm27_non_rl import asm2_contact_control as control
    recovery = initialization_reference()
    # Adoption happens after the old hold boundary. Starting blend at the old
    # event clock here would introduce a jump and must fail this test.
    state = replace(motion(.050, position=.04, velocity=-.11),
                    velocity=np.array([-.11, .02, .03]),
                    rotation=Rotation.from_euler('xyz', [.05, -.28, .02]).as_matrix(),
                    omega_body=np.array([.08, .13, -.04]))
    before = recovery.reference(state)
    status = recovery.observe(state, measurement_time_s=.030, delivery_time_s=.045)
    after = recovery.reference(state)
    assert status['anchor_acquisition_s'] == .030
    assert status['anchor_delivery_s'] == .045
    assert status['anchor_adoption_s'] == .050
    assert status['blend_start_s'] == .050
    for left, right in zip(before, after):
        np.testing.assert_array_equal(left, right)
    requested_before = control._requested_force(state, before)
    requested_after = control._requested_force(state, after)
    np.testing.assert_array_equal(requested_before, requested_after)
    np.testing.assert_array_equal(oriented_frame(requested_before, [1., 0., 0.]),
                                  oriented_frame(requested_after, [1., 0., 0.]))
    config = json.loads(DEFAULT_CONFIG.read_text())
    action_before, clip_before = control.feedback(state, before, .142, 4., config)
    action_after, clip_after = control.feedback(state, after, .142, 4., config)
    assert action_before.collective_force_N == action_after.collective_force_N
    np.testing.assert_array_equal(action_before.body_rates_flu_rad_s,
                                  action_after.body_rates_flu_rad_s)
    assert clip_before == clip_after


def test_initialization_duplicate_and_later_samples_do_not_restart_anchor():
    recovery = initialization_reference()
    first = motion(.035, velocity=-.08)
    initial = recovery.observe(first, measurement_time_s=.020, delivery_time_s=.035)
    query = motion(.050, velocity=.42)
    # Duplicate timestamp with a different value must not become a new anchor.
    duplicate = recovery.observe(query, measurement_time_s=.020, delivery_time_s=.035)
    # A newer delivered sample is used by normal feedback, not reinitialization.
    newer = recovery.observe(query, measurement_time_s=.040, delivery_time_s=.050)
    # Nor can an older, age-valid sample rewind initialization.
    older = recovery.observe(query, measurement_time_s=.010, delivery_time_s=.025)
    for status in (duplicate, newer, older):
        assert status['anchor_velocity_m_s'] == initial['anchor_velocity_m_s']
        assert status['anchor_acquisition_s'] == .020
        assert status['anchor_delivery_s'] == .035
        assert status['anchor_adoption_s'] == .035
        assert status['blend_start_s'] == initial['blend_start_s']
        assert len(status['transitions']) == 1
        assert status['velocity_initialization_end_s'] == pytest.approx(.335)


def test_initialization_anchor_has_no_memory_after_finite_existing_envelope():
    references = [initialization_reference(), initialization_reference()]
    for recovery, velocity in zip(references, [-.18, .24]):
        recovery.observe(motion(.035, velocity=velocity), measurement_time_s=.020,
                         delivery_time_s=.035)
    same_query = motion(.400, velocity=.09)
    left, right = [recovery.reference(same_query) for recovery in references]
    for lv, rv in zip(left, right):
        np.testing.assert_array_equal(lv, rv)
    normal = references[0].normal
    expected = np.r_[.30 * normal[:2] / np.linalg.norm(normal[:2]), .08]
    np.testing.assert_allclose(left[1], expected, atol=1e-15)
    np.testing.assert_array_equal(left[2], np.zeros(3))
    np.testing.assert_array_equal(left[3], np.zeros(3))


def test_initialization_provisional_timeout_is_explicit_and_post_sample_wins_at_deadline():
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    unsupported = initialization_reference()
    deadline = unsupported.status(.025)['provisional_deadline_s']
    unsupported.reference(motion(deadline - .001))
    with pytest.raises(RecoveryInitializationError, match='POST_EVENT_TRANSLATION_UNAVAILABLE'):
        unsupported.reference(motion(deadline))
    with pytest.raises(RecoveryInitializationError, match='POST_EVENT_TRANSLATION_UNAVAILABLE'):
        unsupported.observe(motion(deadline), measurement_time_s=.0151, delivery_time_s=.030)
    assert unsupported.status(deadline)['anchor_velocity_m_s'] is None
    supported = initialization_reference()
    status = supported.observe(motion(deadline, velocity=-.09),
                               measurement_time_s=.040, delivery_time_s=deadline)
    assert status['phase'] == 'POST_EVENT_INITIALIZED'
    assert status['anchor_adoption_s'] == deadline
    assert np.isfinite(np.concatenate(supported.reference(motion(deadline)))).all()


@pytest.mark.parametrize('acquired, delivered, valid, reason', [
    (None, .030, True, 'TRANSLATION_TIMESTAMPS_UNAVAILABLE'),
    (.020, None, True, 'TRANSLATION_TIMESTAMPS_UNAVAILABLE'),
    (float('nan'), .030, True, 'TRANSLATION_TIMESTAMPS_UNAVAILABLE'),
    (.040, .035, True, 'TRANSLATION_NOT_YET_DELIVERED_OR_INVALID_CLOCK'),
    (.030, .045, True, 'TRANSLATION_NOT_YET_DELIVERED_OR_INVALID_CLOCK'),
    (-.001, .030, True, 'TRANSLATION_NOT_YET_DELIVERED_OR_INVALID_CLOCK'),
    (.020, .030, False, 'TRANSLATION_UNAVAILABLE'),
])
def test_initialization_invalid_measurement_contract_is_rejected(acquired, delivered, valid, reason):
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    recovery = initialization_reference()
    with pytest.raises(RecoveryInitializationError, match=reason):
        recovery.observe(motion(.035), measurement_time_s=acquired,
                         delivery_time_s=delivered, translation_valid=valid)
    assert recovery.status(.035)['anchor_velocity_m_s'] is None


def test_initialization_stale_samples_and_nonmonotonic_queries_are_rejected():
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    recovery = initialization_reference()
    with pytest.raises(RecoveryInitializationError, match='TRANSLATION_STALE_OR_FUTURE'):
        recovery.observe(motion(.060), measurement_time_s=0., delivery_time_s=.015)
    recovery.observe(motion(.035), measurement_time_s=.020, delivery_time_s=.035)
    with pytest.raises(RecoveryInitializationError, match='NONMONOTONIC_CONTROL_TIME'):
        recovery.observe(motion(.034), measurement_time_s=.020, delivery_time_s=.030)
    with pytest.raises(RecoveryInitializationError, match='BACKDATED_REFERENCE_QUERY'):
        recovery.reference(motion(.030))


@pytest.mark.parametrize('arguments', [
    dict(event_lower_s=.016),
    dict(event_reference_s=.016),
    dict(event_upper_s=float('nan')),
    dict(hold_s=-.001),
    dict(blend_s=0.),
    dict(translation_max_age_s=0.),
])
def test_initialization_invalid_event_or_timing_is_not_silently_replaced(arguments):
    with pytest.raises(ValueError):
        initialization_reference(**arguments)


def test_initialization_does_not_change_legacy_recovery_reference():
    from rm27_non_rl import asm2_contact_control as control
    state = motion(.090, position=.03, velocity=-.11)
    normal = initialization_reference().normal
    initial_velocity = np.array([.05, .01, -.13])
    since = .075
    before = control.recovery_reference(state, normal, since, initial_velocity)
    recovery = initialization_reference()
    recovery.observe(replace(state, timestamp=.035),
                     measurement_time_s=.020, delivery_time_s=.035)
    recovery.reference(state)
    after = control.recovery_reference(state, normal, since, initial_velocity)
    fraction = since / .30
    weight = fraction * fraction * (3. - 2. * fraction)
    target = np.r_[.30 * normal[:2] / np.linalg.norm(normal[:2]), .08]
    expected_velocity = initial_velocity + weight * (target - initial_velocity)
    np.testing.assert_array_equal(after[0], state.position)
    np.testing.assert_array_equal(after[1], expected_velocity)
    np.testing.assert_array_equal(after[2], np.zeros(3))
    np.testing.assert_array_equal(after[3], np.zeros(3))
    for left, right in zip(before, after):
        np.testing.assert_array_equal(left, right)


def test_initialization_zero_velocity_requires_a_real_post_event_sample():
    recovery = initialization_reference()
    assert recovery.status(.025)['anchor_velocity_m_s'] is None
    assert recovery.status(.025)['latest_translation'] is None
    status = recovery.observe(motion(.035), measurement_time_s=.020, delivery_time_s=.035,
                              source='SYNTHETIC_EXTERNAL_TRANSLATION')
    assert status['phase'] == 'POST_EVENT_INITIALIZED'
    assert status['anchor_velocity_m_s'] == [0., 0., 0.]
    assert status['latest_translation']['event_applicability'] == 'POST_EVENT'
    assert status['latest_translation']['source'] == 'SYNTHETIC_EXTERNAL_TRANSLATION'


def test_initialization_bad_state_and_undeclared_source_cannot_freeze_anchor():
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    recovery = initialization_reference()
    invalid = replace(motion(.035), velocity=np.array([np.nan, 0., 0.]))
    with pytest.raises(RecoveryInitializationError, match='INVALID_STATE_OR_EVENT_TIME'):
        recovery.observe(invalid, measurement_time_s=.020, delivery_time_s=.035)
    with pytest.raises(RecoveryInitializationError, match='TRANSLATION_SOURCE_UNAVAILABLE'):
        recovery.observe(motion(.035), measurement_time_s=.020, delivery_time_s=.035,
                         source='')
    assert recovery.status(.035)['anchor_velocity_m_s'] is None
    assert not recovery.status(.035)['transitions']


def test_initialization_fresh_post_event_data_cannot_adopt_after_provisional_deadline():
    from rm27_non_rl.asm2_recovery_initialization import RecoveryInitializationError
    recovery = initialization_reference()
    deadline = recovery.status(.025)['provisional_deadline_s']
    now = deadline + .001
    # The sample is post-event and fresh, but reference initialization has
    # already exceeded its bounded provisional window before this first call.
    with pytest.raises(RecoveryInitializationError, match='POST_EVENT_TRANSLATION_UNAVAILABLE'):
        recovery.observe(motion(now, velocity=-.08),
                         measurement_time_s=now - .010, delivery_time_s=now)
    status = recovery.status(now)
    assert status['phase'] == 'MODEL_NORMAL_RELEASE'
    assert status['anchor_velocity_m_s'] is None
    assert status['anchor_adoption_s'] is None
    assert not status['transitions']
