"""Sensor-only numerical observer tests; no aircraft physics or sensor claims."""
import math

import numpy as np
import pytest

from rm27_non_rl.asm2_sensor_estimator import (
    SYNTHETIC, SensorEstimator, SensorEstimatorConfig, SensorPacket, time_ns,
)


def packet(kind, acquired, value, delivered=None, **flags):
    return dict(kind=kind, acquisition_s=acquired,
                delivery_s=acquired if delivered is None else delivered,
                value=value, source=SYNTHETIC, **flags)


def imu(estimator, timestamp, gyro=(0., 0., 0.), accel=(0., 0., 9.81), now=None):
    for kind, value in [('gyro', gyro), ('accel', accel)]:
        assert estimator.ingest(packet(kind, timestamp, value),
                                now_s=timestamp if now is None else now)['accepted']


def external(t, p=(2., 4., 6.), v=(1., 2., 3.), delivered=None):
    return packet('external', t, {'position': p, 'velocity': v}, delivered)


def registered():
    return SensorEstimator(SensorEstimatorConfig(external_frame_registration='DECLARED_TEST_WORLD_EQUALS_LOCAL'))


def flight_packets(excited=True):
    from scipy.spatial.transform import Rotation

    orientation = Rotation.from_euler('xyz', [8., -6., 31.], degrees=True).as_matrix()
    records = []
    velocity, position = np.zeros(3), np.zeros(3)
    for index in range(1001):
        timestamp = index*.001
        acceleration = np.array([2.*np.sin(8.*timestamp), np.cos(5.*timestamp), 0.]) if excited else np.zeros(3)
        force = orientation.T @ (acceleration+[0., 0., 9.81])
        records.extend([packet('accel', timestamp, force.tolist()), packet('gyro', timestamp, [0., 0., 0.])])
        if index % 20 == 0:
            records.append(external(timestamp, position.tolist(), velocity.tolist(), timestamp+.015))
        position += velocity*.001
        velocity += acceleration*.001
    return records, orientation


def test_flight_alignment_uses_excitation_and_delayed_measurements():
    from rm27_non_rl.asm2_flight_alignment import solve_alignment

    records, orientation = flight_packets()
    result = solve_alignment(records, now_s=1.)
    assert result is not None
    np.testing.assert_allclose(result.rotation, orientation, atol=1e-10)
    assert result.acquisition_s == .98
    assert result.residual_m_s < 1e-10


def test_static_history_cannot_observe_world_heading():
    from rm27_non_rl.asm2_flight_alignment import solve_alignment

    records, _ = flight_packets(excited=False)
    assert solve_alignment(records, now_s=1.) is None


def test_flight_alignment_rejects_unknown_gyro_interval():
    from rm27_non_rl.asm2_flight_alignment import solve_alignment

    records, _ = flight_packets()
    records = [record for record in records if not (
        record['kind'] == 'gyro' and .3 < record['acquisition_s'] < .4)]
    assert solve_alignment(records, now_s=1.) is None


def test_flight_alignment_replays_at_measurement_time_without_truth_input():
    records, orientation = flight_packets()
    est = registered()
    for record in sorted(records, key=lambda record: record['delivery_s']):
        if record['delivery_s'] <= 1.:
            est.ingest(record)
    assert est.initialize_flight_history(records, now_s=1.) is not None
    result = est.estimate(1.)
    np.testing.assert_allclose(result['rotation_body_to_local'], orientation, atol=1e-10)
    assert result['world_alignment_observed']
    assert all(channel['valid'] for channel in result['channels'].values())
    assert result['covariance'] is None
    assert not result['world_attitude_trusted']


def calibrate(estimator, now=0.):
    estimator.reinitialize_stationary(now_s=now, mean_specific_force_body=(0., 0., 9.81),
                                     stationary_status=True, evidence_id='HAND_AUTHORED_STATIC_TEST')


def test_actual_m3_inertial_integration_without_truth_or_covariance():
    est = registered()
    for t in [0., .005, .01]:
        imu(est, t, accel=(2., 0., 9.81))
    result = est.estimate(.01)
    np.testing.assert_allclose(result['position_m'], [.0001, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(result['velocity_m_s'], [.02, 0., 0.], atol=1e-14)
    assert result['covariance'] is None
    assert not result['channels']['attitude']['valid']
    assert result['channels']['body_rate']['valid']  # No world alignment needed.
    assert not result['world_attitude_trusted']


def test_new_gyro_is_not_integrated_backwards_and_body_rate_clock_is_original():
    est = registered()
    imu(est, 0.)
    imu(est, .005, gyro=(0., 0., 1.))
    result = est.estimate(.01)
    rotation = np.array(result['rotation_body_to_local'])
    assert math.atan2(rotation[1, 0], rotation[0, 0]) == pytest.approx(.005)
    assert result['channels']['body_rate']['represented_s'] == .005
    assert result['state_time_s'] == .01


def test_all_axes_use_actual_external_complementary_updates():
    est = registered()
    imu(est, 0.)
    assert est.ingest(external(0.))['accepted']
    result = est.estimate(0.)
    np.testing.assert_allclose(result['position_m'], [1., 2., 3.])
    np.testing.assert_allclose(result['velocity_m_s'], [.35, .7, 1.05])
    assert result['channels']['position']['acquisition_s'] == 0.


def test_missing_registration_never_silently_mixes_world_and_local():
    est = SensorEstimator()
    imu(est, 0.)
    est.ingest(external(0.))
    result = est.estimate(0.)
    np.testing.assert_array_equal(result['position_m'], [0., 0., 0.])
    assert 'EXTERNAL_FRAME_REGISTRATION_NOT_DECLARED' in result['channels']['position']['reasons']


def test_delayed_reordered_updates_rewind_to_acquisition_and_match_ordered_result():
    ordered, delayed = registered(), registered()
    events = []
    for t in np.arange(0., .0301, .005):
        events += [packet('gyro', float(t), (0., 0., .2)),
                   packet('accel', float(t), (1., 0., 9.81))]
    ext = external(.01, delivered=.025)
    for event in sorted(events + [ext], key=lambda e:e['acquisition_s']):
        ordered.ingest(event, now_s=.03)
    for event in events:
        delayed.ingest(event, now_s=.03)
    delayed.estimate(.03)
    delayed.ingest(ext, now_s=.03)
    a, b = ordered.estimate(.03), delayed.estimate(.03)
    for name in ('position_m', 'velocity_m_s', 'rotation_body_to_local'):
        np.testing.assert_array_equal(a[name], b[name])
    assert b['channels']['velocity']['acquisition_s'] == .01
    assert b['channels']['velocity']['delivery_s'] == .025


def test_no_future_packet_is_ingested_or_used():
    est = registered()
    assert est.ingest(external(.01, delivered=.02), now_s=.015)['reason'] == 'NOT_DELIVERED'
    np.testing.assert_array_equal(est.estimate(.015)['position_m'], [0., 0., 0.])
    assert est.accepted_packets == 0


def test_impact_increments_generation_preserves_state_rejects_old_external():
    est = registered()
    calibrate(est)
    imu(est, 0.)
    est.ingest(external(.001, p=(1., 0., 0.), v=(0., 0., 0.)), now_s=.001)
    before = est.estimate(.001)
    event = packet('impact', .002, {'lower_s': .0015, 'upper_s': .002, 'event_id': 'i1'})
    est.ingest(event)
    after = est.estimate(.002)
    assert after['generation'] == before['generation'] + 1
    np.testing.assert_array_equal(after['position_m'], before['position_m'])
    assert not after['channels']['position']['valid']
    assert 'POST_EVENT_EXTERNAL_REACQUISITION_REQUIRED' in after['channels']['position']['reasons']
    assert est.ingest(external(.002, delivered=.003))['reason'] == 'PRE_IMPACT_OR_REINITIALIZATION_EXTERNAL'
    assert est.ingest(external(.003, p=(1., 0., 0.), v=(0., 0., 0.)))['accepted']
    assert est.estimate(.003)['channels']['position']['valid']
    assert est.ingest(event, now_s=.003)['reason'] == 'DUPLICATE'


def test_delayed_impact_orders_at_interval_upper_not_notification_acquisition():
    est = registered()
    calibrate(est)
    imu(est, 0.)
    est.ingest(external(.003), now_s=.005)
    est.ingest(packet('impact', .004, {'lower_s': .001, 'upper_s': .002}, .005), now_s=.005)
    result = est.estimate(.005)
    assert result['channels']['position']['valid']
    assert result['channels']['position']['measurement_epoch_id'] == result['generation']


def test_gap_latches_attitude_until_explicit_measured_stationary_reinit():
    est = registered()
    calibrate(est)
    imu(est, 0.)
    gap = est.estimate(.02)
    assert 'GYRO_GAP_LATCHED' in gap['faults']
    imu(est, .02)
    result = est.estimate(.02)
    assert not result['channels']['attitude']['valid']
    assert result['channels']['body_rate']['valid']
    calibrate(est, .021)
    assert est.estimate(.021)['channels']['attitude']['valid']


def test_gyro_clipping_latches_attitude_but_new_body_rate_can_be_observed():
    est = registered()
    calibrate(est)
    imu(est, 0.)
    est.ingest(packet('gyro', .002, (0., 0., 3.), clipped=True))
    assert not est.estimate(.002)['channels']['body_rate']['valid']
    est.ingest(packet('gyro', .003, (0., 0., 0.)))
    result = est.estimate(.003)
    assert not result['channels']['attitude']['valid']
    assert result['channels']['body_rate']['valid']


def test_missing_or_clipped_accel_does_not_prevent_aligned_gyro_attitude():
    est = registered()
    calibrate(est)
    est.ingest(packet('gyro', 0., (0., 0., 1.)))
    result = est.estimate(.005)
    assert result['channels']['attitude']['valid']
    assert not result['channels']['position']['valid']
    assert np.array(result['rotation_body_to_local'])[1, 0] == pytest.approx(math.sin(.005))
    est.ingest(packet('accel', .005, (0., 0., 100.), clipped=True))
    assert est.estimate(.005)['channels']['attitude']['valid']


def test_stationary_gravity_reinit_is_measured_roll_pitch_with_unobserved_yaw():
    est = registered()
    force = np.array([-2., 3., math.sqrt(9.81**2-13.)])
    est.reinitialize_stationary(now_s=0., mean_specific_force_body=force,
                               stationary_status=True, evidence_id='DECLARED_TEST_STATIC_WINDOW')
    imu(est, 0., accel=force)
    result = est.estimate(0.)
    np.testing.assert_allclose(np.array(result['rotation_body_to_local']) @ force, [0., 0., 9.81], atol=1e-14)
    assert result['channels']['attitude']['valid']
    assert not result['world_attitude_trusted']
    assert result['alignment_evidence_id'] == 'DECLARED_TEST_STATIC_WINDOW'


@pytest.mark.parametrize('status,force', [(False,[0.,0.,9.81]), (True,[0.,0.,0.])])
def test_no_stationary_alignment_without_supported_evidence(status, force):
    with pytest.raises(ValueError):
        registered().reinitialize_stationary(now_s=0., mean_specific_force_body=force,
                                           stationary_status=status, evidence_id='test')


def test_history_is_bounded_and_expired_late_samples_rejected():
    est = registered()
    for t in np.arange(0., .2001, .005):
        imu(est, float(t))
        est.estimate(float(t))
    assert len(est._events) <= 44
    assert est.ingest(external(.01, delivered=.2))['reason'] == 'OUTSIDE_RETAINED_HISTORY'


@pytest.mark.parametrize('field,value', [('truth',[1.,2.,3.]), ('model',object()),
                                         ('rotation_body_to_world',np.eye(3))])
def test_unknown_gt_and_model_fields_rejected(field, value):
    record = packet('gyro', 0., [0.,0.,0.])
    record[field] = value
    with pytest.raises(ValueError, match='unknown packet fields'):
        SensorPacket.from_mapping(record)


def test_gt_sources_and_unknown_nested_fields_rejected():
    record = packet('gyro', 0., [0.,0.,0.])
    record['source'] = 'GROUND_TRUTH'
    with pytest.raises(ValueError, match='synthetic'):
        SensorPacket.from_mapping(record)
    record = external(0.)
    record['value']['true_position'] = [0.,0.,0.]
    with pytest.raises(ValueError):
        SensorPacket.from_mapping(record)


def test_integer_ns_conversion_and_duplicate_packet_order():
    assert time_ns(.03) == time_ns(.030000000000000002)
    est = registered()
    sample = packet('gyro', .03, [0.,0.,0.])
    assert est.ingest(sample)['accepted']
    assert est.ingest(sample)['reason'] == 'DUPLICATE'
    assert not est.estimate(.03)['channels']['attitude']['valid']


@pytest.mark.parametrize('query_during_gap', [False, True])
def test_missing_first_gyro_after_calibration_latches_unknown_interval(query_during_gap):
    est = registered()
    calibrate(est)
    if query_during_gap:
        missing = est.estimate(.015)
        assert not missing['channels']['attitude']['valid']
        assert 'GYRO_GAP_LATCHED' in missing['faults']
    # This first measurement cannot reconstruct the 20 ms since gravity alignment.
    imu(est, .02, gyro=(0., 0., 1.))
    fresh = est.estimate(.02)
    assert fresh['channels']['body_rate']['valid']
    assert not fresh['channels']['attitude']['valid']
    assert 'GYRO_GAP_LATCHED' in fresh['faults']
    calibrate(est, .021)
    assert est.estimate(.021)['channels']['attitude']['valid']


@pytest.mark.parametrize('flags', [{'valid': False}, {'clipped': True}])
def test_invalid_body_rate_vector_and_clock_are_from_the_same_raw_sample(flags):
    est = registered()
    imu(est, 0., gyro=(.1, .2, .3))
    est.ingest(packet('gyro', .002, (1., 2., 3.), delivered=.003, **flags))
    result = est.estimate(.003)
    np.testing.assert_array_equal(result['angular_velocity_body_rad_s'], [1., 2., 3.])
    channel = result['channels']['body_rate']
    assert not channel['valid']
    assert channel['acquisition_s'] == channel['represented_s'] == .002
    assert channel['delivery_s'] == .003
