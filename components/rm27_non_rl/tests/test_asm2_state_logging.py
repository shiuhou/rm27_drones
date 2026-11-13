"""Offline logger time semantics, with no native physics."""
import numpy as np
import pytest

from rm27_sim.experiments.asm2_state_logging import (
    exact_time_index, historical_estimator_generation, translation_clock, vector_error,
)


def test_translation_epochs_do_not_relabel_delayed_velocity_as_current():
    row = translation_clock(.025, 0., latency_s=.015, valid=True)
    assert row['acquisition_time_s'] == 0.
    assert row['modeled_delivery_time_s'] == .015
    assert row['position_state_epoch_s'] == .025
    assert row['velocity_state_epoch_s'] == 0.
    assert row['position_propagated'] and not row['velocity_propagated']
    assert row['age_s'] == .025 and row['covariance'] is None
    assert row['delivery_provenance'] == 'DECLARED_FIXED_LATENCY_MODEL_NOT_TRANSPORT_LOG'


def test_unavailable_translation_remains_unavailable():
    row = translation_clock(.005, None, latency_s=.015, valid=False)
    assert not row['valid'] and row['acquisition_time_s'] is None
    assert row['position_state_epoch_s'] is None
    with pytest.raises(ValueError, match='acquisition'):
        translation_clock(.005, None, latency_s=.015, valid=True)
    with pytest.raises(ValueError, match='delivery'):
        translation_clock(.01, 0., latency_s=.015, valid=True)


def test_alignment_requires_saved_state_instead_of_interpolation():
    assert exact_time_index([0., .005, .010000000000001], .01) == 2
    with pytest.raises(ValueError, match='No saved state'):
        exact_time_index([0., .005, .01], .0075)
    with pytest.raises(ValueError, match='time axis'):
        exact_time_index([0., .005, .005], .005)


def test_analytical_timestamp_misalignment_can_hide_tracking_error():
    # Analytical constant acceleration only, not aircraft simulation or evidence.
    acceleration = np.array([2., 0., 0.])
    captured, queried = 0., .025
    stale_velocity = acceleration * captured
    actual_query_velocity = acceleration * queried
    correct = vector_error(stale_velocity, actual_query_velocity)
    wrong_epoch = vector_error(stale_velocity, acceleration * captured)
    assert correct['norm'] == pytest.approx(.05)
    assert wrong_epoch['norm'] == 0.


def test_estimator_generation_is_not_inferred_from_state_times_or_events():
    assert historical_estimator_generation() == {
        'estimator_epoch_id': None, 'estimator_epoch_provenance': 'NOT_RECORDED'}
    for now, acquired in [(.025, 0.), (.045, .02), (.065, .04)]:
        row = translation_clock(now, acquired, latency_s=.015, valid=True)
        assert row['estimator_epoch_id'] is None
        assert row['estimator_epoch_provenance'] == 'NOT_RECORDED'
        assert row['position_state_epoch_s'] == now
        assert row['velocity_state_epoch_s'] == acquired
        assert row['position_propagated'] and not row['velocity_propagated']
    # Missing initial acquisition is not evidence of an estimator reset either.
    assert translation_clock(.005, None, latency_s=.015, valid=False)['estimator_epoch_id'] is None
