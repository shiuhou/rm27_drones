"""Capability contracts using fabricated examples, never real sensor evidence.

REAL_SENSOR/VIO labels and approval tuples below are hand-authored unit-test
inputs. They do not qualify any RM27 estimator, report, hardware, or controller.
"""
from dataclasses import replace

import numpy as np
import pytest

from rm27_non_rl.asm2_estimation_integrity import (
    CapabilitySupervisor, EstimatorMode, ImpactEvent, RecoveryMode, Source, StateSample,
    PropagationApproval, EventApproval,
)


# Physical meanings used by the contract tests, with explicit independent frames.
CHANNELS = {
    'attitude': (np.eye(3), 'body_FLU_to_world'),
    'body_rate': (np.zeros(3), 'body_FLU'),
    'gravity_body': (np.array([0., 0., -1.]), 'body_FLU'),
    'position': (np.array([.2, .1, .8]), 'world'),
    'velocity': (np.array([.1, 0., .02]), 'world'),
    'surface_normal': (np.array([-.6, 0., .8]), 'world'),
    'vertical_velocity': (.02, 'world_up'),
    'position_target': (np.array([.2, .1, .8]), 'world'),
    'safe_contact_response': (True, 'predicate'),
    'separated': (True, 'predicate'),
    'attitude_stable': (True, 'predicate'),
    'vertical_clearance': (True, 'predicate'),
    'vertical_stable': (True, 'predicate'),
    'velocity_stable': (True, 'predicate'),
}


def test_simulation_execution_never_grants_trusted_state_or_real_event():
    supervisor = CapabilitySupervisor()
    supervisor.impact(ImpactEvent('synthetic-contact', .09, .10, .105, Source.SYNTHETIC), now=.13)
    samples = [sample(name, epoch=1, source=Source.SYNTHETIC) for name in CHANNELS]
    passive = supervisor.evaluate(now=.13, samples=samples, actuator_available=True)
    assert not passive.recovery_modes
    experimental = supervisor.evaluate(now=.13, samples=samples, actuator_available=True,
                                        simulation_execution=True)
    assert experimental.brief_normal_release_eligible
    assert all(not capability.trusted for capability in experimental.capabilities.values())
    assert experimental.estimator_mode == EstimatorMode.SUSPECTED
    assert 'SIMULATION_ONLY_UNCALIBRATED_EXECUTION' in experimental.blocked_by
    samples = [replace(item, source=Source.GROUND_TRUTH) if item.quantity == 'velocity' else item
               for item in samples]
    rejected = supervisor.evaluate(now=.13, samples=samples, actuator_available=True,
                                    simulation_execution=True)
    assert not rejected.brief_normal_release_eligible


def test_simulation_execution_preserves_post_event_translation_gate():
    supervisor = CapabilitySupervisor()
    supervisor.impact(ImpactEvent('synthetic-contact', .09, .12, .125, Source.SYNTHETIC), now=.13)
    samples = [sample(name, epoch=1, source=Source.SYNTHETIC) for name in CHANNELS]
    decision = supervisor.evaluate(now=.13, samples=samples, actuator_available=True,
                                    simulation_execution=True)
    assert not decision.brief_normal_release_eligible
    assert not decision.capabilities['velocity'].usable


def sample(name, *, epoch=0, source=Source.REAL_SENSOR, **overrides):
    value, frame = CHANNELS[name]
    arguments = dict(quantity=name, value=value, acquisition_s=.110, delivery_s=.120,
                     state_time_s=.130, epoch_id=epoch, frame=frame, source=source,
                     max_age_s=.050, propagated=True,
                     integrity_id=f'fabricated-test-report:{name}',
                     propagation_method='fabricated-test-propagation',
                     propagation_integrity_id=f'fabricated-propagation-report:{name}')
    arguments.update(overrides)
    return StateSample(**arguments)


def approval(item, report=None):
    return (report or item.integrity_id, item.quantity, item.frame,
            item.epoch_id, item.source)


def propagation_approval(item):
    # Explicit fabricated review of the exact time interval, not sensor evidence.
    return PropagationApproval(
        item.propagation_integrity_id, item.propagation_method, item.quantity,
        item.frame, item.epoch_id, item.source, item.acquisition_s, item.state_time_s)


def full_state(**kwargs):
    return [sample(name, **kwargs) for name in ('attitude', 'body_rate', 'position', 'velocity')]


def event(*, confirmed=True, source=Source.REAL_SENSOR, event_id='test-impact'):
    return ImpactEvent(event_id, .090, .100, .105, source, confirmed,
                       'fabricated-event-report')



def event_approval(item):
    return EventApproval(item.integrity_id, item.event_id, item.lower_s,
                         item.upper_s, item.delivery_s, item.source)

def test_fresh_synthetic_full_state_is_diagnostic_without_integrity_or_commands():
    samples = full_state(source=Source.SYNTHETIC)
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    decision = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert decision.estimator_mode == EstimatorMode.NORMAL
    assert all(c.usable and not c.trusted for c in decision.capabilities.values())
    assert all('INTEGRITY_UNSUPPORTED' in c.reasons for c in decision.capabilities.values())
    assert not decision.recovery_modes and not decision.brief_normal_release_eligible
    assert decision.emits_commands is False


@pytest.mark.parametrize('source', [Source.SYNTHETIC, Source.RECONSTRUCTED, Source.MODEL])
def test_approved_diagnostic_source_does_not_become_real_or_trusted(source):
    item = sample('attitude', source=source)
    decision = CapabilitySupervisor(approved_integrity={approval(item)}).evaluate(
        now=.130, samples=[item])
    assert decision.capabilities['attitude'].usable
    assert not decision.capabilities['attitude'].trusted


def test_ground_truth_channels_and_events_cannot_supply_runtime_capabilities():
    supervisor = CapabilitySupervisor()
    decision = supervisor.evaluate(now=.130, samples=full_state(source=Source.GROUND_TRUTH),
                                   actuator_available=True)
    assert all(not c.usable and 'PRIVILEGED_OR_UNKNOWN_SOURCE' in c.reasons
               for c in decision.capabilities.values())
    assert not decision.recovery_modes
    with pytest.raises(ValueError, match='privileged'):
        supervisor.impact(event(source=Source.GROUND_TRUTH), now=.130)
    with pytest.raises(ValueError, match='privileged'):
        supervisor.impact(event(source=Source.MODEL), now=.130)


@pytest.mark.parametrize('change,reason', [
    ({'delivery_s': .140}, 'INVALID_OR_UNDELIVERED_CLOCK'),
    ({'acquisition_s': .140, 'delivery_s': .140, 'state_time_s': .140},
     'INVALID_OR_UNDELIVERED_CLOCK'),
    ({'state_time_s': .140, 'propagated': True}, 'INVALID_OR_UNDELIVERED_CLOCK'),
    ({'state_time_s': .125, 'propagated': False}, 'PROPAGATION_NOT_DECLARED'),
    ({'acquisition_s': .010, 'state_time_s': .010}, 'STALE_OR_FUTURE_MEASUREMENT'),
])
def test_measurement_delivery_state_and_query_clocks_are_independent(change, reason):
    decision = CapabilitySupervisor().evaluate(now=.130, samples=[sample('velocity', **change)])
    assert not decision.capabilities['velocity'].usable
    assert reason in decision.capabilities['velocity'].reasons


def test_declared_propagation_does_not_refresh_acquisition_age():
    item = sample('velocity', acquisition_s=.100, delivery_s=.115,
                  state_time_s=.130, propagated=True, max_age_s=.025)
    supervisor = CapabilitySupervisor(approved_integrity={approval(item)})
    decision = supervisor.evaluate(now=.130, samples=[item])
    capability = decision.capabilities['velocity']
    assert capability.propagated and capability.age_s == pytest.approx(.030)
    assert not capability.usable and not capability.trusted
    timely = replace(item, max_age_s=.040)
    assert supervisor.evaluate(now=.130, samples=[timely]).capabilities['velocity'].usable


def test_impact_invalidates_old_epoch_and_relabelled_or_propagated_preimpact_translation():
    old = sample('velocity', acquisition_s=.080, delivery_s=.120, state_time_s=.080)
    relabelled = replace(old, epoch_id=1, state_time_s=.130, propagated=True,
                         continuity_id='fabricated-continuity')
    supervisor = CapabilitySupervisor(approved_integrity={
        approval(relabelled), approval(relabelled, relabelled.continuity_id)})
    supervisor.impact(event(), now=.105)
    first = supervisor.evaluate(now=.130, samples=[old])
    assert 'ESTIMATOR_EPOCH_MISMATCH' in first.capabilities['velocity'].reasons
    second = supervisor.evaluate(now=.130, samples=[relabelled])
    assert 'PRE_EVENT_OR_AMBIGUOUS_ACQUISITION' in second.capabilities['velocity'].reasons
    assert not second.capabilities['velocity'].usable
    at_upper = replace(relabelled, acquisition_s=.100, state_time_s=.100, propagated=False)
    assert not supervisor.evaluate(now=.130, samples=[at_upper]).capabilities['velocity'].usable
    fresh_post = replace(relabelled, acquisition_s=.110, state_time_s=.110, propagated=False)
    assert supervisor.evaluate(now=.130, samples=[fresh_post]).capabilities['velocity'].trusted


def test_duplicate_impact_can_confirm_but_never_retime_epoch_or_erase_failure():
    supervisor = CapabilitySupervisor()
    suspected = event(confirmed=False)
    supervisor.impact(suspected, now=.105)
    assert supervisor.epoch_id == 1 and supervisor.cutoff_s == .100
    supervisor.stop(now=.106, reason='forbidden-contact')
    supervisor.impact(replace(suspected, confirmed=True), now=.107)
    assert supervisor.epoch_id == 1 and supervisor.cutoff_s == .100
    assert supervisor.failure == 'forbidden-contact'
    with pytest.raises(ValueError, match='rewrite'):
        supervisor.impact(replace(suspected, upper_s=.101, confirmed=True), now=.108)
    with pytest.raises(ValueError, match='rewrite'):
        supervisor.impact(suspected, now=.108)
    assert supervisor.epoch_id == 1


def test_new_impact_revokes_previously_qualified_hold_and_preserves_stop_on_reset():
    samples = [sample(name) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    assert RecoveryMode.HOLD in supervisor.evaluate(
        now=.130, samples=samples, actuator_available=True).recovery_modes
    supervisor.impact(ImpactEvent('second-impact', .131, .132, .133, Source.REAL_SENSOR, True),
                      now=.133)
    decision = supervisor.evaluate(now=.134, samples=samples, actuator_available=True)
    assert not decision.recovery_modes
    supervisor.stop(now=.135, reason='latched-safety')
    supervisor.reset(now=.136, reason='explicit-estimator-reset')
    assert supervisor.failure == 'latched-safety'
    with pytest.raises(ValueError, match='historical|Historical'):
        supervisor.impact(event(), now=.137)


def test_reset_requires_explicit_alignment_evidence_and_never_silent_realignment():
    item = sample('attitude', epoch=1, acquisition_s=.120, delivery_s=.125,
                  state_time_s=.120, alignment_id='fabricated-new-alignment')
    supervisor = CapabilitySupervisor(approved_integrity={approval(item)})
    supervisor.reset(now=.100, reason='gyro-gap')
    decision = supervisor.evaluate(now=.130, samples=[item])
    assert not decision.capabilities['attitude'].usable
    assert 'ALIGNMENT_REACQUISITION_REQUIRED' in decision.capabilities['attitude'].reasons
    aligned = CapabilitySupervisor(approved_integrity={
        approval(item), approval(item, item.alignment_id)})
    aligned.reset(now=.100, reason='gyro-gap')
    assert aligned.evaluate(now=.130, samples=[item]).capabilities['attitude'].trusted
    assert aligned.reset_reason == 'gyro-gap'


@pytest.mark.parametrize('fault', [{'gap': True}, {'clipped': True}, {'valid': False},
                                   {'delivery_s': .140}])
def test_rate_fault_revokes_actuation_modes_even_when_other_channels_qualified(fault):
    good = [sample(name) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in good},
                                      approved_propagation={propagation_approval(s) for s in good},
                                      approved_events={event_approval(event())})
    assert supervisor.evaluate(now=.130, samples=good, actuator_available=True).recovery_modes
    bad = [replace(s, **fault) if s.quantity == 'body_rate' else s for s in good]
    decision = supervisor.evaluate(now=.130, samples=bad, actuator_available=True)
    assert not decision.recovery_modes and not decision.brief_normal_release_eligible


def test_lost_translation_retains_attitude_capability_but_revokes_hold_and_braking():
    samples = [sample(name) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    decision = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert decision.selected_recovery_mode == RecoveryMode.HOLD
    remaining = [s for s in samples if s.quantity not in ('position', 'velocity')]
    decision = supervisor.evaluate(now=.130, samples=remaining, actuator_available=True)
    assert decision.estimator_mode == EstimatorMode.ATTITUDE_ONLY
    assert RecoveryMode.ATTITUDE in decision.recovery_modes
    assert RecoveryMode.HOLD not in decision.recovery_modes
    assert RecoveryMode.BRAKING not in decision.recovery_modes
    assert not decision.brief_normal_release_eligible


def test_gravity_only_does_not_grant_world_attitude_or_surface_separation():
    samples = [sample(n, epoch=1) for n in ('body_rate', 'gravity_body', 'velocity', 'surface_normal')]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    supervisor.impact(event(), now=.105)
    decision = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert RecoveryMode.ATTITUDE in decision.recovery_modes
    assert RecoveryMode.SEPARATION not in decision.recovery_modes
    assert not decision.brief_normal_release_eligible
    assert 'attitude' not in decision.capabilities


def test_evidence_is_scoped_to_quantity_frame_epoch_and_source_not_identifier_alone():
    item = sample('velocity', source=Source.VIO)
    key = approval(item)
    wrong_keys = [
        (key[0], 'position', key[2], key[3], key[4]),
        (key[0], key[1], 'body_FLU', key[3], key[4]),
        (key[0], key[1], key[2], 1, key[4]),
        (key[0], key[1], key[2], key[3], Source.REAL_SENSOR),
    ]
    for wrong in wrong_keys:
        capability = CapabilitySupervisor(approved_integrity={wrong}).evaluate(
            now=.130, samples=[item]).capabilities['velocity']
        assert capability.usable and not capability.trusted
    assert CapabilitySupervisor(approved_integrity={key},
                                approved_propagation={propagation_approval(item)}).evaluate(
        now=.130, samples=[item]).capabilities['velocity'].trusted


def test_independent_test_approval_allows_skipping_modes_and_overlapping_recovery_without_commands():
    samples = [sample(name, epoch=1) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    supervisor.impact(event(), now=.105)
    decision = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert decision.estimator_mode == EstimatorMode.TRUSTED
    assert set(decision.recovery_modes) == set(RecoveryMode)
    assert decision.selected_recovery_mode == RecoveryMode.HOLD
    assert decision.brief_normal_release_eligible
    assert decision.emits_commands is False
    assert decision.transition == (None, EstimatorMode.TRUSTED.value)
    # Qualification describes this hand-authored contract fixture, no sensor study.
    supervisor.stop(now=.131, reason='forbidden-contact')
    stopped = supervisor.evaluate(now=.132, samples=samples, actuator_available=True)
    assert not stopped.recovery_modes and not stopped.brief_normal_release_eligible
    assert stopped.emits_commands is False


def test_unconfirmed_event_and_partial_channels_show_concurrent_validation_not_false_trust():
    samples = full_state(epoch=1, source=Source.SYNTHETIC)
    supervisor = CapabilitySupervisor()
    supervisor.impact(event(confirmed=False), now=.105)
    decision = supervisor.evaluate(now=.130, samples=samples)
    assert decision.estimator_mode == EstimatorMode.SUSPECTED
    assert EstimatorMode.VALIDATION in decision.concurrent_estimator_modes
    assert EstimatorMode.ATTITUDE_ONLY in decision.concurrent_estimator_modes
    assert EstimatorMode.REACQUIRE in decision.concurrent_estimator_modes
    assert EstimatorMode.TRUSTED not in decision.concurrent_estimator_modes
    assert not decision.recovery_modes


def test_trusted_delivered_measurements_are_not_automatically_current_control_estimates():
    # Both sets carry the same fabricated integrity scope, original acquisition
    # and delivery. Only the declared represented state time changes.
    current = [sample(name, epoch=1) for name in CHANNELS]
    old = [replace(s, state_time_s=s.acquisition_s, propagated=False)
           if s.quantity in ('attitude', 'body_rate', 'position', 'velocity',
                              'vertical_velocity', 'position_target', 'gravity_body') else s
           for s in current]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in current},
                                      approved_propagation={propagation_approval(s) for s in current},
                                      approved_events={event_approval(event())})
    supervisor.impact(event(), now=.105)
    held = supervisor.evaluate(now=.130, samples=old, actuator_available=True)
    for name in ('attitude', 'body_rate', 'position', 'velocity'):
        assert held.capabilities[name].trusted
    assert held.estimator_mode == EstimatorMode.VALIDATION
    assert 'STATE_EPOCHS_NOT_ALIGNED_TO_CONTROL_TIME' in held.blocked_by
    assert not held.brief_normal_release_eligible
    assert RecoveryMode.SEPARATION not in held.recovery_modes
    assert RecoveryMode.ATTITUDE not in held.recovery_modes
    assert RecoveryMode.VERTICAL not in held.recovery_modes
    assert RecoveryMode.BRAKING not in held.recovery_modes
    assert RecoveryMode.HOLD not in held.recovery_modes
    aligned = supervisor.evaluate(now=.130, samples=current, actuator_available=True)
    assert aligned.estimator_mode == EstimatorMode.TRUSTED
    assert aligned.brief_normal_release_eligible
    assert RecoveryMode.HOLD in aligned.recovery_modes
    assert current[0].acquisition_s == old[0].acquisition_s
    assert aligned.emits_commands is False


def test_vertical_world_normal_does_not_qualify_unsupported_brief_release_executor():
    from rm27_non_rl.asm2_recovery_initialization import EventAwareRecovery
    samples = [sample(name, epoch=1) for name in CHANNELS]
    samples = [replace(s, value=np.array([0., 0., 1.]))
               if s.quantity == 'surface_normal' else s for s in samples]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    supervisor.impact(event(), now=.105)
    decision = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert decision.capabilities['surface_normal'].trusted
    assert not decision.brief_normal_release_eligible
    with pytest.raises(ValueError, match='horizontal component'):
        EventAwareRecovery([0., 0., 1.], event_lower_s=.090, event_upper_s=.100,
                           event_reference_s=.100, hold_s=.025, blend_s=.065,
                           translation_max_age_s=.050)


def test_unspecified_measurement_age_limit_cannot_grant_capability():
    item = sample('velocity', max_age_s=None)
    result = CapabilitySupervisor().evaluate(now=.130, samples=[item])
    assert not result.capabilities['velocity'].usable
    assert 'INVALID_OR_UNDELIVERED_CLOCK' in result.capabilities['velocity'].reasons
    assert not result.brief_normal_release_eligible


@pytest.mark.parametrize('field,value', [
    ('evidence_id', 'other-report'), ('method', 'other-method'),
    ('quantity', 'position'), ('frame', 'body_FLU'), ('epoch_id', 2),
    ('source', Source.VIO), ('acquisition_s', .109), ('state_time_s', .129),
])
def test_propagation_report_is_scoped_to_method_channel_epoch_source_and_interval(field, value):
    item = sample('velocity')
    report = replace(propagation_approval(item), **{field: value})
    supervisor = CapabilitySupervisor(approved_integrity={approval(item)},
                                      approved_propagation={report})
    cap = supervisor.evaluate(now=.130, samples=[item]).capabilities['velocity']
    assert cap.usable and cap.measurement_trusted
    assert not cap.propagation_trusted and not cap.trusted
    assert 'PROPAGATION_INTEGRITY_UNSUPPORTED' in cap.reasons


def test_arbitrary_propagation_flag_cannot_authorize_recovery_with_measurement_approval_only():
    samples = [sample(name, epoch=1) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_events={event_approval(event())})
    supervisor.impact(event(), now=.105)
    result = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert all(cap.measurement_trusted for cap in result.capabilities.values())
    assert not result.recovery_modes and not result.brief_normal_release_eligible
    assert result.estimator_mode != EstimatorMode.TRUSTED


@pytest.mark.parametrize('source', [Source.SYNTHETIC, Source.RECONSTRUCTED])
def test_synthetic_confirmed_contact_cannot_authorize_runtime_contact_recovery(source):
    samples = [sample(name, epoch=1) for name in CHANNELS]
    contact = event(source=source)
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={event_approval(contact)})
    supervisor.impact(contact, now=.105)
    result = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert result.estimator_mode == EstimatorMode.SUSPECTED
    assert RecoveryMode.RESPONSE not in result.recovery_modes
    assert RecoveryMode.SEPARATION not in result.recovery_modes
    assert not result.brief_normal_release_eligible
    assert 'EVENT_INTEGRITY_UNSUPPORTED' in result.blocked_by


@pytest.mark.parametrize('field,value', [
    ('evidence_id', 'other-report'), ('event_id', 'other-event'),
    ('lower_s', .089), ('upper_s', .101), ('delivery_s', .106), ('source', Source.VIO),
])
def test_event_confirmation_report_is_scoped_to_id_interval_delivery_and_source(field, value):
    samples = [sample(name, epoch=1) for name in CHANNELS]
    supervisor = CapabilitySupervisor(approved_integrity={approval(s) for s in samples},
                                      approved_propagation={propagation_approval(s) for s in samples},
                                      approved_events={replace(event_approval(event()), **{field: value})})
    supervisor.impact(event(), now=.105)
    result = supervisor.evaluate(now=.130, samples=samples, actuator_available=True)
    assert result.estimator_mode == EstimatorMode.SUSPECTED
    assert not result.brief_normal_release_eligible
    assert RecoveryMode.RESPONSE not in result.recovery_modes
    assert RecoveryMode.SEPARATION not in result.recovery_modes
