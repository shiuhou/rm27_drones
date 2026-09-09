"""V0 unit tests for UAVGuidance.

These tests define the frozen V0 behavior of the guidance layer:

    TargetEstimate -> UAVGuidance -> GuidanceCommand

Conventions (docs/frames.md):
- yaw_rad > 0 means the target is RIGHT of boresight.
- yaw_rate_rad_s > 0 commands a yaw to the RIGHT.
- All timing uses injected timestamps (monotonic or simulation time),
  never wall-clock reads inside the guidance module.
"""

import math

import pytest

from uav_guidance import (
    GuidanceConfig,
    GuidanceState,
    TargetEstimate,
    UAVGuidance,
)

DEG = math.pi / 180.0

T0_US = 1_000_000  # arbitrary sim-time origin


def make_estimate(yaw_rad=0.0, pitch_rad=0.0, timestamp_us=T0_US, **overrides):
    """A fresh, safe, valid, centered estimate unless overridden."""
    fields = dict(
        timestamp_us=timestamp_us,
        measurement_age_us=20_000,  # 20 ms, comfortably fresh
        valid=True,
        safe_for_control=True,
        predicted=False,
        yaw_rad=yaw_rad,
        pitch_rad=pitch_rad,
        confidence=0.9,
    )
    fields.update(overrides)
    return TargetEstimate(**fields)


@pytest.fixture
def guidance():
    return UAVGuidance(GuidanceConfig())


def assert_zero_motion(command):
    assert command.vx_body_mps == 0.0
    assert command.vy_body_mps == 0.0
    assert command.vz_body_mps == 0.0
    assert command.yaw_rate_rad_s == 0.0


# 1/2. Sign conventions: target right -> positive yaw rate, and mirror.

def test_right_target_aligns_with_positive_yaw_rate(guidance):
    out = guidance.update(make_estimate(yaw_rad=10 * DEG), T0_US)
    assert out.state == GuidanceState.ALIGN
    assert out.command.vx_body_mps == 0.0
    assert out.command.yaw_rate_rad_s > 0.0
    assert out.command.valid


def test_left_target_aligns_with_negative_yaw_rate(guidance):
    out = guidance.update(make_estimate(yaw_rad=-10 * DEG), T0_US)
    assert out.state == GuidanceState.ALIGN
    assert out.command.vx_body_mps == 0.0
    assert out.command.yaw_rate_rad_s < 0.0


# 3. Centered target -> APPROACH at +0.30 m/s.

def test_centered_target_approaches(guidance):
    out = guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    assert out.state == GuidanceState.APPROACH
    assert out.command.vx_body_mps == pytest.approx(0.30)
    assert out.command.yaw_rate_rad_s == pytest.approx(0.0)
    assert out.command.valid


def test_vy_vz_are_always_zero(guidance):
    for yaw in (0.0, 8 * DEG, -8 * DEG):
        out = guidance.update(make_estimate(yaw_rad=yaw), T0_US)
        assert out.command.vy_body_mps == 0.0
        assert out.command.vz_body_mps == 0.0


# 4. Yaw-rate saturation at 15 deg/s.

def test_yaw_rate_saturates_at_configured_limit(guidance):
    max_rate = GuidanceConfig().max_yaw_rate_rad_s
    assert max_rate == pytest.approx(15 * DEG)

    for yaw in (60 * DEG, -60 * DEG):
        out = guidance.update(make_estimate(yaw_rad=yaw), T0_US)
        assert abs(out.command.yaw_rate_rad_s) <= max_rate + 1e-9
        assert abs(out.command.yaw_rate_rad_s) == pytest.approx(max_rate)


def test_yaw_rate_is_proportional_below_limit(guidance):
    k = GuidanceConfig().k_yaw_per_rad
    out = guidance.update(make_estimate(yaw_rad=10 * DEG), T0_US)
    assert out.command.yaw_rate_rad_s == pytest.approx(k * 10 * DEG)


# 5. Alignment deadband: at/inside threshold -> APPROACH, outside -> ALIGN.

def test_alignment_deadband_boundary():
    align = GuidanceConfig().align_threshold_rad
    assert align == pytest.approx(2 * DEG)

    # Independent instances: this checks the acquisition boundary,
    # not the hysteresis behavior of a single machine.
    inside = UAVGuidance(GuidanceConfig()).update(
        make_estimate(yaw_rad=align), T0_US)
    assert inside.state == GuidanceState.APPROACH

    outside = UAVGuidance(GuidanceConfig()).update(
        make_estimate(yaw_rad=align + 0.5 * DEG), T0_US)
    assert outside.state == GuidanceState.ALIGN
    assert outside.command.vx_body_mps == 0.0


# 6. Hysteresis: jitter between thresholds must not chatter.

def test_hysteresis_prevents_align_approach_chatter(guidance):
    cfg = GuidanceConfig()
    assert cfg.align_threshold_rad < cfg.realign_threshold_rad

    t = T0_US
    out = guidance.update(make_estimate(yaw_rad=0.0), t)
    assert out.state == GuidanceState.APPROACH

    mid = 0.5 * (cfg.align_threshold_rad + cfg.realign_threshold_rad)
    for sign in (+1, -1, +1, -1):
        t += 33_000
        out = guidance.update(make_estimate(yaw_rad=sign * mid, timestamp_us=t), t)
        assert out.state == GuidanceState.APPROACH
        assert out.command.vx_body_mps == pytest.approx(cfg.approach_vx_mps)


# 7-10. Safety gates all produce zero motion.

def test_unsafe_for_control_zeroes_command(guidance):
    out = guidance.update(
        make_estimate(yaw_rad=10 * DEG, safe_for_control=False), T0_US)
    assert_zero_motion(out.command)
    assert out.reject_reason == "unsafe_for_control"


def test_stale_measurement_zeroes_command(guidance):
    stale_us = GuidanceConfig().stale_threshold_us + 1
    out = guidance.update(
        make_estimate(yaw_rad=10 * DEG, measurement_age_us=stale_us), T0_US)
    assert_zero_motion(out.command)
    assert out.reject_reason == "stale_measurement"


def test_low_confidence_zeroes_command(guidance):
    out = guidance.update(
        make_estimate(yaw_rad=10 * DEG, confidence=0.05), T0_US)
    assert_zero_motion(out.command)
    assert out.reject_reason == "low_confidence"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_yaw_zeroes_command(guidance, bad):
    out = guidance.update(make_estimate(yaw_rad=bad), T0_US)
    assert_zero_motion(out.command)
    assert out.reject_reason == "non_finite"


# 11. Target loss from APPROACH -> immediate SAFE_HOLD, zero motion.

def test_target_loss_from_approach_goes_safe_hold_immediately(guidance):
    out = guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    assert out.state == GuidanceState.APPROACH

    out = guidance.update(make_estimate(valid=False), T0_US + 33_000)
    assert out.state == GuidanceState.SAFE_HOLD
    assert_zero_motion(out.command)
    assert not out.command.valid

    out = guidance.update(make_estimate(valid=False), T0_US + 66_000)
    assert out.state == GuidanceState.SAFE_HOLD
    assert_zero_motion(out.command)


def test_safe_hold_recovers_through_align(guidance):
    guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    guidance.update(make_estimate(valid=False), T0_US + 33_000)
    out = guidance.update(make_estimate(yaw_rad=10 * DEG), T0_US + 66_000)
    assert out.state == GuidanceState.ALIGN
    assert out.command.valid


# 12. Error grows beyond realign threshold while APPROACH: vx -> 0.

def test_realign_threshold_cancels_forward_motion(guidance):
    cfg = GuidanceConfig()
    out = guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    assert out.state == GuidanceState.APPROACH

    out = guidance.update(
        make_estimate(yaw_rad=cfg.realign_threshold_rad + 1 * DEG),
        T0_US + 33_000,
    )
    assert out.state == GuidanceState.ALIGN
    assert out.command.vx_body_mps == 0.0
    assert out.command.yaw_rate_rad_s > 0.0


# 13. Timing: injected sim/monotonic time only; producer timeout.

def test_producer_timeout_uses_injected_time(guidance):
    cfg = GuidanceConfig()
    est = make_estimate(yaw_rad=0.0, timestamp_us=T0_US)

    out = guidance.update(est, T0_US)
    assert out.state == GuidanceState.APPROACH

    later = T0_US + cfg.producer_timeout_us + 1
    out = guidance.update(est, later)
    assert out.state == GuidanceState.SAFE_HOLD
    assert_zero_motion(out.command)
    assert out.reject_reason == "producer_timeout"


def test_none_estimate_before_timeout_holds_state_with_zero_motion(guidance):
    out = guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    assert out.state == GuidanceState.APPROACH

    out = guidance.update(None, T0_US + 100_000)
    assert_zero_motion(out.command)
    assert out.state == GuidanceState.APPROACH


def test_none_estimate_after_timeout_safe_hold(guidance):
    cfg = GuidanceConfig()
    guidance.update(make_estimate(yaw_rad=0.0), T0_US)
    out = guidance.update(None, T0_US + cfg.producer_timeout_us + 1)
    assert out.state == GuidanceState.SAFE_HOLD
    assert_zero_motion(out.command)


# Extra: initial state and SEARCH behavior.

def test_initial_state_is_search_with_zero_hold(guidance):
    out = guidance.update(make_estimate(valid=False), T0_US)
    assert out.state == GuidanceState.SEARCH
    assert_zero_motion(out.command)
    assert out.command.valid


def test_search_to_align_on_first_safe_target(guidance):
    guidance.update(make_estimate(valid=False), T0_US)
    out = guidance.update(make_estimate(yaw_rad=10 * DEG), T0_US + 33_000)
    assert out.state == GuidanceState.ALIGN
    assert out.command.yaw_rate_rad_s > 0.0
