"""V1 component unit tests (no SITL required).

Covers:
- GuidanceMavlinkAdapter: GuidanceCommand -> SET_POSITION_TARGET_LOCAL_NED
  packing (frame, type mask, field mapping, invalid-command behavior).
- SyntheticTargetProducer: scripted scenario producer semantics.
"""

import math

import pytest

from uav_guidance import GuidanceCommand, GuidanceState, TargetEstimate, UAVGuidance

from v1_sitl.mavlink_adapter import (
    GUIDANCE_TYPE_MASK,
    MAV_FRAME_BODY_NED,
    GuidanceMavlinkAdapter,
)
from v1_sitl.synthetic_target import SyntheticTargetProducer

DEG = math.pi / 180.0


class FakeMav:
    def __init__(self):
        self.sent = []

    def set_position_target_local_ned_send(self, *args):
        self.sent.append(args)


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.target_system = 1
        self.target_component = 1


def test_adapter_packs_body_frame_velocity_and_yaw_rate():
    master = FakeMaster()
    adapter = GuidanceMavlinkAdapter(master)
    cmd = GuidanceCommand(
        timestamp_us=1_000_000,
        vx_body_mps=0.30,
        vy_body_mps=0.0,
        vz_body_mps=0.0,
        yaw_rate_rad_s=0.20,
        valid=True,
    )
    adapter.send(cmd)

    assert len(master.mav.sent) == 1
    (time_boot_ms, sys_id, comp_id, frame, mask,
     x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate) = master.mav.sent[0]

    assert frame == MAV_FRAME_BODY_NED == 8
    assert mask == GUIDANCE_TYPE_MASK == 0x05C7
    assert (x, y, z) == (0, 0, 0)
    assert (vx, vy, vz) == (0.30, 0.0, 0.0)
    assert (afx, afy, afz) == (0, 0, 0)
    assert yaw == 0.0                    # yaw angle ignored by mask
    assert yaw_rate == pytest.approx(0.20)


def test_adapter_invalid_command_sends_zero_motion():
    master = FakeMaster()
    adapter = GuidanceMavlinkAdapter(master)
    cmd = GuidanceCommand(
        timestamp_us=1_000_000,
        vx_body_mps=0.30,
        yaw_rate_rad_s=0.20,
        valid=False,  # SAFE_HOLD / rejected input
    )
    adapter.send(cmd)

    args = master.mav.sent[0]
    vx, vy, vz, yaw_rate = args[8], args[9], args[10], args[15]
    assert (vx, vy, vz, yaw_rate) == (0.0, 0.0, 0.0, 0.0)


def test_adapter_rejects_target_estimate_input():
    # The boundary: the adapter must never consume vision objects directly.
    adapter = GuidanceMavlinkAdapter(FakeMaster())
    with pytest.raises(TypeError):
        adapter.send(TargetEstimate(valid=True, safe_for_control=True))


def test_adapter_sanitizes_non_finite_motion():
    master = FakeMaster()
    adapter = GuidanceMavlinkAdapter(master)
    cmd = GuidanceCommand(
        timestamp_us=1, vx_body_mps=float("nan"),
        yaw_rate_rad_s=float("inf"), valid=True,
    )
    adapter.send(cmd)
    args = master.mav.sent[0]
    assert args[8] == 0.0 and args[15] == 0.0
    assert args[8] == 0.0 and args[15] == 0.0


# Synthetic target producer

def test_synthetic_producer_scripted_sequence():
    producer = SyntheticTargetProducer(
        [
            ("bearing", 1.0, {"yaw_deg": 10.0}),
            ("bearing", 1.0, {"yaw_deg": 5.0}),
            ("bearing", 1.0, {"yaw_deg": 0.0}),
            ("dropout", 0.5, {}),
        ],
        start_us=0,
    )
    e1 = producer.update(0)
    assert e1.yaw_rad == pytest.approx(10 * DEG)
    assert e1.valid and e1.safe_for_control

    e2 = producer.update(1_100_000)  # 1.1 s: second step
    assert e2.yaw_rad == pytest.approx(5 * DEG)

    e3 = producer.update(2_100_000)  # third step
    assert e3.yaw_rad == pytest.approx(0.0)

    none_est = producer.update(3_100_000)  # dropout step
    assert none_est is None


def test_synthetic_producer_unsafe_and_stale_events():
    producer = SyntheticTargetProducer(
        [
            ("unsafe", 1.0, {}),
            ("stale", 1.0, {}),
        ],
        start_us=0,
    )
    unsafe = producer.update(0)
    assert unsafe is not None
    assert not unsafe.safe_for_control

    stale = producer.update(1_100_000)
    assert stale is not None
    assert stale.measurement_age_us > 200_000


def test_synthetic_scenario_drives_expected_guidance_states():
    """Headless rehearsal of the V1 scenario: 10 -> 5 -> 1 -> 0 deg,
    then dropout. Guidance state sequence must match V1 expectations."""
    producer = SyntheticTargetProducer(
        [
            ("bearing", 2.0, {"yaw_deg": 10.0}),
            ("bearing", 2.0, {"yaw_deg": 5.0}),
            ("bearing", 2.0, {"yaw_deg": 1.0}),
            ("bearing", 2.0, {"yaw_deg": 0.0}),
            ("dropout", 1.0, {}),
        ],
        start_us=0,
    )
    guidance = UAVGuidance()
    states = []
    t_us = 0
    for _ in range(int(9.0 / 0.1)):  # 10 Hz for 9 s
        est = producer.update(t_us)
        out = guidance.update(est, t_us)
        states.append(out.state)
        t_us += 100_000

    assert GuidanceState.ALIGN in states[:20]        # 10 deg: aligning
    assert GuidanceState.APPROACH in states          # centered: approach
    assert states[-1] == GuidanceState.SAFE_HOLD     # dropout -> safe hold


def test_dropout_zeroes_forward_motion_immediately():
    guidance = UAVGuidance()
    producer = SyntheticTargetProducer(
        [("bearing", 1.0, {"yaw_deg": 0.0}), ("dropout", 2.0, {})],
        start_us=0,
    )
    t_us = 0
    vx_after_dropout = []
    dropped = False
    for _ in range(30):
        est = producer.update(t_us)
        if est is None:
            dropped = True
        out = guidance.update(est, t_us)
        if dropped:
            vx_after_dropout.append(out.command.vx_body_mps)
        t_us += 100_000
    assert vx_after_dropout and all(v == 0.0 for v in vx_after_dropout)
