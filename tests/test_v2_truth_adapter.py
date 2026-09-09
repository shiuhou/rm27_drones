"""V2 truth-adapter frame math, modes, and pose parsing tests."""

import math

from v2_gazebo.truth_adapter import (
    STALE_LATENCY_US,
    TruthTargetAdapter,
    enu_yaw_rad,
    vision_pitch_rad,
    vision_yaw_rad,
    wrap_pi,
)
from v2_gazebo.truth_source import parse_pose_message


def quat_yaw_enu(yaw_rad):
    """Quaternion (x, y, z, w) for a pure z-up yaw (CCW from East)."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def pose(x, y, z, quat=(0.0, 0.0, 0.0, 1.0)):
    return {"pos": (x, y, z), "quat": quat}


class FakeSource:
    def __init__(self, poses):
        self._poses = poses

    def poses(self):
        return dict(self._poses)


class TestWrapPi:
    def test_within_range_unchanged(self):
        assert wrap_pi(0.5) == 0.5

    def test_wraps_above_pi(self):
        assert math.isclose(wrap_pi(3.0 * math.pi), math.pi, abs_tol=1e-12)

    def test_wraps_below_negative_pi(self):
        assert math.isclose(wrap_pi(-3.5 * math.pi), 0.5 * math.pi,
                            abs_tol=1e-12)


class TestEnuYaw:
    def test_identity_faces_east(self):
        # ENU x = East; identity quaternion => yaw 0 = nose East.
        assert math.isclose(enu_yaw_rad((0, 0, 0, 1)), 0.0, abs_tol=1e-12)

    def test_north_is_plus_90(self):
        assert math.isclose(enu_yaw_rad(quat_yaw_enu(math.pi / 2)),
                            math.pi / 2, abs_tol=1e-12)

    def test_west_is_minus_90(self):
        assert math.isclose(enu_yaw_rad(quat_yaw_enu(-math.pi / 2)),
                            -math.pi / 2, abs_tol=1e-12)


class TestVisionYawSign:
    """frames.md section 5/6: target right => yaw_rad > 0."""

    def setup_method(self):
        # UAV at origin facing NORTH (+y in ENU).
        self.uav_pos = (0.0, 0.0, 1.5)
        self.uav_quat = quat_yaw_enu(math.pi / 2)

    def test_target_right_positive(self):
        # East of a north-facing UAV = right side.
        yaw = vision_yaw_rad(self.uav_pos, self.uav_quat, (5.0, 5.0, 0.5))
        assert yaw > 0.2

    def test_target_left_negative(self):
        yaw = vision_yaw_rad(self.uav_pos, self.uav_quat, (-5.0, 5.0, 0.5))
        assert yaw < -0.2

    def test_target_ahead_near_zero(self):
        yaw = vision_yaw_rad(self.uav_pos, self.uav_quat, (0.0, 8.0, 0.5))
        assert abs(yaw) < 1e-9

    def test_20_deg_right_maps_to_minus_20_heading_err(self):
        d = 6.0
        target = (d * math.cos(math.pi / 2 - math.radians(20)),
                  d * math.sin(math.pi / 2 - math.radians(20)), 0.5)
        yaw = vision_yaw_rad(self.uav_pos, self.uav_quat, target)
        assert math.isclose(yaw, math.radians(20), abs_tol=1e-9)

    def test_20_deg_left(self):
        d = 6.0
        target = (d * math.cos(math.pi / 2 + math.radians(20)),
                  d * math.sin(math.pi / 2 + math.radians(20)), 0.5)
        yaw = vision_yaw_rad(self.uav_pos, self.uav_quat, target)
        assert math.isclose(yaw, -math.radians(20), abs_tol=1e-9)


class TestVisionPitch:
    def test_target_below_boresight_negative(self):
        # UAV at 1.5 m, target on the ground 6 m ahead => looks down.
        pitch = vision_pitch_rad((0, 0, 1.5), (0, 6, 0.25))
        assert pitch < 0.0
        assert math.isclose(pitch, -math.atan2(1.25, 6.0), abs_tol=1e-12)


class TestAdapterModes:
    def setup_method(self):
        self.uav = pose(0.0, 0.0, 1.5, quat_yaw_enu(math.pi / 2))
        self.target = pose(1.0, 5.0, 0.25)
        source = FakeSource({"iris_with_gimbal": self.uav,
                             "rm27_target": self.target})
        self.adapter = TruthTargetAdapter(source)

    def test_normal_estimate(self):
        est = self.adapter.update(1_000_000, mode="normal")
        assert est is not None
        assert est.valid and est.safe_for_control
        assert est.measurement_age_us < 200_000
        assert est.confidence == 1.0
        assert est.yaw_rad > 0.0  # target is to the right

    def test_unsafe_mode(self):
        est = self.adapter.update(1_000_000, mode="unsafe")
        assert est is not None and est.valid
        assert est.safe_for_control is False

    def test_stale_mode(self):
        est = self.adapter.update(1_000_000, mode="stale")
        assert est is not None and est.safe_for_control
        assert est.measurement_age_us == STALE_LATENCY_US
        assert est.measurement_age_us > 200_000

    def test_dropout_returns_none(self):
        assert self.adapter.update(1_000_000, mode="dropout") is None

    def test_missing_target_returns_none(self):
        source = FakeSource({"iris_with_gimbal": self.uav})
        adapter = TruthTargetAdapter(source)
        assert adapter.update(1_000_000) is None

    def test_nonfinite_pose_returns_none(self):
        bad = pose(float("nan"), 0.0, 0.0)
        source = FakeSource({"iris_with_gimbal": self.uav,
                             "rm27_target": bad})
        adapter = TruthTargetAdapter(source)
        assert adapter.update(1_000_000) is None

    def test_unknown_mode_raises(self):
        try:
            self.adapter.update(1_000_000, mode="bogus")
        except ValueError:
            return
        raise AssertionError("expected ValueError for unknown mode")


CANNED_MESSAGE = """\
header {
  stamp {
    sec: 480
    nsec: 501000000
  }
}
pose {
  name: "axes"
  id: 4
  position {
  }
  orientation {
    w: 1
  }
}
pose {
  name: "runway"
  id: 10
  position {
    x: -29
    y: 545
  }
  orientation {
    z: 0.02617694830787318
    w: 0.99965732497555726
  }
}
pose {
  name: "iris_with_gimbal"
  id: 15
  position {
    x: 4.1700631138754369
    y: -1.3755517258759908
    z: 0.19499970501624442
  }
  orientation {
    x: -1.8322761114003847e-07
    y: -2.4251558924280783e-07
    z: -0.45502322112294624
    w: 0.890479572052607
  }
}
"""


class TestParsePoseMessage:
    def test_parses_all_models(self):
        poses = parse_pose_message(CANNED_MESSAGE)
        assert set(poses) == {"axes", "runway", "iris_with_gimbal"}

    def test_defaults_for_empty_sections(self):
        poses = parse_pose_message(CANNED_MESSAGE)
        assert poses["axes"]["pos"] == (0.0, 0.0, 0.0)
        assert poses["axes"]["quat"] == (0.0, 0.0, 0.0, 1.0)

    def test_float_precision(self):
        poses = parse_pose_message(CANNED_MESSAGE)
        iris = poses["iris_with_gimbal"]
        assert math.isclose(iris["pos"][0], 4.1700631138754369,
                            rel_tol=1e-15)
        assert math.isclose(iris["quat"][2], -0.45502322112294624,
                            rel_tol=1e-15)

    def test_runway_orientation(self):
        poses = parse_pose_message(CANNED_MESSAGE)
        runway = poses["runway"]
        yaw = enu_yaw_rad(runway["quat"])
        # Small positive CCW yaw ~ 2 * atan2(z, w).
        assert math.isclose(yaw, 2 * math.atan2(0.02617694830787318,
                                                0.99965732497555726),
                            abs_tol=1e-12)
