"""V2 supervisor gating tests + truth-isolation proof for guidance."""

import inspect
import math

import uav_guidance
import uav_guidance.guidance
import v1_sitl.mavlink_adapter
from uav_guidance import GuidanceCommand
from v2_gazebo.supervisor import TruthSupervisor


def pose(x, y, z):
    return {"pos": (x, y, z), "quat": (0.0, 0.0, 0.0, 1.0)}


class FakeSource:
    def __init__(self, poses):
        self._poses = poses

    def poses(self):
        return dict(self._poses)


class TestSupervisorDistance:
    def test_known_distance(self):
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5),
                          "rm27_target": pose(3, 4, 0.25)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        assert math.isclose(sup.distance_m(), 5.0, abs_tol=1e-12)

    def test_missing_target_none(self):
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        assert sup.distance_m() is None

    def test_rejects_nonpositive_stop(self):
        src = FakeSource({})
        try:
            TruthSupervisor(src, stop_distance_m=0.0)
        except ValueError:
            return
        raise AssertionError("expected ValueError")


class TestSupervisorGate:
    def setup_method(self):
        self.cmd = GuidanceCommand(
            timestamp_us=5_000_000, vx_body_mps=0.30,
            yaw_rate_rad_s=0.05, valid=True,
        )

    def test_passes_through_when_far(self):
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5),
                          "rm27_target": pose(6, 0, 0.25)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        out, gated = sup.apply(self.cmd, now_us=5_000_000)
        assert gated is False and out is self.cmd

    def test_zeros_command_when_close(self):
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5),
                          "rm27_target": pose(1.0, 0.5, 0.25)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        out, gated = sup.apply(self.cmd, now_us=5_000_000)
        assert gated is True
        assert out.vx_body_mps == 0.0
        assert out.yaw_rate_rad_s == 0.0
        assert out.valid is False

    def test_boundary_is_strictly_inside(self):
        # Exactly at the stop distance: NOT gated (no motion yet needed).
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5),
                          "rm27_target": pose(2.0, 0, 0.25)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        _, gated = sup.apply(self.cmd, now_us=5_000_000)
        assert gated is False

    def test_unknown_distance_passes_through(self):
        src = FakeSource({"iris_with_gimbal": pose(0, 0, 1.5)})
        sup = TruthSupervisor(src, stop_distance_m=2.0)
        out, gated = sup.apply(self.cmd, now_us=5_000_000)
        assert gated is False and out is self.cmd


class TestTruthIsolation:
    """Proof that guidance never sees Gazebo truth.

    The guidance layer and the MAVLink adapter must not import (or even
    name) the truth adapter, pose source, or supervisor.
    """

    FORBIDDEN = ("v2_gazebo", "gazebo", "truth", "pose_source",
                 "supervisor", "gz.")

    def test_guidance_source_is_truth_free(self):
        source = inspect.getsource(uav_guidance.guidance).lower()
        for token in self.FORBIDDEN:
            assert token not in source, f"guidance.py mentions {token!r}"

    def test_uav_guidance_package_imports_nothing_v2(self):
        modules = [
            uav_guidance,
            uav_guidance.guidance,
        ]
        for mod in modules:
            names = dir(mod)
            for token in ("v2_gazebo", "truth_source", "truth_adapter",
                          "supervisor"):
                assert token not in names, (
                    f"{mod.__name__} exposes {token!r}"
                )

    def test_mavlink_adapter_source_is_truth_free(self):
        source = inspect.getsource(v1_sitl.mavlink_adapter).lower()
        for token in self.FORBIDDEN:
            assert token not in source, (
                f"mavlink_adapter.py mentions {token!r}"
            )

    def test_guidance_import_graph_has_no_v2(self):
        # Walk the module objects guidance.py actually references and
        # prove none of them is a truth/v2 module.
        import types
        mod = uav_guidance.guidance
        deps = {
            value.__name__
            for value in mod.__dict__.values()
            if isinstance(value, types.ModuleType)
        }
        allowed_external = {"dataclasses", "enum", "math"}
        illegal = {
            name for name in deps
            if not name.startswith("uav_guidance")
            and name not in allowed_external
        }
        assert illegal == set(), f"guidance imports {illegal}"
