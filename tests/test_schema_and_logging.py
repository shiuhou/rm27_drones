"""Schema adapter and logging tests for V0.

The schema test pins the Python adapter to the exact JSON line format
emitted by maixcam2_dart `target_json.cpp` (schema_version = 2).
"""

import json
import math

import pytest

from uav_guidance import (
    GuidanceConfig,
    GuidanceState,
    TargetEstimate,
    UAVGuidance,
)
from uav_guidance.logging import GuidanceLogger

# A real-format line as produced by target_json.cpp (std::fixed, 6 dp),
# control-relevant fields plus nested blocks the adapter must ignore.
REAL_JSON_LINE = (
    '{"schema_version":2,"timestamp_us":123456789,'
    '"measurement_age_us":15000,"valid":true,"safe_for_control":true,'
    '"predicted":false,"classical_detection_ran":true,'
    '"classical_detection_ms":3.210000,"model_ran":false,'
    '"model_inference_ms":0.000000,"guidance_state":"tracking",'
    '"guidance_mode":"lamp_approach","state":"tracking",'
    '"center_x":320.000000,"center_y":240.000000,'
    '"yaw_rad":0.174533,"pitch_rad":-0.034907,"confidence":0.850000,'
    '"green":{"valid":true},'
    '"armor":{"valid":false,"color":"unknown"},'
    '"pose":{"valid":false,"translation_m":[0.0,0.0,0.0],'
    '"orientation_wxyz":[1.0,0.0,0.0,0.0],"distance_m":0.0,'
    '"reprojection_error_px":0.0},'
    '"aim_point":{"x":320.0,"y":240.0},'
    '"line_of_sight_camera":[0.176327,0.034878,0.983656],'
    '"line_of_sight_rate_rad_s":[0.0,0.0],'
    '"angular_covariance":[0.0,0.0]}'
)


def test_from_json_parses_real_maixcam2_line():
    est = TargetEstimate.from_json(REAL_JSON_LINE)
    assert est.schema_version == 2
    assert est.timestamp_us == 123456789
    assert est.measurement_age_us == 15000
    assert est.valid and est.safe_for_control
    assert not est.predicted
    assert est.yaw_rad == pytest.approx(0.174533)
    assert est.pitch_rad == pytest.approx(-0.034907)
    assert est.confidence == pytest.approx(0.85)


def test_from_json_rejects_wrong_schema_version():
    bad = json.loads(REAL_JSON_LINE)
    bad["schema_version"] = 99
    with pytest.raises(ValueError, match="schema_version"):
        TargetEstimate.from_json(json.dumps(bad))


def test_logger_records_target_and_guidance(tmp_path):
    log_path = tmp_path / "v0.jsonl"
    guidance = UAVGuidance(GuidanceConfig())
    est = TargetEstimate(
        timestamp_us=1_000_000,
        measurement_age_us=20_000,
        valid=True,
        safe_for_control=True,
        yaw_rad=math.radians(10.0),
        confidence=0.9,
    )
    with GuidanceLogger(log_path) as logger:
        out = guidance.update(est, 1_000_000)
        logger.log(1_000_000, est, out)
        out = guidance.update(None, 1_033_000)
        logger.log(1_033_000, None, out)

    lines = log_path.read_text().strip().split("\n")
    assert len(lines) == 2

    first = json.loads(lines[0])
    assert first["target"]["yaw_rad"] == pytest.approx(math.radians(10.0))
    assert first["guidance"]["state"] == "ALIGN"
    assert first["guidance"]["yaw_rate_rad_s"] > 0.0
    assert first["guidance"]["reject_reason"] == ""

    second = json.loads(lines[1])
    assert second["target"] is None
    assert second["guidance"]["vx_body_mps"] == 0.0
    assert second["guidance"]["reject_reason"] == "awaiting_estimate"


def test_logged_zero_after_target_loss(tmp_path):
    log_path = tmp_path / "v0_loss.jsonl"
    guidance = UAVGuidance(GuidanceConfig())
    good = TargetEstimate(
        timestamp_us=1_000_000, measurement_age_us=20_000,
        valid=True, safe_for_control=True, yaw_rad=0.0, confidence=0.9,
    )
    lost = TargetEstimate(timestamp_us=1_033_000, valid=False)
    with GuidanceLogger(log_path) as logger:
        logger.log(1_000_000, good, guidance.update(good, 1_000_000))
        out = guidance.update(lost, 1_033_000)
        logger.log(1_033_000, lost, out)

    last = json.loads(log_path.read_text().strip().split("\n")[-1])
    assert last["guidance"]["state"] == GuidanceState.SAFE_HOLD.name
    assert last["guidance"]["vx_body_mps"] == 0.0
    assert last["guidance"]["valid"] is False
