import struct
import mujoco
import numpy as np
import pytest
from ardupilot_bridge.bridge import Bridge
from ardupilot_bridge.plant_adapter import Plant
from ardupilot_bridge.protocol import ProtocolError


def packet(count, pwm=1500, rate=500):
    return struct.pack("<HHI16H", 18458, rate, count, *([pwm] * 16))


def test_reset_and_duplicates():
    b = Bridge()
    for count in range(50):
        b.process(packet(count))
    reply, _ = b.process(packet(50))
    qpos = b.plant.data.qpos.copy()
    t = b.plant.data.time
    assert b.process(packet(50)) == (reply, None)
    assert b.plant.data.time == t
    np.testing.assert_array_equal(b.plant.data.qpos, qpos)
    response, row = b.process(packet(0, 1000))
    fresh = Bridge()
    expected, _ = fresh.process(packet(0, 1000))
    assert response == expected
    assert row["epoch"] == 1 and b.stats["resets"] == 1
    for attr in ("qpos", "qvel", "act", "ctrl", "qacc"):
        np.testing.assert_array_equal(
            getattr(b.plant.data, attr), getattr(fresh.plant.data, attr)
        )
    assert b.plant.data.time == 0.002


def test_gaps_wrap_invalid_rate():
    b = Bridge()
    b.process(packet(0xFFFFFFFE))
    b.process(packet(0xFFFFFFFF))
    b.process(packet(0))
    assert b.stats["resets"] == 0 and b.stats["missing"] == 0
    b.process(packet(3))
    assert b.stats["missing"] == 2
    t = b.plant.data.time
    with pytest.raises(ProtocolError):
        b.process(packet(4, rate=400))
    assert b.plant.data.time == t and b.last_count == 3


@pytest.mark.parametrize(
    "channel,signs",
    [(0, [-1, 1, 1]), (1, [1, -1, 1]), (2, [1, 1, -1]), (3, [-1, -1, -1])],
)
def test_single_rotor_ap_torques(channel, signs):
    p = Plant()
    p.data.qpos[2] = 2
    pwm = [1000] * 4
    pwm[channel] = 1500
    s = p.step(pwm)
    np.testing.assert_array_equal(np.sign(s["imu"]["gyro"]), signs)


def test_freefall_and_stationary_imu():
    p = Plant()
    p.data.qpos[2] = 3
    s = p.step([1000] * 4)
    np.testing.assert_allclose(s["imu"]["accel_body"], [0, 0, 0], atol=0.001)
    p.reset()
    for _ in range(1500):
        s = p.step([1000] * 4)
    np.testing.assert_allclose(s["imu"]["accel_body"], [0, 0, -9.81], atol=0.001)
    np.testing.assert_allclose(s["velocity"], [0, 0, 0], atol=0.001)


def test_motor_lag_once():
    p = Plant()
    p.step([2000] * 4)
    np.testing.assert_allclose(p.data.act, 8 * (1 - np.exp(-0.002 / 0.040)), rtol=0.03)


def test_startup_rate_negotiation():
    b = Bridge()
    b.process(packet(0, rate=1200))
    assert b.plant.data.time == 0.002
    b.process(packet(1, rate=500))
    with pytest.raises(ProtocolError):
        b.process(packet(2, rate=1200))


def test_startup_retry_and_restart():
    b = Bridge()
    first, _ = b.process(packet(0, rate=1200))
    assert b.process(packet(0, rate=1200)) == (first, None)
    b.process(packet(1))
    restart, _ = b.process(packet(0, rate=1200))
    assert restart == first and b.stats["resets"] == 1


def test_invalid_is_counted_without_step():
    b = Bridge()
    with pytest.raises(ProtocolError):
        b.process(b"bad")
    assert b.stats["invalid"] == 1 and b.plant.data.time == 0


def test_rotated_body_gyro_matches_mujoco_object_velocity():
    p = Plant()
    p.data.qpos[2] = 3
    p.data.qpos[3:7] = [0.5, 0.5, 0.5, 0.5]
    p.data.qvel[3:6] = [0.2, -0.3, 0.4]
    mujoco.mj_forward(p.model, p.data)
    local = np.empty(6)
    mujoco.mj_objectVelocity(
        p.model, p.data, mujoco.mjtObj.mjOBJ_BODY, p.body, local, 1
    )
    np.testing.assert_allclose(local[:3], p.data.qvel[3:6])
    np.testing.assert_allclose(p.state()["imu"]["gyro"], [0.2, 0.3, -0.4])
