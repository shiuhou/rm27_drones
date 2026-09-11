import json
import struct
import numpy as np
import pytest
import mujoco
from ardupilot_bridge.protocol import decode_servos, encode_state, ProtocolError
from ardupilot_bridge.frames import to_ap, attitude_to_ap, specific_force
from ardupilot_bridge.actuators import pwm_to_thrust, AP_ROTORS


@pytest.mark.parametrize("n,magic", [(16, 18458), (32, 29569)])
def test_packet(n, magic):
    p = decode_servos(struct.pack("<HHI" + "H" * n, magic, 500, 123, *([1500] * n)))
    assert (p.frame_rate, p.frame_count, len(p.pwm)) == (500, 123, n)


@pytest.mark.parametrize(
    "blob",
    [
        b"",
        bytes(40),
        struct.pack("<HHI16H", 18458, 0, 0, *([1000] * 16)),
        bytes(72),
        bytes(41),
    ],
)
def test_invalid(blob):
    with pytest.raises(ProtocolError):
        decode_servos(blob)


def test_json():
    blob = encode_state({"timestamp": 0.002})
    assert blob.startswith(b"\n") and blob.endswith(b"\n")
    assert json.loads(blob)["timestamp"] == 0.002
    with pytest.raises(ValueError):
        encode_state({"timestamp": float("nan")})


@pytest.mark.parametrize(
    "v,w", [([1, 0, 0], [1, 0, 0]), ([0, 1, 0], [0, -1, 0]), ([0, 0, 1], [0, 0, -1])]
)
def test_vectors(v, w):
    np.testing.assert_array_equal(to_ap(v), w)


@pytest.mark.parametrize("axis,sign", [(0, 1), (1, -1), (2, -1)])
@pytest.mark.parametrize("angle", [0.3, np.pi / 2, np.pi])
def test_attitude_axes(axis, sign, angle):
    q = np.zeros(4)
    q[0] = np.cos(angle / 2)
    q[axis + 1] = np.sin(angle / 2)
    expected = q.copy()
    expected[axis + 1] *= sign
    actual = attitude_to_ap(q)
    assert abs(np.dot(actual, expected)) == pytest.approx(1)


def test_known_composed_rotation():
    q = np.array([0.5, 0.5, 0.5, 0.5])
    out = attitude_to_ap(q)
    a = np.empty(9)
    b = np.empty(9)
    mujoco.mju_quat2Mat(a, q)
    mujoco.mju_quat2Mat(b, out)
    for v in np.eye(3):
        np.testing.assert_allclose(
            b.reshape(3, 3) @ to_ap(v), to_ap(a.reshape(3, 3) @ v), atol=1e-12
        )


@pytest.mark.parametrize(
    "acc,expected",
    [
        ([0, 0, 0], [0, 0, -9.81]),
        ([0, 0, -9.81], [0, 0, 0]),
        ([2, 3, 4], [2, -3, -13.81]),
    ],
)
def test_specific_force(acc, expected):
    np.testing.assert_allclose(specific_force(np.eye(3), acc), expected)


def test_rotated_specific_force():
    r = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    np.testing.assert_allclose(specific_force(r, [0, 0, 0]), [-9.81, 0, 0])


def test_level():
    np.testing.assert_allclose(attitude_to_ap([1, 0, 0, 0]), [1, 0, 0, 0])


def test_mapping():
    assert AP_ROTORS == ("rotor_fr", "rotor_rl", "rotor_fl", "rotor_rr")
    np.testing.assert_allclose(
        pwm_to_thrust([1100, 1200, 1300, 1400]), [2.4, 0.8, 3.2, 1.6]
    )
    np.testing.assert_allclose(pwm_to_thrust([0, 1000, 2000, 2200]), [8, 0, 8, 0])
