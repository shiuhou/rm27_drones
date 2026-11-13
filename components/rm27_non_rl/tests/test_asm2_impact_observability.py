"""Offline command-aware specific-force model, with no native simulation."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rm27_non_rl.asm2_recovery_diagnostics import command_specific_force


CONFIG = dict(accel=dict(odr_hz=1600., internal_cutoff_hz=280.), px4_accel_cutoff_hz=30.)


def test_constant_hover_predicts_same_specific_force_and_filter_steady_state():
    times = np.arange(0., .101, .000625)
    expected = command_specific_force(times, [], [], initial_collective_N=.142*9.81,
                                      mass_kg=.142, motor_tau_s=.04, accel_config=CONFIG)
    np.testing.assert_allclose(expected, np.tile([0., 0., 9.81], (len(times), 3)), atol=1e-12)


def test_future_commands_do_not_change_past_force_and_step_follows_motor_lag():
    times = np.arange(0., .101, .000625)
    expected = command_specific_force(times, [.04], [2.], initial_collective_N=1.,
                                      mass_kg=1., motor_tau_s=.04, accel_config=CONFIG)
    np.testing.assert_allclose(expected[times <= .04, 2], 1., atol=1e-12)
    # Preceding 0.1ms mean stays strictly below endpoint force after the step.
    now = times[-1]
    endpoint = 2. - np.exp(-(now-.04)/.04)
    assert 1. < expected[-1, 2] < endpoint
    mean = 2. - .04/.0001*(np.exp(-(now-.0001-.04)/.04)-np.exp(-(now-.04)/.04))
    assert expected[-1, 2] == pytest.approx(mean)
    assert expected[-1, 8] < expected[-1, 2]


def test_com_specific_force_gravity_cancels_at_arbitrary_attitude():
    gravity = np.array([0., 0., 9.81])
    for angles in ([0., 0., 0.], [18., -30., 60.], [90., 70., -25.]):
        rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        world_accel = rotation[:, 2]*12. - gravity
        np.testing.assert_allclose(rotation.T@(world_accel+gravity), [0., 0., 12.], atol=1e-12)


@pytest.mark.parametrize('commands,times', [([1., 2.], [.01, .01]), ([-1.], [.01]), ([1.], [float('nan')])])
def test_invalid_command_history_is_rejected(commands, times):
    with pytest.raises(ValueError):
        command_specific_force([0., .01], times, commands, initial_collective_N=1.,
                               mass_kg=.142, motor_tau_s=.04, accel_config=CONFIG)


def test_command_history_before_declared_initial_sample_is_not_silently_collapsed():
    with pytest.raises(ValueError, match='timestamped collective'):
        command_specific_force([.02, .03], [0., .01], [1., 2.],
                               initial_collective_N=1., mass_kg=.142, motor_tau_s=.04,
                               accel_config=CONFIG)


def test_overlapping_native_average_windows_are_rejected_instead_of_rewinding_clock():
    with pytest.raises(ValueError, match='timestamped collective'):
        command_specific_force([0., .00005, .0001], [], [], initial_collective_N=1.,
                               mass_kg=.142, motor_tau_s=.04, accel_config=CONFIG)
