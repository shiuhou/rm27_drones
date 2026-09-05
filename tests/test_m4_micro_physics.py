"""M4-A micro-UAV physics tests for models/quadrotor_m4a.xml.

M4-A replaces the generic 1 kg plant with an RM27-direction micro-UAV:
180 g, 120 mm diagonal wheelbase, X quad, thrust authority from an assumed
thrust-to-weight ratio of 2.5 (ENGINEERING_ESTIMATE -- no 1103 motor/prop
bench data exists yet; see m4_params.py provenance table).

These tests pin:

* config geometry/thrust arithmetic (hover thrust, T/W, X-layout radius);
* the M4-A XML matching the nominal MicroUAVConfig (mass, inertia, sites,
  actuator range);
* free fall / hover equilibrium / thrust direction on the micro plant;
* roll/pitch/yaw mixer signs (same conventions as M0, re-verified at the
  new scale);
* the canonical motor order: name -> site position -> allocation column
  -> M1 effectiveness slot (the M1 report-order ambiguity must not recur);
* the m * L^2 inertia scaling law for geometrically similar variants;
* runtime variant application (mass / wheelbase / T/W sensitivities).

Sign conventions are identical to M0 (body frame: +X nose, +Y left, +Z up):

* roll torque  tau_x = sum(y_i * F_i)
* pitch torque tau_y = sum(-x_i * F_i)   (rear-up reads positive in Z-up ZYX)
* yaw torque   tau_z = sum(gear_z_i * F_i); FL/RR +0.018, FR/RL -0.018 N*m/N
"""

from __future__ import annotations

import math
from pathlib import Path
import unittest

import mujoco
import numpy as np

from fly import ROTOR_NAMES, load_simulation, rotation_to_euler
from fly import ControllerParameters
from m1_realism import M1_MOTOR_EFFECTIVENESS
from m4_params import (
    GRAVITY,
    M4A_NOMINAL,
    PROVENANCE_LABELS,
    MicroUAVConfig,
    apply_micro_variant,
    compute_inertia,
    geometrically_scaled,
    micro_controller_parameters,
    provenance_table,
)


M4A_MODEL = Path(__file__).resolve().parents[1] / "models" / "quadrotor_m4a.xml"
START_ALTITUDE = 2.0
# Motor order everywhere: front-left, front-right, rear-right, rear-left.
FL, FR, RR, RL = 0, 1, 2, 3


def make_airborne_micro():
    """Release the micro-UAV at 2 m; every later step is pure physics."""

    model, data, controller = load_simulation(M4A_MODEL)
    data.qpos[controller.qpos_address + 2] = START_ALTITUDE
    mujoco.mj_forward(model, data)
    return model, data, controller


def step_with_thrusts(model, data, thrusts, duration):
    data.ctrl[:] = np.asarray(thrusts, dtype=float)
    steps = int(round(duration / model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    return steps * model.opt.timestep


class MicroConfigArithmeticTest(unittest.TestCase):
    """Nominal M4-A numbers: 180 g, 120 mm wheelbase, T/W = 2.5."""

    def test_motor_geometry_derived_from_wheelbase(self) -> None:
        config = M4A_NOMINAL
        self.assertAlmostEqual(config.motor_radius_m, 0.060)
        expected_xy = 0.120 / (2.0 * math.sqrt(2.0))
        self.assertAlmostEqual(config.motor_xy_m, expected_xy)
        self.assertAlmostEqual(config.motor_xy_m, 0.04243, places=5)

    def test_hover_thrust_is_calculated_not_measured(self) -> None:
        # m*g/4 = 0.180 * 9.81 / 4 = 0.44145 N per motor.
        self.assertAlmostEqual(M4A_NOMINAL.hover_thrust_per_motor_n, 0.44145)

    def test_max_thrust_from_assumed_thrust_to_weight(self) -> None:
        # 2.5 * m * g / 4 = 1.103625 N per motor (ENGINEERING_ESTIMATE).
        self.assertAlmostEqual(M4A_NOMINAL.max_thrust_per_motor_n, 1.103625)
        total_max = 4.0 * M4A_NOMINAL.max_thrust_per_motor_n
        weight = M4A_NOMINAL.mass_kg * GRAVITY
        self.assertAlmostEqual(total_max / weight, 2.5)


class MicroModelMatchesConfigTest(unittest.TestCase):
    """The M4-A XML must be the nominal MicroUAVConfig, not magic numbers."""

    def test_mass(self) -> None:
        model, _, controller = load_simulation(M4A_MODEL)
        self.assertAlmostEqual(controller.mass, 0.180)
        self.assertAlmostEqual(
            float(model.body_mass[controller.body_id]), M4A_NOMINAL.mass_kg
        )

    def test_wheelbase_and_x_geometry(self) -> None:
        model, _, _ = load_simulation(M4A_MODEL)
        positions = {}
        for name in ROTOR_NAMES:
            site_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SITE, f"{name}_site"
            )
            self.assertGreaterEqual(site_id, 0)
            positions[name] = model.site_pos[site_id][:2].copy()

        a = M4A_NOMINAL.motor_xy_m
        expected = {
            "rotor_fl": (a, a),
            "rotor_fr": (a, -a),
            "rotor_rr": (-a, -a),
            "rotor_rl": (-a, a),
        }
        for name, xy in expected.items():
            np.testing.assert_allclose(positions[name], xy, atol=1e-9)
            # Center-to-motor radius is wheelbase / 2.
            self.assertAlmostEqual(
                float(np.linalg.norm(positions[name])),
                M4A_NOMINAL.motor_radius_m,
            )
        # Diagonal motor-to-motor distance is the wheelbase.
        diagonal = np.linalg.norm(positions["rotor_fl"] - positions["rotor_rr"])
        self.assertAlmostEqual(float(diagonal), M4A_NOMINAL.wheelbase_m)

    def test_inertia_matches_component_model(self) -> None:
        model, _, controller = load_simulation(M4A_MODEL)
        np.testing.assert_allclose(
            model.body_inertia[controller.body_id],
            compute_inertia(M4A_NOMINAL),
            rtol=1e-9,
        )

    def test_actuator_range_matches_max_thrust(self) -> None:
        model, _, controller = load_simulation(M4A_MODEL)
        np.testing.assert_allclose(controller.minimum_thrusts, 0.0)
        np.testing.assert_allclose(
            controller.maximum_thrusts, M4A_NOMINAL.max_thrust_per_motor_n
        )


class MotorOrderRegressionTest(unittest.TestCase):
    """Canonical mapping: name -> site quadrant -> allocation column ->
    M1 effectiveness slot. The M1 report-order ambiguity must not recur."""

    EXPECTED_QUADRANT = {  # (sign_x, sign_y) in the body frame
        "rotor_fl": (1.0, 1.0),
        "rotor_fr": (1.0, -1.0),
        "rotor_rr": (-1.0, -1.0),
        "rotor_rl": (-1.0, 1.0),
    }
    EXPECTED_EFFECTIVENESS = {
        "rotor_fl": 0.97,
        "rotor_fr": 1.00,
        "rotor_rr": 0.99,
        "rotor_rl": 1.02,
    }

    def test_name_site_allocation_effectiveness_binding(self) -> None:
        for path in (
            Path(__file__).resolve().parents[1] / "models" / "quadrotor.xml",
            M4A_MODEL,
        ):
            model, _, controller = load_simulation(path)
            for column, name in enumerate(ROTOR_NAMES):
                actuator_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
                )
                self.assertEqual(
                    actuator_id, column, f"{path.name}: {name} actuator slot"
                )
                site_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_SITE, f"{name}_site"
                )
                xy = model.site_pos[site_id][:2]
                sx, sy = self.EXPECTED_QUADRANT[name]
                self.assertGreater(xy[0] * sx, 0.0, f"{path.name}: {name} x")
                self.assertGreater(xy[1] * sy, 0.0, f"{path.name}: {name} y")
                # Allocation column for this actuator produces the torques
                # of its quadrant: tau_x = y*F, tau_y = -x*F.
                column_wrench = controller.allocation_matrix[:, column]
                self.assertAlmostEqual(column_wrench[0], 1.0)  # unit thrust
                self.assertAlmostEqual(column_wrench[1], xy[1], places=12)
                self.assertAlmostEqual(column_wrench[2], -xy[0], places=12)
                self.assertAlmostEqual(
                    self.EXPECTED_EFFECTIVENESS[name],
                    M1_MOTOR_EFFECTIVENESS[column],
                )


class MicroFreeFallTest(unittest.TestCase):
    """Motors off: a 180 g brick in the same 9.81 m/s^2 field."""

    def test_free_fall_matches_gravity_only(self) -> None:
        model, data, controller = make_airborne_micro()
        gravity = float(np.linalg.norm(model.opt.gravity))
        self.assertAlmostEqual(gravity, GRAVITY)

        state0 = controller.read_state(data)
        elapsed = step_with_thrusts(model, data, np.zeros(4), 0.25)
        state1 = controller.read_state(data)

        az = (state1.velocity_world[2] - state0.velocity_world[2]) / elapsed
        self.assertAlmostEqual(az, -gravity, places=9)
        np.testing.assert_allclose(state1.velocity_world[:2], 0.0, atol=1e-12)
        np.testing.assert_allclose(state1.angular_velocity_body, 0.0, atol=1e-12)
        expected_z = START_ALTITUDE - 0.5 * gravity * elapsed**2
        self.assertAlmostEqual(state1.position[2], expected_z, places=9)


class MicroHoverEquilibriumTest(unittest.TestCase):
    """Four motors at m*g/4 = 0.44145 N: zero acceleration, zero torque."""

    def test_hover_thrust_gives_zero_acceleration(self) -> None:
        model, data, controller = make_airborne_micro()
        per_motor = controller.mass * GRAVITY / 4.0
        self.assertAlmostEqual(per_motor, 0.44145)

        state0 = controller.read_state(data)
        elapsed = step_with_thrusts(model, data, np.full(4, per_motor), 0.5)
        state1 = controller.read_state(data)

        acceleration = (state1.velocity_world - state0.velocity_world) / elapsed
        np.testing.assert_allclose(acceleration, 0.0, atol=1e-9)
        # Equal thrusts on a symmetric X layout cancel every torque.
        np.testing.assert_allclose(state1.angular_velocity_body, 0.0, atol=1e-12)

    def test_increased_collective_thrust_accelerates_upward(self) -> None:
        model, data, controller = make_airborne_micro()
        per_motor = controller.mass * GRAVITY / 4.0
        state0 = controller.read_state(data)
        elapsed = step_with_thrusts(model, data, np.full(4, 1.5 * per_motor), 0.2)
        state1 = controller.read_state(data)
        az = (state1.velocity_world[2] - state0.velocity_world[2]) / elapsed
        self.assertAlmostEqual(az, 0.5 * GRAVITY, places=9)


class MicroMixerSignTest(unittest.TestCase):
    """Roll/pitch/yaw signs on the micro plant (same conventions as M0).

    DELTA is 0.2 N so hover +/- DELTA stays inside [0, 1.1036] N.
    """

    DELTA = 0.2

    def _measure(self, pattern: np.ndarray):
        model, data, controller = make_airborne_micro()
        hover = np.full(4, controller.mass * GRAVITY / 4.0)
        thrusts = hover + self.DELTA * np.asarray(pattern, dtype=float)
        self.assertTrue(np.all(thrusts >= 0.0))
        self.assertTrue(np.all(thrusts <= controller.maximum_thrusts))

        wrench = controller.allocation_matrix @ thrusts
        self.assertAlmostEqual(wrench[0], controller.mass * GRAVITY, places=12)

        state0 = controller.read_state(data)
        data.ctrl[:] = thrusts
        mujoco.mj_step(model, data)
        state1 = controller.read_state(data)
        alpha = (state1.angular_velocity_body - state0.angular_velocity_body) / (
            model.opt.timestep
        )

        step_with_thrusts(model, data, thrusts, 0.1)
        state2 = controller.read_state(data)
        euler = rotation_to_euler(state2.rotation_body_to_world)
        expected_alpha = wrench[1:] / model.body_inertia[controller.body_id]
        return alpha, expected_alpha, euler, state2

    def test_roll_positive_torque_raises_left_side(self) -> None:
        alpha, expected_alpha, euler, state2 = self._measure(
            [+1.0, -1.0, -1.0, +1.0]
        )
        np.testing.assert_allclose(alpha, expected_alpha, rtol=1e-3, atol=1e-6)
        self.assertGreater(alpha[0], 0.0)
        self.assertLess(abs(alpha[1]), 1e-6)
        self.assertLess(abs(alpha[2]), 1e-6)
        self.assertGreater(euler[0], 0.0)
        self.assertGreater(state2.angular_velocity_body[0], 0.0)

    def test_pitch_rear_up_rotates_nose_down(self) -> None:
        alpha, expected_alpha, euler, state2 = self._measure(
            [-1.0, -1.0, +1.0, +1.0]
        )
        np.testing.assert_allclose(alpha, expected_alpha, rtol=1e-3, atol=1e-6)
        self.assertGreater(alpha[1], 0.0)
        self.assertLess(abs(alpha[0]), 1e-6)
        self.assertLess(abs(alpha[2]), 1e-6)
        self.assertGreater(euler[1], 0.0)  # nose-down reads positive (Z-up ZYX)
        self.assertGreater(state2.angular_velocity_body[1], 0.0)

    def test_yaw_diagonal_pair_spins_counter_clockwise(self) -> None:
        alpha, expected_alpha, euler, state2 = self._measure(
            [+1.0, -1.0, +1.0, -1.0]
        )
        np.testing.assert_allclose(alpha, expected_alpha, rtol=1e-3, atol=1e-6)
        self.assertGreater(alpha[2], 0.0)
        self.assertLess(abs(alpha[0]), 1e-6)
        self.assertLess(abs(alpha[1]), 1e-6)
        self.assertGreater(euler[2], 0.0)
        self.assertGreater(state2.angular_velocity_body[2], 0.0)


class InertiaScalingTest(unittest.TestCase):
    """I must scale as m * L^2 for geometrically similar variants."""

    def test_component_model_values(self) -> None:
        ixx, iyy, izz = compute_inertia(M4A_NOMINAL)
        # All entries positive, Z the largest (flat quad), finite.
        self.assertGreater(ixx, 0.0)
        self.assertGreater(iyy, 0.0)
        self.assertGreater(izz, max(ixx, iyy))
        # Order-of-magnitude sanity: m * r^2 = 0.18 * 0.06^2 = 6.5e-4 is an
        # upper bound for any component.
        self.assertLess(izz, M4A_NOMINAL.mass_kg * M4A_NOMINAL.motor_radius_m**2)

    def test_geometric_scaling_follows_mass_times_length_squared(self) -> None:
        for scale in (0.5, 2.0):
            bigger = geometrically_scaled(M4A_NOMINAL, scale)
            ratio = compute_inertia(bigger) / compute_inertia(M4A_NOMINAL)
            # Constant-density scaling: m -> s^3 * m, L -> s * L,
            # so I -> s^5 * I == (m ratio) * (L ratio)^2.
            np.testing.assert_allclose(ratio, scale**5, rtol=1e-12)
            mass_ratio = bigger.mass_kg / M4A_NOMINAL.mass_kg
            length_ratio = bigger.wheelbase_m / M4A_NOMINAL.wheelbase_m
            np.testing.assert_allclose(
                ratio, mass_ratio * length_ratio**2, rtol=1e-12
            )

    def test_pure_mass_scaling_doubles_inertia(self) -> None:
        doubled = MicroUAVConfig(
            mass_kg=2.0 * M4A_NOMINAL.mass_kg,
            motor_mass_kg=2.0 * M4A_NOMINAL.motor_mass_kg,
        )
        np.testing.assert_allclose(
            compute_inertia(doubled) / compute_inertia(M4A_NOMINAL), 2.0
        )


class ControlAuthorityTest(unittest.TestCase):
    """Finite, nonzero, physically consistent angular acceleration authority."""

    def test_angular_acceleration_authority(self) -> None:
        model, _, controller = load_simulation(M4A_MODEL)
        inertia = model.body_inertia[controller.body_id]
        max_thrust = M4A_NOMINAL.max_thrust_per_motor_n
        a = M4A_NOMINAL.motor_xy_m
        # Raw torque ceilings: two motors per side at full vs zero thrust.
        max_roll_torque = 2.0 * a * max_thrust
        max_pitch_torque = 2.0 * a * max_thrust
        max_yaw_torque = 2.0 * M4A_NOMINAL.yaw_torque_per_thrust * max_thrust
        alphas = np.array(
            [
                max_roll_torque / inertia[0],
                max_pitch_torque / inertia[1],
                max_yaw_torque / inertia[2],
            ]
        )
        self.assertTrue(np.all(np.isfinite(alphas)))
        self.assertTrue(np.all(alphas > 0.0))
        # Consistency: the allocation matrix must reproduce the same
        # analytic torque ceilings.
        wrench = controller.allocation_matrix @ np.array(
            [max_thrust, 0.0, 0.0, max_thrust]
        )
        self.assertAlmostEqual(wrench[1], max_roll_torque, places=12)
        # The micro plant must be much more agile than the 1 kg M0 plant;
        # assert only the direction (the report carries the numbers).
        m0_model, _, m0_controller = load_simulation(
            Path(__file__).resolve().parents[1] / "models" / "quadrotor.xml"
        )
        m0_inertia = m0_model.body_inertia[m0_controller.body_id]
        m0_alpha_roll = (2.0 * 0.163 * 8.0) / m0_inertia[0]
        self.assertGreater(alphas[0], m0_alpha_roll)


class VariantApplicationTest(unittest.TestCase):
    """apply_micro_variant must turn the loaded model into the variant."""

    def test_mass_wheelbase_thrust_variants(self) -> None:
        variant = MicroUAVConfig(mass_kg=0.150, wheelbase_m=0.100,
                                 thrust_to_weight_max=3.0)
        model, data, _ = load_simulation(M4A_MODEL)
        apply_micro_variant(model, data, variant)

        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.assertAlmostEqual(float(model.body_mass[body_id]), 0.150)
        np.testing.assert_allclose(
            model.body_inertia[body_id], compute_inertia(variant), rtol=1e-9
        )
        a = variant.motor_xy_m
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "rotor_fl_site"
        )
        np.testing.assert_allclose(model.site_pos[site_id][:2], (a, a))
        np.testing.assert_allclose(
            model.actuator_ctrlrange[:, 1], variant.max_thrust_per_motor_n
        )
        self.assertAlmostEqual(
            variant.max_thrust_per_motor_n, 3.0 * 0.150 * GRAVITY / 4.0
        )

    def test_provenance_table_covers_key_parameters(self) -> None:
        rows = {row["parameter"]: row for row in provenance_table(M4A_NOMINAL)}
        for key in (
            "mass",
            "wheelbase",
            "thrust_to_weight_max",
            "hover_thrust_per_motor",
            "max_thrust_per_motor",
            "inertia",
            "motor_kv",
            "yaw_torque_per_thrust",
        ):
            self.assertIn(key, rows)
        for row in rows.values():
            self.assertIn(row["provenance"], PROVENANCE_LABELS)
        # The two numbers most likely to be misquoted must be labeled.
        self.assertEqual(rows["mass"]["provenance"], "ENGINEERING_ESTIMATE")
        self.assertEqual(rows["max_thrust_per_motor"]["provenance"],
                         "ENGINEERING_ESTIMATE")
        self.assertEqual(rows["hover_thrust_per_motor"]["provenance"],
                         "CALCULATED")


class MicroControllerGainTest(unittest.TestCase):
    """The M4-A controller keeps the M0 gains for the position loop but
    scales attitude torque gains by the inertia ratio. Evidence for why:
    with M0 attitude_kd=0.18 on Ixx=8.5e-5 kg m^2 at 500 Hz, the explicit
    rate-damping factor is 1 - kd*dt/I = -3.24, an unstable pole that
    destroyed takeoff in the first M4-A hover run (unsafe altitude abort).
    """

    M0_INERTIA = np.array([0.018, 0.018, 0.030])  # models/quadrotor.xml

    def test_attitude_gains_scale_with_inertia_ratio(self) -> None:
        params = micro_controller_parameters(M4A_NOMINAL)
        base = ControllerParameters()
        ratio = compute_inertia(M4A_NOMINAL) / self.M0_INERTIA
        np.testing.assert_allclose(params.attitude_kp, base.attitude_kp * ratio)
        np.testing.assert_allclose(params.attitude_kd, base.attitude_kd * ratio)
        np.testing.assert_allclose(params.max_torque, base.max_torque * ratio)

    def test_position_loop_gains_unchanged(self) -> None:
        params = micro_controller_parameters(M4A_NOMINAL)
        base = ControllerParameters()
        np.testing.assert_allclose(params.position_kp, base.position_kp)
        np.testing.assert_allclose(params.position_kd, base.position_kd)
        self.assertEqual(params.position_ki_z, base.position_ki_z)
        self.assertEqual(params.max_tilt_radians, base.max_tilt_radians)

    def test_scaled_loop_is_discretely_stable_at_500hz(self) -> None:
        params = micro_controller_parameters(M4A_NOMINAL)
        inertia = compute_inertia(M4A_NOMINAL)
        dt = 0.002
        # Explicit-Euler rate-damping pole: |1 - kd*dt/I| < 1. This is the
        # exact condition the unscaled M0 gains violated on the micro plant.
        pole = 1.0 - params.attitude_kd * dt / inertia
        self.assertTrue(np.all(np.abs(pole) < 1.0))
        # Closed-loop attitude bandwidth/damping must equal the M0 loop.
        omega_n_micro = np.sqrt(params.attitude_kp / inertia)
        omega_n_m0 = np.sqrt(
            ControllerParameters().attitude_kp / self.M0_INERTIA
        )
        np.testing.assert_allclose(omega_n_micro, omega_n_m0, rtol=1e-12)


if __name__ == "__main__":
    unittest.main()
