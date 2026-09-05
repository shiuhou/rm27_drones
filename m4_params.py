"""M4-A micro-UAV plant parameters with explicit provenance.

M4-A scales the verified M0-M3 plant (generic 1 kg quad) down to the RM27
direction: a 150-200 g / 2-2.5 inch micro quad. M4-A covers ONLY mass,
geometry, inertia and thrust-authority scaling. Motor/prop/battery realism
(1103 9500 KV + 2"/2.5" + 2S/3S) is NOT validated here and belongs to M4-B.

Every parameter carries one of these provenance labels:

    MEASURED             -- measured on real hardware (none yet)
    DATASHEET            -- vendor datasheet/product spec available in-repo
    CAD_DERIVED          -- derived from CAD geometry (none yet)
    CALCULATED           -- pure physics arithmetic from other parameters
    ENGINEERING_ESTIMATE -- assumed value, must be replaced by measurement
    USER_PROVIDED        -- user-supplied hardware selection without an
                            in-repo datasheet source
    DESIGN_CHOICE        -- project design decision / candidate option

CRITICAL: max thrust comes from an ASSUMED thrust-to-weight ratio (2.5
nominal, 2.0/3.0 sensitivity). It is NOT a measured 1103 motor value.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math

import mujoco
import numpy as np

from fly import ROTOR_NAMES, ControllerParameters


# Same gravity as every model XML in this project.
GRAVITY = 9.81

PROVENANCE_LABELS = (
    "MEASURED",
    "DATASHEET",
    "CAD_DERIVED",
    "CALCULATED",
    "ENGINEERING_ESTIMATE",
    "USER_PROVIDED",
    "DESIGN_CHOICE",
)


@dataclass(frozen=True)
class MicroUAVConfig:
    """Physical description of the M4-A micro-UAV plant.

    Defaults are the nominal M4-A configuration. Sensitivity variants are
    built with dataclasses.replace (mass 0.150/0.200 kg, wheelbase
    0.100/0.140 m, thrust_to_weight_max 2.0/3.0).
    """

    # ENGINEERING_ESTIMATE: RM27 target class; final AUW not yet measured.
    mass_kg: float = 0.180
    # ENGINEERING_ESTIMATE: nominal inside the 100-140 mm design range.
    wheelbase_m: float = 0.120
    # ENGINEERING_ESTIMATE: assumed max thrust-to-weight; no bench data.
    thrust_to_weight_max: float = 2.5
    # ENGINEERING_ESTIMATE: 1103-class motor with short wires, ~8 g each.
    motor_mass_kg: float = 0.008
    # ENGINEERING_ESTIMATE: central body box (frame + FC + battery lump).
    body_length_m: float = 0.060
    body_width_m: float = 0.040
    body_height_m: float = 0.025
    # M4-A keeps a centered COM; COM offset realism is M4-B.
    com_offset_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # ENGINEERING_ESTIMATE: inherited from the M0 model convention. The real
    # 2"/2.5" prop reaction-torque coefficient is unknown until M4-B.
    yaw_torque_per_thrust: float = 0.018
    # USER_PROVIDED: 1103 9500 KV motor selection; no datasheet source is
    # present in this repository. KV alone does NOT determine thrust; it is
    # metadata until M4-B motor modeling.
    motor_kv: int = 9500
    # DESIGN_CHOICE: candidate options (informational only in M4-A).
    prop_diameter_in: float = 2.5
    battery_cells: int = 3

    @property
    def motor_radius_m(self) -> float:
        """Center-to-motor distance (CALCULATED from wheelbase)."""

        return self.wheelbase_m / 2.0

    @property
    def motor_xy_m(self) -> float:
        """|x| = |y| of each motor in an X layout (CALCULATED)."""

        return self.wheelbase_m / (2.0 * math.sqrt(2.0))

    @property
    def motor_positions_xy(self) -> tuple[tuple[float, float], ...]:
        """Motor (x, y) in canonical FL/FR/RR/RL order (CALCULATED)."""

        a = self.motor_xy_m
        return ((a, a), (a, -a), (-a, -a), (-a, a))

    @property
    def hover_thrust_per_motor_n(self) -> float:
        """CALCULATED: m*g/4. For 0.180 kg this is 0.44145 N."""

        return self.mass_kg * GRAVITY / 4.0

    @property
    def max_thrust_per_motor_n(self) -> float:
        """ENGINEERING_ESTIMATE: T/W assumption * m * g / 4.

        For the nominal config: 2.5 * 0.180 * 9.81 / 4 = 1.103625 N.
        This is NOT a measured 1103 + prop + battery thrust value.
        """

        return self.thrust_to_weight_max * self.mass_kg * GRAVITY / 4.0


M4A_NOMINAL = MicroUAVConfig()


# Verified M0 model inertia (models/quadrotor.xml diaginertia). Reference
# for the micro controller gain derivation below.
M0_REFERENCE_INERTIA = np.array([0.018, 0.018, 0.030])


def micro_controller_parameters(config: MicroUAVConfig) -> ControllerParameters:
    """Controller gains for a micro-UAV plant, derived from the verified M0
    gains by inertia-ratio scaling.

    Evidence this is required: with the M0 attitude_kd = 0.18 on the micro
    inertia Ixx = 8.5e-5 kg m^2 at the fixed 500 Hz control rate, the
    explicit rate-damping pole is 1 - kd*dt/I = -3.24 (|pole| > 1), which
    diverged within 80 ms and destroyed the first M4-A takeoff (unsafe
    altitude abort at 10 m).

    Scaling attitude_kp, attitude_kd and max_torque by I_micro / I_M0
    preserves the M0 closed-loop attitude bandwidth and damping EXACTLY:

        omega_n = sqrt(kp / I)   (unchanged)
        zeta    = kd / (2 sqrt(kp I))   (unchanged)

    and restores the discrete stability margin kd*dt/I = 0.02 that the M0
    loop had. Position-loop gains are untouched: thrust = m * a makes the
    position loop mass-invariant, so it transfers unchanged.
    """

    base = ControllerParameters()
    ratio = compute_inertia(config) / M0_REFERENCE_INERTIA
    return ControllerParameters(
        attitude_kp=base.attitude_kp * ratio,
        attitude_kd=base.attitude_kd * ratio,
        max_torque=base.max_torque * ratio,
    )


def compute_inertia(config: MicroUAVConfig) -> np.ndarray:
    """Simple component inertia model (all inputs ENGINEERING_ESTIMATE).

    Components:
    * central rectangular body (frame + FC + battery lump), mass
      m_body = mass - 4 * motor_mass, dimensions l x w x h:
          Ixx = (1/12) m (h^2 + w^2)
          Iyy = (1/12) m (h^2 + l^2)
          Izz = (1/12) m (l^2 + w^2)
    * four motor point masses at (+-a, +-a, 0), a = wheelbase / (2 sqrt(2)):
          Ixx += m_i (y_i^2 + z_i^2) = m_i a^2
          Iyy += m_i (x_i^2 + z_i^2) = m_i a^2
          Izz += m_i (x_i^2 + y_i^2) = 2 m_i a^2

    Returns [Ixx, Iyy, Izz] about the body origin (COM is centered in M4-A).
    """

    a = config.motor_xy_m
    m_motor = config.motor_mass_kg
    m_body = config.mass_kg - 4.0 * m_motor
    if m_body <= 0.0:
        raise ValueError(
            f"central body mass {m_body:.4f} kg <= 0: check mass/motor_mass"
        )
    length = config.body_length_m
    width = config.body_width_m
    height = config.body_height_m
    ixx = m_body * (height**2 + width**2) / 12.0 + 4.0 * m_motor * a**2
    iyy = m_body * (height**2 + length**2) / 12.0 + 4.0 * m_motor * a**2
    izz = m_body * (length**2 + width**2) / 12.0 + 8.0 * m_motor * a**2
    return np.array([ixx, iyy, izz], dtype=float)


def geometrically_scaled(
    config: MicroUAVConfig, length_scale: float
) -> MicroUAVConfig:
    """Constant-density geometric scaling: lengths x s, masses x s^3.

    Used by tests to demonstrate I proportional to m * L^2 (= s^5 * I).
    """

    s = float(length_scale)
    return replace(
        config,
        mass_kg=config.mass_kg * s**3,
        wheelbase_m=config.wheelbase_m * s,
        motor_mass_kg=config.motor_mass_kg * s**3,
        body_length_m=config.body_length_m * s,
        body_width_m=config.body_width_m * s,
        body_height_m=config.body_height_m * s,
    )


def apply_micro_variant(
    model: mujoco.MjModel, data: mujoco.MjData, config: MicroUAVConfig
) -> None:
    """Apply a MicroUAVConfig to a loaded model (mass, inertia, motor
    geometry, actuator thrust range).

    Call BEFORE constructing the controller so the controller caches the
    variant mass/allocation/limits, and BEFORE any initial-condition yaw:
    mj_setConst resets qpos to qpos0.
    """

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    if body_id < 0:
        raise ValueError("model must contain body 'quadrotor'")
    model.body_mass[body_id] = config.mass_kg
    model.body_inertia[body_id] = compute_inertia(config)

    for rotor_name, (x, y) in zip(ROTOR_NAMES, config.motor_positions_xy):
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{rotor_name}_site"
        )
        if site_id < 0:
            raise ValueError(f"model must contain site '{rotor_name}_site'")
        model.site_pos[site_id][0] = x
        model.site_pos[site_id][1] = y

    for rotor_name in ROTOR_NAMES:
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, rotor_name
        )
        if actuator_id < 0:
            raise ValueError(f"model must contain actuator '{rotor_name}'")
        model.actuator_ctrlrange[actuator_id] = (
            0.0,
            config.max_thrust_per_motor_n,
        )

    mujoco.mj_setConst(model, data)


def provenance_table(config: MicroUAVConfig) -> list[dict]:
    """Machine-readable provenance for every important M4-A parameter."""

    inertia = compute_inertia(config)
    return [
        {
            "parameter": "mass",
            "value": config.mass_kg,
            "unit": "kg",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "medium",
            "reason": "RM27 target class 150-200 g; final AUW not measured",
        },
        {
            "parameter": "wheelbase",
            "value": config.wheelbase_m,
            "unit": "m",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "medium",
            "reason": "nominal within 100-140 mm design range",
        },
        {
            "parameter": "thrust_to_weight_max",
            "value": config.thrust_to_weight_max,
            "unit": "-",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "low",
            "reason": "assumed authority; no 1103/prop/battery bench data",
        },
        {
            "parameter": "hover_thrust_per_motor",
            "value": config.hover_thrust_per_motor_n,
            "unit": "N",
            "provenance": "CALCULATED",
            "confidence": "high",
            "reason": "m*g/4 from assumed mass",
        },
        {
            "parameter": "max_thrust_per_motor",
            "value": config.max_thrust_per_motor_n,
            "unit": "N",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "low",
            "reason": "T/W assumption * m * g / 4; NOT a motor measurement",
        },
        {
            "parameter": "inertia",
            "value": [float(v) for v in inertia],
            "unit": "kg m^2",
            "provenance": "CALCULATED",
            "confidence": "low",
            "reason": "box + point-motor model on estimated masses/dims",
        },
        {
            "parameter": "motor_mass",
            "value": config.motor_mass_kg,
            "unit": "kg",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "medium",
            "reason": "1103-class motor typical weight, not weighed",
        },
        {
            "parameter": "body_dimensions",
            "value": [
                config.body_length_m,
                config.body_width_m,
                config.body_height_m,
            ],
            "unit": "m",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "low",
            "reason": "lumped frame/FC/battery box; no CAD yet",
        },
        {
            "parameter": "com_offset",
            "value": list(config.com_offset_xyz_m),
            "unit": "m",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "low",
            "reason": "centered COM assumed in M4-A; real COM is M4-B",
        },
        {
            "parameter": "yaw_torque_per_thrust",
            "value": config.yaw_torque_per_thrust,
            "unit": "N m / N",
            "provenance": "ENGINEERING_ESTIMATE",
            "confidence": "low",
            "reason": "inherited M0 convention; prop reaction torque unknown",
        },
        {
            "parameter": "motor_kv",
            "value": config.motor_kv,
            "unit": "RPM/V",
            "provenance": "USER_PROVIDED",
            "confidence": "medium",
            "reason": (
                "user-selected 1103 9500 KV; no in-repo datasheet source; "
                "KV does not set thrust"
            ),
        },
        {
            "parameter": "prop_diameter",
            "value": config.prop_diameter_in,
            "unit": "inch",
            "provenance": "DESIGN_CHOICE",
            "confidence": "medium",
            "reason": "candidate 2.0/2.5 inch; metadata only in M4-A",
        },
        {
            "parameter": "battery_cells",
            "value": config.battery_cells,
            "unit": "S",
            "provenance": "DESIGN_CHOICE",
            "confidence": "medium",
            "reason": "candidate 2S/3S; metadata only in M4-A",
        },
    ]
