import mujoco
import numpy as np
import pytest
from ardupilot_bridge.plant_adapter import Plant
from rm_world.field_adapter import FieldPlant, FIELD, SPAWNS


@pytest.fixture(scope="module")
def plant():
    return FieldPlant()


def test_preserved_uav(plant):
    baseline = Plant()
    m = plant.model
    b = baseline.model
    assert m.opt.timestep == b.opt.timestep == 0.002
    assert m.opt.integrator == b.opt.integrator
    assert m.opt.density == b.opt.density and m.opt.viscosity == b.opt.viscosity
    np.testing.assert_array_equal(m.opt.gravity, b.opt.gravity)
    for attr in ["body_mass", "body_inertia", "body_ipos", "body_iquat"]:
        np.testing.assert_allclose(
            getattr(m, attr)[plant.body], getattr(b, attr)[baseline.body], atol=1e-14
        )
    for attr in [
        "actuator_gear",
        "actuator_ctrlrange",
        "actuator_dynprm",
        "actuator_dyntype",
        "actuator_gainprm",
    ]:
        np.testing.assert_array_equal(getattr(m, attr), getattr(b, attr))
    assert m.nq == 7 and m.nv == 6 and m.nu == 4


def test_all_field_geoms_preserved(plant):
    m = mujoco.MjModel.from_xml_path(str(FIELD))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert m.ngeom == 555
    for i in range(m.ngeom):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i)
        j = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert j >= 0
        for attr in ["geom_xpos", "geom_xmat"]:
            np.testing.assert_allclose(
                getattr(plant.data, attr)[j], getattr(d, attr)[i], atol=1e-12
            )
        for attr in [
            "geom_type",
            "geom_size",
            "geom_contype",
            "geom_conaffinity",
            "geom_group",
            "geom_friction",
            "geom_solref",
            "geom_solimp",
        ]:
            np.testing.assert_array_equal(
                getattr(plant.model, attr)[j], getattr(m, attr)[i]
            )


def test_spawn_clear_and_local_frame(plant):
    plant.reset()
    assert plant.data.ncon == 0
    np.testing.assert_allclose(
        plant.data.qpos[:3], SPAWNS["safe_test_spawn"] + [0, 0, 0.12]
    )
    np.testing.assert_allclose(plant.state()["position"], [0, 0, -0.12], atol=1e-14)
    plant.data.qpos[:3] += np.array([1, 2, 3])
    np.testing.assert_allclose(plant.state()["position"], [1, -2, -3.12], atol=1e-14)
    plant.reset()


def test_rotor_geometry_unchanged(plant):
    baseline = Plant()
    for i in range(baseline.model.nsite):
        name = mujoco.mj_id2name(baseline.model, mujoco.mjtObj.mjOBJ_SITE, i)
        j = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_SITE, name)
        assert j >= 0
        np.testing.assert_array_equal(
            plant.model.site_pos[j], baseline.model.site_pos[i]
        )
    for name in [
        "fuselage",
        "arm_fl_rr",
        "arm_fr_rl",
        "nose",
        "prop_fl",
        "prop_fr",
        "prop_rr",
        "prop_rl",
    ]:
        a = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        b = mujoco.mj_name2id(baseline.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for attr in [
            "geom_pos",
            "geom_quat",
            "geom_size",
            "geom_contype",
            "geom_conaffinity",
        ]:
            np.testing.assert_allclose(
                getattr(plant.model, attr)[a],
                getattr(baseline.model, attr)[b],
                atol=1e-14,
            )
