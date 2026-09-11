"""Preserve native field coordinates and compose the verified UAV in memory."""

from copy import deepcopy
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from ardupilot_bridge.plant_adapter import Plant, MODEL
from ardupilot_bridge.frames import to_ap
from ardupilot_bridge.actuators import pwm_to_thrust

ROOT = Path(__file__).resolve().parents[1]
FIELD = ROOT.parent / "rmuc2026_battlefield_mjcf/model.xml"
SPAWNS = {"safe_test_spawn": np.array([-9.55, 3.75, 0.5])}


def compose_xml(field=FIELD, spawn="safe_test_spawn"):
    uav = ET.parse(MODEL).getroot()
    terrain = ET.parse(field).getroot()
    allowed = {
        "compiler",
        "option",
        "statistic",
        "visual",
        "default",
        "asset",
        "worldbody",
    }
    if any(e.tag not in allowed for e in terrain):
        raise ValueError("field root section requires explicit composition handling")
    if terrain.find("compiler").get("angle") != "radian":
        raise ValueError("field angle convention changed")
    # The UAV tree is degrees. Convert its orientation attributes before merging.
    for e in uav.iter():
        for name in ("euler", "axisangle"):
            if name in e.attrib:
                values = np.fromstring(e.get(name), sep=" ")
                if name == "euler":
                    values = np.deg2rad(values)
                else:
                    values[-1] = np.deg2rad(values[-1])
                e.set(name, " ".join(map(str, values)))
    uav.find("compiler").set("angle", "radian")
    uav_names = {e.get("name") for e in uav.iter() if e.get("name")}
    field_names = {e.get("name") for e in terrain.iter() if e.get("name")}
    if uav_names & field_names:
        raise ValueError(f"composition name collision: {uav_names & field_names}")
    if uav.find("default") is not None:
        raise ValueError("new UAV defaults require composition review")
    defaults = deepcopy(terrain.find("default"))
    if defaults.attrib or any(e.tag != "default" for e in defaults):
        raise ValueError("field has global defaults that could affect UAV")
    uav.append(defaults)
    for asset in terrain.find("asset"):
        asset = deepcopy(asset)
        if "file" in asset.attrib:
            path = (field.parent / asset.get("file")).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            asset.set("file", str(path))
        uav.find("asset").append(asset)
    world = uav.find("worldbody")
    ground = world.find("geom[@name='ground']")
    world.remove(ground)
    body = world.find("body[@name='quadrotor']")
    body.set("pos", " ".join(map(str, SPAWNS[spawn] + [0, 0, 0.12])))
    for child in terrain.find("worldbody"):
        world.append(deepcopy(child))
    return ET.tostring(uav, encoding="unicode")


class FieldPlant(Plant):
    def __init__(self, spawn="safe_test_spawn"):
        self.origin = SPAWNS[spawn].copy()
        t = time.perf_counter()
        self.xml = compose_xml(spawn=spawn)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.compile_seconds = time.perf_counter() - t
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.step_us = []
        self.reset()

    def state(self):
        state = super().state()
        state["position"] = to_ap(self.data.qpos[:3] - self.origin).tolist()
        return state

    def step(self, pwm):
        self.pre_step_velocity = self.data.qvel[:3].copy()
        self.data.ctrl[:] = pwm_to_thrust(pwm)
        start = time.perf_counter_ns()
        mujoco.mj_step(self.model, self.data)
        self.step_us.append((time.perf_counter_ns() - start) / 1000)
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def contacts(self):
        result = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            names = [
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(j))
                for j in c.geom
            ]
            if not any(n and n.startswith("field__collision__") for n in names):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            result.append(
                {
                    "geoms": names,
                    "distance_m": float(c.dist),
                    "position": c.pos.tolist(),
                    "normal_geom0_to_geom1": c.frame[:3].tolist(),
                    "force_contact_frame": force.tolist(),
                }
            )
        return result

    def truth(self):
        result = super().truth()
        result["navigation_origin_nwu"] = self.origin.tolist()
        result["contacts"] = self.contacts()
        result["mj_step_us"] = self.step_us[-1] if self.step_us else None
        return result
