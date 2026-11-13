"""Compile native collision vertices into non-RL planning support geometry."""
import numpy as np
import mujoco

def _compiled_contact_geometry(plant, scorer):
    from scipy.spatial import ConvexHull
    from scipy.spatial.transform import Rotation

    from .asm2_contact_control import ContactGeometry

    model = plant.model
    collision_geoms = np.flatnonzero(
        (model.geom_bodyid == plant.body) & (model.geom_contype | model.geom_conaffinity).astype(bool))
    pads, airframe = [], []
    for geom in collision_geoms:
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            raise ValueError('ASM2 planning expects the registered native mesh collision geometry')
        mesh = model.geom_dataid[geom]
        start, length = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        rotation = Rotation.from_quat(model.geom_quat[geom], scalar_first=True)
        vertices = rotation.apply(model.mesh_vert[start:start+length]) + model.geom_pos[geom]
        (pads if geom in scorer.patch_ids else airframe).append(vertices)
    hull = np.concatenate(airframe)
    return ContactGeometry(np.concatenate(pads), hull[ConvexHull(hull).vertices], scorer.halfwidth.copy())


