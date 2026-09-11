# RM27 G1 Gazebo→MuJoCo Field Migration Report

Status: **source audit completed; G1 is not complete**. No MuJoCo field or
field-flight success is claimed. The located source has no interior collision
geometry, which conflicts with the requested golden-reference collision gates.
The source selection/interior-collision decision is pending.

## 1. Gazebo field inventory

Audited source:
`/home/shiuhou/Projects/rm27_drones/rm_uav_lab/rm27_uav_sim/src/rm27_field`.
Repository HEAD: `3619543`; field introduced at C0 (`9403950`). Existing unrelated
untracked scripts in that repository were left untouched.

World: `worlds/rmuc_2025_world.sdf`, SDF 1.9, world `rmuc_2025`.
It nests static `rmuc_2025_field` → include `model://rmuc_2025` at identity →
static `rmuc_2025` → link `body` at `(14.5, 8, 0.2, 0, 0, 0)`.
Source and installed model copies both contain only the five colliders below.
The PX4 `rm27_rmuc_2025_test.sdf` wrapper includes this same C0 field.

[Machine-readable inventory](gazebo_inventory.json) records file SHA-256 hashes,
asset scale, raw plugin/material definitions, transforms, each collider's world
bounds and all 516 connected mesh components without repairing the source.

| Collider | Gazebo world center (m) | Full dimensions (m) |
|---|---|---|
| floor | (14.5, 8, 0.1) | (30, 18, 0.2) |
| north wall | (14.5, 16.9, 1.2) | (30, 0.2, 2) |
| south wall | (14.5, −0.9, 1.2) | (30, 0.2, 2) |
| east wall | (29.4, 8, 1.2) | (0.2, 18, 2) |
| west wall | (−0.4, 8, 1.2) | (0.2, 18, 2) |

World plugins: Physics, SceneBroadcaster, UserCommands, Sensors (ogre2).
No field model/collision plugins, joints or moving components. The world places
`max_step_size` inside the Physics plugin, rather than a world `<physics>`
element; this audit does not claim that Gazebo applies that timestep setting.

## 2. Source assets reused

Inspected `models/rmuc_2025/meshes/rmuc_2025.stl`, 114,234 triangles, mesh scale
(1,1,1), 516 connected components, not globally watertight. It is currently used
only by a `<visual>`, never by `<collision>`. No textures; one uniform material.
Source README attributes the visual asset to
SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator. No source asset was copied,
modified, rescaled or used to invent colliders at this stage.

## 3. Coordinate-frame mapping

The source's `docs/simulation_frames.md` defines Gazebo ENU (+X east, +Y north,
+Z up), field floor top at world Z=0.2 m, and test origin at (3.5,9.5,0.2).
The audited standalone world does not explicitly declare spherical coordinates;
ENU is the project convention, not an inferred SDF element.

Proposed mapping preserves the verified bridge's NWU world convention:
RM logical frame is ENU with origin at Gazebo (0,0,0.2).
For a Gazebo point `(x,y,z)`, logical coordinates are `(x,y,z−0.2)`, MuJoCo NWU
coordinates `(y,−x,z−0.2)`, and absolute field NED coordinates `(y,x,−z+0.2)`.
Autopilot local NED also requires subtraction of a documented spawn origin.
Rotations would use the same explicit basis matrices. This is a proposal;
landmark tests and field composition have not yet been implemented.

## 4. Conversion approach

Await correct golden-reference identification, or a decision to use this C0
source and add independently derived interior colliders. That latter option is
an extension beyond the original Gazebo collision model, and must be reported
as such. An untouched reference can demonstrate pass-through at interior
structures; it cannot demonstrate a matching positive contact there.

## 5. Simplifications

No new simplifications performed. Existing source discrepancies:

- Visual floor bounds correspond to 29 × 16 m; collision floor is 30 × 18 m.
- Floor collider top is Z=0.2 m, with bounds X=[−0.5,29.5], Y=[−1,17].
- Perimeter colliders top out at Z=2.2 m.
- Visual mesh includes four tall perimeter components reaching Z≈18.7546 m.
- All ramps, base/outpost-like structures and central obstacles are visual-only.
- SDF has no semantic names for those interior mesh parts; component index is
  not a verified base/outpost identification.

The source README explicitly states that original mesh collision was replaced
with floor/perimeter boxes for C0. Treating this as a mature interior collision
reference would conceal a known gap.

## 6. Collision architecture

Existing Gazebo field: five static boxes; no per-collider surface overrides.
MuJoCo counterpart: not created pending the source decision. The verified UAV
model, bridge and nominal hover configuration remain untouched.

## 7. Landmark comparison

Gazebo collider floor top and outer corners are inventoried. Base/outpost and
major obstacle semantics are unresolved in the monolithic visual mesh. No
cross-simulator landmark result is claimed.

## 8. Collision test results

Not run. SDF inspection proves that interior visual structures have no matching
collision elements; this is not presented as an observed dynamic test.
No floor/obstacle/Gazebo-equivalence gate is marked passing yet.

## 9. ArduPilot flight-in-field results

Not run. Prior nominal ArduPilot–MuJoCo flight evidence remains unchanged and
must not be relabeled as flight in the RM field. No flight or mission code was
added during this audit.

## 10. Remaining mismatches

The requested golden reference appears more capable than the available C0
collision implementation. G1 requires a source decision before geometry and
collision equivalence can be assessed honestly. No original Gazebo files were
changed; no current simulator was stopped or reconfigured.

Audit reproduction, from `mujoco_uav_ardupilot`:

```bash
/home/shiuhou/venvs/mujoco/bin/python -m rm_field_g1.audit
```

Output: `Inventoried 5 colliders and 516 visual components`.
The audit program intentionally handles the inspected source structure and
rejects unsupported new transform/geometry cases instead of silently converting.

## 11. Recommendation for G2

Do not begin G2. First resolve the G1 source mismatch, then continue field
composition, landmark checks, physical collision probes and real flight-in-field
verification. No perception or mission implementation is authorized by this report.
