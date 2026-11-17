# Geometry Contract

Contract ID: `rm27.geometry` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
Shared evidence/version rules are in the [index](README.md); transforms follow
[State](STATE.md). Hull count is an implementation property, not a task gate.

## Purpose and ownership

Sim owns canonical collision semantics, asset selection and geometry profiles.
External asset repositories retain source CAD/mesh ownership and licensing.
Non-RL may derive conservative screening geometry; RL consumes Sim's geometry.
A screening hull, render mesh, designated contact patch and complete native
collision model are different objects with distinct identities.

## Canonical definitions and fields

| Field / object | Definition, units and required evidence |
|---|---|
| `geometry_id/version/hash` | identifies the complete effective collision model, including material/contact settings and exclusions; aircraft label alone is insufficient |
| `aircraft_frame`, `T_Bpart` | registered rigid transform from every part into the body frame; translation m, rotation per State |
| `collision_hulls[]` | stable part/geom ID, mesh/content hash, transform, collision mask, enabled state, group and material/contact profile for **all** aircraft collision parts |
| `underside_shell` | explicit set of shell geom IDs and body-relative shape; cannot be inferred as “all patches” |
| `designated_patches[]` | 15 stable patch IDs for the currently selected ASM2 release, group, shape, transform, source surface, normal orientation and area definition |
| `forbidden_aircraft_regions` | all collision-enabled aircraft surfaces outside the admissible patch set; includes shell, guards and other body parts as registered |
| `target_id`, `target_geometry_hash` | selected target and its complete collision/reference geometry, not a visual lamp identity |
| `armor_surface` | registered surface geom set, plane/reference shape, finite contact region and any backing set |
| `target_frame_WT` | position and orientation of a right-handed frame, source registration and time/epoch |
| `surface_normal` | unit outward normal from target into free space; patch outward normals are separately defined |
| `obstacles[]` | registered surrounding geometry with IDs, transforms and collision/clearance roles; scene subset exclusions recorded |
| `collision_pair_policy` | admissible/forbidden/ignored pairs, masks and phase/time rules; ignored pairs are never implicitly safe |
| `tolerances` | numerical collision distance, contact plane, penetration, region edge, normal-angle and sweep-resolution tolerances with units, purpose and profile hash |
| `derived_geometry` | source model hash, algorithm/version, reduction method, coverage/conservatism assumptions and applicability |

“15 patches” identifies this release's designated material surfaces, not 15
sensor taxels or measured force areas. Metadata footprint area is a geometric
quantity, not actual pressure-bearing area. Front and rear groups need not touch
simultaneously under the current engineering profile. A new release changing
patch membership must explicitly version the geometry and evaluate compatibility.

## Target frame and normal convention

The current mapping S10 stores `across_axis`, `slope_axis`, `armor_normal` and a
surface center. The proposed canonical frame is `T = [across, slope, normal]`
(columns resolved in W), with `across × slope = normal`. Validate unit length,
orthogonality and determinant +1 within registered tolerances after frame
conversion; small rounding in input must have an explicit normalization policy.

The native planner also uses `[normal, across, slope]`, and native metadata
contains another `target_rotation_to_world` ordering. These are not interchangeable
matrix layouts. Consumers must name axes and register the permutation/signs;
frame compatibility is **not yet implemented** by this draft.

For point `x_B` on an aircraft part, `x_W = p_WB + R_WB x_B`;
`x_T = R_WTᵀ(x_W-p_WT)`. Normals use rotation only. Target clearance is positive
on the outward free-space side. A pad's outward material normal generally faces
opposite the target's outward normal at contact; use pair-specific orientation
rather than treating both as the same vector. Global scene translation or target
motion requires a registered transform/epoch, not edits to frozen data.

## Native collision classification

Record every candidate pair with physics interval, aircraft/target IDs, contact
point, signed distance m, contact normal, solver-active status and compressive
normal force N. Register the point and its sampling interval in the evaluation
world frame for [point-velocity derivation](EVALUATION.md#contact-point-kinematics).
The solver's pair normal and the registered target surface normal serve different
checks. A visual overlap or distance threshold is not a physical impulse.

1. Identify whether the pair includes an aircraft geom; retain other relevant
   safety contacts according to the scene policy.
2. Classify patch-to-registered-armor, wrong aircraft region, wrong surface,
   ground/obstacle, or unknown/unregistered pair. Unregistered is not admissible.
3. Evaluate the finite surface region, plane, normal and penetration tolerances.
4. Pass classification and geometry evidence to [Evaluation](EVALUATION.md),
   which applies the profile's motion/attitude gates and records forbidden events.
   A physically first forbidden collision cannot be replaced by a later valid pad hit.

All enabled aircraft parts must be covered. An identical set of 15 patches does
not establish identical shell clearance, forbidden-contact behavior or model
response when the remaining hulls differ.

## Investigation of the reported 11,874 hulls

The task input reports an **11,874-hull implementation**. This is **REPORTED**,
not a verified property of the locally selected asset. Inspection of registry
S00 and XML S01 found:

- Registry: ASM2 `1.0.0`, simplified proxy `0.1.0`.
- XML SHA-256: `2356bc3f8372fe14002521a82d4a9533867c688f8ed3d13cf172c74e17905ae4`.
- Static XML count: 3,536 geoms total, **3,522 collision-enabled** by explicit
  local masks, including **15 named contact patches**. The remaining 14 are
  visual-only. This phase did not compile or step the model.
- The inspected native scene provenance S18 references this registry/XML path.
  It records a field subset; it is not evidence of an 11,874-aircraft-hull profile.
- No complete-model ID, asset manifest/hash or equivalence evidence for the
  reported 11,874 count was located in the inspected local contracts/configs.
  It may refer to another release, decomposition or environment; exact origin
  remains **UNKNOWN**, not inferred or remotely audited.

Keep both identities separate if the fuller model is supplied later. Required
comparison includes all body transforms, masks, patch identities, enabled hulls,
mesh hashes and pair classifications, not merely counts or matching pads.

## Clearance and numerical tolerances

S11 `SceneClearance` creates an independent model for static aircraft/environment
queries, using a conservative superset of cross-pair masks. It samples reference
poses (default spacing `0.1 s`) and permits designated pad/plate overlap only in
the candidate contact window. It returns a sampled collision check, not a
continuous swept-volume certificate or native contact qualification.

S07 reduces nonpatch aircraft vertices to a convex hull for planning support;
this is not a replacement for the full collision model. S12's footprint reserve
is a bounded local plane projection under assumed stress, not surrounding-scene
clearance or calibrated collision probability.

The inspected contact-profile snapshot S05 uses plane error `0.005 m`, maximum
penetration `0.005 m`, normal error `20°` and finite halfwidths from S10. These are
**engineering profile settings**, not universal geometry tolerances or official
judging thresholds. Mesh approximation error, distance-query tolerance and
continuous-sweep error bounds are still **UNKNOWN**. Do not make them up from a
contact allowance. Declared conservative bounds are required for a continuous
claim; otherwise label results `SAMPLED_ONLY` and coverage as incomplete.

## Time, compatibility and conformance

Static geometry has content identity and an effective registration epoch. Moving
objects carry pose time/clock/uncertainty as in State. Missing, stale or mismatched
assets block physical use. Hash the XML include closure, referenced collision
meshes, transformations, masks, target mapping, subset selection and effective
contact/solver configuration. Machine-specific compiled MJB hashes alone are
not portable model identities.

Required tests in the next phase:

1. Full asset-reference closure; every enabled hull belongs to a registered
   aircraft part, patch or obstacle. Missing hulls and altered masks are detected.
2. Transform/normal and finite-region checks under nonzero attitude and
   translation; explicitly test the different target-axis orderings.
3. Native pad-first, guard-first, shell-first, edge, back-face, simultaneous mixed
   pairs and early forbidden contacts, including the same pads with different shells.
4. Sampled-versus-swept near-miss cases, transient between samples and conservative
   reduction containment; incomplete coverage cannot return continuous-clearance PASS.
5. Geometry/hash/profile mismatch refusal and replay of old geometry without substitution.

**IMPLEMENTED:** native geometry registration/classification and planning queries
(S04/S07/S10/S11). **TESTED in this phase:** static XML counting/hash consistency.
**PROPOSED:** complete cross-consumer geometry identity and conformance suite.
**UNKNOWN:** 11,874-hull provenance, continuous-sweep certification, final physical
contact/material calibration and **PENDING RL CONSUMER VALIDATION** of full geometry.
