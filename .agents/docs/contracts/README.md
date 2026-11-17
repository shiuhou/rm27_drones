# Six Shared Contracts — Phase 1.1 Review Baseline

Date: **2026-11-13** · Set version: **0.1.0-draft** · Status: **PROPOSED / REVIEWED AND FROZEN FOR PHASE 1.1**.

The main repository owns this documentation baseline. The focused design review
is complete for all six drafts. Freeze identifies these reviewed definitions;
it does not accept Contract V1.0.0, approve Phase 2, certify runtime compatibility
or replace historical experiment criteria. Review and checks were local only.

## Contract review status

| Contract | Phase 1.1 status | Review outcome and open dependency |
|---|---|---|
| [State](STATE.md) | Reviewed draft · frozen | Keep origin-based state and derive contact-point velocity; runtime frame/clock mappings and RL observation lineage still unverified. |
| [Dynamics](DYNAMICS.md) | Reviewed draft · frozen | Separate rotor/actuator physics from allocation; full-tensor native model versus diagonal prediction and unverified actuator parameters remain. |
| [Geometry](GEOMETRY.md) | Reviewed draft · frozen | Register contact points, interval and surface normal for Evaluation; 11,874-hull identity and swept coverage remain unknown. |
| [Control](CONTROL.md) | Reviewed draft · frozen | Separate simulation/flight allocation and the two consumer adapters; effective allocation, action mappings and timing still need evidence. |
| [Evaluation](EVALUATION.md) | Reviewed draft · frozen | Define point-motion gates and persistent forbidden-event evidence; keep the four-Q formula; runtime scorer parity and full event coverage unverified. |
| [Feasibility](FEASIBILITY.md) | Reviewed draft · frozen | Record provenance per stage and distinguish oracle prediction from sensor autonomy; continuous clearance and handover region remain unvalidated. |

## Review decisions

1. **Motor allocation:** `rm27_sim` owns canonical rotor/actuator physics and
   simulation allocation. `rm27_drone_px4` owns actual flight allocation through
   PX4. `rm27_RL` owns the action adapter, and `rm27_non_rl` owns the controller
   adapter. Log the effective allocator/profile; matching CTBR does not imply
   matching motor outputs.
2. **Contact-point velocity:** [Evaluation](EVALUATION.md#contact-point-kinematics)
   derives both point velocities using origin velocity and angular velocity
   crossed with the lever arm, with the aircraft COM registered via Dynamics.
   Use the difference in W at the declared interval. State's target-frame
   coordinate derivative is distinct; no new mandatory State field.
3. **Forbidden contact:** `forbidden_events` is persistent evidence. Earlier or
   simultaneous forbidden contact prevents first-contact qualification; any
   forbidden event in the clearance window fails `Q_clearance`. Later events
   do not rewrite `Q_contact`. No extra Q term; the completion formula is unchanged.
4. **Feasibility provenance:** [stage evidence](FEASIBILITY.md#stage-evidence-provenance)
   records state ancestry, feedback, contact-event source and effective model and
   evaluator. A native forecast with oracle feedback or trigger remains optimistic
   prediction. Scorer-only truth is separate from control or trigger inputs.

All six use the sections and common conventions here as one specification set.
Their proposed fields are semantic definitions, **not generated JSON schemas or
new Python/C++ interfaces**. A `MUST`-level condition or requirement in this set
means a proposed future conformance requirement, not existing implementation.

## Ownership and consumer responsibility

| Owner | Canonical responsibility | Consumer obligation |
|---|---|---|
| Main repository | six contract meanings, versioning, cross-component compatibility and review | preserve source/model/evidence identities; no duplicate contracts in consumers |
| `rm27_sim` | canonical rotor/actuator physics, simulation allocation, geometry, native scorer, replay and evaluation | preserve independent physics/scoring without importing consumer algorithms |
| `rm27_drone_px4` | actual flight allocation through its PX4 firmware/configuration | identify firmware, allocator, rotor order, limits and mappings independently of Sim |
| `rm27_non_rl` | controller adapter, planning, guidance, estimation, recovery and candidate feasibility generation | declare approximations and available inputs; consume Sim's physical/evaluation services |
| `rm27_RL` | action adapter, policy, observations, training and RL experiment evaluation protocol | provide policy/normalizer/action/model identities and consume shared physical scoring; reward does not replace final success |

MJCF/CAD repositories remain source asset providers under Sim's selection and
provenance responsibility. Main owns specifications, not another physical plant.
Existing explicit legacy wrappers and frozen experiment code retain their meaning.

## Common identity and missing-data conventions

Each record has a conceptual envelope:

| Field | Semantics |
|---|---|
| `contract_id/version` | record meaning and version; this draft is not a currently accepted wire version |
| `run_id`, `record_id`, `source`, `sequence` | run/stream identity and ordered records; sequence restarts need a new stream generation |
| `input_kind`, `lineage` | `GROUND_TRUTH`, `SYNTHETIC`, `RECORDED`, `REAL_MEASURED`, `ESTIMATED` or `PREDICTED`; derivation preserves original source and transformations |
| `clock_id`, `frame_id`, `reset_epoch` | exact time/world/state registrations, not implicit global defaults |
| `model_id/hash`, `geometry_id/hash`, `config_hash` | effective identities when physical state, commands or evaluation depend on them |
| `availability`, `reason` | supported/valid/missing/unknown per capability; null with reason for unavailable values |

A recording retains its original source kind and adds replay/receipt clock identity;
“recorded” alone is not a sensor qualification. GroundTruthState is identified as
truth even when it uses a struct named `VehicleEstimate`. Predictions retain the
latest informative measurement time. Validity is not calibrated probability.
No zero covariance, zero RPM, perfect clock or static target motion is invented
to make incomplete data pass. Optional fields remain optional; only an asserted
capability's required inputs gate that capability.

No transport is selected here. The old six-worker JSONL profile remains an
independent, source-locked synthetic integration snapshot with its own format.
Its earlier PASS does not approve this set or new consumer/model combinations.

## Clocks, timing and evidence

Physical simulation time, sensor acquisition, estimator state epoch, publication,
receiver receipt and host monotonic solver timing are distinct. Use seconds and
declare each clock. A verified clock mapping carries direction, scale/offset,
valid interval and uncertainty; no mapping means no cross-clock freshness claim.
Receivers compute age at use and enforce their registered limits. Sequence order
and a producer age field do not establish freshness. Unknown historical clock
or epoch data stays unknown. Prediction and resampling retain informative ages.

| Evidence label | Meaning in these documents |
|---|---|
| `IMPLEMENTED` | inspected local source contains the named behavior; no new execution implied |
| `TESTED` | a stated check actually ran; Phase 1 static checks remain historical and Phase 1.1 only checks documentation |
| `REPORTED` | task input or existing local reports record a result; not rerun/recertified |
| `PROPOSED` | draft contract or conformance requirement; design review does not imply runtime adoption |
| `UNKNOWN` | evidence absent or insufficient; RL-specific gaps also carry `PENDING RL CONSUMER VALIDATION` |
| `UNVERIFIED` | identified value or behavior lacks the required confirming evidence |

Result verdicts are separate: `PASS`, `FAIL`, `NOT_RUN`, `BLOCKED`. Required checks
aggregate FAIL first, then BLOCKED, then NOT_RUN; PASS requires all required checks.
Record individual statuses and reasons even when an overall verdict exists.
A static documentation check PASS is not a flight, dynamics or evaluator PASS.

## Phase 1 local architecture snapshot

| Repository | Local branch | HEAD |
|---|---|---|
| `.` | `astra/research` | `651f8a220b7b9df3051763ce14a62da6322a02b5` |
| `components/rm27_sim` | `feat/moving-base-target` | `f414b4c7517db7eae79ffee112d60bc45f2ef302` |
| `components/rm27_RL` | `i4/informed-reset-arrival` | `ed4ebc550800baf032348f67af5699b44e7953f2` |

Local worktrees are dirty; HEAD alone cannot reproduce inspected behavior.
Parent gitlinks already differ from some child HEADs. Staged renames, the local
Non-RL component and the `rm_uav_mujoco → rm27_sim` compatibility alias were
present before this task and were not changed. Non-RL has no independent Git
repository/remote assigned here. Local RL contains historical Contact/Stage1
source; it is **not assumed to be the complete current RL research workspace**.

## Source register

This is the Phase 1 inspection register, not a Phase 1.1 source audit or a lock
on the working tree. Paths are relative to the root workspace, and hashes identify inspected bytes,
not a clean published release. Sources, configs, XML and historical results were
read only. Small source excerpts and definitions were inspected without importing
simulators or running physics. Test source inspection does not mean test execution.
Local raw assets and artifact paths may be absent in a standalone clone; preserve
UNKNOWN/BLOCKED rather than fabricating them.

| ID | Local source | Inspected role | SHA-256 |
|---|---|---|---|
| S00 | [data/shared_aircraft/aircraft.json](../../../data/shared_aircraft/aircraft.json) | Shared ASM2 registry | `c818545af9cc3c96af7491366464428c40f5b305cddd7696ed19d5e54b4d0621` |
| S01 | [data/shared_aircraft/v1.0.0/models/asm2_isaac_proxy.xml](../../../data/shared_aircraft/v1.0.0/models/asm2_isaac_proxy.xml) | Current complete proxy XML, static inspection only | `2356bc3f8372fe14002521a82d4a9533867c688f8ed3d13cf172c74e17905ae4` |
| S02 | [data/shared_aircraft/v1.0.0/models/plant.json](../../../data/shared_aircraft/v1.0.0/models/plant.json) | ASM2 physical metadata and parameter provenance | `f4a5c52e8c06de893d3dc99b5544ba4408f03eed812bee52b595ee52db860b08` |
| S03 | [components/rm27_sim/packages/physics/src/rm27_contact_physics/model.py](../../../components/rm27_sim/packages/physics/src/rm27_contact_physics/model.py) | VehicleEstimate / TargetState and legacy model helpers | `2c5f95f6b680ca0a026e06c012b730284802edd27f1d6894c5faac99bf420146` |
| S04 | [components/rm27_sim/rm27_sim/simulation/asm2.py](../../../components/rm27_sim/rm27_sim/simulation/asm2.py) | Native scene, Plant, contact predicate | `b332be4323939e189abeac53227a3aba559e3d81ca91c50e7f8d5f4c09a1387d` |
| S05 | [components/rm27_sim/configs/asm2_contact_recovery.json](../../../components/rm27_sim/configs/asm2_contact_recovery.json) | Current engineering contact/recovery profile snapshot | `90fb3dda1e46f6f31b432ca6a43196a221247cdaf9a6dbcee8dd22a5f98e3acb` |
| S06 | [components/rm27_non_rl/rm27_non_rl/asm2_contact_control.py](../../../components/rm27_non_rl/rm27_non_rl/asm2_contact_control.py) | Observation, Reference, ContactGeometry, planner and tracker | `ff501ed06aad74147cc085ecd0c7199fea6957e0f08c52a21d932a5296ad40dc` |
| S07 | [components/rm27_non_rl/rm27_non_rl/geometry.py](../../../components/rm27_non_rl/rm27_non_rl/geometry.py) | Native geometry to planning support reduction | `568fb947cc4d7192122587f9f20c1f54926533fcb71fbb936e52fe303be61625` |
| S08 | [components/rm27_non_rl/rm27_non_rl/asm2_guidance.py](../../../components/rm27_non_rl/rm27_non_rl/asm2_guidance.py) | PendingPlan, adoption, reference continuity and sampled query use | `031281b964b0b346eec6a0aae4dea96a00df365195711c29294287917c1ce455` |
| S09 | [components/rm27_sim/rm27_sim/experiments/asm2_state_logging.py](../../../components/rm27_sim/rm27_sim/experiments/asm2_state_logging.py) | Per-channel clocks and unrecorded reset generation | `6ac28b6d897e6ef83c5cbdcb8671c9ef61cee83a9e2957a85a9b8b1b306f2874` |
| S10 | [components/rm27_sim/configs/base_target.json](../../../components/rm27_sim/configs/base_target.json) | Registered armor plane, axes, region and geometry IDs | `898b3c71b214768e7b3811112906deea5294fd93aaea3fc36b30432033e2e84e` |
| S11 | [components/rm27_sim/rm27_sim/simulation/clearance.py](../../../components/rm27_sim/rm27_sim/simulation/clearance.py) | Independent sampled scene queries | `6ab01749fe4c1fbfe89265ee8e5129a5f3766109b68c853c3b229bbf8fdf2e09` |
| S12 | [components/rm27_non_rl/rm27_non_rl/asm2_recoverability.py](../../../components/rm27_non_rl/rm27_non_rl/asm2_recoverability.py) | Footprint reserve and offline native forecast | `9c0f2c4c92d29cda93820bb8b7c540e12e51c1366ead025fd864ca1f0d00379a` |
| S13 | [components/rm27_RL/src/rm27_contact_rl/env.py](../../../components/rm27_RL/src/rm27_contact_rl/env.py) | Local historical ContactRecoveryEnv, not external RL audit | `4468077ffc887a409dc46e139ccec8c4243ef6066b468cd20ec17395f1ba0286` |
| S14 | [components/rm27_RL/src/rm27_contact_rl/stage1_env.py](../../../components/rm27_RL/src/rm27_contact_rl/stage1_env.py) | Local historical Stage1ContactEnv | `d404155e46582be3a2eb10ddd1a8e19856ce3319a6c3fb3ee0d0153e4b40e02b` |
| S15 | [components/rm27_non_rl/rm27_non_rl/asm2_contact.py](../../../components/rm27_non_rl/rm27_non_rl/asm2_contact.py) | Native experiment-loop qualification and recovery result | `3f3055bc07e63df2afb09045440b6b1be4e0cc9a65bc9c0500660f10154f386b` |
| S16 | [components/rm27_sim/packages/physics/src/rm27_contact_physics/evaluation.py](../../../components/rm27_sim/packages/physics/src/rm27_contact_physics/evaluation.py) | Legacy proxy ContactEvaluator, different from ASM2 profile | `655b46a7b3aff735fa9dc4aad805364d95968043749814b4141810cbfbe65c6c` |
| S17 | [components/rm27_non_rl/tests/test_asm2_contact.py](../../../components/rm27_non_rl/tests/test_asm2_contact.py) | Existing regression code inspected, NOT_RUN this phase | `d5eb880f0df5883e145f1164f9201d64b048e11b05d412214acc76e23e1320b8` |
| S18 | [components/rm27_sim/artifacts/asm2_contact_recovery/base_comparison_02/scene_provenance.json](../../../components/rm27_sim/artifacts/asm2_contact_recovery/base_comparison_02/scene_provenance.json) | Recorded historical scene provenance, no rerun | `0aa5165e762728f2d817e05d3ace6c0f3db6b7bc0953aefdbfb1b796d59d698f` |
| S19 | [components/rm27_non_rl/artifacts/asm2_contact_recovery/base_comparison_02/nominal_fixed/summary.json](../../../components/rm27_non_rl/artifacts/asm2_contact_recovery/base_comparison_02/nominal_fixed/summary.json) | Recorded failure FLIGHT_ENVELOPE | `c35a3350a6d1a72769d6f481269083f5d9dc75c5bee736461f895731004fd95c` |
| S19b | [components/rm27_non_rl/artifacts/asm2_contact_recovery/base_comparison_02/changed_handover_online/summary.json](../../../components/rm27_non_rl/artifacts/asm2_contact_recovery/base_comparison_02/changed_handover_online/summary.json) | Recorded failure EPISODE_WALL_BUDGET | `b03b241d073806e16486ad8a2350c7acd8e85c29b6dff1a27fffc36ce8c6e47d` |
| S20 | [components/rm27_RL/src/rm27_contact_rl/stage1_plant.py](../../../components/rm27_RL/src/rm27_contact_rl/stage1_plant.py) | Local model/scene adapter and source-hash checks | `c0c3900765e5fe476eb5302b5f8f36983e1e1ab5d605e5e159bfbd0f9f91c650` |
| S21 | [components/rm27_sim/packages/physics/src/rm27_contact_physics/teacher_interface.py](../../../components/rm27_sim/packages/physics/src/rm27_contact_physics/teacher_interface.py) | FLU force/rate proxy and MAVLink field mapping | `10311dd86d697b13c07958e74f987ab357ec21782488912988eb4c60edc09fc6` |
| S22 | [components/rm27_non_rl/configs/asm2_autonomous_handover.json](../../../components/rm27_non_rl/configs/asm2_autonomous_handover.json) | Current local Non-RL timing and synthetic assumptions | `5a90302acd49014ec44f946f088c8918b37d3f29e64d1c66d5e1de3ccda2dc17` |
| S23 | [components/rm27_sim/.agents/docs/ASM2_ESTIMATION_MODES.md](../../../components/rm27_sim/.agents/docs/ASM2_ESTIMATION_MODES.md) | Estimator/recovery capabilities and source restrictions | `1c22f14ce8fddac9c2067809b64e01521ad3da5302695fb9b08ddaf059fbb19a` |

Concurrent-source note (2026-11-13): the table retains the initially inspected
snapshot hashes. During the final check, two Non-RL sources changed outside this
documentation task. Relevant control and forecast definitions were reread; the
contract gaps described here still apply. Separately observed current hashes:

| Source | SHA-256 observed when definitions were reread |
|---|---|
| S06 | `34c1681358e0bfdb656ef08de4753ee6daeddca5758616a1abee7d4db5abc496` |
| S12 | `67815ef37d00cecdf8cba9fcd1dfda95b264fbb9ba6bea214a34f4a7e6a081ba` |

The other 23 registered source hashes matched at the documentation check. S12
continued changing afterward; later bytes are outside the inspected snapshot.
This register records inspection evidence; it is not a frozen source lock,
reproducible bundle or conformance certification. No concurrent source edits were
overwritten. Reinspect current consumer code before implementing these drafts.

Architecture and requirements additionally read: root and relevant component
AGENTS, root SPEC/DESIGN/ROADMAP, Non-RL SPEC/DESIGN/ROADMAP/README,
Sim SPEC/DESIGN/ROADMAP/README, local RL SPEC/DESIGN/ROADMAP/README and local
shared-aircraft registry/catalog. Detailed native definitions are referenced by
source ID in the individual contracts.

## Compatibility and versioning

All six drafts remain `0.1.0`; the set is identified by the Phase 1.1 freeze.
Record subsequent changes explicitly against this baseline. A deployed consumer
records the accepted contract versions, its own source/version/config, field
capability map, frame/time mapping, model/geometry/scorer/allocator profile and
exact adapter/policy/normalizer hashes. A contract version does not establish physical
model equivalence or reproducibility of an uncommitted worktree.

After adoption, incompatible frame/unit/order/field meaning or required-behavior
changes require a major contract version; additive optional capabilities use a
minor version and editorial corrections use a patch. Changes during this draft
must still be explicit. Parameter/geometry/scorer tuning always changes the
relevant content/profile identity even if message shape stays compatible.
No current consumer is declared conformant by documentation alone. Unknown
versions, required fields or identity mappings block the affected physical use.
Historical runs retain their original profile and verdict.

## Open questions and RL validation dependencies

- Native Z-up/FLU rotation matrices, planner-specific target frames and proposed
  NED/FRD Hamilton quaternions need explicit adapters; covariance/epoch fields
  are incomplete in legacy data. The proposed target-axis order is frozen as
  a draft definition; mapping and equivalence of actual producer registrations
  remain unverified.
- Native XML full inertia differs from diagonal-inertia controller prediction;
  provisional thrust/lag/yaw parameters, rotor speed, drag and voltage need evidence.
  The effective PX4 allocator, firmware/configuration and thrust mapping are
  unverified against the simulation profile.
- Full 11,874-hull provenance is unconfirmed. Current static asset audit finds
  3,522 collision-enabled geoms and 15 patches; pad identity is not model identity.
- ASM2 `ContactScorer` and legacy proxy `ContactEvaluator` are distinct; existing
  `joint_success` is not the proposed four-Q result. One Sim-owned shared
  evaluation service is the intended next-phase direction. Full forbidden-event
  coverage and moving-target point-speed parity are unverified; official judging
  equivalence is not established.
- Sampled clearance, local footprint reserves and short offline forecasts do not
  certify a continuous swept path, sensor-driven recovery or a handover region.
- **PENDING RL CONSUMER VALIDATION:** actual observation dimensions/feature ordering,
  history, masks and normalizer; action transform, units/frame, policy rate,
  saturation/hold/inner loop and allocator/rotor order; actuator states available
  to policy; physical model and complete geometry; checkpoint and reward-versus-
  evaluator separation; contact-motion/event coverage and initialization,
  feedback and trigger lineage in evaluation/feasibility. Local historical source
  is evidence only for itself.

## Phase 2 approval gate

Phase 1.1 ends with this reviewed draft freeze and documentation checks.
Phase 2 requires explicit approval before schema/IDL, effective-profile
manifests, adapters or executable conformance tests are implemented. Actual
consumers must supply implementation evidence for frame/clock registration,
allocation and actuator semantics, geometry identity, evaluator coverage and
feedback/trigger provenance before compatibility can be claimed. No remote
inspection, runtime or model changes, simulation campaigns or Git publication
were part of this review.
