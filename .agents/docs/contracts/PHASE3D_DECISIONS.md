# RM27 Phase 3D — Design Decision Record

Date: **2026-11-14**

Contract set: **`0.1.0-draft / PROPOSED`**

Record status: **ACCEPTED** — the user explicitly approved D1–D4.

This record freezes the four design choices identified by
[Phase 3C](../../runs/phase3_cross_track_audit/REPORT.md) as the Phase 4 design
baseline. Approval covers D1–D4 only. It does not amend the six frozen contracts,
accept Contract V1.0.0, certify an existing consumer, or start Phase 4 work.
Runtime implementation and experimental qualification remain separate work.

Evidence precedence for these decisions is:

1. exact local artifact and source identity;
2. immutable run or evaluation manifest with hashes;
3. implementation and test evidence;
4. documentation-only claims, marked as such.

A shared model name, patch count or task label is never an equivalence proof.

## D1 — Canonical Model and Geometry

### Decision

**Owner:** `rm27_sim` owns the canonical native verification assets and their
model, dynamics and geometry manifests. The main repository owns the cross-track
identity registry and comparison policy.

For Phase 4 local native verification, adopt the locally reproducible **ASM2
shared-aircraft release `1.0.0`, MuJoCo proxy `0.1.0`** as the only canonical
baseline:

| Artifact | Required identity |
|---|---|
| Catalog | `data/shared_aircraft/aircraft.json` · SHA-256 `c818545af9cc3c96af7491366464428c40f5b305cddd7696ed19d5e54b4d0621` |
| Native MuJoCo model | `data/shared_aircraft/v1.0.0/models/asm2_isaac_proxy.xml` · SHA-256 `2356bc3f8372fe14002521a82d4a9533867c688f8ed3d13cf172c74e17905ae4` |
| Plant metadata | `data/shared_aircraft/v1.0.0/models/plant.json` · SHA-256 `f4a5c52e8c06de893d3dc99b5544ba4408f03eed812bee52b595ee52db860b08` |
| Registered geometry | 3,522 collision-enabled geoms and 15 designated contact patches, as identified by the catalog and compiled model |

“Canonical” here means the common Phase 4 engineering verification reference;
it does not mean highest-fidelity physical truth. Controller-side diagonal inertia,
reduced planning hulls and other approximations must carry their own profile
identity and may not inherit the native model identity.

The documented 4090 Phase 4 RL training proxy remains a **named RL training
proxy only**: `rm27_asm2_collision_proxy_v1`, 3,521 entries, proxy-manifest
SHA-256 `247f076685d84796980232073b9bf12f27b6ce065d82227d890fc02d2a11834b`,
flight-profile SHA-256
`20243853d293b524521f9273e34b269d0c668f64f208e599bcf606fd3e4b8d3e`,
and contact-contract SHA-256
`fbfd2d9499e70c36838d041573120b88f7a099feb90b2dea4642afcaa22b401b`.
This identity applies only to the documented `phase4-meaningful` baseline. It
does not authorize new training or establish equivalence to the local baseline.

The 4090 native evaluation result reporting 11,874 detailed colliders is retained
as **documented historical evaluation evidence**. Because no locally reproducible
artifact hash or local reconstruction was established in Phase 3, it is not the
Phase 4 canonical local baseline. Local Ver2/full-field RL assets, earlier 150 g
models and other prior aircraft remain **HISTORICAL / LEGACY**.

Comparison policy:

- Direct aggregation or pass-rate comparison requires identical model, dynamics
  profile, geometry, target/scene, evaluator profile, initial-condition protocol
  and contract identities.
- Different identities may be compared only as a paired **cross-model sensitivity
  study**. Results stay separate and must name both sides.
- Geometry or profile equivalence requires a predeclared correspondence,
  tolerances, coverage and independent validation. Matching aircraft names,
  masses or 15 patch identities is insufficient.
- Results from 3,521, 3,522 and 11,874 geometry representations may not be pooled
  or substituted under this baseline.

### Evidence

- [Phase 3C final Dynamics and Geometry matrix](../../runs/phase3_cross_track_audit/REPORT.md), section 14.
- [Shared aircraft catalog](../../../data/shared_aircraft/aircraft.json) and
  [plant metadata](../../../data/shared_aircraft/v1.0.0/models/plant.json).
- [Dynamics Contract](DYNAMICS.md) and [Geometry Contract](GEOMETRY.md).
- Phase 3B run identities and proxy/native geometry findings in the same audit
  report, sections 4–6.

### Rationale

This baseline is the only candidate with exact local files, hashes and a usable
Sim/Non-RL integration path. Selecting it permits deterministic Phase 4 adapter
and conformance work without asserting that unavailable remote geometry is
reproducible or equivalent.

### Scope / Limitations

The baseline contains provisional or assumed actuator parameters, and some
consumer controllers use diagonal inertia. It does not validate continuous
swept clearance, real-aircraft dynamics, the 11,874-collider model, real PX4
allocation or RK3576 performance. The 4090 proxy and historical models remain
valid evidence only for their original manifests.

### Phase 4 Gate

Before any Phase 4 result is accepted, its manifest must bind all model,
dynamics, geometry, scene and evaluator identities above. A consumer using a
different identity must be rejected from the canonical result set or explicitly
run as a cross-model study. No “same ASM2” fallback is allowed.

### Status

**ACCEPTED.** Equivalence to the 3,521 proxy
or 11,874-collider model remains **BLOCKED** pending separate validation.

## D2 — Canonical Engineering Evaluation

### Decision

**Owner:** `rm27_sim` owns the authoritative **Engineering Evaluator** and the
trace-to-result transformation. The main repository owns contract-version
compatibility; RL and Non-RL may produce local metrics but do not define final
engineering success.

The canonical result interface is the Phase 2 draft
`EpisodeEvaluationResult` in `contracts/v0.1.0/schemas/evaluation.schema.json`,
with the semantics in the frozen [Evaluation Contract](EVALUATION.md). The
result must preserve:

`Q_complete = Q_contact ∧ Q_clearance ∧ Q_separation ∧ Q_stable`

Each Q carries its own `PASS`, `FAIL`, `NOT_RUN` or `BLOCKED` verdict, evidence
coverage, event references and evaluator identity. Aggregation is:

1. `PASS` only when all four required Q terms are `PASS` and their required
   time/geometry coverage is complete;
2. any required `FAIL` makes `Q_complete=FAIL`;
3. otherwise a required `BLOCKED` dominates `NOT_RUN`;
4. otherwise any unexecuted required term makes `Q_complete=NOT_RUN`.

`forbidden_events` remains a persistent, independently observable event list.
Earlier or simultaneous forbidden contact prevents qualifying `Q_contact`; any
forbidden event in the declared clearance window fails `Q_clearance`; a later
event does not rewrite the historical first-contact classification. No
`Q_forbidden` is added.

`rm27_contact_physics.evaluation.ContactEvaluator` is the strongest current
implementation candidate because it records physical first contact, contact-point
velocity, environment/ground/undesignated contact, separation, stable hold,
recovery, impulse and actuator saturation. It is **not declared conformant yet**.
`rm27_sim.simulation.asm2.ContactScorer`, local historical `Stage1Evaluator`,
Non-RL `joint_success`, 4090 first-contact qualification and RL reward/termination
remain scoped metrics.

Historical metrics must retain their original names and evaluator identities.
They may populate a canonical Q only when an explicit, lossless evidence mapping
and required coverage are demonstrated. They may never be relabelled wholesale
as `Q_complete`.

### Evidence

- [Phase 3C Evaluation row](../../runs/phase3_cross_track_audit/REPORT.md), section 14.
- `components/rm27_sim/packages/physics/src/rm27_contact_physics/evaluation.py`:
  `ContactEvaluator`.
- `components/rm27_sim/rm27_sim/simulation/asm2.py`: `ContactScorer`.
- `components/rm27_non_rl/rm27_non_rl/asm2_contact.py`: `qualified_contact`,
  `recovered` and `joint_success`.
- `components/rm27_RL/src/rm27_contact_rl/stage1_evaluation.py`:
  `recovery_evaluated=False`.
- [Phase 2 machine-readable draft](../../../contracts/v0.1.0/README.md).

### Rationale

A single owner and result vocabulary make cross-track results comparable while
preserving local training and diagnostic metrics. Keeping evidence coverage and
forbidden events explicit prevents a first-contact pass or reward threshold from
silently becoming full mission success.

### Scope / Limitations

This decision defines ownership and output semantics. It does not prove that the
current evaluator emits every draft field, provides continuous clearance
coverage or has cross-consumer parity. Official referee area/time judging is
outside this Engineering Evaluator unless separately registered.

### Phase 4 Gate

Phase 4 must first establish an evaluator profile identity, deterministic trace
fixture and semantic conformance tests for all four Q terms, incomplete coverage,
forbidden-event timing and contact-point kinematics. Runtime compatibility or
`Q_complete=PASS` may not be claimed until these tests pass against the canonical
D1 baseline.

### Status

**ACCEPTED.** Evaluator ownership and result semantics are frozen. Runtime
conformance remains **BLOCKED** until Phase 4 implementation and tests.

## D3 — Frame and Clock Registry

### Decision

**Owners:** the main repository owns registry format and compatibility rules;
each producer owns the evidence for its registrations; each receiver must enforce
them before control, feasibility admission or evaluation use.

The exchange boundary remains right-handed local **NED world (`W`) / FRD body
(`B`)**, SI units, COM body origin and Hamilton quaternion `[w,x,y,z]`. Native
Sim/Non-RL world frames and FLU body data remain valid internal representations,
but may cross the boundary only through a registered transform.

Every frame registration must identify:

- frame ID, parent ID, axes/handedness, origin and heading definition;
- body/sensor origin and extrinsics, including the transform to COM;
- rotation and translation direction, source identity and numerical precision;
- validity interval and registration/reset epoch.

The fixed FLU↔FRD axis change may be used only after confirming the same body
origin and a valid registration. Native world→NED additionally requires explicit
origin translation and yaw/alignment; a Z-up label alone is insufficient.

Every clock mapping must identify source and destination clock IDs, mapping
method and parameters, uncertainty/tolerance, validity interval and mapping
epoch. Acquisition time, represented state time, publication time and
receiver-local receipt time remain separate. A cross-clock latency or age may be
computed only through a valid mapping. Estimator/world reset increments
`reset_epoch`; plans, histories and commands tied to an earlier epoch are invalid.

Receiver policy:

- reject control, feasibility-admission and evaluation use when frame or clock
  registration is missing, expired, ambiguous or outside its validity interval;
- reject invalid, non-finite, stale, impermissibly future, out-of-order or
  superseded-epoch data according to the named consumer profile;
- preserve missing covariance, actuator state, origin, yaw or epoch as unknown;
  adapters must not fabricate them;
- diagnostic recording may retain rejected data with the rejection reason, but
  does not make it admissible.

Freshness limits are profile-specific test/config identities, not universal
vehicle constants.

### Evidence

- [State Contract](STATE.md), especially Frames and transformations plus Time,
  validity and reset.
- [Control Contract](CONTROL.md), command timing, authority and epoch rules.
- [Phase 3C State and Control rows](../../runs/phase3_cross_track_audit/REPORT.md),
  section 14.
- `components/rm27_non_rl/rm27_non_rl/asm2_estimation_integrity.py`:
  `StateSample` acquisition, delivery, state time, frame, source and epoch.
- `components/rm27_sim/rm27_sim/experiments/asm2_state_logging.py`: historical
  clock and unknown-epoch preservation helpers.
- Phase 2 `common.schema.json` frame, identity, lineage and time definitions.

### Rationale

The local sources expose enough timing and frame information to build explicit
mappings, but not enough to infer world origin, heading or cross-clock alignment.
A registry makes the conversion auditable and converts unknown mapping from a
silent control error into a deterministic rejection.

### Scope / Limitations

This decision does not choose an arena survey origin, derive a live clock
synchronization algorithm or set production freshness thresholds. It does not
prove any existing consumer currently enforces the policy. Native logs lacking
historical epoch evidence remain unknown rather than retroactively assigned.

### Phase 4 Gate

Before adapters process a fixture, the fixture must bind frame and clock registry
entries, transform direction, validity interval, source epoch and consumer
freshness profile. Conformance fixtures must include valid conversion plus
missing registration, stale/future data, reset, out-of-order and epoch-change
rejection cases.

### Status

**ACCEPTED.** Registry semantics are frozen. Concrete arena/world and
runtime clock registrations remain **BLOCKED** until measured/configured inputs
exist; offline Phase 4 fixtures can use explicit synthetic registrations.

## D4 — Feasibility Admission Policy

### Decision

**Owners:** `rm27_non_rl` owns candidate generation and screening requests;
`rm27_sim` owns native plant, geometry/contact/recovery validation and executed
Engineering Evaluation; the main repository owns admission profiles and
cross-stage provenance rules. RL candidates use the same policy when consumed.

Keep the four stages from the frozen [Feasibility Contract](FEASIBILITY.md):

1. cheap geometric screening;
2. dynamics feasibility;
3. detailed swept geometry;
4. native contact/recovery rollout.

The required-stage set must be named before execution. Use these admission
profiles:

| Profile | Required stages | Permitted claim |
|---|---|---|
| `SCREENING_ONLY` | Explicit subset, normally 1–2 | Candidate survived the named screens only. Report as **PARTIAL SCREENING**; never “physically feasible” or task success. |
| `NATIVE_VALIDATION` | 1–4 | Candidate passed the declared canonical native model, geometry coverage and rollout gates. This is simulated validation for that identity, not executed mission success or sensor autonomy. |
| `EXECUTED_OUTCOME` | Not a screening profile; consumes an executed trace | Use D2 `EpisodeEvaluationResult`. Only complete evidence with `Q_complete=PASS` supports Engineering task success. |

`PARTIAL` is a human-readable coverage label, not a new contract verdict. Encode
it with `ValidationLevel.scope=SCREENING_ONLY`, `highest_completed_stage`,
coverage and per-stage `PASS`／`FAIL`／`NOT_RUN`／`BLOCKED` values.

Verdict rules:

- A stage is `PASS` only when its named check ran and all declared gates and
  provenance requirements passed.
- A scoped feasibility result is `PASS` only when every predeclared required
  stage is `PASS`; its claim remains limited to that profile.
- Any required `FAIL` makes admission `FAIL`; otherwise required `BLOCKED`
  dominates `NOT_RUN`.
- An omitted stage is `NOT_RUN`. A missing model, transform, state lineage,
  coverage bound, evaluator identity or required evidence is `BLOCKED`.
- Stage 3 sampled poses remain `SAMPLED_ONLY`. They cannot satisfy continuous
  clearance for `NATIVE_VALIDATION` unless a certified conservative sweep/error
  bound covers the declared interval.
- Stage 4 must preserve full initial/terminal physical state and actuator history
  and identify feedback, contact trigger, plant, allocator, geometry and scorer.
  A horizon shorter than the configured stability hold cannot pass full recovery.

Evidence adoption:

- **Predicted:** may reject or shortlist candidates under the named model; never
  establishes executed success.
- **Ground truth:** may support simulation/model feasibility and independent
  scoring when declared; it cannot support an estimate-only autonomy claim.
- **Oracle-assisted:** remains optimistic even if initialized from an estimate;
  it may support model studies but not sensor-driven admission.
- **Sensor-driven:** requires decisions and triggers to use registered sensor/
  estimator channels with valid timing and reset provenance. Independent scoring
  may use truth only when scorer truth is isolated from controller/trigger inputs.
- Predicted, oracle-assisted and sensor-driven rates must be reported separately.

### Evidence

- [Phase 3C Feasibility row](../../runs/phase3_cross_track_audit/REPORT.md),
  section 14.
- [Feasibility Contract](FEASIBILITY.md), hierarchical screening, stage evidence
  provenance and verdict semantics.
- `components/rm27_non_rl/rm27_non_rl/asm2_recoverability.py`: estimated seed,
  command-derived activation, copied tracker, optimistic planning-model recovery,
  oracle contact, model hash and short post-contact horizon.
- `components/rm27_sim/rm27_sim/simulation/clearance.py`: sampled clearance.
- Phase 2 `FeasibilityResult` and `ValidationLevel` definitions.

### Rationale

The policy retains inexpensive candidate rejection while preventing partial or
optimistic prediction from being reported as native physical validation,
sensor-driven autonomy or completed task success. It also gives every result an
unambiguous next gate.

### Scope / Limitations

No current evidence provides certified continuous swept clearance, sensor-driven
recovery or a validated handover feasibility region. Native validation remains
model-specific. This policy does not prescribe a planner or policy architecture.

### Phase 4 Gate

Conformance work must define an admission profile before running, bind all stage
inputs and identities, preserve per-stage provenance, and test verdict precedence.
`NATIVE_VALIDATION` remains blocked wherever continuous clearance, full rollout
coverage or the canonical evaluator is unavailable. `EXECUTED_OUTCOME` requires
the D2 interface and cannot be synthesized from feasibility output.

### Status

**ACCEPTED.** The admission policy is frozen. Full native admission and
sensor-driven claims remain **BLOCKED** by missing evidence and implementation.

## Remaining Open Evidence Questions

These open evidence questions limit what can be claimed under the approved
decisions:

- whether the 3,521 proxy or 11,874-collider remote geometry can ever be validated
  as equivalent to the D1 local baseline;
- continuous swept-clearance evidence and tolerances;
- runtime consumption of frame/clock registrations, command leases and epoch
  invalidation;
- full four-Q canonical evaluator conformance;
- sensor-driven state, contact detection and recovery lineage;
- real PX4 allocation and RK3576 performance, which remain outside Phase 3D.

## Phase 4 Readiness

| Work item | Technical readiness | Required prerequisite / boundary |
|---|---|---|
| Exact model/profile/geometry identity manifest and mismatch rejection | **READY** | Use D1 hashes; reject or separately label every mismatch. |
| Offline State frame/clock envelope and rejection fixtures | **READY** | Use explicit synthetic registry entries under D3; do not infer arena origin or live clock mapping. |
| Offline FLU↔FRD CTBR/action mapping and identity fixtures | **READY** | Preserve allocator ownership and source limits; no real PX4 claim. |
| Historical event/metric to draft Evaluation record adapters | **READY WITH LIMITS** | May emit scoped or incomplete Q evidence only; must not produce false `Q_complete=PASS`. |
| Feasibility per-stage result/provenance envelope | **READY WITH LIMITS** | Start with `SCREENING_ONLY`; retain oracle/predicted/sensor labels and verdict precedence. |
| Canonical runtime four-Q evaluator conformance | **BLOCKED** | Needs D2 implementation, deterministic traces and complete coverage tests. |
| `NATIVE_VALIDATION` with continuous clearance | **BLOCKED** | Needs certified sweep/error bounds plus canonical evaluator coverage. |
| Proxy/detailed/local geometry equivalence or pooled statistics | **BLOCKED** | Needs predeclared correspondence and independent equivalence validation. |
| Sensor-driven autonomous feasibility or recovery claim | **BLOCKED** | Needs registered estimator/detector inputs and independent executed evidence. |
| RL training, PX4/RK3576 or hardware work | **OUT OF SCOPE** | Requires separate authorization and evidence; not a Phase 4 substitute. |

**Overall readiness:** The D1–D4 design approval gate is cleared. The bounded
offline work marked READY or READY WITH LIMITS is technically ready within the
listed boundaries. Phase 4 implementation has not started. Physics qualification,
continuous-clearance admission, geometry equivalence and sensor-driven claims
remain blocked independently of design approval.
