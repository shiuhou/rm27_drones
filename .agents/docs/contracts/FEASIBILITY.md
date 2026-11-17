# Feasibility Contract

Contract ID: `rm27.feasibility` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
Use [State](STATE.md), [Dynamics](DYNAMICS.md), [Geometry](GEOMETRY.md),
[Control](CONTROL.md) and [Evaluation](EVALUATION.md) together under the [index](README.md).

## Purpose, workflow and ownership

Make candidate screening and its evidence explicit, without declaring a solved
handover region or requiring an online planner to run every expensive check.

```text
Initial Full State
    → Candidate Trajectory
    → Dynamics Feasibility
    → Geometry / Clearance
    → Terminal State
    → Post-contact Recovery Assessment
```

Non-RL owns candidate/reference generation and its planning approximations.
RL may propose or consume candidates but retains its own policy internals.
Sim owns canonical physical rollout, geometry queries, contact scoring and replay.
Main repository owns this result boundary. Validation code must not create a
second physical model or scorer inside a planner.

## Inputs and structured result

The complete request includes an immutable initial-state record, candidate,
model/geometry/scorer/profile identities and screening budget. Missing required
input blocks the dependent stage; independent checks may still complete.

| Field | Definition and required content |
|---|---|
| `request_id`, `candidate_id`, `parent_reference_id` | provenance of the proposed path and currently active parent |
| `initial_full_state` | p/v/q/body rates plus actual or estimated rotor activation/held inputs, represented time, uncertainty, source/reset epoch; Sim replay also needs full integration snapshot and layout hash |
| `state_completeness` | channel availability and assumed/unknown fields; a partial estimate is not relabelled complete truth |
| `candidate` | timed reference or physical command sequence per Control; generation algorithm/version, initial/terminal conditions, interpolation and valid interval |
| `dynamics_feasible` | verdict, constraints checked, integration/prediction model, margins/units, saturation and model-approximation limits |
| `geometry_feasible` | verdict, geometry/obstacle coverage, forbidden pair policy, sampled/swept/native method and tolerance/bound |
| `clearance_valid` | verdict and clearance metric/margin, authorized contact window, uncertainty/stress assumptions and temporal coverage |
| `contact_admissible` | predicted or native engineering contact verdict, source of evidence and expected first pair; prediction is never a measured contact event |
| `terminal_state` | full terminal p/v/q/rates/rotor states at specified time, target-relative quantities, uncertainty and predicted-versus-observed source; null when stage did not produce it |
| `recovery_assessed` | verdict of the declared recovery assessment, separate predicted recovery outcome, horizon and controller/event assumptions; NOT_RUN is normal for early screens |
| `validation_status` | overall PASS/FAIL/NOT_RUN/BLOCKED, explicitly limited to declared coverage/stage |
| `validation_level` | highest stage completed and its coverage; no stage-1 result may claim native execution |
| `versions` | contract, physics/parameter model, geometry/scene/subset, effective stage evaluator and scorer/profile, planner, controller/allocator, simulator/build/integrator and config hashes |
| `request_state_time_s`, `acquisition_time_s` | physical state epoch and per-channel informative measurements; preserve delayed input history |
| `request_wall_time`, `solve_wall_s`, `deadline` | monotonic host timing and clock identity; separate from simulation time |
| `apply_time_s`, `expires_at_s`, `epoch` | consumption window and state lineage; reject stale/superseded solutions |
| `assumptions`, `limitations`, `failure_reasons` | structured named entries, affected stage, units/ranges and uncertainty source |
| `evidence` | per-stage provenance and trace/config/input/output identities, queried intervals, terminal snapshot and recovery branch identifiers; use existing lineage and version records |

Units and frames follow State and Control; margins in m, force in N, time in s,
angles/rates in rad and rad/s. A one-line feasible boolean cannot encode these
required conditions. Unknown covariance, rotor speed or voltage remains null;
a force-state surrogate may be used only with its explicit model/assumption.

## Hierarchical screening

| Stage | Work and cost discipline | What a pass establishes |
|---|---|---|
| 1. Cheap geometric screening | finite target footprint, normal/approach side, coarse workspace and conservative support envelopes; reject obvious failures before expensive search | only declared local geometry conditions; no dynamic feasibility or complete obstacle clearance |
| 2. Dynamics feasibility | force/rate/torque demand, rotor allocation, limits, lag, initial actuator state, timing and model uncertainty against the named dynamics | bounded prediction under specified model/assumptions; not actual contact/recovery |
| 3. Detailed swept geometry | complete enabled aircraft hulls against scene over continuous path, or sampled poses explicitly marked with coverage bounds | continuous clearance only if a certified conservative sweep/error bound is supplied; otherwise SAMPLED_ONLY |
| 4. Native contact/recovery rollout | selected short list through Sim's native plant, contact scorer and recovery branch; preserve physical state through first contact and separation | simulated outcomes for that rollout, with its actual feedback and trigger sources; neither sensor autonomy nor a general handover region follows from native physics alone |

Stage-specific tests may run independently, but later passes cannot erase earlier
failures. Expensive full-geometry/native validation is reserved for shortlisted
candidates with recorded ranking and selection; do not allocate a native rollout
to every search candidate by default. Record budget consumption, deadline misses
and untested candidates. Screening failures are evidence, not task completions.

## Stage evidence provenance

For each stage, reference the actual state inputs and their `lineage` from
[State](STATE.md), the feedback and contact-event sources used by the method,
and the effective model and evaluator in `versions`. Record mixed lineage
where applicable: e.g., a prediction seeded from an estimate remains a prediction
and retains the estimate's ancestry and timestamp. Use the following mapping
within `evidence`; no additional state channels are required.

| Stage | State and lineage | Feedback and contact-event source | Effective model and evaluator |
|---|---|---|---|
| 1. Cheap geometry | initial state/target source and any derived support quantities | N/A when no feedback or event is used | coarse geometry or registered reduction and named screen/profile; native scorer N/A if unused |
| 2. Dynamics | initial state and actuator source or assumptions, and propagated state lineage | open loop or named estimate, truth or predicted feedback; event N/A for a contact-free check | dynamics, controller/allocator if used, and constraint evaluator/profile |
| 3. Detailed geometry | queried path source (reference, prediction or trace), state ancestry and scene pose source | feedback used to generate the path by reference; geometric query hits are not native contact events | complete geometry or declared subset, sweep/sample method and clearance evaluator/profile |
| 4. Native rollout | initialization source, assumptions, native simulated truth, and estimate/prediction lineage by channel when used | identify actual feedback and the recovery trigger (native oracle, detector, injected event or N/A); separately identify scorer events | native plant, geometry, controller/allocator and scorer/profile; sensor/estimator/detector configuration when used |

N/A means a source is unused, distinct from `UNKNOWN` for an unverified source
or `NOT_RUN` for a stage that did not execute. Missing required provenance blocks
the affected claim. A native forecast with perfect predicted-state feedback or
an oracle recovery trigger is optimistic prediction, even when initialized
from an estimate. A sensor-driven claim requires that decisions and triggers
use the declared sensor/estimator channels and their timing, validity and reset
semantics. Native truth may generate measurements and score outcomes independently;
it does not then count as controller feedback. Compare or aggregate evidence only
within declared source eligibility and coverage per [Evaluation](EVALUATION.md).

A contact-free predictor returns its pre-contact terminal state. Stage 4 must
also record the native post-contact state, event interval and actuator memory.
A recovery branch begins from that state without resetting pose, velocity,
motor activation, controller memory or contact history. An independently reset
recovery seed is a different experiment and cannot certify the candidate's
continuous task completion.

## Verdict semantics and admission

Every subresult uses four statuses:

- `PASS`: the named check actually ran and all its declared gates/coverage passed.
- `FAIL`: it ran and violated a gate; include failure location and measured margin.
- `NOT_RUN`: no execution for this check, including omitted expensive stages.
- `BLOCKED`: a required model, transform, input, time budget or evidence is unavailable.

Declare the required stage set before execution. Overall PASS is allowed only
when every required stage passes; a PASS limited to stages 1–2 must state that
limitation. Any required FAIL yields FAIL; otherwise required BLOCKED dominates
NOT_RUN. Expiry, changed parent reference, reset epoch, frame/model mismatch or
unavailable current state prevents adoption even if past screening passed.
A deadline interruption cannot return a successful uncompleted check.

`recovery_assessed=PASS` states that the **specified assessment** completed with
its required evidence; it must also carry `predicted_recovery_outcome`. A short
forecast revealing failure may be a completed assessment while the candidate
is FAIL. A horizon too short for the configured stability hold cannot claim
full `Q_stable`; report its early indicators and limited coverage instead.

## Current local implementation and gaps

**IMPLEMENTED:** Non-RL `ContactGeometry` and `plan_reference` perform local
geometric/force screening; `ApproachTracker.reference_demand` checks motor demand
with a diagonal-inertia/force-lag prediction (S06/S07). `SceneClearance` (S11)
queries sampled poses. `PendingPlan` (S08) rejects changed epoch/parent, expired
results and unavailable estimates; current Non-RL timing settings remain those
in Control. These are not continuous-time guarantees.

S12 `footprint_reserve` checks quadratic tangent extrema with assumed attitude/
position/velocity stresses and a first-order rotation bound; it explicitly
excludes contact/friction/full-body guarantees. S12 `native_forecast` has its own
plant, initialized from estimated state and command-derived motor activation,
with perfect predicted-state feedback and oracle contact as optimistic modelling
assumptions. It is explicitly **offline**, outside the 30 ms online solver gate.
Its default short post-contact horizon does not establish the one-second stable
flight requirement. It can reject a candidate without qualifying sensor recovery.

**REPORTED:** prior fixed or generated successful witnesses support particular
initial states, alongside retained failures. **PROPOSED:** structured results,
full-stage compatibility, continuous sweep and independent native recovery
validation. **UNKNOWN:** full-state/actuator uncertainty, worst-case execution,
continuous swept-geometry bounds and remote RL candidate/terminal interfaces
(**PENDING RL CONSUMER VALIDATION**).

The **Handover Feasibility Region** remains a research objective. It requires an
explicit state/uncertainty domain, model/profile, sampling protocol, prospective
independent validation and failure analysis. A few successful paths, projected
boxes or exposed development starts do not validate a region.

## Version rules and required conformance tests

Breaking changes to candidate/terminal meaning, required stage set, state time,
feasibility gates or verdict semantics require explicit version compatibility.
Changing any planner/model/geometry/scorer/config produces a new result identity;
cache entries are keyed by full initial state, lineage, uncertainty, candidate,
versions and assessment scope, not endpoint position alone.

Next-phase tests (not implemented/executed here):

1. Feasible endpoint but infeasible intervening rotor/torque/lag demand; initial
   motor history changes the outcome and prevents false cache reuse.
2. Same designated patches with a different shell, transient collision between
   samples and forbidden early contact; sampled-only cannot pass a sweep gate.
3. Prediction/native disagreement, full terminal-state continuity and no reset
   at contact; short forecasts cannot satisfy a longer recovery hold.
4. Missing covariance/clock/geometry, numerical faults, deadline/worker exit,
   stale parent/epoch and correct partial-result status aggregation.
5. Independent evaluator rejects rewards/optimizer flags as task success;
   verify that estimated seeds, predicted feedback, oracle triggers and scorer-only
   truth stay distinguishable through all stages, including replay.
6. Consumer compatibility and prospective held-out validation before any
   handover-region claim. No RL-specific dimension or architecture is prescribed.
