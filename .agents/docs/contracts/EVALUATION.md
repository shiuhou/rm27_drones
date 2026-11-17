# Evaluation Contract

Contract ID: `rm27.evaluation` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
The [index](README.md) defines status aggregation, evidence labels and identity.
[Geometry](GEOMETRY.md) defines collision membership; [State](STATE.md) defines
state frames and clocks; [Dynamics](DYNAMICS.md) defines the COM offset.

## Purpose, ownership and independence

Sim owns one authoritative engineering contact/evaluation service with named,
versioned profiles. Non-RL and RL provide trajectories, commands, state lineage
and experiment identity; neither can independently redefine final success.
Policy reward, planner feasibility score, native contact count and evaluator
result are distinct outputs. Reward functions cannot certify task completion.

This is an engineering task contract, **not official competition judging**.
No equivalence has been established between a native contact wrench/impulse,
a discretized collision mesh and the referee's area/time-dependent criterion.
Official thresholds, spatial sensing and event segmentation are outside this
profile; do not substitute `force × timestep` for an official damage decision.

## Completion expression and stage meanings

Preserve:

```text
Q_complete = Q_contact AND Q_clearance AND Q_separation AND Q_stable
```

Each `Q` has a four-state verdict and an evidence record. `Q_complete=PASS` only
when all four are PASS under **one** run/profile/model and complete coverage.
Any FAIL makes completion FAIL; otherwise BLOCKED dominates NOT_RUN, which
dominates PASS. A missing event or segment cannot be replaced by an optimistic
boolean. For a completed trial, failure to meet a required stage within its
horizon is FAIL. A phase never executed is NOT_RUN; an unavailable required
model/input/evaluator is BLOCKED with a named dependency.

| Stage / term | Required meaning and evidence |
|---|---|
| First contact | Earliest physical collision event satisfying the declared active-contact predicate and interval resolution; record all simultaneous relevant pairs. Not first proximity or first later acceptable hit. |
| `Q_contact` | The physical first contact is by registered patches on registered armor and satisfies the profile's geometry, surface, attitude and point-motion gates. Any earlier or simultaneous forbidden event prevents qualification. Record event interval, raw checks and reasons. |
| Forbidden contact | Wrong aircraft part, unregistered/wrong target surface, surrounding obstacle/ground or excluded collision according to profile; retain a persistent event history, not only the last contact. |
| `Q_clearance` | Executed path over the profile's declared episode window has no disallowed collisions and satisfies any additional specified clearance margin. Authorized patch/armor contact is the explicit exception. Continuous margins require coverage evidence; sampled planner checks alone do not pass this term. |
| `Q_separation` | After contact, the declared separation metric exceeds its threshold and the no-contact dwell condition is satisfied, with times and geometry definition. Ending penetration or a planner's release command is insufficient. |
| Recovery | Transition from the post-contact state to available state/control capabilities and stable flight; report start/event detection, channel validity, mode changes, separation and terminal time. A truth-started recovery has different provenance from detected/sensor-driven recovery. |
| `Q_stable` | Following separation, declared attitude, rate, speed, height and valid-state conditions remain satisfied continuously for the configured hold, meeting the recovery deadline. A last-frame snapshot cannot pass it. |
| Mission continuation | Post-completion state/control availability, target reacquisition and next task readiness or actual continued execution, with its own time window and verdict. Not implied by `Q_complete`. |

Use the profile's forbidden-pair/region rules and permitted contact window to
classify events; retain all `forbidden_events` independently of the Q verdicts.
A forbidden event inside the clearance window fails `Q_clearance`, including
one after an otherwise qualifying first contact. Such a later event does not
rewrite the first-contact verdict. An allowed pair can fail a qualification
check without being a forbidden event unless the profile explicitly disallows
that contact. Record window coverage: an empty event list with missing intervals
is not evidence of absence. There is no separate forbidden-contact verdict term
in `Q_complete`.

Physical geometry clearance, state-estimation health and continuation eligibility
must be separately visible even if a historical profile combines them internally.
A safe no-contact exit is useful evidence with its own verdict; it is **not**
contact success. Recovery seeded from a historical contact state is a branch
experiment, not a fresh end-to-end trial.

## Evaluation input and result fields

| Field | Definition / unit |
|---|---|
| `run_id`, `case_id`, `seed`, `trial_role` | immutable experiment, independent random start and development/held-out/replay role |
| `execution_provenance` | one of `PLANNER_GENERATED`, `FIXED_A_AUTONOMOUS`, `RL_AUTONOMOUS`; definitions below |
| `observation_provenance`, `event_source` | origin and ancestry of state, feedback and trigger inputs; distinguish the controller's contact trigger from the independent scorer's event source |
| `model/geometry/scene/profile/scorer` identities | exact versions/hashes, effective overrides, simulator/integrator/timestep, and effective controller/allocator identity for executed commands |
| `episode_start/end`, `window_coverage` | physical time and clock, continuous history coverage, missing intervals and invalid intervals |
| `first_contact` | `[interval_start, interval_end]`, pair IDs, points m, normals, force N, native impulse Ns if available, and checks |
| `contact_checks` | per-pair part/surface/region/plane/penetration/attitude/speed results; thresholds and reject reasons |
| `forbidden_events` | ordered forbidden events with interval, pair/region identity, reasons and coverage under the named profile; persistent across phase transitions |
| `secondary_contacts` | subsequent contact events with their own classifications and segmentation; may overlap `forbidden_events` by event reference, but does not replace it |
| `clearance` | metric, minimum m or null, measurement method, margin threshold, time span and limitations |
| `separation` | plane/pad or full-body distance metric, threshold m, no-contact dwell s, achieved time and interval |
| `stability` | attitude error/tilt convention rad, rate rad/s, speed m/s, height/loss m, hold s, deadline s and actual extrema |
| `recovery` | start basis, detection latency, control phase and state validity trace; no invented detection event |
| `Q_contact`, `Q_clearance`, `Q_separation`, `Q_stable`, `Q_complete` | PASS/FAIL/NOT_RUN/BLOCKED, reasons, coverage and evidence references |
| `continuation_status`, `safe_exit_status` | independently assessed outcomes; null/NOT_RUN if untested |
| `termination_reason`, `numerical_health` | physical/constraint/budget/timeout/numerical termination, separate from success |
| `artifacts` | trace, config, event and source hashes; replay identity and relation to original trial |

All timings use completed native solver intervals; wall-time deadlines remain
separate. Consumer units may be converted for display (e.g. degrees), but serialized
angle semantics are explicit and never confused with radian thresholds.

## Contact-point kinematics

Use a common registered world frame W and the profile's declared contact-motion
sample (for the inspected ASM2 profile, pre-interval motion). Record the point
and interval used. Let P be the aircraft point associated with the collision.
Use State's `position_W_m`, `velocity_W_m_s`, `q_WB_wxyz` and
`body_rates_B_rad_s`, and Dynamics' `com_B_m`. `p_WB`, `v_WB` and `omega_B`
below are the position, velocity and angular rate from that state:

```text
omega_W   = R_WB omega_B
c_W       = R_WB com_B_m
p_COM_W   = p_WB + c_W
v_COM_W   = v_WB + omega_W × c_W
v_P_W     = v_COM_W + omega_W × (p_P_W - p_COM_W)
```

The lever arm is from COM to P in W (m); all linear velocities are m/s and
angular velocities rad/s. For COM-centered ASM2, `v_WB` already is COM velocity.
If the source supplies a velocity about another origin, register and shift it
before use. Do not add the rotational term to an already computed point velocity.

For the other body's corresponding point Q, apply the same rigid-body relation
about its registered origin: `v_Q_W = v_WT + (R_WT omega_T) × (p_Q_W - p_WT)`
for the registered target, where `v_WT` and `omega_T` come from State's target
velocity and angular rate. Use the solver's declared point convention for both sides; if it supplies one shared world point then `p_Q_W = p_P_W`. Compute
`v_rel_W = v_P_W - v_Q_W`, `normal_speed = -dot(v_rel_W, n_W)` and
`tangent_speed = norm(v_rel_W + normal_speed n_W)`, with `n_W` the registered
unit surface normal into approach space. Positive normal speed means closing.
A registered static surface has zero point velocity. Unknown moving-surface
motion or missing registered inputs blocks a required speed check. The State
body-origin coordinate derivative in T is not a substitute for `v_rel_W`.

## Inspected native ASM2 engineering profile

S04 `ContactScorer.active` reads solver-active compressive contacts (`wrench[0] >
1e-8 N`), before endpoint recomputation, preserving the pre-interval motion.
This numerical predicate is **not referee alpha**. It rejects wrong aircraft
geom, wrong surface, outside region, plane error, surface normal, penetration,
attitude, normal speed and tangent speed. Approach speed is positive toward the
plate and includes angular motion at the contact point. Current target velocity
is zero by static-map assumption.

The S05 snapshot has normal speed `0–0.4 m/s`, tangent speed at most `0.25 m/s`,
attitude error at most `18°`, surface-normal error at most `20°`, and plane/
penetration allowances `0.005 m`. These values illustrate the **hashed local
profile only**. Thresholds are not frozen as universal contracts.

S15's current experiment loop records `qualified_contact`, `secondary_contact`,
`separated`, `recovered`, `joint_success` and failure. Separation uses the
COM-to-plane projection minus a pad-height approximation: threshold `0.08 m`
and no recent contact for `0.03 s`. It is not a minimum full-hull distance.
Stability uses tilt `10°`, body rate `0.5235987756 rad/s`, speed `0.3 m/s`, maximum
height loss `0.3 m`, a `1 s` hold, and a `2 s` deadline relative to recovery start.
The start of the stable interval must meet the deadline; hold completion occurs
later. Preserve this precise historical deadline meaning when replaying.

The loop qualifies first contact only in APPROACH and computes
`joint_success = qualified_contact and recovered`. It **does not publish** the
four independently evidenced Q records above. Its separation/stability booleans
and safety checks cannot simply be relabelled as the complete shared evaluator.
In particular, `secondary_contact` reflects the historical time-gap and selected
rejection checks; it does not establish complete `forbidden_events` or coverage
for the proposed `Q_clearance`. A missing log is not an empty one. Do not infer
a missing term from an aggregate success flag.

## Experiment provenance and statistics

| Value | Eligibility |
|---|---|
| `PLANNER_GENERATED` | planner/trajectory-selected witness, solver-generated feasibility rollout or offline oracle feedback experiment; not proof of autonomous fixed-input adoption |
| `FIXED_A_AUTONOMOUS` | fixed conventional algorithm with declared available inputs, state-dependent decisions and no human/per-trial selected success path; includes frozen configuration and allowed initialization |
| `RL_AUTONOMOUS` | fixed policy/checkpoint/normalizer/action adapter at evaluation time, explicit input eligibility and no updates using evaluation trials |

Record training method, inner controller, initial-state lineage, feedback and
contact-trigger source independently. Truth used only by the independent scorer
does not give the controller oracle access. Truth or oracle inputs used for
initialization, decisions or recovery triggering must be disclosed; the class
label alone cannot qualify sensor-driven autonomy. A hybrid run must declare
who selects decisions/actions and the attribution policy before testing. If classification is ambiguous, block comparative
statistics rather than inventing another group's denominator. Existing native
labels `fixed`/`online` do not prove either of the above autonomous categories.

Never combine rates across these categories, models, scorers, provenance or
trial roles. Report successes, failures, total attempts, skipped/blocked counts,
termination reasons and held-out exposure. Replays are not new independent
samples. Previously examined cases are development/regression evidence.

## Compatibility, tests and current status

A result is comparable only under the same scorer/profile/model identities and
input eligibility or an explicit registered comparison protocol. Changing event
segmentation, forbidden sets, thresholds, time windows, recovery start, reset or
termination rules requires a new evaluation profile identity. Do not rescore old
trials with new thresholds under their old result label.

Required next-phase tests:

1. Native first contact versus proximity; simultaneous pad/guard, wrong surface,
   earlier/simultaneous and later forbidden events, incomplete event coverage,
   grazing/edge/backface, non-COM origins, and rotating aircraft/moving target
   point velocities under a nonidentity frame transform.
2. Completed interval consumed once before `mj_forward`, independent of logging;
   duplicate timestamps/reset/missing intervals cannot accumulate extra evidence.
3. No contact, qualified contact without separation, early stable sample,
   interrupted hold, deadline boundary, secondary contact and real safe exit.
4. Full Q truth table, partial coverage, timeout/numerical failure and branch-only
   recovery; reward/score manipulation cannot change independent verdicts.
5. Same trace/model/profile through both consumers yields identical event/Q
   results. Confirm data-access isolation and separate statistical denominators.

**IMPLEMENTED:** S04 native collision predicate, S15 experiment orchestration and
S16 legacy proxy `ContactEvaluator`; S17 has related test code, not run here.
**REPORTED:** local historical summaries include failures as well as witnesses;
S19's two inspected cases fail and remain failures. **PROPOSED:** Sim-owned unified
result service and four-Q evidence. **UNKNOWN:** equivalence of the ASM2 and
legacy RL evaluators, remote RL scoring and official judging equivalence.
RL adoption is **PENDING RL CONSUMER VALIDATION**. Do not create a second scorer.
