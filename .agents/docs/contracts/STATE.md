# State Contract

Contract ID: `rm27.state` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
The shared [index](README.md) defines evidence labels, identity, clock and compatibility rules.

## Purpose and scope

Define the physical meaning of state exchanged between Sim, Non-RL and RL,
without imposing a policy tensor, estimator algorithm or transport. All fields
below are proposed semantic fields, not a description of an installed schema.

| Record | Meaning and permitted use |
|---|---|
| `GroundTruthState` | Sim integration state and observations of the simulated plant; available to sensor generation and independent offline scoring. A truth-input baseline must declare that source; it cannot claim estimate-only autonomy. |
| `EstimatedState` | State actually available to a consumer at its decision time, including uncertainty, validity and measurement lineage. Prediction does not refresh the last measurement. |
| `PolicyObservation` | Versioned consumer-specific selection, history and normalization of permitted inputs. It must declare whether it uses truth, estimates or asymmetric training information. It is not the complete physical state. |

A numeric pose alone cannot certify a resumable simulation state. Exact replay or
branching additionally requires the simulator's full-state snapshot, model and
state-layout identity, integration time, actuator state, held commands and relevant
controller/estimator history. MuJoCo warm-start and solver state must follow the
chosen snapshot profile. Mark missing history rather than silently resetting it.

## Frames and transformations

The proposed boundary uses SI and right-handed **local NED** world frame `W`
(x north/reference-forward, y east/reference-right, z down) and **FRD** body frame
`B` (forward, right, down). A local yaw reference is not evidence of true north.
The world origin, yaw alignment, body origin and sensor extrinsics are explicit
registrations. For the current ASM2 model, the native body origin is COM-centered;
other models must declare their transform to COM.

`R_AB` maps coordinates from B into A; `p_WB` is the position of B's origin in W.
`q_WB` is Hamilton `[w,x,y,z]`, the unit quaternion for `R_WB`. Composition is
`q_AC = q_AB ⊗ q_BC`, and `v_A = R_AB v_B`. `q` and `-q` represent the same
orientation. Consumers compare rotations, not raw quaternion signs. Accept only
finite, nonzero, unit quaternions within a declared numerical tolerance; do not
normalize arbitrary invalid data into a valid estimate.

Native Sim/Non-RL currently use a Z-up world convention and FLU body axes. Their
world registration must explicitly identify NWU or its local equivalent. For an
aligned NWU registration and FLU body, let `D = diag(1,-1,-1)`:

- `p_NED = D p_NWU` (plus the registered origin translation).
- `R_NED_FRD = D R_NWU_FLU Dᵀ`, `omega_FRD = D omega_FLU`.
- ENU to NED uses `A = [[0,1,0],[1,0,0],[0,0,-1]]`;
  `R_NED_FRD = A R_ENU_FLU Dᵀ` when the source body is FLU.

These are proper rotations, not quaternion component swaps. Acceleration, normals,
forces, covariance and lever arms must use the same registered transformations.
Unknown axis alignment blocks physical command use.

## Field-level definition

R = required identity or capability field; C = required when that capability is
claimed; U = explicitly nullable with an unavailability reason. Vectors have three
entries unless stated otherwise.

| Field | Requirement | Units, frame and semantics |
|---|---|---|
| `position_W_m` | C | m; B origin in W; not an unregistered camera origin |
| `velocity_W_m_s` | C | m/s; derivative of the same origin in W |
| `q_WB_wxyz` | C | unit quaternion as above; orientation availability independent of position |
| `body_rates_B_rad_s` | C | rad/s; actual/estimated angular velocity, FRD; not Euler angle derivatives or rate setpoints |
| `rotor_ids` | C | ordered stable IDs matching the Dynamics model and motor allocation |
| `rotor_speed_rad_s` | U | actual/estimated nonnegative angular-speed magnitude per rotor; spin signs and axes are separate Dynamics fields; RPM requires explicit conversion |
| `rotor_thrust_N` | U | actual/estimated nonnegative force magnitude along each model-defined thrust axis; source may be simulator activation or observer, not requested force |
| `motor_command_applied` | U | last applied per-rotor input, with input kind, units, order, command ID and application time; not the actuator response |
| `target_id`, `target_pose_WT` | C | target identity and registered pose; orientation uses the same quaternion convention |
| `target_velocity_W_m_s`, `target_omega_T_rad_s` | U | target-origin velocity and target-frame angular velocity; zero only for an explicitly static registered target |
| `relative_position_T_m` | C | `R_WTᵀ (p_WB - p_WT)`; vehicle relative to target, never the negation without a new field name |
| `relative_velocity_T_m_s` | C | derivative of the above: `R_WTᵀ(v_WB-v_WT) - omega_T × relative_position_T`; static target is a declared special case |
| `surface_normal_W` | C | unit normal pointing from target surface into free approach space; no translation in its transform |
| `state_time_s` | R | physical epoch represented by this state, in the declared clock |
| `measurement_times_s` | C | per-channel acquisition times; propagation to state time preserves these ages |
| `published_time_s`, `received_time_s` | C | producer completion and receiver-local receipt; latency claims require a verified clock map |
| `validity` | R | per-channel availability, initialized/aligned, finite, current, and reason; a single `valid=true` cannot imply all channels |
| `uncertainty` | U | covariance or declared bounds with units, frame, field order, error convention, source and time; unknown is null, not zero |
| `data_age_s` | C | receiver-computed age of each last informative measurement at decision time; supplied ages are diagnostic only |
| `reset_epoch`, `epoch_provenance` | R | estimator/world reset generation and whether actually recorded; historical unrecorded epochs remain null |
| `source`, `input_kind`, model/frame/clock identities | R | common envelope from the index; synthetic and truth lineage persists through adapters |

`relative_velocity_T_m_s` is a coordinate derivative for the body origin in the
rotating target frame. It is not the relative physical velocity of the two
contact points. [Evaluation](EVALUATION.md#contact-point-kinematics) derives the
latter from the existing pose, origin velocity and angular velocity, using the
registered COM offset from [Dynamics](DYNAMICS.md) and the contact point from
[Geometry](GEOMETRY.md). No mandatory contact-point State field is added.

Covariance uses small-angle rotation error, not four quaternion coordinates.
Each covariance declares its state ordering and left/right attitude perturbation.
For independent vector blocks rotated by A, use `P' = A P Aᵀ`; mixed blocks use
the full declared Jacobian, including lever-arm terms where applicable. Do not
assume independence or calibrated Gaussian confidence. Missing covariance only
blocks consumers that require it, not every offline diagnostic.

## Time, validity and reset

Clock domains and mappings are from the index. A prediction timestamp cannot
hide old measurements. Reject future or out-of-order state for a control use
according to a declared profile; retain rejected input for diagnostics. On reset,
invalidate plans, histories and commands tied to the old epoch. Re-registration
must establish frame/time validity before control resumes. A contact/impact epoch
and a localization reset epoch are distinct; neither is inferred from the other.

`PolicyObservation` additionally declares `observation_id/version`, named feature
order, dimensions, history stride, scaling/normalizer hash, masking, missing-data
policy, permitted sources, state epoch and source timestamps. Its exact RL tensor
and deployed policy metadata are **PENDING RL CONSUMER VALIDATION**.

## Ownership and compatibility

Main repository owns these meanings. Sim produces truth/sensor lineage and replay
snapshots. Non-RL owns its estimator and planner input selection. RL owns its
observation builder, normalization and privileged-training declarations. Neither
consumer may turn a missing sensor or actuator state into measured truth.
A changed frame, quaternion direction, origin, feature meaning or time meaning
is a breaking version change; a different observation layout requires its own
consumer version and compatibility record. Legacy structs remain unchanged.

## Required conformance tests — proposed, not executed

1. Nonidentity NED/ENU/NWU and FLU/FRD transformations, a 90° axis rotation,
   quaternion composition and sign equivalence; covariance and lever-arm checks.
2. Static and rotating target relative velocities against finite differences.
3. Distinct requested thrust, applied command, activation and measured rotor speed;
   absent RPM/covariance remain absent through round trips.
4. Delayed and predicted measurements retain source age; future/duplicate/reset
   data cannot rearm old plans; clocks with unknown mappings are rejected where required.
5. Deny truth access in estimate-only runs; verify replay restores the complete
   state and controller history without a reset at contact.
6. RL consumer supplies feature-order, units, masks and normalizer fixtures;
   do not approve compatibility from tensor shape alone.

## Current implementation and gaps

**IMPLEMENTED:** `VehicleEstimate` has p/v/rotation/body rate/time/source;
`TargetState` has position/normal/covariance/time/confidence ([S03](README.md#source-register)).
Non-RL `Observation` and `PendingPlan` implement prediction and some epoch/parent
checks (S06/S08). Sim state logging explicitly marks historical estimator reset
generation `NOT_RECORDED` (S09). Native motor activation is thrust, not rotor speed.

**REPORTED:** earlier interface and estimator experiments have their own gates;
none certifies this draft. **PROPOSED:** the complete boundary above and its tests.
**UNKNOWN:** actual rotor RPM, unrecorded uncertainty/epochs, all target orientation
capabilities beyond the registered static map, and remote RL observation lineage.
Local RL features inspected in S13 are historical examples, not a shared layout.
