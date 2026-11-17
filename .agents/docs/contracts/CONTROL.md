# Control Contract

Contract ID: `rm27.control` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
[State](STATE.md) defines frames/time; [Dynamics](DYNAMICS.md) defines physical
actuation. This document specifies boundary semantics, not a common controller.

## Purpose and separate command layers

Non-RL and RL may use different planners, tracking laws, policy rates and inner
controllers. Comparison requires compatible **physical command semantics** and a
named actuator/inner-loop profile, not identical internal algorithms.

| Layer | Input / output meaning | Owner |
|---|---|---|
| `ReferenceTrajectory` | desired p/v/a, attitude and angular motion over an interval; no direct actuator effect without a named tracker | generating consumer, e.g. Non-RL; RL may bypass this layer |
| `DirectCTBR` | collective physical thrust N and three body-rate setpoints rad/s; consumed by a named rate/actuator interface | `rm27_non_rl` controller adapter or `rm27_RL` action adapter, according to the producer |
| `MotorAllocation` | maps collective/torque demand into ordered rotor requests or declared motor inputs, subject to allocation limits; actuator response follows separately | `rm27_sim` for simulation; `rm27_drone_px4` for actual flight through its PX4 configuration |

A reference cannot simultaneously exercise control authority with a direct CTBR
producer. A policy vector does not become a motor command without an explicit
adapter/profile. The existing FLU `RateThrust` object remains unchanged.

## Field-level reference and command definitions

All records carry common identity, source sequence, frame/clock/reset epoch,
control owner and source state/target identities from the index.

| Field | Meaning and units |
|---|---|
| `reference_id`, `parent_id` | unique trajectory generation and reference being replaced; prevents stale asynchronous adoption |
| `knots[].time_s`, interpolation | named clock, continuous segment domain; polynomial order/boundary derivatives or interpolation rule declared |
| `position_ref_W_m` | desired body-origin position in W |
| `velocity_ref_W_m_s` | derivative in W, not body velocity |
| `acceleration_ref_W_m_s2` | inertial acceleration, excluding gravity; specific thrust is a different quantity |
| `jerk_ref_W_m_s3` | optional acceleration derivative, required if tracker feedforward uses it |
| `q_ref_WB_wxyz` | desired body orientation per State; optional when explicitly derived from acceleration/heading |
| `body_rate_ref_B_rad_s` | desired angular velocity in FRD, distinct from Euler derivatives; interpolation/body basis at each sample declared |
| `angular_acceleration_ref_B_rad_s2` | optional feedforward, with derivative convention |
| `collective_thrust_N` | nonnegative total rotor force magnitude, not throttle, body-z signed force or thrust-to-weight ratio |
| `body_rate_sp_B_rad_s` | three FRD angular-rate setpoints, independent of actual rates |
| `rotor_thrust_request_N[]` | ordered force targets before actuator response; IDs/order from Dynamics |
| `motor_command_requested[]` | requested hardware/model input with kind (`FORCE_TARGET_N`, normalized throttle, shaft speed, etc.), units and mapping identity |
| `motor_command_applied[]` | actually admitted/clipped/held input, command ID, application time and saturation flags |
| `generated_time_s`, `apply_time_s` | when computation completed and when command first applies, in a registered clock |
| `valid_from_s`, `expires_at_s` | allowed application interval; proposed convention is `[valid_from, expires_at)` |
| `source_state_time_s`, `measurement_age_s` | represented state time plus independently computed source age; prediction cannot hide old input |
| `sample_period_s`, `hold_policy` | declared update period and interpolation/zero-order hold, including late/missing sample behavior |
| `authority` | owner/lease ID, generation, allowed layer, phase, lease expiry and handoff state |
| `limits`, `saturation` | configured vector/norm limits, raw versus applied values, allocation infeasibility and hardware/profile identities |

The reference must explain which fields are authoritative when both acceleration
and attitude are supplied. Inconsistent demands are rejected or handled by an
explicit tracker policy; they are not averaged implicitly. Motor/thrust fields
in State distinguish applied input from actual or estimated actuator outputs;
none are interchangeable with a request.

## Physical signs and allocation

Boundary CTBR is `[T_N, p_FRD, q_FRD, r_FRD]`, where p/q/r here denote angular-rate
setpoints, not position/quaternion. Existing native FLU commands transform rates
with `diag(1,-1,-1)`; collective magnitude stays positive. Upward rotor force in
FRD is along the registered negative-z axis. For a force vector use the actual
axis transform, not merely the collective magnitude.

`rm27_sim` owns canonical rotor and actuator physics and the simulation
allocator. `rm27_drone_px4` owns integration and configuration of the actual
flight allocator in PX4. `rm27_RL` owns conversion from policy outputs to the
chosen command layer; `rm27_non_rl` owns the controller adapter to that layer.
A consumer's internal allocation estimate or proposed rotor requests do not
own the allocator or the actuator response of the selected execution path.

Record the effective allocator and inner-loop profile for each path, including
rotor order, thrust/torque mapping, limits and any inertia approximation used
in rate-to-torque conversion. Simulation and PX4 allocation can differ in
mapping, priorities, saturation and outputs. Neither matching CTBR values nor
a shared rotor geometry establishes equivalence. When an interface accepts
normalized thrust or throttle, use its declared mapping; `T/(4*T_max)` is not
a measured calibration. Log infeasible allocation, pre-clip requests, applied
bounds and rate limiting separately. Clipping does not establish feasibility.

Native `Plant.command` uses `apply_proxy`: rate error to torque, inverse mixer,
then per-rotor force clipping (S04). Non-RL `ApproachTracker` additionally computes
rate/feedforward and motor-lead behavior (S06). Identical CTBR numbers with a
different inner loop are not equivalent physical actuation. Preserve this
profile difference when comparing policies and conventional control.

## Timing, authority and failure behavior

Exactly one valid command owner per vehicle/layer is admitted at a time. Authority
registration includes run/model identity and state epoch. Handoff has an explicit
effective application time, acknowledged successor and generation; stale commands
from the previous owner are rejected. Reset invalidates old plans, command leases
and state-dependent holds. Do not use stale requested commands to infer actuator
state without an observer and its uncertainty.

Hold/interpolation may operate only within a valid command/lease interval.
At timeout, select a **named, prevalidated** fallback compatible with available
state channels and model. If none exists, return `BLOCKED/NO_VALID_CONTROL` to the
supervisor and invoke the run's declared termination mechanism. This draft does
not define a safe hover, motor-stop maneuver or physical recovery controller.
Record which fallback actually executed and whether it succeeded independently.

The current Non-RL profile uses 5 ms control updates (200 Hz), 200 ms planner
query period, 30 ms solver deadline and 60 ms application lead (S05/S06/S08).
These are local settings and budgets, not a general real-time guarantee or a
required RL policy frequency. Physical timestep/substeps and command phase
within each interval are recorded separately. Deadline measurement uses host
monotonic time; trajectory/application times use the physical clock.

## Versions and compatibility

A consumer compatibility record states command layer, frame, units, thrust
mapping, limits, rates, hold/expiry/lease rules, rotor ordering and effective
plant, allocator and inner-loop identities (including PX4 firmware/configuration
for flight). Any semantic change in these requires a new contract
major version or explicit incompatible profile. Tuning values require new
profile identity even when the data fields do not change. Unknown mapping or
clock/frame/epoch prevents physical command admission.

## Required conformance tests — proposed, not executed

1. NED/FRD and native-frame axis impulses through allocator, including yaw signs
   and motor-order permutation; compare actual response to requested input.
2. Reference derivative consistency, knot boundaries, attitude/feedforward
   conventions, zero-order hold and no interpolation across a reset.
3. Zero/max/infeasible allocation and saturation, including full-tensor versus
   diagonal approximation errors and motor lag after command steps.
4. Delayed, duplicate, future, expired, reset-epoch and wrong-model commands;
   conflicting owners, acknowledged handoff and timeout without an available fallback.
5. Non-RL reference replacement retains tracker state and command continuity;
   old parent/reference/epoch adoption is refused.
6. RL provides its actual action transform, rate, normalization and saturation
   fixtures. Compare matching physical inputs, not raw action vectors.

## Current implementation and known gaps

**IMPLEMENTED:** FLU force/rate commands and native motor force execution;
Non-RL planner identity/epoch/expiry checks and command-continuous reference
splicing have test code (S04/S06/S08/S17). Those tests were **not run here**.
Local RL env and stage1 files show historical CTBR boundaries, not remote policy
compatibility (S13/S14). **REPORTED:** earlier six-worker command/lease tests;
those are a separate synthetic interface snapshot.

**PROPOSED:** the shared command envelope, canonical FRD layer and cross-consumer
conformance. **UNKNOWN:** deployment thrust mapping, remote RL action semantics,
rate/hold/normalization, current policy motor allocation and validated fallback
coverage. They remain **PENDING RL CONSUMER VALIDATION**. No runtime adapters,
lease implementation or controller behavior changed in Phase 1.
