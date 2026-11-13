# RM27 Non-RL — DESIGN

The `rm27_non_rl` package owns candidate search, trajectory/reference tracking,
guidance decisions, M3-based state estimation, estimator/recovery capability
supervision, recovery initialization and complete experiment orchestration.

The `rm27_sim` package owns scene/model composition, native Plant and contact
scoring (`simulation.asm2`), sensor generation, static-scene geometry queries
(`simulation.clearance`) and replay. The shared `rm27_contact_physics` package
remains byte-preserved and independent of parent/RL/Non-RL code. Historical
benchmark controllers in that frozen package are not moved or requalified.

The opt-in `forward_braking_s` candidate uses two joined seventh-order segments.
`braking_forward_waypoint` and `braking_vertical_waypoint` jointly adjust the
intermediate position, velocity, acceleration and jerk in forward and vertical
coordinates. Both endpoints and derivative continuity remain constrained;
the original contact/dynamic acceptance gates remain active. Offline coefficient
development is separate from runtime candidate evaluation.
The per-tick check permits at most 0.001 m/s numerical-scale speed rebound and
0.01 m/s² positive acceleration. This is a sampled check, not a continuous
monotonicity proof. Segment coefficients and the world forward axis are recorded
with the adopted reference, including vertical coefficients. Existing
configurations retain their original path.

The opt-in `leveling_clearance_m` gate postpones the existing smooth leveling
transition until valid estimated translation, outward normal velocity and
complete-body rotation clearance are available. It checks the current pose to
upright at a fixed estimated COM against the registered plane, subtracting an
angular interpolation bound between sampled orientations. This half-space test
is conservative around the finite plate edge but does not certify future moving
COM clearance, other scene geometry or uncalibrated estimation errors. An absent
translation estimate rejects the complete-upright gate rather than substituting truth.
The detector and authoritative scorer remain unchanged.
These are experimental candidates; tests alone do not qualify their flight use.

The opt-in `incremental_clearance_recovery` checks smaller rotations toward the
existing recovery reference instead of waiting for the entire upright arc.
It limits upward reference blending to the original 65 ms transition scale;
downward changes can occur when a fresh estimate invalidates a candidate.
If no candidate passes, the normal-release fallback is explicit and is not a
clearance certificate. Missing translation retains the existing explicitly
labeled attitude-only degraded branch; it cannot claim geometry-gated separation.
`separation_normal_accel_m_s2` changes requested normal
acceleration only; the default remains 1.2 and actuator/safety limits remain fixed.
`recovery_motor_lead` reuses the existing three-control-tick motor inverse and
command-history activation estimate for recovery CTBR; clipping is recorded.
`recovery_reference_feedforward` supplies the causal backward difference of
reference acceleration to the existing bounded angular feedforward. The initial
derivative is unavailable and set to zero explicitly; no sensor state is fabricated.
These options default off and retain the original recovery path for comparisons.

Scheduled separation uses the adopted reference's selected preview clock. A
predicted plane crossing and a 5 mm proximity window are different events;
neither is physical contact detection. Native contacts and IMU event timestamps
are reported independently. A rejected estimate-based crossing prototype remains
archived only; a vertex crossing outside the finite plate cannot trigger release.

The data path remains SensorRecorder → SensorEstimator → FlightRuntime →
guidance/planner/tracker/recovery commands → native plant. Offline scoring reads
truth independently. The estimator consumes IMU/external packets, never the
plant. Initial state/covariance limitations remain unchanged by relocation.

The optional `speed_diagnostic.hard_stop_m_s` affects experiment termination only.
`SpeedEnvelopeObservation` latches the first native-interval violation of the
unchanged `maximum_speed_m_s`. It prevents formal recovery/success after that
violation while separately reporting subsequent observed stabilization and the
actual diagnostic stop reason. Planner, sensor, controller, physical state and
other envelope checks are unchanged. Without this option the original speed
violation stops the experiment immediately.

Task config and code roots belong here; shared geometry, sensor config and
historical evidence use the simulator root explicitly. The legacy `native`
module in old experiment imports exposes shared primitives and lazily forwards
only task execution. Shared sensors import the native simulator directly.
Legacy estimator and ASM2 module aliases point to canonical objects to preserve
patching/class identity; they contain no second algorithm implementation.

The static-scene query compiles separate convex enclosures of all forbidden
aircraft mesh vertices and all legal patch vertices, preserving every enabled
static environment geometry. It checks 20 ms samples with native collision
queries. Enclosure is conservative at each sampled pose; temporal gaps remain
and are not a swept-volume certificate. Native mission physics and scoring
continue to use the original detailed aircraft geometry.

Reference blending optionally preserves one old control sample for the
tracker's 5 ms lookahead. Guidance projects current state into a target-relative
region and evaluates terminal candidates, rather than resetting toward a
single nominal handover. This development candidate remains unqualified until
the registered physical and held-out tests pass.

Replacement waits for the active blend to finish to limit tracking demand from
overlapping transitions. Short terminal references may have no remaining admissible blend
window; report this as a bounded-search limitation. A kinematically continuous
splice alone does not establish closed-loop rate margin or recoverable contact.
Offline native forecasts now use the same scheduled separation reference
function when that branch is configured. Their exact model feedback can disagree
with sensor-driven outcomes, so they do not grant online acceptance.

Offline geometry distances, sampled collision queries and the native scorer's
active force-bearing contacts are separate measurements. A positive saved-state
distance cannot exclude collision between samples; a tiny negative distance
alone does not replace the scorer's force/constraint activation criteria.
First-contact qualification and later recovery outcomes must be reported
independently, including failures that precede any recovery command.
