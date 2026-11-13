# RM27 Non-RL — SPEC

## Research contract

Implement state-conditioned autonomous handover and continuous controlled
underside contact, separation and flight recovery without RL. Handover must
not require one fixed coordinate or fixed activation time. Guidance may brake,
reposition, approach, adopt a feasible terminal plan, or reject an attempt.

The frozen native simulator scorer remains authoritative: qualified first
contact, no forbidden contact, physical separation and at least one second of
stable flight are required for complete success. Front/rear contact regions
need not load simultaneously. Report their geometry and contact sequence
without replacing the original scoring criteria.

## State and timing

Runtime uses acquired/delivered synthetic sensor observations and a registered
static target map. This is not visual/VIO or real sensor evidence. Ground truth
is permitted only for generating measurements and offline evaluation. Preserve
validity, impact epoch, measurement age and state time; unavailable uncertainty
is `null`, not calibrated confidence.

New references must preserve the existing tracker's command continuity and
carry parent reference, estimator generation and acquisition/validity/apply
timestamps. Reject stale, superseded or late results. Retain 5 ms control,
200 ms query period, 30 ms solver deadline and 60 ms application lead. A sampled
geometry check does not establish continuous swept-volume clearance.

Coordinated braking must preserve terminal position/velocity/acceleration and
segment derivative continuity while checking height, thrust-aligned attitude,
motor response and original contact constraints together. A monotonic sampled
reference does not prove monotonic actual velocity. Report closed-loop rebound.
Recovery trials must distinguish desired rotation, actual rate response and
geometric clearance. Predictive or estimated separation timing is never labeled
as detected physical contact. Optional actuator compensation retains original
motor limits and records saturation; it grants no change to physical qualification.

## Validation boundaries

Keep successful engineering witnesses and all failed evidence. Archived exposed
cases are diagnostic/regression data, not fresh held-out evidence. Freeze code
before independent evaluation. A rejected candidate, old-reference fallback,
simulation stop or clean no-contact exit is not successful contact/recovery.
Safe exit requires actual contact-free braking and stable hold under native
physics. No automatic hardware, RL, MPC, publication or budget expansion.

Local candidate feasibility on recorded states is distinct from runtime splice
adoption and physical task success. Perturbations around archived estimates are
model sensitivity checks, not a reachable-set or robustness certificate. Report
observed estimator errors against planning assumptions without treating nominal
stress values as calibrated bounds. Failure classification starts at the first
physical qualification violation, not only the later runtime termination code.

Explicitly authorized diagnostic continuation may observe flight after exceeding
the configured speed envelope. The original speed limit remains a qualification
criterion and first violation permanently invalidates complete success, even if
stable flight is subsequently observed. Report diagnostic recovery separately;
retain all other stopping conditions and a finite diagnostic speed/time budget.
The configured 0.95 m/s is an experiment envelope, not a physical speed limit or
a demonstrated boundary of recoverability.
