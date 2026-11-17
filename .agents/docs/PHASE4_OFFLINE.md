# Phase 4 — Local offline adapter subset

The six contracts and schemas remain `0.1.0-draft / PROPOSED`. The parent
repository owns this offline adapter; it does not participate in runtime.

| Entry point | What it checks | Limits |
| --- | --- | --- |
| `verify_local_asm2(repo_root)` | Verifies D1's ASM2 `1.0.0` catalog, local model/geometry XML and plant metadata against their pinned SHA-256 digests. | Local 3,522 geometry only. The documented 3,521 RL proxy and historical 11,874 geometry are distinct and not locally verified. The XML digest does not certify compiled geometry or transitive assets. |
| `verify_sha256(path, expected_digest)` | Checks explicit configuration or recording bytes against an independently obtained digest. | It does not identify a scene or evaluator from a filename. |
| `adapt_sim_truth(sample, context, ...)` | Converts the ego fields of a Sim `VehicleEstimate`-shaped truth record from native/FLU to NED/FRD. | Preserves truth ancestry; no simulated execution or control admission. |
| `adapt_non_rl_sample(sample, context, ...)` | Converts one supported, non-propagated `StateSample` VIO channel; preserves acquisition and represented time and rejects invalid source, time or epoch. | No sensor verification, estimator invocation, cross-channel fusion, covariance inference or inferred actuator values. |
| `adapt_physical_ctbr(values, ...)` | Converts synthetic target-free `[T_N, p_rad_s, q_rad_s, r_rad_s]` records to the existing FRD CTBR shape; checks source IDs, current epoch, application/lease expiry, and declared lease ownership. | Returns a record only. No allocator runs, no authority is acquired, and no normalized action is mapped. |

For State, pass an independently known `NativeContext`, `FrameRegistration`
and `ClockRegistration`. The frame must specify a reviewed static world-to-NED
rotation and translation and a common COM body origin. The clock mapping is
`destination_s = scale × source_s + offset_s`, with an explicit destination
uncertainty bound. The adapter checks both registrations at the sample's
timestamp, applies the supplied current epoch and uses the receiver's time and
maximum age. The receiver must supply the current registrations and separately
review their physical provenance. This helper does not discover world origin,
estimate clock calibration, or track stream ordering between calls.

Compare the registered and observed identity manifests independently. Include
an independently verified `config_hash` in both manifests when a configuration
is part of the claimed identity. Unsupported or unregistered input raises
`Blocked`; unavailable channels in otherwise valid State records stay `UNKNOWN`.
Synthetic ancestry remains explicit. The offline tests validate produced
records with the existing Phase 2 schema registry and semantic checks.

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/contracts -v
```

The checks use NumPy, SciPy and `pyulog==1.2.4` in addition to `jsonschema`.
`python3 -m rm27_exchange.recorded_attitude` reads the pinned `1113_z.ulg`
and emits the registered `vehicle_attitude` window as State JSON Lines:

```sh
python3 -m rm27_exchange.recorded_attitude \
  data/flight_logs/data/2026-11-13_quad-132p6g_sysid/1113_z.ulg \
  --receiver-boot-us 79160000 --max-age-us 60000
```

The test replay clock is local to the recorded boot. The frame comes from
PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`'s
`msg/versioned/VehicleAttitude.msg`; `EKF2::PublishAttitude` establishes the
valid-attitude publication and reset semantics. The nine samples are attitude
only. No COM position, UTC mapping or vehicle-wide reset epoch is established.
The historical hover projection still tests rejection of unregistered data.

`assess_coverage` checks required windows using explicit, scoped evidence
spans. A missing claim, uncovered interval or unrelated span blocks acceptance.
Point observations do not establish continuous coverage. The helper does not
score a rollout or certify the producer that supplied a span.

Reference trajectory integration, normalized RL action mapping, contact
Evaluation and contact Feasibility integration are outside this subset. Existing
Phase 2 evaluation/feasibility fixture checks remain available, but do not
establish D2/D4 source adapter conformance, continuous clearance, recovery or
`Q_complete=PASS`. Cross-machine and runtime compatibility and full mission
evaluation have not been tested.

G0 subsequently adds a bounded offline Navigation → Handover Snapshot and
ReferenceTrajectory lease handshake using these same contracts and registrations.
See [G0 delivery](G0_HANDOVER.md) for runnable entry points, actual recorded-data
coverage and the still-BLOCKED complete scenario dataset. This does not expand
the historical Phase 4 subset's physical or runtime validation claims above.
