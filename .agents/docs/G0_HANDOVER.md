# G0 — Shared Handover Integration

G0 local offline delivery, 2026-11-18 (Asia/Shanghai). Contracts remain
`0.1.0-draft / PROPOSED`; D1–D4 remain ACCEPTED. No frozen contract, schema,
source lock, component core or gitlink was changed.

`rm27_exchange/handover.py` wraps the existing State, Control and optional
TargetGeometry records. Strict JSON serialization preserves values, timestamps,
sequence, epoch, uncertainty, lineage, identities and available ordered actuator
history. It does not fill missing channels. Structural readability and handover
admission are separate: native evidence without a registered Shared State is
readable with `state=null`, while admission is BLOCKED.

Receiver-owned `HandoverRegistry` supplies the identity, frame/clock mappings,
validity intervals, current epoch, freshness policy and any target registration.
World origins and clocks are never inferred. The existing Phase 4 registration
classes are reused; Snapshot does not implement another coordinate adapter.
Ground truth ancestry is rejected for runtime handover even after recording.
Prediction preserves informative acquisition times. `SYNTHETIC_TEST` is a
separate profile; its thresholds and identities do not qualify a real aircraft.
`RECORDED_SIM_ESTIMATED` additionally admits only receiver-registered recorded
simulation estimates. It binds the source run/artifact, estimator and sensor
generator identities, sensor-only runtime input artifacts and each exact Snapshot
digest. Truth may appear only as an explicitly registered sensor-generation
ancestor; direct truth state and runtime promotion are still rejected.
Unknown actuator or uncertainty channels remain unknown; passing this interface
gate does not establish an exact physical branching state or estimator accuracy.

`HandoverProtocol` implements REQUEST, ACCEPT/ADJUST/ABORT and ACK/TIMEOUT.
ADJUST requires a validated replacement reference. ACK must echo the same
boundary and accepted reference; a different boundary requires a new request.
Only quaternion sign may differ. No plant propagation or reinitialization occurs.
The predecessor owns control until ACK's effective time. Expiry removes ownership;
timeout does not invent hover or another fallback. Epoch reset revokes both
pending and active ownership and requires new registrations. Commands cannot
reuse an accepted reference ID with different motion, timing or target content.
This bounded protocol supports the existing ReferenceTrajectory layer only;
it does not implement a CTBR policy, allocator or flight controller.

Scenario Set v1 is in `configs/handover_scenarios_v1.json`. Three consecutive
moving WAYPOINT records (source lines 1176–1178, 23.504–23.544 s) come from
`fixed-waypoints-half-height-001/flight/frames.jsonl`. The small distributable
projection keeps PX4 telemetry, original message/receipt times, normalized
controls and separate MuJoCo truth. Raw trace SHA-256:
`e3d8666efab977c3525cebd21f6cc62686498a3f9f473175b71e9fb3648d712d`.
Manifest/fixture hashes and exact projected values were checked against local
originals. All rows share one source family and `dev` split; no independent
Train/Held-out distribution or fixed handover position is asserted.

The recording's world/COM registration, clock mapping, full estimator epoch,
actuator activation/profile history and equivalence to canonical ASM2 are
unestablished. Consequently its diagnostic Snapshots retain native evidence,
and **Dataset Readiness remains BLOCKED**. The synthetic fixture exercises the
fully populated format and protocol, including nonzero actuator values/history;
it is not navigation reachability or physical feasibility evidence. A separate
test archives nine genuine ULog attitude records through the same Snapshot
format; missing p/v/rates/model registration remain missing and block admission.

The scenario admission closure adds **three ADMITTED recorded simulation
Snapshots** from `asm2_handover/moving_handover_02`, source lines 165–167 at
0.820/0.825/0.830 s. This existing 2.6 s navigation recording has no contact;
the estimator initializes from synthetic IMU/external velocity history without
receiving truth pose. The original `heading_at_transfer=FAIL` (3.47°) is retained.
Admission establishes data availability under a named test profile, not
calibrated estimation accuracy or navigation-task success.

`handover_recording.py` projects only recorded estimator channels through the
existing State adapter. Scene +X is explicitly chosen as local N, +Y as −E and
+Z as −D, preserving the scene origin and COM; this is a configured simulated
local frame, not surveyed North/East. Clock mapping is identity on the source's
nearest-nanosecond representation. Original floating-point channel times,
`trusted=false`, missing covariance and raw command history are retained in a
versioned evidence extension. Hidden motor activation remains diagnostic
GROUND_TRUTH, never an EstimatedState actuator measurement. Original command
leases/expiry were not recorded, so no qualified AppliedActuatorCommand is
fabricated. Protocol tests use explicitly synthetic, non-actuating authority
messages anchored to the recorded estimate.

The three rows share one source family and `dev` split; moving_handover_01/02
share a recorded trajectory prefix. **Dataset Readiness remains BLOCKED** for
insufficient independent admitted families and held-out coverage. A passing
Snapshot does not establish a handover distribution or physical feasibility.
Raw traces, sensor packets, summary, configuration, scene provenance and exact
original estimator/alignment source bytes are retained locally under
`.agents/runs/g0_handover/navigation/moving_handover_02/`. Some historical runtime
sources differ from the current checkout; recorded-data replay does not claim
that rerunning today's controller/estimator reproduces the original experiment.

`handover_consumers.py` validates first, then uses the actual local environments
to construct legacy RL `rm27_contact_physics.model.VehicleEstimate` and Non-RL
`rm27_non_rl.asm2_estimation_integrity.StateSample`. It inverses registered world
and clock mappings and explicitly converts FRD to the consumers' FLU convention.
The native view is bound to the unchanged Snapshot by its digest; the original
Snapshot remains authoritative. Source distinguishes fixture reconstruction from
a registered recorded simulation estimate, never the native truth default or a
fabricated VIO qualification. Native type construction is offline-only:
policy observation is BLOCKED without target,
previous action and horizon; Non-RL sensor trust is BLOCKED without approvals.
No estimator, supervisor, policy, environment or plant is executed.

Run from the repository root:

```sh
PYTHONDONTWRITEBYTECODE=1 /home/shiuhou/venvs/mujoco/bin/python -m unittest discover -s tests/contracts -q
python3 -m rm27_exchange.handover_scenarios --snapshots --verify-originals
python3 -m rm27_exchange.handover_recording
```

The first command includes existing Phase 2/4 tests and G0 negative, replay and
cross-process checks: **101 tests PASS** in the root MuJoCo interpreter during
scenario closure. The original delivery's 80 tests passed both `/usr/bin/python3`
(jsonschema 4.10.3) and that interpreter (4.26.0); the enlarged suite was not rerun
in system Python. Relevant localization
and historical exchange regression: **12 PASS, 1 pre-existing FAIL**. The failure
is the original preflight rejection, before any full-chain worker is launched.
Native-constructor tests use the existing local RL `.venv`
and root MuJoCo environment, without modifying either environment or PYTHONPATH.
RL's environment lacks `jsonschema`; admission runs in the Exchange environment
before the isolated native constructor. The constructor's own output is marked
`UNADMITTED_NATIVE_CONSTRUCTOR_ONLY` and cannot independently claim conformance.
Both old jsonschema 4.10.3 and modern 4.26.0 use the same frozen, offline bundle.

For the JSONL worker, launch:

```sh
python3 -m rm27_exchange.handover --fixture tests/contracts/fixtures/handover_synthetic.json --consumer non_rl
```

Send one object per stdin line. Operations: `inspect` (lossless archive plus
admission verdict), `read` (admitted state), `native-read` (actual native type
projection), `request`, `decide`, `ack`, `tick`, `command`, `reset`.
`--consumer rl` reads the identical Snapshot; internal observations need not
match. Diagnostic startup identity goes to stderr; stdout is JSONL only.
The request/registry fixture is a test configuration, not a deployment profile.
See `tests/contracts/test_handover_protocol.py` for executable message sequences.

| Scope | Result | Evidence / limitation |
| --- | --- | --- |
| Schema / semantic conformance | PASS | Existing bundle plus strict Snapshot and rejection checks. |
| Local offline exchange / handshake | PASS | Same Snapshot in two reader processes; ACK continuity, stale/epoch/authority/parent/expiry and immutable-reference checks. |
| Recorded simulation Snapshot admission | PASS | Three source-backed rows ADMITTED under RECORDED_SIM_ESTIMATED; hashes, registrations and source roles verified. |
| Native consumer type construction | PASS | Three recorded estimates in actual RL/Non-RL types and environments; no policy/controller execution. |
| Recorded-data fidelity / replay | PASS | Source hashes, native row projection, ULog preservation and family isolation. |
| Dataset readiness | BLOCKED | One admitted simulation family, no independent held-out distribution; legacy telemetry remains inadmissible. |
| Historical full native worker regression | BLOCKED | Existing source lock rejects changed RL `__main__.py` before worker startup; test reports FAIL. Lock retained. |
| Cross-machine / live runtime compatibility | NOT_RUN | No remote operations or live control. |
| Physical feasibility / full mission | NOT_RUN | Closure replays existing non-contact records; no new rollout, physics, training or recovery validation. |

Detailed command output and source/type identities are local in
`.agents/runs/g0_handover/`. `source_lock_mismatches.json` lists four existing
source differences; the first rejected file is RL `src/rm27_contact_rl/__main__.py`
(expected `205f7f97…`, observed `0ad09b5a…`). None was relocked. Frozen documents/schema/source-lock files and indexed component entries match
the task-start snapshots. Closure results and evidence are in
`.agents/runs/g0_handover/REPORT.md` and `RESULTS.json`. The preceding table is
the historical G0 closure scope. The separately authorized N1-A extension below
does not revise those archived results or start R9.

## N1-A bounded live simulation integration

`rm27_exchange/non_rl_navigation.py` connects the existing Snapshot, protocol and
native adapter to FlightRuntime / GreenGuidance for non-contact development.
`LIVE_SIM_ESTIMATED` is a separate receiver-owned registration: it binds a named
run/family, source artifact, estimator, synthetic sensor generator, scene and
sensor-only runtime inputs. Its mandatory scope is
`NONCONTACT_DEVELOPMENT_ONLY`, with `trusted=false` and unknown covariance.
It does not make recorded admission runtime trust or change frozen schemas.

Runtime admission checks acquisition/delivery/state times, source, frame, clock,
epoch, validity, command-derived motor history, reference content, current
authority and expiry. The inverse adapter supplies the current estimate at
dispatch; the estimator and controller continue without reinitialization.
All issued actuator commands are retained. Motor activation is a one-pole model
estimate from a declared initial condition, not a measurement of `data.act`.
Truth is confined to sensor generation and offline evaluation.

`HandoverProtocol.cancel_pending` permits cancellation before an ACK becomes
effective; committed transfers cannot be rolled back. Successful structural
schema validation has a bounded exact-content cache. Dynamic admission,
freshness and ownership checks are never cached. Both measures preserve the
existing 30 ms whole-query and 60 ms application-lead contract.

The N1-A matched accept / explicit ABORT runs exercise actual authority routing
and native non-contact braking/hold. They do not qualify estimator-loss fallback,
independent source families, hardware timing, contact or recovery. Native
timeout/expired-plan exits remain untested. Reproduction is in the
[Non-RL README](../../components/rm27_non_rl/README.md#n1-a-non-contact-handover),
and all current verdicts/budget/next actions remain solely in the
[Non-RL activity plan §0](../../components/rm27_non_rl/.agents/plans/plan_asm2_autonomous_handover_01.md).

## Source publication and local inputs

The preceding G0 publication (9ea8051) includes its required Phase 1–4 contract documents, schema
bundle, offline helper modules and regression fixtures, which had not previously
been committed. Unrelated working-tree changes, component pins, native models,
raw recordings, virtual environments and local plans/runs are excluded.
`data/shared_aircraft` is only a relative alias; its ASM2 payload is not supplied
by this root commit or the currently pinned MJCF commit. Provision the D1 catalog,
XML and plant metadata at that target and verify their declared hashes before
running model-dependent checks. No model or geometry equivalence is implied.

The original 80-test result belongs to the preceding provisioned local delivery.
The 101-test closure suite additionally requires the hash-pinned navigation archive
above. Closure changes have not been committed or pushed.
A fresh checkout additionally needs the pinned ULog, navigation JSONL/summary
and native consumer environments to repeat all 80 tests. Tests requiring these
inputs must not be reported PASS when inputs are unavailable. The recorded
projection and synthetic fixture are included; raw-data fidelity and full native
consumer validation remain conditional on the original inputs and environments.

With NumPy, SciPy and jsonschema installed, a checkout without those local
payloads can run the portable checks independently:

```sh
python3 -m unittest discover -s tests/contracts -p 'test_schema_validation.py' -q
python3 -m unittest discover -s tests/contracts -p 'test_semantics.py' -q
python3 -m unittest discover -s tests/contracts -p 'test_validation.py' -q
python3 -m unittest discover -s tests/contracts -p 'test_handover_protocol.py' -q
python3 -m rm27_exchange.handover_scenarios --snapshots
```

The last command replays the legacy projection and registered recorded-simulation
Snapshots and reports Dataset Readiness
BLOCKED; `--verify-originals` additionally requires the original recordings.
Publication does not upgrade any local-input, runtime or mission qualification.

Publication verification in an isolated checkout of `origin/astra/research`: **33 portable tests PASS**, plus all three included diagnostic Snapshots round-trip without the original raw recordings or model payloads. Full local-input/native-environment reproduction was not claimed for that checkout.
