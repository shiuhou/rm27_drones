# Dynamics Contract

Contract ID: `rm27.dynamics` · Draft version: `0.1.0` · Status: **PROPOSED — PHASE 1.1 REVIEWED DRAFT**.
Use the [index](README.md) for evidence and version rules and [State](STATE.md)
for frames. This specification does not change the plant or controller.

## Purpose and ownership

Sim owns the canonical physical model, parameter set, actuator interpretation
and native integration. Canonical rotor and actuator physics describe how
applied inputs produce forces and moments. Motor allocation is a control
operation with separate Sim and flight owners; see [Control](CONTROL.md#physical-signs-and-allocation).
Non-RL predicts feasibility against a named Sim model; RL consumes that model
or declares an explicitly different surrogate. Neither may silently create
another current physical model. External MJCF/CAD repositories remain asset
dependencies administered through Sim, with original provenance.

## Canonical parameter record

Each parameter carries `value`, `units`, frame/origin if relevant, `provenance`,
`source_ref` and source hash, applicable hardware/configuration, identification
conditions, uncertainty or null, and verification status. Allowed provenance:
`MEASURED`, `CAD_DERIVED`, `SYSTEM_IDENTIFIED`, `ESTIMATED`, `ASSUMED`, `UNVERIFIED`.
These are not confidence scores. Record user-reported measurements as such until
the raw measurement is available. Composite models retain per-parameter provenance.

| Field | Definition and constraints |
|---|---|
| `model_id`, version/hash | full vehicle, parameter, actuator and solver profile identities; not merely aircraft name |
| `mass_kg` | positive total mass of declared configuration, including payload/battery |
| `com_B_m` | COM relative to the named body/model origin, in m |
| `inertia_COM_B_kg_m2` | symmetric positive-definite full 3×3 inertia about COM, resolved in B; includes off-diagonal terms |
| `inertia_source_frame`, transform | CAD axes, length/mass scaling, COM shift, principal-axis orientation and conversion history |
| `rotors[].id/order` | explicit association between geometry, model state, command vector and mixer columns |
| `rotors[].position_B_m` | thrust application point relative to B origin, m; subtract COM to get the moment arm |
| `rotors[].thrust_axis_B` | unit force direction; body FRD downward axis does not imply positive thrust is downward |
| `rotors[].spin_sign`, `spin_view` | physical rotation about the declared positive shaft axis; right-hand positive; CW/CCW requires viewing direction |
| `rotors[].reaction_torque_model` | signed reaction moment on vehicle, Nm; sign stated independently of rotor visual spin |
| `motor_lag` | state variable, differential/update equation, time constants in s, initial activation, sample time and discretization |
| `thrust_model` | maps actuator state, shaft speed and voltage to thrust N; may be direct-force proxy or identified speed model |
| `torque_model` | reaction torque Nm and geometric `r × f`; no double-counting |
| `drag_model` | translational/angular drag, frame, coefficients with dimensional units and validity envelope; absence labelled as a model omission |
| `saturation` | per-rotor min/max force or speed/input bounds, slew/current limits if modelled, and clipping location |
| `voltage_model` | voltage V, sag/state dependence, calibration range, motor/thrust mapping and missing-data behavior |
| `gravity_W_m_s2` | vector, magnitude and frame; NED positive down |
| `integration_profile` | simulator/version, integrator, physical timestep, substeps, contacts and solver settings |

Transform full tensors as `I_A = R_AB I_B R_ABᵀ`; moving an inertia to another
origin requires the parallel-axis theorem. A frame change is not permission to
discard products of inertia or overwrite the mass configuration.

For positive scalar thrust `f_i` and unit direction `a_i`:
`F_B = Σ a_i f_i`, `tau_B = Σ ((r_i-com_B) × a_i f_i + tau_reaction_i)`.
In FRD an upward rotor commonly has `a_i = [0,0,-1]`; its identity must come from
the model, not this example. Native FLU signs convert through State's rotation.
The physical free-flight equations are `m a_W = m g_W + R_WB F_B + F_external_W`
and `I d(omega_B)/dt = tau_B - omega_B × (I omega_B)` about COM. Contacts add
external wrenches; a free-flight feasibility pass does not predict their response.

## Inspected ASM2 profile — evidence, not final hardware parameters

Values below come from local metadata S02 and XML S01, with hashes in the index.
They are the current registered **ASM2 v1.0.0 / proxy 0.1.0**, not a finalized
system identification or the proposed NED/FRD wire representation.

| Parameter | Native FLU value | Provenance / limitation |
|---|---|---|
| Mass | `0.142 kg` | `MEASURED` user-reported, according to metadata; raw weighing record not inspected |
| COM | `[0,0,0] m` | `CAD_DERIVED` COM-centered assembly registration |
| Full inertia | matrix below | `CAD_DERIVED`; native XML stores equivalent principal values and inertial quaternion |
| Rotor positions | `(0.0275,0.041,0.00144473)`, `(0.0275,-0.041,0.00144473)`, `(-0.0275,-0.041,0.00144473)`, `(-0.0275,0.041,0.00144473)` m, rounded display | `CAD_DERIVED`; exact values stay in hashed metadata; indices 0–3 |
| Thrust axes | four `[0,0,1]` | CAD alignment with upright thrust direction `ASSUMED` |
| Reaction torque / thrust | `[+0.018,-0.018,+0.018,-0.018] m` | `ASSUMED` provisional Ver2 magnitude/signs; not measured shaft rotation |
| Rotor cap | `1.103625 N` each | `ASSUMED` provisional Ver2 value; no voltage-dependent ASM2 calibration |
| Lag | `0.04 s`, native `filterexact` | `ASSUMED` first-order force activation, not measured rotor-speed dynamics |
| Rate proxy gain | `17 s^-1` | `ASSUMED` proxy/controller parameter, not firmware PID qualification |
| Timestep | contact `0.0001 s`; metadata free-flight `0.001 s` | integration settings; actual scene/config must record which is used |
| Drag / voltage / shaft speed | no identified model established | `UNVERIFIED`; fixed thrust cap does not establish battery behavior |

The FLU inertia matrix (kg·m²) is:

```text
[ 1.1150257838327968e-4,  1.0350512712770064e-7, -4.8322420247145470e-6 ]
[ 1.0350512712770064e-7,  1.0455706507428649e-4,  1.1561894898317493e-7 ]
[-4.8322420247145470e-6,  1.1561894898317493e-7,  1.8709338833666170e-4 ]
```

Current XML motor inputs and `data.act` are **rotor force targets and force
activation in N**; neither is normalized ESC throttle or angular velocity.
Native filter activation uses the configured integration semantics (`actearly`
included). Analytic first-order predictions must document their discretization
and timing; their equality to solver updates is not assumed.

## Timing, validity and compatibility

Parameters are valid for a configuration/version interval; dynamic quantities
such as voltage carry State's clock and measurement age. Unknown voltage mapping
cannot be replaced by a normalized-throttle guess. Model overrides (mass scale,
lag scale or payload) produce a distinct effective model hash and applicability.
Log applied limits and saturation separately from pre-clipped requests.

A model hash must cover metadata, actuator equations, XML/included assets,
solver settings and effective overrides. A changed motor order, axis, tensor,
lag equation or units is a breaking semantic change; updated measured values
still require a new parameter/model identity and compatibility assessment.
Historical Ver2 experiments retain their original inputs.

## Required conformance tests — proposed, not executed

1. Reconstruct XML principal inertia into the registered full tensor, verify
   symmetry/positive definiteness, COM origin and SI conversion.
2. Single-motor impulse tests verify axis, lever-arm and reaction signs in both
   native and boundary frames; reject swapped motor order.
3. Compare gravity/hover, free rotation and first-order step response with
   analytic references under declared tolerances and timesteps.
4. Exercise cap/clipping, zero/high commands, initialization, battery extremes
   if a voltage model exists, and missing mapping rejection otherwise.
5. Compare the same physical command sequence in Non-RL and RL consumers against
   Sim's plant, including actual actuator history; quantify surrogate differences.

## Current implementation and unresolved mismatches

**IMPLEMENTED:** native `Plant`, `RateThrust` proxy and metadata/XML linkage
(S01–S05). **TESTED in this phase:** static XML/metadata inspection and source
hashes only. **REPORTED:** prior physical trials and model audits; not rerun here.

Native `Plant` and Non-RL `ApproachTracker` take `diag(I)` for rate/torque prediction
(S04/S06), while the XML retains full inertia. Thus prediction and plant are not
full-tensor equivalent. The contract exposes this as a model approximation, not
a repaired implementation. Actual spin direction, drag, voltage dependency,
thrust calibration and remote RL model remain **UNKNOWN / PENDING RL CONSUMER
VALIDATION**. No Ver2-derived value is promoted to a measured ASM2 value.
