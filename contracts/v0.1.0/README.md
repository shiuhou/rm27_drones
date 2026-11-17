# RM27 shared contracts — Phase 2 draft

Schema set **`0.1.0-draft` · PROPOSED**. These schemas encode the [Phase 1.1
reviewed documentation baseline](../../.agents/docs/contracts/README.md). The
baseline remains `0.1.0` and controls the meaning of the records. This release
chooses neither a runtime transport nor a production interface.

| Schema | Documentation | Scope |
|---|---|---|
| [common](schemas/common.schema.json) | [Index](../../.agents/docs/contracts/README.md) | Shared identities, frames, version, lineage, availability, coverage and verdicts |
| [state](schemas/state.schema.json) | [State](../../.agents/docs/contracts/STATE.md) | Truth, estimates, policy observations, target state and actuator state |
| [dynamics](schemas/dynamics.schema.json) | [Dynamics](../../.agents/docs/contracts/DYNAMICS.md) | Vehicle manifest, parameters, tensor, actuator models and provenance |
| [geometry](schemas/geometry.schema.json) | [Geometry](../../.agents/docs/contracts/GEOMETRY.md) | Collision asset manifest, geometry identities, transforms and classified regions |
| [control](schemas/control.schema.json) | [Control](../../.agents/docs/contracts/CONTROL.md) | References, CTBR, rotor requests, applied inputs and authority |
| [evaluation](schemas/evaluation.schema.json) | [Evaluation](../../.agents/docs/contracts/EVALUATION.md) | Contact events, clearance, separation, recovery, stability and independent episode verdict |
| [feasibility](schemas/feasibility.schema.json) | [Feasibility](../../.agents/docs/contracts/FEASIBILITY.md) | Candidate requests, per-stage evidence and scoped feasibility results |

## Reading the bundle

The schemas use JSON Schema Draft 2020-12. Load all seven files in a local
registry keyed by `$id`; the `rm27.example` URI namespace identifies documents
and is never a retrieval endpoint. Select the entry point by its `$defs`
fragment, as each fixture does. `x-rm27` gives the draft version, status and
repository-relative document. The package directory carries the documentation
version; envelope `contract_version` distinguishes this machine-readable draft.

Field suffixes and descriptions identify SI units and the applicable coordinate
basis. Local NED and FRD, with Hamilton `[w,x,y,z]`, are the State boundary
convention. Registered geometry and model frames are explicit. Neither a matching
frame label nor a well-formed transform proves clock or frame registration.
Identity objects carry `id`, `version` and `sha256`; structural validation checks
shape, not the contents of an external asset. Values not available remain absent
or explicitly null with their associated availability or provenance reason.

Objects are closed except for documented maps and explicitly versioned
`metadata.extensions`. Extensions cannot override core fields or silently relax
required checks. Keep the meaning of existing fields stable. After adoption,
changes to frames, units, order, required behavior or field semantics need a new
major version; optional additions use a minor version and editorial corrections
a patch. Model, geometry, evaluator and allocator profile content can change
independently and needs its own identity. Changes to this proposed draft are
recorded explicitly against the reviewed baseline.

## Validation boundaries

- **Schema validation** checks record types, required capabilities/identities,
  structural references, enums and explicit unavailable values. It performs no
  frame conversion, clock mapping, contact scoring or vehicle configuration.
- **Semantic validation** exercises relationships on synthetic fixtures with
  declared context. This covers constraints such as quaternion normalization,
  cross-record identity and rotor order, command timing and authority, and
  verdict and coverage consistency. Fixture checks are not a general semantic
  admission API: they do not establish external asset identity, interval coverage
  from raw traces, clock mappings, contact-point kinematics, exact replay, or
  physical truth from a declared value.
- **Runtime compatibility** is untested. Real allocator/rotor calibration,
  geometry equivalence, continuous sweep and scorer coverage, estimator quality,
  recovery, official judging and actual RL observations/actions are not established.

The [`examples`](examples/) directory contains synthetic fixtures only. A valid
fixture may intentionally describe a failed, blocked, or partial engineering
result; validity of a record is separate from its verdict. See the
[test guide](../../tests/contracts/README.md) for the offline command and negative
fixture format. Before any runtime integration, obtain explicit approval and
verify the actual consumers' frame/time registration, model/profile identities,
allowed input lineage and adapter behavior against this draft.
