# Offline contract checks

From the repository root, with Python 3.12 and the locally available
`jsonschema==4.10.3` (tested with `attrs==23.2.0` and `pyrsistent==0.20.0`):

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests/contracts -v
```

The suite uses the frozen local Draft 2020-12 registry and rejects unregistered
references. The shared production validator also supports jsonschema 4.26.0.
Phase 4/G0 tests additionally use NumPy, SciPy, pyulog and existing local native
consumer environments; they do not install dependencies or execute a plant,
policy, estimator, training run or network operation. See the
[G0 delivery and commands](../../.agents/docs/G0_HANDOVER.md) for Snapshot,
recorded navigation replay, negative tests and native-constructor scope.

Each Phase 2 bundle fixture selects `schema` (a bundle file and `$defs` pointer) and wraps a
synthetic `record` with `synthetic: true`. Negative fixtures specify either
`expect.keyword` with `expect.path` (JSON instance path to the failing value), or
`expect.semantic` (a test rule identifier). When a rule compares a record to
other evidence, `context` supplies the comparison inputs. Semantic negatives
must still pass structural validation. Fixture wrappers are test metadata, not
part of the proposed records. Recorded fixtures under this test directory retain
their actual source lineage; G0's application envelope reuses these same schemas
without adding another frozen contract or claiming dataset readiness.

`test_schema_validation.py` checks the schema bundle, references and fixtures.
`test_semantics.py` exercises rules whose outcome depends on numeric or
cross-record relationships. These checks do not change or admit runtime records.
