## Description

<!-- What does this PR change and why? -->

## Manual verification before merging

CI does not install Drake and cannot run any Drake-dependent code (see `TESTING.md`). Check the box(es) below for whatever this PR actually touches, after running the corresponding checks from `TESTING.md`:

- [ ] Install / environment
- [ ] `algorithms/nlp_sampling/`
- [ ] `ManipulationPlanner` (inline copies across notebooks)
- [ ] Model / scene loading (Drake plant construction)
- [ ] IRIS-ZO / clique-cover
- [ ] Data files (`data/cfree/*`)
- [ ] N/A - doesn't touch runtime behavior (docs-only, CI config, issue/PR tooling, etc.)
