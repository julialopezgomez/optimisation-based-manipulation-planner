# `iris_zo_cliquecover` (placeholder)

Reimplementation of Drake's IRIS-ZO and clique-cover algorithms with an NLP-sampling twist for drawing polytopes, to be integrated with `algorithms/nhr/nhr.py` into the full-arm pipeline.

Not started yet - this package is an empty placeholder so the eventual code has a home from day one, mirroring how `algorithms/nhr/nhr.py` is structured (flat module(s), importable via `algorithms/nhr/` on `sys.path`, no packaging ceremony).

**Existing precedent to build from**: `experiments/grasping_space.ipynb` is the closest existing usage of Drake's built-in IRIS-ZO/clique-cover APIs (`IrisZoOptions`, `IrisFromCliqueCoverOptions`, `IrisInConfigurationSpaceFromCliqueCover`) on the full Panda-arm scene - a markdown header there literally reads "Generate C-free for full franka arm with IRIS-ZO". No custom/partial reimplementation of these algorithms exists anywhere else in the repo (confirmed by search) - every other usage just calls Drake's shipped implementations directly.

Do this work on its own branch (e.g. `algo/iris-zo-clique-cover`), cross-checked against `experiments/grasping_space.ipynb` and `data/generation/full_arm_c_free.ipynb`'s Drake-builtin results before merging.
