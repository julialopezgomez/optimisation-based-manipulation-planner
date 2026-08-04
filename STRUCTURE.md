# Repository Structure

Where everything lives, what depends on what, and why. This complements `README.md` (installation/setup) - this file is about navigating the codebase.

## 1. Purpose & scope

This repo grew from a single 3/4-DOF toy manipulation planner into three parallel bodies of work: the original planner, an NHR/mRRT sampling library for grasp-pose sampling, and (upcoming) a reimplementation of IRIS-ZO/clique-cover with an NLP-sampling twist. This file exists so none of that gets harder to navigate as it grows. If you add a new notebook or data file, this is the file to update.

## 2. Root-level core pipeline

The root of the repo holds the "main" notebooks, in roughly pedagogical/execution order:

1. **`manipulation_planner_3dof.ipynb`** - toy WSG-gripper scene; defines an inline `ManipulationPlanner` class (grasp/placement polytopes -> MIP grasp-path -> GCS trajectory) and demos it. 3-DOF task.
2. **`manipulation_planner_4dof.ipynb`** - same pattern, 4-DOF task. Its `ManipulationPlanner` is a separate copy-pasted class, not shared with (1) - see "Known issues" below.
3. **`full_pipeline.ipynb`** - a separate, more granular, from-scratch pipeline (clique-cover C-free generation -> region certification -> MIQP path -> GCS trajectory) on the same toy scene. Does **not** import or reuse `ManipulationPlanner` from (1)/(2), despite the similar subject matter - it's self-contained.
4. **`full_arm_blocked_joints_3dof.ipynb`** - the real Panda arm + hand + bottle cap, with most joints locked via a locked URDF (3 active positions) - the "blocked joints" analogue of (1) on the real arm. Yet another inline `ManipulationPlanner` copy. Reads `data/cfree/cfree_drake_1_48.yaml`/`cfree_drake_1_49.yaml`/`path_drake_1_48.npy`.
5. **`full_arm_blocked_joints_4dof.ipynb`** - same pattern, 4 active positions - the analogue of (2). Reads+writes `data/cfree/cfree_4dof.yaml` and writes `data/cfree/connected_components_4dof.yaml`.
6. **`full_arm_nhr.ipynb`** - work in progress, the eventual master notebook: integrates the NHR/mRRT sampling library (`algorithms/nhr.py`) with the full (unblocked) Panda arm + Drake IRIS/GCS machinery to sample grasp poses and plan in the full configuration space. Reads `data/cfree/cfree_full_98coverage.yaml`, produced by `data/generation/full_arm_c_free.ipynb` (see below) - **this is a real pipeline dependency**, even though the generation notebook itself isn't at root.

Root also holds the two shared helper modules imported by nearly everything above (`ciris_plant_visualizer.py`, `visualization_utils.py`) and `my_sdfs/` (shared SDF/URDF scene assets).

## 3. `algorithms/` - shared, importable code

Flat modules, not a package, so every existing `import nhr` / `from nhr import (...)` keeps working unmodified regardless of where a notebook itself lives:

- **`nhr.py`** - the NLP-sampling library (Nonlinear Hit-and-Run + Manifold-RRT interior methods, two-phase restart sampler). See `nhr_code_walkthrough.md` in this folder for a full function-by-function guide.
- **`nhr_standalone_test.py`** - a runnable harness that validates `nhr.py` against the real Panda+cap grasp problem with a minimal Drake plant (no meshcat/IRIS/GCS overhead).
- **`nhr_code_walkthrough.md`**, **`Restarting Two-Phase NLP Sampler: Algorithm Reference.md`** - docs for the above.
- **`iris_zo_cliquecover/`** - empty placeholder package for the upcoming IRIS-ZO/clique-cover-with-NLP-sampling reimplementation. See its own `README.md` for the closest existing precedent to build from.

**Standard bootstrap cell** for a new notebook anywhere in the repo that needs `algorithms/` and/or the root helper modules on `sys.path`:

```python
import sys
from pathlib import Path

def _find_repo_root(start: Path, markers=(".git", "README.md")) -> Path:
    for candidate in (start, *start.parents):
        if any((candidate / m).exists() for m in markers):
            return candidate
    raise RuntimeError(f"Could not locate repo root above {start}")

REPO_ROOT = _find_repo_root(Path.cwd())
for _p in (REPO_ROOT, REPO_ROOT / "algorithms"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ciris_plant_visualizer import CIrisPlantVisualizer
```

This works regardless of how deep the notebook lives - no more editing a `parent = Path.cwd().parent` line every time a file moves. Skip this entirely for notebooks that sit exactly at repo root and only need the root helper modules (they resolve via a plain import with no path manipulation needed).

No proper Python package (`pyproject.toml` + editable install) yet - deliberately deferred until `iris_zo_cliquecover` is mature and there's a second real consumer of `algorithms/` beyond `nhr`. Revisit then.

## 4. `experiments/` - exploratory, prototype, or superseded notebooks

- **`grasping_space.ipynb`** - full 10-DOF unblocked Panda arm scene, similar scope to `data/generation/full_arm_c_free.ipynb` but ~2 weeks older; likely an earlier draft. **This is the closest existing precedent for the upcoming IRIS-ZO/clique-cover work** - it has a markdown header literally titled "Generate C-free for full franka arm with IRIS-ZO." Cross-referenced from `algorithms/iris_zo_cliquecover/README.md`.
- **`grasping_space_3d.ipynb`** - oldest file in this group, toy WSG-gripper (not the Panda arm) IRIS-ZO/clique-cover exploration. Superseded prototype.
- **`hit_and_run_grasping.ipynb`** - a 4-cell, no-markdown sanity check of `nhr.py`'s Hit-and-Run sampler on a toy 2D unit-circle constraint, no robot/Drake involved. Predates `algorithms/nhr_standalone_test.py`.

## 5. `data/` - c-free polytope data and the notebook that generates it

- **`data/generation/full_arm_c_free.ipynb`** - full 10-DOF unblocked Panda arm, generates C-free regions via IRIS-ZO/clique-cover, writes `data/cfree/cfree_full_98coverage.yaml`. Not at root tier (by design - it's a data-producing notebook, not a main demo), but its output is actively consumed by root's `full_arm_nhr.ipynb`.
- **`data/cfree/`** - all c-free polytope/path/edge data:
  - `cfree_full_98coverage.yaml` - written by `data/generation/full_arm_c_free.ipynb`, read by `full_arm_nhr.ipynb`.
  - `cfree_4dof.yaml`, `connected_components_4dof.yaml` - read+written by `full_arm_blocked_joints_4dof.ipynb`.
  - `cfree_drake_1_48.yaml`, `cfree_drake_1_49.yaml`, `path_drake_1_48.npy` - **frozen legacy inputs**: actively read by `full_arm_blocked_joints_3dof.ipynb` and `data/generation/full_arm_c_free.ipynb`, but no notebook currently has an active (uncommented) writer for them. Treat as pinned reference data, not reproducible from a fresh run without uncommenting the relevant save cells.
  - `cfree_full_90coverage.yaml`, `region_intersection_edges_1_48_0.json`, `region_intersection_edges_1_49_0.json` - **orphaned**: no reader or writer found anywhere in the repo. Kept as-is during the reorg rather than guessing whether they're safe to delete.

## 6. `artifacts/` - ephemeral, gitignored run outputs

- **`artifacts/joint_samples_plots/`** - per-run NHR sampling diagnostics (per-joint histogram PNGs + `info.json`), written by `nhr.save_joint_sample_artifacts(..., output_root=...)`. `full_arm_nhr.ipynb` points `output_root` at `artifacts/joint_samples_plots`; `algorithms/nhr_standalone_test.py` defaults to a self-anchored `algorithms/joint_samples_plots` instead (run from inside `algorithms/`) - these are two intentionally different locations for now; unifying them is a content change, not part of this reorg (see Known issues).

Matched by `**/joint_samples_plots/` in `.gitignore` - new runs are never tracked. (Some older runs predating that gitignore rule are still tracked in git history; that's fine, just legacy.)

## 7. `testing/` and `legacy/`

- **`testing/`** - dormant since a single squashed commit in April 2025 (untouched since). Early toy-gripper IRIS/clique-cover/QP explorations, with its own (now stale) copies of `ciris_plant_visualizer.py`/`my_sdfs/`. Left completely untouched by this reorg - it's self-contained and touching it buys nothing. If reviving any of it, check it against the current root `ciris_plant_visualizer.py`/`my_sdfs/` first, since the copies have drifted.
- **`legacy/`** - the prior Docker-based dev setup, preserved for reference per the main README.

## 8. Shared root helpers

`ciris_plant_visualizer.py` and `visualization_utils.py` stay at repo root (not moved into `algorithms/`) because nearly every notebook in the repo - old and new alike - already imports them directly, and they predate the `algorithms/` convention. Moving them would touch far more files for no structural benefit.

## 9. Import-mechanism note

See the bootstrap cell in §3. Use it for any new notebook that needs `algorithms/` on `sys.path`; skip it for anything living exactly at repo root that only needs the root helpers.

## 10. Git branching conventions

No git-flow ceremony, just enough separation that risky work can't destabilize what already runs:

- **`chore/repo-reorg`** - this reorg (file moves + mechanical path fixes only).
- **`algo/iris-zo-clique-cover`** - all new IRIS-ZO/clique-cover-with-NLP-sampling work, under `algorithms/iris_zo_cliquecover/`. Keep isolated until cross-checked against `data/generation/full_arm_c_free.ipynb`/`experiments/grasping_space.ipynb`'s Drake-builtin results.
- **`full_arm_nhr.ipynb` continuation** - routine iteration is fine directly on `main` (a single notebook diffs/merges poorly as JSON regardless of branching). If a session starts wiring in the new `iris_zo_cliquecover` library, do that specific change on `algo/iris-zo-clique-cover` instead.
- **Deferred content-change work** - each on its own small branch, only when/if decided (see §11).

## 11. Known issues / not fixed in this pass

Flagged during the reorg, deliberately left alone (pure reorg = moves + mechanical path fixes only, no logic changes):

- **`ManipulationPlanner` is copy-pasted**, not shared: at least 5 near-identical inline definitions exist (`manipulation_planner_3dof.ipynb`, `manipulation_planner_4dof.ipynb`, `full_arm_blocked_joints_3dof.ipynb`, `full_arm_blocked_joints_4dof.ipynb`, `data/generation/full_arm_c_free.ipynb`, `full_arm_nhr.ipynb`), with real drift between copies (e.g. only some perform a Mosek-license-file validity check in `__init__`). Worth deduplicating into a shared module once there's a stable interface to converge on.
- **`full_pipeline.ipynb` doesn't use `ManipulationPlanner`** at all, despite the similar subject matter to (1)/(2) above - it's an independent implementation. Worth being aware of before treating a future `ManipulationPlanner` dedup as automatically wiring `full_pipeline.ipynb` in too.
- **Some `artifacts/joint_samples_plots/` runs are tracked in git**, predating the `.gitignore` rule meant to stop that. Left as tracked history rather than force-removed during the reorg; a future `git rm --cached` pass could untrack them if desired.
- **`artifacts/joint_samples_plots` vs `algorithms/joint_samples_plots` split** - `full_arm_nhr.ipynb` and `nhr_standalone_test.py`'s default now point at two different locations (see §6). Unifying them is a one-line content change, not bundled into this reorg.
