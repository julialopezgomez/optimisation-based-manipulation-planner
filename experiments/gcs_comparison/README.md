# Reachability A* vs GCS on the same sets

Regenerates the GCS comparison of the paper (Table III, Run A) and the side-by-side pages in
`artifacts/plan_comparison/`. All planning code is the notebook's own: `full_arm_planner_gcs.ipynb`. These
scripts only run its cells, time and measure each method, and render the plans.

| file | what it does |
|---|---|
| `run_notebook.py` | Runs the notebook's code cells headlessly, up to its "GCS baseline" section (scene, free-set fixing, brute force, A*). |
| `full_gcs_child.py` | One full GCS solve in a fresh process, started by `compare_plans.py`. |
| `compare_plans.py` | From one start and goal, and for each turn: A*, GCS on the A*'s sequence and full GCS, with time and peak memory. Renders every plan and writes the comparison pages. |

## Run

From anywhere, with the `obmp` conda env. MOSEK uses the repo's `mosek.lic`.

```bash
# a new scenario (the start and goal come from a random grasp seed, so each run differs)
python experiments/gcs_comparison/compare_plans.py --out artifacts/plan_comparison_new

# reproduce Run A exactly, from its saved start, goal and sets
python experiments/gcs_comparison/compare_plans.py --scenario artifacts/plan_comparison --out artifacts/run_a_again
```

Options:

- `--turns 0.5,1.5,6` (cap turns in rad): the A* and GCS on its sequence.
- `--full-turns 0.5,1.5`: also run the full GCS.
- `--page-turns 0.5,1.5`: also write a comparison page.
- `--memory-limit 28` (GB): stop the run above this.

Expected on an AMD Ryzen 9 9950X with 60 GB:

| turn | grasps | full GCS time | full GCS memory |
|---|---|---|---|
| 0.5 rad | 1 | ~45 s | ~4 GB |
| 1.5 rad | 2 | ~95 s | ~6 GB |
| 6 rad | 8 | does not fit | — |

Everything else takes seconds. A full run with the defaults takes ~8 min, mostly loading the notebook
once per process and rendering.

## Output (`--out`)

- `summary.json`: per turn and method, the grasps, path length, time (with its parts), peak extra
  memory, graph size, and certificate check; plus the shared preprocessing time.
- `compare_D*.html`: self-contained pages that play the plans side by side on one timeline, with the
  table. Share the file itself; no other file is needed.
- `scenario.npz`, `cfree_fixed.yaml`: the start, goal and sets, for `--scenario`.
- `render_meshes/`: the OBJ meshes used to draw the markers (ghost hands, dial, cap arc).

## What is timed

The same as the paper:

- A*: the search + placing the waypoints.
- GCS on the A* sequence: the A* search + building the layered GCS graph + one convex program.
- Full GCS: building the layered graph + `SolvePath`. Each solve runs in a fresh process
  (`full_gcs_child.py`), which reloads the saved scenario. That way its memory is not hidden by memory an
  earlier solve left allocated, and MOSEK stays multithreaded: a forked child ran the same solve ~4×
  slower.

Not counted for any method: the shared preprocessing that builds the A*'s graph (transition sets,
extreme points, transit and switch edges, ~1 s), which the GCS graph reuses; and the decomposition.
