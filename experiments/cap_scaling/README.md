# Reachability A* with 1 to 8 caps

The 8-cap scene, its C-free, and the run time of the reachability A* as the number of caps to turn grows.
All planning code is the notebook's own: `full_arm_planner_8caps.ipynb`. The scene (cap poses, obstacle
boxes) is defined once, in `multicap_scene.py` at the repo root, which the notebook and these scripts import.

| file | what it does |
|---|---|
| `generate_cfree.py` | Standalone. Computes C-free for the scene (9-D, caps welded) and writes `data/cfree/cfree_9dof_8caps_3obs_capsafe.yaml`, the grasps it was built around (`*_grasps.npz`, loaded by the notebook) and a validation report (`*_report.json`). |
| `run_notebook.py` | Runs the notebook's code cells headlessly, up to its "Step 7" section (scene, grasp polytopes, C-free, A*). |
| `cap_scaling.py` | For K = 1..8 caps and N random queries each, times the A* (search + placing the waypoints), checks every plan, and writes `summary.json`, `table.tex` and a self-contained page. |

## Run

From anywhere, with the `obmp` conda env.

```bash
# 1. C-free (once per scene; ~30-60 min on 32 threads). Rerun after editing multicap_scene.py.
python experiments/cap_scaling/generate_cfree.py
#    options: --out data/cfree/<name>.yaml  --coverage 0.98  --grasps-per-cap 2  --cut-rounds 6  --threads 32

# 2. Run time against the number of caps (~40 min with the defaults, mostly exact A* at its time limit;
#    --no-exact: ~3 min)
python experiments/cap_scaling/cap_scaling.py --out artifacts/cap_scaling
#    options: --ks 1,2,...,8  --queries 20  --seed 0  --exact-time-limit 60  --no-exact  --render-k 8
#             --no-render  --max-expansions 2000000
```

If you write C-free under another name, point the notebook's `CFREE_FILE` (section 1) at it.

## Why C-free is 9-D

A cap's collision geometry is a cylinder about its own joint axis, so no collision depends on a cap angle:
the true free set is (9-D arm + finger free set) × (cap angles). Generating it with the caps welded is
therefore exact, and its dimension stays 9 however many caps the scene has. The A* already worked in the
cap-free space (it tracks each cap's angle as an interval), so it uses the regions directly.

## How C-free is generated (`generate_cfree.py`)

1. **Grasps.** For each cap, the first random seeds of `find_grasp_seed` that an NLP refines onto the
   grasp (|h| ≤ 1e-6, g ≤ 0, arm joints ≥ 0.05 rad inside their limits) and that are collision-free.
   Grasp 0 of each cap is the notebook's q0ₖ.
2. **Seeded regions.** IRIS-NP2 along each collision-free wrist run of each grasp, from a starting
   ellipsoid that is long along the wrist (the run's half-length + 1 rad) and thin across it (1e-4). In
   IRIS-NP2's metric, collisions beside the grasp line are then far away, so its planes run parallel to
   the line and one region holds the whole run. A round starting sphere left regions only ~0.2 rad long;
   the clique cover alone left one cap's grasp only 40% covered in the two-cap scene.
3. **Clique cover** (IRIS-NP2) with the seeded regions as the initial sets. Settings: ε = 1e-3, δ = 1e-2,
   3 outer iterations, relative termination 1e-3, coverage 0.98. The stricter settings of the earlier
   notebooks (50 iterations, termination 1e-4) ran for hours on 11-D, or crashed with a degenerate region.
4. **Cutting false positives**: points a region calls free but that collide. A plane through the
   closest collision-free configuration (an NLP), never cutting the collision-free part of a grasp's
   wrist sweep that the region holds. On the wrist sweeps (an obstacle blocks part of a cap's wrist
   range; fallback: a plane normal to the wrist at the end of the free run), around the caps (wrist held
   fixed in the NLP, so the plane is parallel to the wrist axis), and uniformly (arm against the walls).
5. **Validation** on fresh samples, saved in `*_report.json`.

## What is timed (`cap_scaling.py`)

As in `experiments/gcs_comparison`: the A* search and placing the waypoints, with the peak extra resident
memory. Not timed: the shared preprocessing that builds the A*'s graph (Steps 1-4, reported once), and
C-free.

Every query is solved twice:

- **weighted A\*** (the notebook's `HEURISTIC_WEIGHT` = 1.5: nodes ordered by g + 1.5·h), guaranteed to
  need at most 1.5× the fewest actions;
- **exact A\*** (weight 1.0: the fewest actions), stopped after `--exact-time-limit` (60 s) per query. Once
  no query of some K solves within the limit, it is skipped for larger K.

Why both: with 8 caps the exact search is exponential in K. Each action costs 1 and each hop only
μ = 0.001, so it expands essentially every (region, α-box) state with fewer hops than the optimum. The
α-boxes multiply across caps, and the scene's 356 regions have ~140 neighbours each. The weighted search
finishes the remaining actions first and grows about linearly with K. The page compares their action
counts on the queries both solve.

A query with K caps picks K of the 8 caps at random; the A* may grasp only those (the other caps' grasp
components leave the graph) and every cap's angle stays in its state. The active caps turn by random
amounts in [0.5, π] rad that sum to 2.5·K rad (`sample_turns` in the notebook), from α = 3.0. Start and goal
are random configurations in C-free.

## Output (`--out`)

- `summary.json`: every query (caps, turns, nodes expanded, times, memory, grasps, path length,
  certificate check) and per-K statistics (median, quartiles, min, max).
- `table.tex`: the per-K table in the paper's style.
- `cap_scaling.html`: self-contained page with the table, charts of run time and nodes expanded per
  query, and a render of one K = 8 plan.
- `render_meshes/`, `render_first_frame.jpg`: what the render used.
