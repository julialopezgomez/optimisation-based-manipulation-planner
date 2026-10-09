# GCS baseline: what it is, in short

Code: section "GCS baseline on the same polytopes" in `full_arm_planner_gcs.ipynb`.

## 1. What GCS does

GCS (graph of convex sets) gets a graph whose **vertices are convex sets** and returns:
- a **path** through the graph (which sets, in which order), and
- **one curve inside each visited set**.

The curves of consecutive sets meet at a common point, and the total cost is minimised. Each
vertex can be used **at most once** on the path.

We use straight segments (Bézier order 1). In vertex v the segment goes from r₀ to r₁, and both
points must lie in the set X_v. A convex set contains the whole segment, so every move is certified
collision-free: the same certificate the A* waypoints have.

```python
gcs = GcsTrajectoryOptimization(10)                       # q = (7 arm joints, 2 fingers, cap α)
F = gcs.AddRegions(sets, edges, order=1, name="free")     # one vertex per set
r = F.vertex_control_points()                             # placeholder r[:, 0] = r₀, r[:, 1] = r₁
```

## 2. The sets: the same as the A*

| | set | from the A* |
|---|---|---|
| free vertex | F_i, the cap column dropped and α free in its limits | `free_sets` (Step 1) |
| grasp vertex | G_c = Grasp ∩ F_k (with the trust region) | `components` / `pieces` (Steps 2–3) |
| edge F_i – F_j | F_i ∩ F_j ≠ ∅ | `ff_door` (Step 4) |
| edge F_i – G_c | F_i ∩ G_c ≠ ∅ | `fc_links` (Step 4) |

## 3. The manipulation rule: two modes, as linear constraints

Write w for the wrist and α for the cap. The constraint depends only on the type of the vertex:

**Free vertex** (approach, transit, return): the cap does not move.
$$r_1[\alpha] = r_0[\alpha]$$

**Grasp vertex** (a stroke): the cap turns with the wrist, α −= Δw, and only forward.
$$r_1[\alpha] + r_1[w] = r_0[\alpha] + r_0[w], \qquad r_1[w] \ge r_0[w]$$

```python
F.AddVertexConstraint(r[idx_cap, 1] == r[idx_cap, 0])                                        # free
G.AddVertexConstraint(r[idx_cap, 1] + r[idx_wrist, 1] == r[idx_cap, 0] + r[idx_wrist, 0])    # grasp
G.AddVertexConstraint(r[idx_wrist, 1] >= r[idx_wrist, 0])                                    # forward only
```

**Sign.** The code turns the cap as α −= Δw (3 → −3 rad). The paper uses the opposite convention
(α += Δw, so α_init ≤ α_goal); there the grasp constraint reads r₁[α] − r₁[w] = r₀[α] − r₀[w]. Same
constraint, mirrored axis.

These are the same equations as in `ManipulationPlanner._solve_for_n_grasps_CC` (placement:
cap equal; grasp: cap − wrist constant, sign flipped for this plant). The difference is what they
attach to. In the MIQP the mode of each pair of points is fixed by its index, and binaries with
big-M pick the polytopes. In GCS the mode comes from the vertex type, and the path through the
graph picks the polytopes.

## 4. Layers: so the same piece can be grasped several times

A full turn needs several strokes in the same piece G_c, with a return in between. A GCS path cannot
visit G_c twice, so the graph is copied once per grasp:

```
source → approach → grasp1 → between1 → grasp2 → between2 → … → graspN → depart → target
            └────────────────────────────── (no grasp) ──────────────────────────┘
          every graspₙ can also go straight to depart
```

- `between` layers: the 8 free regions that touch a grasp piece (returns and regrasps happen there).
- `approach` / `depart`: the start (or goal) regions, those 8, and the regions the A* path uses.
- N = ⌈|α_goal − α_init| / L_max⌉ + 1 grasp layers: the fewest strokes that could work, plus one spare.

**Where the "at most once" comes from.**

- Paper (Marcucci et al., *Shortest Paths in Graphs of Convex Sets*, arXiv 2101.11565, §2): "an s-t
  path p is a sequence of **distinct** vertices", and there is one point x_v per vertex,
  constraint (2.1c).
- Drake v1.48, `geometry/optimization/graph_of_convex_sets.cc`:
  - l. 1541–1543, the degree constraint `∑ ϕ_out ≤ 1 − δ(is_target)`: a vertex is left at most once;
  - l. 1516, spatial conservation `∑ z_in = ∑ y_out`: one copy of the vertex's variables, so one segment;
  - l. 1905–1916, the rounding's depth-first search skips vertices already in `visited_vertex_ids`.
- Checked on the toy (a 4 rad turn, strokes ≤ 2 rad): with one grasp vertex and an edge back to
  the free set it is infeasible, both relaxed and as the exact MIP; with two grasp layers it solves.

**Pruning.** Only free sets are pruned, never grasp pieces:

- `between` layers keep the 8 free sets that touch a transition set;
- `approach` and `depart` keep those 8, the free sets containing the start or the goal, and the A*
  path's free sets.
850 of the 861 pairs of free sets intersect, so every set left out removes ~40 edges from each copy.
What is lost: detours through free sets that touch neither the start, the goal nor a transition set.

**Why not copy only the grasp pieces?** Between two strokes the hand lets go and turns the wrist back
with the cap fixed. That move happens in a free vertex (F_k ⊇ G_k), and F_k was already used for the
previous return, so it needs a fresh copy too. That is why a `between` layer exists, and it is
already only 8 free sets, not the whole graph.

You could also do the return inside a copy of the piece itself: a "return" vertex over G_k, with the
cap frozen and the wrist free. Each layer is then only grasp-piece copies (stroke G_k → return G_k →
stroke G_k …). Free sets appear only in `approach` and `depart`, and it matches the A* exactly (its
returns are inside the piece). Not implemented yet. Most of the edges per layer are the grasp ↔ free
links (~50 each way), so this would roughly halve the graph.

## 5. The cost

$$J = \underbrace{\sum_{\text{vertices}} \lVert r_1 - r_0 \rVert}_{\text{path length}}
\;+\; 10 \cdot \#\text{grasps} \;+\; 0.001 \cdot \#\text{other polytope changes}$$

```python
gcs.AddPathLengthCost()                                          # Σ ‖r₁ − r₀‖
edges.AddEdgeCost(10.0 + 0 * edges.edge_constituent_vertex_durations()[0])   # on every free → grasp edge
```

The `0 * duration` is only there because Drake needs a variable in an edge cost.

10 per grasp is much more than any path length here, so the fewest grasps win first and path length
breaks ties. That is the A*'s objective. Every grasp has exactly one release, so "grasps + ungrasps"
in the A* is 2 × grasps here, which gives the same ranking.

## 6. Where it matches the A*, and where it doesn't

| | A* | GCS |
|---|---|---|
| sets and edges | `free_sets`, pieces, `ff_door`, `fc_links` | the same, lifted to 10-D |
| cap rule | α frozen outside a stroke, α −= Δw in a stroke | the same, as linear constraints |
| forward-only stroke | yes | yes (r₁[w] ≥ r₀[w]) |
| objective | fewest grasp/ungrasp actions, then hops | 10·grasps + path length + 0.001·hops |
| where a return happens | in the piece G (cap fixed) | in F_k ⊇ G (cap fixed); see §4 for doing it in G |
| configurations | placed afterwards by a heuristic (`build_waypoints`) | optimised together with the path |
| optimality | exact on its abstraction (α intervals) | relaxation + rounding: not guaranteed |
| free regions per layer | all | pruned (see §4) |

## 7. How `SolvePath` works, and how it is timed

`SolvePath` runs three stages in one call:
1. **Relaxation.** One big convex program over the whole graph, with fractional "how much of each
   edge is used". It gives a lower bound on the cost.
2. **Rounding.** Random walks that follow those fractions give up to `max_rounded_paths` candidate
   sequences of sets.
3. **Restriction.** For each candidate, a small convex program finds the segments with the sequence
   fixed. The best candidate is returned.

Timing in the notebook:
- relaxation alone: `max_rounded_paths = 0`;
- the trajectory for a fixed sequence: `SolveConvexRestriction(sequence)`;
- the sequence: the total minus that.

**GCS on the A* sequence** is only stage 3, run on the A*'s sets. It answers "same polytopes, better
configurations?". When the A* leaves a piece after a stroke, it first moves ungrasped inside the
piece, so the mapped sequence goes stroke G_k → F_k → F_j, not G_k → F_j.

**What each time covers.** Both methods start from the same shared preprocessing: the transition
sets, their extreme wrist points, and the transit and switch edges (A* Steps 1–4). It takes ~1 s and
is reported separately, not in either method's time. Computing the free sets (IRIS) is offline for
both.
- A*: search + placing the waypoints.
- GCS on the A* sequence: A* search + building the layered graph + one convex program.
- Full GCS: building the layered graph + `SolvePath`.
The GCS graph reuses the A*'s edges (`edges_between_regions=`), so GCS does not redo the
intersection checks.

**MOSEK licence.**
- MOSEK reads the licence path from `MOSEKLM_LICENSE_FILE`. Only the conda env's activate script sets
  it, so the GCS notebook now sets it itself (`os.environ.setdefault`). Without it, Drake silently uses
  another solver.
- Every MOSEK solve checks a licence out again unless one is held. The ~1,700 small LPs of the shared
  preprocessing take ~50–90 s that way, and ~1 s with
  `licence = MosekSolver.AcquireLicense()` held for the whole run. The big GCS solve barely changes.

## 8. Results so far, and why the full problem is heavy

Every run so far (Runs A–E, each with its own start and goal) is in Table III of the paper. This is
Run A, the clean comparison.

**One start and goal for every row** (`artifacts/plan_comparison/`; side-by-side videos in
`compare_D0p5.html` and `compare_D1p5.html`). The shared preprocessing (1.1 s) is not counted for
any method.

| turn | method | grasps | path length | time | peak extra memory |
|---|---|---|---|---|---|
| 0.5 rad | A* (graph 50 / 900, 83 nodes expanded) | 1 | 18.06 | 3.7 ms | ~0 |
| | GCS on A* sequence | 1 | 11.49 | 0.016 s (convex program: 3 ms) | <1 MB |
| | full GCS (48 vertices / 543 edges) | 1 | 11.49 | 44 s | 4.0 GB |
| 1.5 rad | A* (122 nodes expanded) | 2 | 20.30 | 4.8 ms | ~0 |
| | GCS on A* sequence | 2 | 13.62 | 0.022 s (5 ms) | <1 MB |
| | full GCS (64 / 749) | 2 | 13.62 | 93 s | 5.6 GB |
| 6 rad | A* (369 nodes expanded) | 8 | 31.16 | 14 ms | ~0 |
| | GCS on A* sequence | 8 | 24.48 | 0.052 s (8 ms) | 1 MB |

Takeaways:

- All methods use the same number of grasps.
- The full GCS returns exactly the path of GCS on the A*'s sets: it picks the same sets.
- The A*'s path is 21–36 % longer only because its placement stage does not optimise length.
- Where the full GCS time goes (0.5 rad): the convex relaxation is 43.9 of the 44.1 s; rounding ~0.3 s.
  It is not branch and bound: `SolvePath` solves the relaxation of the mixed-integer program and rounds.
  Drake's preprocessing (one small program per edge) pays for itself: 61.9 s without it.
- Timings need one MOSEK licence held, and the solve in a normal process: a forked child runs MOSEK
  single-threaded (191 s instead of 44 s for the same solve).

**Why the full problem is heavy.**

- The relaxation has variables for every edge (a copy of both segments per edge), a few MB per edge
  here.
- 850 of the 861 pairs of free regions intersect, so a copy of all 42 free regions adds ~1,700 edges.
- A 6 rad turn needs 8 strokes (L_max = 0.81 rad), so about 10 layers.

| 6 rad version | edges | result |
|---|---|---|
| all 42 free sets in every layer | ~20k | killed by the OS (process at 35–57 GB) |
| all 42 in approach/depart, 8 between | ~5.4k | killed by the OS (59 GB) |
| pruned (start/goal + 8 near) | ~2.3k | relaxation 140 s at 13.7 GB; rounding stopped by our 15 GB limit |

How to read the memory figures:

- **"Killed by the OS"**: how big the process was when the machine ran out of RAM. It depends on what
  else was running, so it is a lower bound, not a requirement.
- **"Stopped by our limit"**: a watchdog we added to protect the machine. It means "needed more than
  that", not "cannot be solved".
- **Comparable figures** come only from solves run in their own child process (the table above).
