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
| where a return happens | in the piece G (cap fixed) | in F_k ⊇ G (cap fixed) |
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
configurations?".

## 8. Why the full problem is heavy

The relaxation has variables for **every edge** (a copy of both segments per edge) and needs roughly
2–6 MB per edge here.

850 of the 861 pairs of free regions intersect, so the free graph is almost complete: about 1,700
edges per layer with all 42 regions. A 6 rad turn needs 8 strokes (the longest piece gives
L_max = 0.81 rad), so about 10 layers.

| version | edges | result |
|---|---|---|
| full layers | ~19k | killed at 35–59 GB |
| pruned layers | ~2.3k | relaxation 140 s at 13.7 GB; rounding passes 15 GB |
| GCS on the A* sequence | none (only the chosen sets) | 0.02 s |

The A* search takes ~0.01 s on the same problem because it works on α intervals and never builds
these copies.
