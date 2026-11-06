# Progress, week to 27 Oct 2026: HPP baseline, GCS and the reachability A*

## 1. Results in one table

Scene `1cap_2obs`: Panda (7 arm joints + 2 fingers) and one cap, 10-DOF, with two small posts beside it and a floor.
All methods share one start and one goal. The cap turns from 3.0 to 3.0 − D rad (cap range [−π, π]). The
posts leave at most 0.95 rad of free wrist per grasp, so the minimum is ⌈D / 0.95⌉ grasps.

| D [rad] | min grasps | Reach. A* (ours) | GCS on the A* sequence | Full GCS | HPP M-RRT, 60 s |
|---|---|---|---|---|---|
| 0.5 | 1 | 1 grasp, length 14.10, 10 ms | 1, 14.10, 19 ms | 1, **11.36**, 0.56 s | **7/10** runs, 1 grasp, 16.67, 40 ms |
| 1.5 | 2 | 2, 16.03, 17 ms | 2, 16.03, 28 ms | **3**, 13.99, 0.73 s | 0/10 |
| 2.4 | 3 | 3, 18.21, 22 ms | 3, 18.21, 35 ms | **4**, 15.52, 0.93 s | not run |
| 3.2 | 4 | 4, 20.14, 19 ms | 4, 20.14, 33 ms | 4, **18.17**, 1.04 s | not run |
| 6.0 | 7 | 7, 26.90, 37 ms | 7, 26.90, 59 ms | **8**, 24.22, 1.82 s | 0/10 |

- At 600 s, on the same scene without the floor, HPP still solved 0/10 at D = 1.5 and at D = 6, and 8/10 at D = 0.5.
- Scene `8caps_3obs`: 8 caps and 3 walls, 17-DOF.
  - A* (weighted, w = 1.5): solves all 145 valid queries, up to 8 caps and 15 grasps, in ≤ 1.7 s each.
  - Exact A*: finishes within 60 s only up to about 4 caps.
  - HPP: 0/120 runs (5 queries × 3 seeds for each K = 1..8, 60 s each).
- Every plan from every method is re-checked by Drake: collisions, the cap turning only with the hand, and reaching the goal.

## 2. HPP: what exactly the constraints are

Built with `ConstraintGraphFactory`, as in tutorial 2. The code is `experiments/hpp_comparison/hpp_plan.py`.

| | what | how |
|---|---|---|
| Gripper | `panda/gripper` = the `panda_hand` frame (the frame our grasp constraint h(q) uses) | SRDF `<gripper>` |
| Object | each cap, loaded with `"anchor"` (not freeflyer): it only moves through its revolute joint `cap_to_base` | `urdf.loadModel` |
| Handle | on the rotating `cap_link`, 0.1075 m above the cap (the middle of our height band), z pointing down, `clearance = 0.05` (pre-grasp 5 cm above) | SRDF `<handle>` |
| Handle mask | `[x, y, z, rx, ry]` enforced, `rz` (about the cap axis) free | `robot.handles()[...].mask` |
| Contacts | none: `setObjects(caps, handles, [[]])`; `environmentContacts` not used | factory |
| Outside a grasp | the factory's placement locks `cap_to_base` with a parametric `LockedJoint`: the cap stays at whatever angle it has | factory, no contact surfaces |
| While grasping | `rz` is the grasp's complement, held constant along a transfer, so the cap turns with the hand (Δα = −Δwrist; checked with `getConfigErrorForTransitionLeaf`) | mask |
| Fingers | `LockedJoint` ±0.02405 m (our grasp width), `EqualToZero`, in every state | `addNumericalConstraintsToGraph` |
| Rules | `Rule([gripper], ["^$"])` (free) + one rule per cap the query may turn; everything else rejected | `setRules` |
| Security margins | −∞ between `panda_joint7`/fingers and `cap<k>/cap_to_base` on the last approach and first retreat (`_12`, `_21`): the fingers close with ~0.05 mm to spare | tutorial 3 |
| Graph | `maxIterations(40)`, `errorThreshold(1e-5)` | |
| Validation | config: `JointBoundValidation` + `CollisionValidation`; paths: `pathValidationFactory(Dichotomy(robot, 0.0))` (continuous), `ProgressiveProjector(0.01)` | as in `ur3-spheres.py` |

**One difference from our model.** HPP lets the whole arm move while grasping. Our grasp sets only let the wrist
turn: the other joints stay within ρ = 0.05 rad of the seed. So HPP gets the larger feasible set.

## 3. HPP: the algorithm and its parameters

**Algorithm:** `ManipulationPlanner`, the Manipulation RRT (Siméon et al. manipulation graph; Mirabel et al.).
Each iteration (`oneStep`, in `hpp-manipulation/src/manipulation-planner.cc`):

1. Sample `q_rand` uniformly with the configuration shooter. This includes the cap angle.
2. For every connected component (the init tree, the goal tree, …) and every graph state:
   - take the nearest node in that state, `q_near`;
   - pick one of its outgoing transitions (`chooseEdge`, by weight);
   - `generateTargetConfig` projects `q_rand` onto that transition's leaf through `q_near`. The leaf's
     right-hand side is read from `q_near`: the cap angle in `free`, and the hand–cap yaw while grasping;
   - build the straight path, project it (`ProgressiveProjector`), validate it (`Dichotomy`), and keep the valid
     part as a new node.
3. Try to connect the new nodes to other components (`tryConnectNewNodes`; otherwise the 7 nearest nodes,
   `tryConnectToRoadmap`). A connection needs a transition whose leaf through q1 contains q2 (`Edge::build`,
   tolerance 1e-5).
4. Stop when the init and a goal are in the same component, or at the time-out.

**Parameters we pass:**
- `maxIterations(1e9)`, i.e. unbounded;
- `timeOut(60)` (also a 600 s run);
- `extendStep` left at its default of 1;
- default transition weights;
- `srand(seed)` for seeds 0–9 (1 cap) or 0–2 (8 caps): HPP samples with `std::rand`, so without a seed every run is identical;
- `addGoalConfig` with one goal configuration;
- paths are not shortcut.

HPP is built from source on the devel branch (hpp-manipulation `1cc7070`, hpp-core `372535b9`, 2026-10-20;
`env/hpp_versions.txt`).

## 4. Why HPP times out (our reading; please correct)

- **What a multi-grasp plan needs.** Each grasp turns the cap by at most 0.95 rad (wrist limited to [−1, 1], minus
  the posts). So D = 1.5 already needs grasp → turn → release → wrist back → regrasp → turn → release → goal.
- **Connections must meet exactly in a leaf.** From the code above, two configurations connect only if they lie on
  the same leaf:
  - in `free`, the same cap angle;
  - while grasping, the same hand–cap yaw.

  Extensions aim at uniform random targets, so a release lands at a continuous, random cap angle. The goal sits on
  one leaf, α_goal, and every intermediate release creates a new leaf that the two trees would have to share
  exactly.
- **What the runs show:**

  | runs | nodes |
  |---|---|
  | D = 0.5, solved | 7–21 (median 0.04 s) |
  | D = 0.5, the 3 that failed | ≈ 12,000 at 60 s |
  | D = 1.5 and D = 6 | 6,000–12,000 nodes at 60 s; ~43,000 at 600 s; never connected |
  | 8 caps | ~4,500 per 60 s (bigger scene); even K = 1 needs 2 grasps (2.5 rad); 0/120 |

- **So** it either finds the single grasp almost immediately or not at all. Its effort grows with the roadmap,
  not towards the goal's leaf.
- **Questions for you:**
  - How does M-RRT land a release exactly on α_goal in the one-grasp case?
  - Is there a recommended way to set up a ratcheting task: a goal on the cap angle only, or `StatesPathFinder`
    (in our build, with the `ur3-spheres-spf.py` example)?

## 5. Running a Drake scene in the HPP container

```
host (conda env obmp, has Drake)                    container hpp:obmp (pyhpp, pinocchio; no Drake)
--------------------------------                    -------------------------------------------------
export_scene.py --scene 1cap_2obs           ───►    hpp_plan.py --stop-after batch --seeds 10 --time-limit 60
  writes artifacts/hpp_comparison/<scene>/:          reads those files (repo mounted at /home/user/obmp)
  panda/cap/env .urdf+.srdf, meshes,                 writes raw/<query>_s<seed>.npz (path sampled every
  queries.json, consistency.npz                      0.002, graph state per sample, time, nodes, memory)
check_and_report.py                         ◄───
  re-checks every path with Drake, writes summary.json, table.tex
```

- **The container:** tutorial 1's Dockerfile and Makefile, copied to `experiments/hpp_comparison/env/`. The repo
  is mounted, `--net host` so the viser viewer opens in the host browser, and the build uses 16 jobs.
- **The export:** the notebooks' own Drake scene code (`multicap_scene.py`) writes URDF/SRDF:
  - one link per Drake body with its collision shapes (Drake's spheres), and Drake's joint limits;
  - SRDF `<disable_collisions>` for every pair Drake filters, so both planners check the same pairs;
  - Drake's Franka glTF meshes, for display only.
- **The checks:**
  - hand pose matches Drake to 1.3e-8 over 500 configurations;
  - collision verdicts agree 500/500 (1 cap and 8 caps).

**Problems we hit, all fixed:**
1. `loadModel` needs absolute paths, and the mesh paths differ between host and container. The URDF writes
   `@SCENE_DIR@`, which is filled in at load time (`loadModelFromString`).
2. A new `Problem` validates nothing (`isConfigValid` returned True in collision); we added the two validations.
3. The default discretized path validation (0.05 step) let a finger pass 4 mm into a post, so we switched to the
   `Dichotomy` factory.
4. Without a path projector, a "solved" 6 rad plan put the hand 3 cm under the floor between samples, so we added
   `ProgressiveProjector`.
5. `problem.pathValidation(...)` (instead of `pathValidationFactory`) replaced the per-transition validation, and
   then no grasp was ever valid. Found by reading `problem.cc` and `graph/edge.cc`.
6. The fingers grazed the cap's fixed base (0.13 mm), out of reach of the joint margins. We moved the base's
   coaxial cylinder onto `cap_link`.
7. Memory grows over many runs in one process, so batches run per query and resume from the saved `.npz` files.
8. One 8-cap query has its goal in collision in Drake, 3.3 cm inside a wall; HPP skips it.

## 6. How the comparison is kept fair

- **Same robot, scene, collision geometry and filters:** exported from the one Drake plant, with forward
  kinematics and collisions cross-checked.
- **Same queries:** one saved start and goal for 1 cap; the same random queries for 8 caps.
- **Same grasp:** the handle in the middle of our height band, the same finger width, Δα = −Δwrist.
- **Same final check for every plan:** Drake re-checks collisions, slip of the hand on the cap, and the goal. The
  metrics (grasps, length in (x, α), x = arm and fingers) are computed the same way for every method.
- **HPP validates motions continuously,** which is stricter than its default and closer to our certified
  convex sets.
- **Time and memory:**
  - online time per query for every method;
  - the offline stage reported separately: C-free 2.8 min (1 cap) and 18.6 min (8 caps), A* graph 0.09 s and 2.0 s.
    It is shared by the A* and GCS; HPP needs none;
  - peak memory measured per run, and each full GCS solve runs in a fresh process.
- **Seeds and time limits:** HPP gets 10 seeds per query and a 60 s limit, plus a 600 s check. GCS gets MOSEK
  with one licence held.
- **Known biases:**
  - HPP may move the whole arm while grasping (in its favour);
  - its paths are not shortcut (against it, in length);
  - the A*'s guarantees are relative to the convex sets, which are validated by dense sampling, not certified.

## 7. Changes to GCS and the A* this week

- **One C-free method for every scene** (`experiments/cap_scaling/generate_cfree.py`):
  - 9-D with the caps welded, which is exact, since a cap is a cylinder about its own axis;
  - IRIS-NP2 regions grown along each grasp's wrist runs, then a clique cover to 0.98 coverage, then separating
    planes that cut off false positives on the wrist sweeps, around the caps and uniformly;
  - result: 0 false positives on the wrist sweeps, and 7 of 20,000 uniform samples (1 cap).
- **The A* graph's LPs use your regularised-QP trick with quadprog.** No licence checkout per LP, so the 8-cap
  graph now takes 2.0 s instead of 15 min.
- **The A* second stage** is the solver-free placement followed by one SOCP: the shortest path through the same
  sets, keeping each move's kind. "GCS on the A* sequence" gives identical lengths, so the waypoints are now
  optimal for the chosen sets.
- **GCS:** same formulation (the graph copied once per grasp, `SolvePath`).
  - It is ~80× faster than before (44 s → 0.56 s) because the new decomposition has 4 transition sets instead of
    8 near-duplicates, so its graph has 130 edges instead of 543.
  - It finds paths 10–19% shorter at equal grasps.
  - On 3 of 5 turns it returns one grasp more than needed. The A*'s plan is a path of the same GCS graph with a
    lower GCS cost, so rounding the relaxation missed the optimum.
- **The A* itself is unchanged** (exact on 1 cap; weighted w = 1.5 on 8 caps). Weighted and exact gave the same
  action counts on all 69 queries both solved.

## 8. Still to do

- **HPP:**
  - your answers to the questions in §4;
  - try `StatesPathFinder` as a second HPP baseline;
  - optionally shortcut HPP paths before comparing lengths.
- **A\*:** add path length to the cost (a door point per node, λ‖p − p_goal‖ in the heuristic) to close the 10–19%
  gap to full GCS, if we want it in the paper.
- **GCS:**
  - run full GCS on the 8-cap scene, where its graph grows again (11 transition sets, 23,729 transit edges);
  - try returns inside the transition sets instead of through free sets.
- **MIQP:** numbers on the current decomposition.
- **Paper:** cut to 8 pages and regenerate Fig. 1. Then move the original notebooks to `legacy/`.
