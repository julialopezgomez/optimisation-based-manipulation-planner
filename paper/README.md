# Paper draft: Reachability Search over Convex Sets for Multi-Regrasp Manipulation Planning

This is the second draft. The first draft was written without access to branch `pipeline/full-arm-planner`, so it only sketched a point-based A*. In this draft, the **reachability A\*** from `full_arm_planner.ipynb` (inspired by CASSR, Wang & Tonneau, arXiv:2603.02989) is the main method. The MIQP is described as the earlier formulation, together with the reasons it fails at 10-DOF.

The format is IEEE conference (`IEEEtran`, two-column). The draft is **8 pages including references**, both with and without the placeholder notes.

## Build

```bash
latexmk -pdf main.tex
```

To check the length without the notes, set `\showtodosfalse` in `main.tex`.

## Placeholder conventions

| Macro | Colour | Meaning |
| --- | --- | --- |
| `\todo{...}` | red | Missing content, or a decision you need to make |
| `\tocheck{...}` | orange | Taken from notebook output or notes. Verify before submission |
| `\tbd` | red | Missing number |

## The pitch

**Takeaway:** the number and location of regrasps are *outputs* of a reachability computation, not inputs.

**What is new** (it must stay precise to survive review):

- A search whose cost is the number of grasp actions, whose nodes are continuous reachable sets (polytope × interval of object angle), and which is complete and optimal **relative to the decomposition**. There is no fixed horizon, no integer variables, and the graph needs only LPs.
- Grasp pieces (grasp polytope ∩ free region) are the only sets where reachability grows. They are the switch sets of the manipulation graph.
- Exact dominance and an admissible, consistent heuristic. In CASSR the heuristic is not admissible and yaw is discretised. In GCS\* the domination checks are approximated in general.

**What is not claimed:** physical optimality or completeness (both are relative to the sets), path-length optimality, or generality beyond Assumption 1 (one object coordinate coupled to one joint).

## Reviewer's assessment of the current draft

Below is what I would write as a reviewer, ordered by severity.

1. **The only 10-DOF plan collides in reality.** Along the wrist sweep at the seed grasp, 46 of 99 points are in collision, yet every decomposition contains them (Sec. VI-C). Honest reporting turns this into a finding, but a planning paper whose headline plan is invalid will be rejected. **Fix:** validate the grasp pieces by restricting them to wrist intervals that the collision checker confirms, re-run Q0, and confirm the expected 5 strokes.
2. **There is one query and no baselines.** The claimed advantages are adapting to goals, starts and obstacle layouts with guarantees. None of these is tested yet. **Fix:** run E1–E4 (Sec. V-B). E1 (the goal sweep) and E4 (time vs. required regrasps, against the MIQP and a sampling-based planner) are the two plots that make the paper stand out.
3. **The MIQP failure is asserted, not documented.** A reviewer will ask whether a better MIQP would work, for example with tight per-row big-M from the joint-limit box, a convex-hull formulation, or rescaling. **Fix:** log the MOSEK status. Ideally also run the MIQP with per-row M_r = max over the box of (a_r·x − b_r), so the baseline is not a straw man.
4. **Proposition 1 is informal.** Two points need care.
   - Exact dominance for grasp nodes: in the holding phase, the wrist position and α are correlated, so the reachable set is not exactly polytope × interval. Prove that the abstraction loses nothing, or state the result for the abstraction.
   - Placement always succeeds: today the code asserts this, but there is no proof.
5. **The scope is narrow.** Assumption 1 covers caps, knobs, valves and cranks. Say so up front and frame general objects (Minkowski-sum propagation, as in CASSR) as future work. A second object or scene would help.
6. **The grasp set is ad hoc.** It uses one seed, a linearisation, and a trust region ρ = 0.05 chosen by hand. Report |h| along every plan, and ideally use several NLP-sampling seeds.
7. **The learning argument is an argument, not an experiment.** It is kept to that level in the text. Do not strengthen it without a learned baseline.
8. **The low-DOF results come from the old pipeline.** Run the reachability A* on T1. It should return 11 grasps, matching the MIQP.

## Sources used

- **Branch `pipeline/full-arm-planner`**:
  - `full_arm_planner.ipynb`, section "Reachability A\* over free and grasp polytopes", cells 33–55 and their outputs: 42 regions, 8 pieces, 850/58 edges, 63 expansions in 4 ms, 2 strokes + 1 return + 1 release, all 6 motions certified, path length 15.25. Cell 58 gives the TP/FP wrist sweep: 53/46 for all four decompositions.
  - `algorithms/manipulation_planner/reachability_astar_guide.md`, for the design rationale and the trust-region numbers (2.27 rad, |h| = 0.95, 0.80 m without it). Those numbers come from a q0-as-start/goal run.
  - `algorithms/manipulation_planner/manipulation_planner.py`, for the MIQP: M = 1e6, component merging through convex hulls with QJ joggling.
  - `full_arm_blocked_joints_3dof.ipynb`, cell 24: the MIQP returns 11 grasps on the 3-DOF task, with GCS succeeding on all segments.
  - `artifacts/diffpoly_runs/stage4_10dof_pilot/baselines/report.json`: IRIS-NP2 at 0.98 coverage took 120 s for 25 regions, a different run from the 42-region set.
- **CASSR** ([arXiv:2603.02989](https://arxiv.org/abs/2603.02989)): the node definition, Minkowski-sum propagation, the non-admissible scaled EPA heuristic, yaw discretisation, the QP placement stage, and the results (30 steps in under 125 ms).
- **MOSEK documentation**: the default `MSK_DPAR_MIO_TOL_ABS_RELAX_INT` is 1e-5, which gives the big-M argument (1e6 × 1e-5 = 10 rad of slack).
- The first draft (MInf report, Year-1 review, IPAB notes) for the problem formulation, the grasp constraints and the low-DOF coverage finding.

## Bibliography

The new entries are at the end of `references.bib`, and those marked `VERIFY` were written from memory or from an arXiv abstract page. CASSR was missing from the first draft and is now `wang2026cassr`. The earlier entries still need checking against DBLP.

## Venue notes

- RSS uses its own template and does not count references towards its page limit.
- ICRA and IROS have typically allowed 6 pages plus 2 extra pages (with a fee, or for references only, depending on the year).
- Check the current CFP. At 8 pages including references, this draft fits all three, but there is no slack for the E1–E4 results until something is cut. Candidates: Fig. 1 (the teaser), the Algorithm 1 box, or the Related Work section on GCS.
