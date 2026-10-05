# Paper draft: Reachability Search over Convex Transition Sets for Multi-Regrasp Manipulation Planning

This is the third draft, a rewrite for clarity of story and contributions. The method and numbers are unchanged from the second draft, which is based on `full_arm_planner.ipynb`.

Format: IEEE conference (`IEEEtran`, two-column). It is 8 pages including references with the placeholder notes hidden, and 9 with them shown.

## Build

```bash
latexmk -pdf main.tex
```

To check the length without the notes, set `\showtodosfalse` in `main.tex`.

## Placeholder conventions

| Macro | Colour | Meaning |
| --- | --- | --- |
| `\todo{...}` | red | Missing content, or a decision you need to make |
| `\tocheck{...}` | orange | Verify against the code, the notebook output or the source before submission |
| `\tbd` | red | Missing number |

## The story (agreed 2026-10-05)

- **Problem.** Multi-regrasp tasks (caps, valves, knobs, cranks) need a number of grasps that depends on the goal, the joint limits and the obstacles. The task looks simple, but it is hard for three reasons:
  - grasps happen on thin sets near contact;
  - the length of the plan is unknown, and obstacles change it discretely;
  - the task has memory, since the cap keeps its angle after release.
- **What other methods lack**, stated specifically for each:
  - **Sampling-based manipulation graphs** (Siméon et al.) are only probabilistically complete. They cannot prove that a goal is infeasible or that a plan uses the fewest grasps, and the grasp sets have zero volume, so reaching them needs dedicated samplers.
  - **Optimisation-based TAMP** fixes the grasp sequence and uses local solvers, so a failure is not a proof of infeasibility.
  - **MIQP** (our earlier work) fixes the number of grasps and needs an outer loop, and its big-M constraints are numerically fragile at 10-DOF.
  - **Learned policies** give no completeness guarantee, cannot certify infeasibility, and have no guarantee on goals that need more regrasps than they were trained on.
- **Footstep planning** (CASSR) has the same combinatorial core. Regrasping needs three additions:
  - the contact sets must be built from nonlinear kinematics;
  - grasps move the object, so reachability must carry the object state;
  - motions must avoid collisions in the full configuration space.
- **Our method.** Free sets come from a convex decomposition, and the grasp set from a seed grasp and the null space of the grasp constraints. Their intersections are the **transition sets** (Siméon's manipulation-graph nodes, but convex). A reachability A\* searches them, with nodes made of a set and an interval of reachable cap angles. Each guarantee comes from one property:
  - convexity: no sampling is needed inside a set;
  - a coupled object coordinate: reachable cap angles form an interval, so propagation and dominance are exact;
  - finitely many sets and a cost that counts grasps: the search terminates with the minimum number of grasps, or proves that no plan exists;
  - linear programs only: no integer variables.
- **Main contribution:** the reachability A\* over transition sets. The MIQP appears as a short paragraph and a baseline.

## Decisions made with Julia

- **Naming:** "transition sets", for grasp set ∩ free set.
- **Optimality claim:** the minimum number of grasps. The code charges stroke = return = release = 1 and every other move μ, so cost = 2 × grasps + μ × set changes.
- **NLP sampling:**
  - The method uses only the Gauss–Newton seed and the null-space idea.
  - The interior samplers (hit-and-run, manifold RRT), growing IRIS from samples, and the max–min network go in the discussion, as attempts and as extensions (multiple seeds).
- **Validating transition sets:**
  - It is a method step (Sec. IV-C), still being implemented (E5).
  - The finding that the decomposition contains collisions at grasps motivates it.
- **Learning:** a short, specific comparison in the intro.
- **Siméon et al.:** the difference is the representation (sampled vs. convex), with the same family of task.

## Open items

1. **Validation (E5):** implement it, then re-run Q0 (expected: 5 grasps) and regenerate Fig. 1.
2. **Cost of a return from the "entered" phase:** the code charges 1 for rotating the wrist back before any grasp. It should cost μ for the cost to be exactly 2 × grasps.
3. **Experiments E1–E4:**
   - Implement the sampling-based baseline: your own manipulation-graph PRM, or HPP.
   - Run the reachability A\* on T1 and check that it returns 11 grasps.
   - Record the MOSEK status for the 10-DOF MIQP.
4. **Proposition 1:** write the full proof, especially part (iii), that the recovery stage always succeeds.
5. **Characterisations to verify:** regrasp maps (Levit & Toussaint), Liu et al. 2026, SL1M.
6. **T1:** decide between the WSG gripper (MInf) and the locked Franka.
7. **Fig. 1:** a figure of the Franka/cap task was removed for space. Add one back if the results leave room.
8. **Bibliography:** entries marked `VERIFY` in `references.bib`, and all older entries, still need checking against DBLP.

## Sources

- **Branch `pipeline/full-arm-planner`:**
  - `full_arm_planner.ipynb`, cells 33–58: the reachability A\*, Q0 and the wrist sweep;
  - `algorithms/manipulation_planner/reachability_astar_guide.md`;
  - `algorithms/manipulation_planner/manipulation_planner.py` (the MIQP).
- **CASSR** (arXiv:2603.02989) and Julia's reading notes in `PhD-Literature/Literature Notes/wang_CASSR_2026/`.
- **Background:** the MInf report, the Year-1 review and the IPAB notes, for the problem formulation and the low-DOF coverage finding.
