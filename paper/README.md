# Paper draft: Reachability Search over Convex Transition Sets for Multi-Regrasp Manipulation Planning

This is the third draft, a rewrite for clarity of story and contributions. The method and numbers are unchanged from the second draft, which is based on `full_arm_planner.ipynb`.

Format: IEEE conference (`IEEEtran`, two-column). It is 8 pages including references with the placeholder notes hidden, and 9 with them shown. There is no slack left, so the E1–E5 results will need cuts.

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

## The point of the paper (settled 2026-10-05)

Abstract and introduction rewritten from scratch, taking CASSR's *logic* as inspiration (one move per paragraph) while deliberately avoiding its wording:

1. **The problem.** A manipulation plan is a sequence of contacts. Choosing it is combinatorial, its length is unknown before planning, and the usual attack is a graph searched by sampling, by a mixed-integer programme, or by fixing the modes first.
2. **What they all need.** A model of reachability between contacts. It is nonlinear and its solution sets are thin, so it gets approximated by enumeration or sampling. Convex decompositions are the continuous alternative.
3. **Why that is not enough.** Those convex models have been consumed by mixed-integer programmes, which need the number of grasps and lose meaning on thin sets; searches that leave the number open discretise instead. A* already has the properties we want; what is missing is a reachability model it can expand without discretising.
4. **Precedent and delta.** CASSR supplies one for footsteps. Manipulation does not inherit it: the contact sets must be constructed from the grasp constraints, and contact displaces the object.
5. **What we do**, then contributions, each stating what is new rather than where it lives.

**Phrasing to keep away from.** CASSR's own sentences, for similarity-check safety: "a discrete problem of exponential complexity", "deterministic and encodes optimality by design", "motivate the search for a continuous formulation of reachability constraints compatible with A*", "act by making and breaking contact with their environment". The current text expresses the same ideas differently; keep it that way if you edit.

**Style:** no em dashes; one idea per paragraph; the cap task belongs to the experiments, not the framing.

**The MInf self-citation is gone**, on the grounds that a project report does not belong in a paper. The mixed-integer formulation is now introduced as the standard way convex decompositions are used in contact planning, cited to [deits2014footstep], and implemented as a baseline. Nothing in the paper now claims it as prior work of ours.

**Still open:** the algorithm has no name. CASSR gains from one; worth choosing before submission.

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
- **A\* formulation (requested by Julia):** the A\* follows CASSR and Griffin et al. 2019 in their syntax.
  - Algorithm 1 is the standard A\* loop, with the parts specific to our problem in blue, each pointing to its own numbered paragraph.
  - Struct 1 describes the node.
  - Algorithm 2 (`expandNode`) replaces the earlier moves table.
  - The numbered paragraphs cover the A\* algorithm, the node structure, `expandNode`, `nodeAlreadyExpanded`, `hasReachedTheGoal` and the cost computation.
  - Specific differences from CASSR are stated in the text: exact dominance (CASSR uses a 2 cm threshold), an admissible heuristic (CASSR's is weighted), and revisiting sets is allowed.

## Reviewer pass (2026-10-05)

A full editing pass for precision, cohesion and repetition. The substantive corrections, as opposed to wording:

- **Problem statement** said plans are made of "transit segments and strokes"; returns were missing even though the cost charges for them.
- **Transition sets were defined twice with different scopes** (manipulation-graph nodes in §III-A, then "transfer motions also happen inside them"). §III-B now states the consequence of Assumption 1(i) once: a transition set is a collision-free piece of the grasp space, so a single set can host a grasp, a whole turn and the release.
- **"One stroke is one grasp"** is now stated, so the claims about grasps connect to the moves named in the method.
- **Extreme-wrist LPs** were written `w_k^± = max/min{±w : x ∈ T_k}`; now two separate LPs.
- **Proposition (ii) termination argument was wrong.** It claimed each stroke lowers α⁻ by at least `L_min`, but a stroke's reach is `w_k⁺ − a⁻`, which can be smaller. Replaced with the correct argument: interval ends come from finitely many LP-derived wrist values, so finitely many nodes exist per key. **Worth confirming against the code.**
- **Cost ordering** ("grasps first, set changes second") holds only while `μ · (set changes) < 1`; the condition is now stated.
- **§VI-A contradicted itself** — "the online stage never calls a collision checker: its only geometric operations are the LPs of the offline stage". Now: the online stage does no geometric computation; it all happened offline.
- **Convexity claims** now say the connecting line stays inside the set, which is the part that matters.
- **TP/FP counts** in the fidelity results are spelled out instead of relying on an unstated classifier convention.
- **Repetition**: the thin-set-near-contact observation was made five times; it is now made once in the introduction, with the technical justification in §IV-C and the rest referring back. "Essentially a wrist interval" / "essentially a segment" replaced with the precise convexity statement.

## Keeping the method independent of the experiment (2026-10-05)

The introduction and method were pinned to the Franka cap task; they are now stated for the general setting, and every task-specific number lives in Sections V--VI.

- **Introduction** no longer names the 10-DOF task, the arm, or big-M. The mixed-integer limitation is stated as a property of the formulation: indicator constraints that select among sets are poorly conditioned when those sets are thin.
- **§IV-B (grasp set)** now defines the grasp conditions abstractly as `h(x)=0, g(x)<=0` and says what each typically contains; the cap instantiation, its constants, and the rank of `J_h` moved to §V-A. The wrist-invariance property is stated as a general condition on the last joint axis, not as a fact about the Panda.
- **§IV-G (mixed-integer)** is now a principled account: a binary certified integral only to tolerance `tau` relaxes its set by `M*tau`, so membership is meaningful only for sets much thicker than that; and the continuous relaxation weakens as `M` grows. Tight per-row constants and convex-hull (extended) formulations are cited as the standard remedies [vielma2015mixed], neither of which removes the outer loop over the number of grasps. The concrete numbers (`M=1e6`, `tau=1e-5`, a 2e-3-thick grasp set) now appear in the MIQP comparison in §VI.
- **New §VI-B, "The trust region on the grasp set"**, carries the evidence that was in the method: without `rho`, extreme points lay 2.27 rad from the seed with `|h|=0.95` and the hand 0.80 m from the cap.
- The running-example figure is now tied to task T2 rather than standing alone as "the 10-DOF cap task".

## Sign convention (changed 2026-10-05)

The paper now has **the wrist and the cap increasing together, and both increasing towards the goal**, so Δα = Δw and α_init ≤ α_goal. Previously the cap decreased, which matched the frames in the notebook but read backwards.

The flip touches only the cap side; every wrist quantity is unchanged:

| | before | now |
| --- | --- | --- |
| w.l.o.g. | α_goal ≤ α_init | α_init ≤ α_goal |
| stroke | β = max(α⁻ − c, α_goal, α_min), child [β, α⁺] | β = min(α⁺ + c, α_goal, α_max), child [α⁻, β] |
| forced turning | α⁺ ← α⁺ − d | α⁻ ← α⁻ + d |
| heuristic | 2⌈(α⁻ − α_goal)/L_max⌉ | 2⌈(α_goal − α⁺)/L_max⌉ |
| c, d | | unchanged |
| Q0 | 3 → −1 rad | −1 → 3 rad |

Verified as a pure reflection: the playground returns the same plan, the same costs and the same 14 expanded nodes under both conventions.

**Remark 1** in §III now states that reversing the task (screwing instead of unscrewing) is the map (α, w) → (−α, −w), which exchanges w⁻ with w⁺ and a⁻ with a⁺. A planner that handles one direction handles both. The notebook currently refuses α_goal > α_init outright; the reimplementation should apply that reflection per query instead.

## Before reimplementing: the d question (corrected)

An earlier version of this note said `d` could claim reachability that does not exist. **That was wrong.** `d` acts only on the least-turned end of the interval, and measuring it from a⁺ is exact there: the least-turned configuration that can leave through a doorway is the one that entered as far forward as possible (at a⁺) and turned just enough to reach a⁻_jk. The most-turned end is untouched. Each end is realised by its own path (α⁻ + d by "enter at a⁺, short stroke"; α⁺ by "enter at a⁻, full strokes"), which is exactly what a reachable *set* means: every α in it is reached by some path, not all by the same one.

The real open question is **physical**: between letting go and crossing the doorway, can the wrist move freely?

- A **return** winds the wrist back inside T_k without turning the object. That only works if letting go frees the wrist.
- Under that same reading, no exit ever forces turning: let go, slide the wrist to the exit window, cross. Then **d is always 0**, the release child is just [α⁻, α⁺], and the rule forbidding a *leave* from *entered* when d > 0 is too strict.
- The current d rule only makes sense if the object must be **carried** to the doorway. But then a return is impossible inside T_k, and nothing handles a wrist that finishes *past* the exit window (a full stroke ends at w⁺ = 1; exiting to F0, window [−1, −0.6], would need the object turned backwards).

So the rules are sound but conservative under the first reading, and inconsistent under the second. **Decide the model before writing the code.** If it is the first, the rewrite gets simpler: forced turning disappears from Algorithm 2 (lines 16–17 collapse to one unconditional add), and the last bar of Fig. 1 becomes [−1, 3] rather than [0.35, 3].

There is also a geometric question underneath: the grasp set holds the fingers within 1 mm of the cap width, so a return that keeps the configuration inside G (as the notebook's `point_at_wrist` does) never opens the fingers. Whether that is a slide or a grip is not something the geometry can tell you.

## Open items

1. **Validation (E5):** implement it, then re-run Q0 (expected: 5 grasps) and regenerate Fig. 1.
2. **Cost of a return from the "entered" phase:** the code charges 1 for rotating the wrist back before any grasp. It should cost μ for the cost to be exactly 2 × grasps.
3. **Experiments E1–E4** (note: the paper is exactly 8 pages now, so the results will need space made for them — candidates: Struct 1, the Related Work GCS paragraph, or Fig. 1 panel (b)):
   - Implement the sampling-based baseline: your own manipulation-graph PRM, or HPP.
   - Run the reachability A\* on T1 and check that it returns 11 grasps.
   - Record the MOSEK status for the 10-DOF MIQP.
4. **Proposition 1:** write the full proof, especially part (iii), that the recovery stage always succeeds.
5. **Characterisations to verify:** regrasp maps (Levit & Toussaint), Liu et al. 2026, SL1M.
6. **T1:** decide between the WSG gripper (MInf) and the locked Franka.
7. **Figures:** there is no teaser figure on page 1 (the Franka/cap photo was dropped for space); `figures/franka_cap_grasp.png` is still in the repo. The E1/E4 results figure is commented out in `sections/experiments.tex` because `figures/generalisation.pdf` does not exist yet — that empty grey box was the old Fig. 2.
8. **Citations removed for space**, when the A\* section grew: Aceituno-Cabezas 2018, Song 2021, Vega-Brown & Roy 2016, Berenson 2011, PDDLStream, Natarajan 2024, Chestnutt 2005 and Amice 2022. They are still in `references.bib`. Long author lists were shortened to "et al.".
9. **Author emails:** Julia's is as she gave it; Steve's (`stonneau@ed.ac.uk`) was taken from the author footnote of his CASSR paper. Confirm it is the address he wants on this submission, and remember both must come out if the venue is double-blind.
10. **Bibliography:** entries marked `VERIFY` in `references.bib`, and all older entries, still need checking against DBLP.

## Sources

- **Branch `pipeline/full-arm-planner`:**
  - `full_arm_planner.ipynb`, cells 33–58: the reachability A\*, Q0 and the wrist sweep;
  - `algorithms/manipulation_planner/reachability_astar_guide.md`;
  - `algorithms/manipulation_planner/manipulation_planner.py` (the MIQP).
- **CASSR** (arXiv:2603.02989) and Julia's reading notes in `PhD-Literature/Literature Notes/wang_CASSR_2026/`.
- **Background:** the MInf report, the Year-1 review and the IPAB notes, for the problem formulation and the low-DOF coverage finding.
