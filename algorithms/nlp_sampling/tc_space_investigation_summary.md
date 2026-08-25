# TC-Space Sampling & Wrist-Axis Grasp Polytope: Investigation Record

Branch: `nlp-sampling/quality-metrics`. This document exists because the
investigation below went through several real corrections across many
turns - this is the record to read instead of reconstructing it from
commit messages and GitHub comments. Two independent pieces of work,
covered in order: **Part 1** (TC-space vs. regular-FK sampling
comparison, #46) and **Part 2** (wrist-axis grasp polytope, #80).

---

## Part 1: TC-space vs. regular-FK sampling comparison (#46)

**Motivation**: Steve's hypothesis (2026-08-25 meeting) - TC-space's
rational-polynomial forward kinematics might make NLP sampling mix
better than regular joint-angle FK, since it's what lets C-IRIS use SOS
instead of a general nonlinear program. Priority one, targeting the
2026-09-08 summer school (M. Toussaint attending).

### Files (current, on disk now)

| File | What it is |
|---|---|
| `tc_space_collision_scenario.py` | The test scenario (#78): a real 2-DOF subproblem of the Panda arm (`panda_joint2`, `panda_joint4` free, rest fixed), collision constraint against a virtual sphere obstacle. Regular (trigonometric) FK. Verified via a fine grid scan to have exactly 2 disjoint free regions. |
| `tc_space_collision_scenario_rational.py` | The TC-space version of the same constraint, built as a genuine ratio of polynomials via `RationalForwardKinematics.CalcBodyPoseAsMultilinearPolynomial` + `ConvertMultilinearPolynomialToRationalFunction`. **`q_star = zeros(7)`** - the textbook stereographic projection (`t_i = tan(theta_i/2)`, no offset), confirmed against the reference material you shared. Cross-validated against the regular-FK constraint (agrees to ~1e-16). |
| `tc_space_collision_scenario_fast.py` | A fast plain-numpy evaluator for the same rational constraint - Drake's own symbolic `Polynomial`/`RationalFunction.Evaluate` walks a full expression tree per call, ~230x slower than necessary. Extracts the polynomial's (exponent, exponent, coefficient) terms once, evaluates as vectorized numpy after. Validated to match Drake's symbolic evaluation to ~1e-15, including through the *entire* stochastic sampling pipeline (identical restart-success patterns), not just isolated points. |
| `tc_space_vs_regular_comparison.py` | Runs both samplers (regular FK vs. TC-space) on the identical scenario, multiple seeds, reports MSTS at both p=1 and p=2, saves restart-colored sample plots. **Current version uses `nlp_sampling`'s plain default step size, unadjusted** (no per-space normalization - see "what's not standard" below). |

### What's NOT currently in the repo (deleted, do not resurrect without redoing)

An earlier attempt used a **non-standard `q_star`** (each joint's own range
midpoint, not zero) to keep the resulting `s`-domain bounded/symmetric.
This was invented mid-implementation and was **not** Drake's convention,
not what the papers do, and not your reference material's definition.
Deleted in commit `92790b6`, along with everything built on top of it:
a `tc_space_per_restart_spread_comparison.py` (paired within-chain spread
test) and `tc_space_shared_phase1_comparison.py` (shared-starting-point
comparison). **Every numeric result reported from those two scripts is
invalid** - they tested a parameterization that isn't real TC-space.
They have not been rebuilt with the corrected `q_star=0`. If that
comparison is still wanted, it needs to be redone from scratch against
the current `tc_space_collision_scenario_fast.py`.

### Timeline of what was found (in order, including the wrong turns)

1. **First comparison** (now-deleted, wrong `q_star`): appeared to show
   TC-space with a mild early-sample coverage advantage. **Not valid** -
   see above.
2. **Methodology review caught two real bugs** in that comparison (commit
   `281e6b6`): MSTS was scored at `p=1` (diversity) when mode-coverage
   needs `p=2`; step size (`nlp_sampling`'s default, a fixed number) was
   applied unadjusted despite the two spaces having differently-sized
   domains. Fixing both made the apparent TC-space advantage mostly
   evaporate even under the (still-wrong) `q_star`.
3. **The `q_star` question** (this session, triggered by you asking why
   the plotted `s`-domain didn't match `tan(-pi/2)`): confirmed the
   per-joint-recentered `q_star` was self-invented, not standard. You
   supplied the actual textbook definition (stereographic projection,
   `t_i = tan(theta_i/2)`, no offset) - confirmed Drake's own
   `RationalForwardKinematics` reduces to exactly that when `q_star=0`.
4. **Rebuilt with `q_star=0`** (commit `ee4dbdb`). Real, honest
   consequence: `panda_joint4`'s actual range (`[-3.07,-0.07]`) sits
   almost entirely on one side of `theta=0`, near the `tan(theta/2)`
   asymptote - `s(joint4)` spans `~[-28.6, -0.03]`, ~24x wider than
   `s(joint2)`'s `~[-1.2, 1.2]`.
5. **Ran the comparison with the correct `q_star=0` and an unadjusted
   default step size** (commit `ee4dbdb`, script `tc_space_vs_regular_comparison.py`).
   Result: **visibly broken TC-space sampling**, not a "no advantage"
   null result like before - a real, mechanistic failure. Confirmed via
   the restart-colored plots: each restart chain gets frozen near its
   starting `s(joint4)` value (the fixed step size is minuscule relative
   to that dimension's true ~28-unit scale), and when mapped back to
   physical `q`-space, nearly all chains - despite occupying wildly
   different `s(joint4)` values - collapse onto the *same* narrow
   physical sliver near `joint4≈-3.0`. What looked like per-restart
   diversity in `s`-space was mostly restarts piled on top of each other
   physically. MSTS numbers from this run exist (`median final MSTS_1:
   regular=40.75, TC-space=27.09`; `median final MSTS_2: regular=3.22,
   TC-space=5.31`) but **should not be read as "TC-space wins on mode
   coverage"** - the visual pathology makes the numbers unreliable as a
   quality signal; a couple of lucky long-range chain excursions inflate
   `p=2` without representing real, systematic exploration.

### Current conclusion (as of the last commit on this topic)

No positive finding for TC-space sampling on this scenario. A genuine,
mechanistically-understood **negative** one: applying the standard
`t=tan(theta/2)` mapping with a single shared step size is structurally
wrong for `panda_joint4`, whose actual range sits near the map's
asymptote. This is a property of the correct formula applied to this
joint's actual range, not a bug to paper over. Whether this generalizes
(does it affect other joints/robots similarly, and is there a principled
per-dimension step-size fix) is open - not pursued further this session,
paused in favor of Part 2.

### How to reproduce

```
cd algorithms/nlp_sampling
python tc_space_collision_scenario_rational.py   # validates q_star=0 against regular FK, prints domain sizes
python tc_space_collision_scenario_fast.py       # validates + benchmarks the fast evaluator
python tc_space_vs_regular_comparison.py --num-seeds 5   # the actual comparison, saves plots under artifacts/tc_space_experiments/<timestamp>/
```

---

## Part 2: Wrist-axis grasp polytope (#80)

**Motivation**: Steve's suggestion (same meeting) - for the actual grasp
use case, don't grow a general IRIS-ZO region at all. If the direction
that preserves the grasp constraint is known exactly (moving along the
wrist joint's own axis), the needed region is just a small volume around
that direction.

**File**: `wrist_axis_grasp_polytope.py`.

### What was confirmed, in order

1. **Exact wrist-axis invariance** (not approximate): from a valid grasp
   configuration `q0`, sweeping `panda_joint7` (the wrist) alone over its
   *entire* range `[-1.0, 1.0]`, with every other coordinate fixed,
   leaves `h_grasp_eq`/`g_grasp_ineq` **bit-for-bit unchanged**
   (`0.00e+00` deviation, 21 swept points). Geometric reason: joint7's
   rotation axis passes through the end-effector frame's own origin, so
   pure rotation about it changes neither the frame's position
   (`p_CapE`) nor its z-axis direction (`z_E_in_Cap`) - the only two
   things the grasp constraint checks. Confirmed matches your hypothesis
   exactly.
2. **Both finger joints and the cap's own joint also have exactly zero
   sensitivity** (checked via the constraint Jacobian directly, not
   assumed) - the grasp constraint doesn't depend on them at all.
3. **First construction attempt**: a naive axis-aligned box around `q0`
   for the remaining 6 arm joints. Margin found by bisecting the
   worst-case corner (all 64 sign combinations of perturbing all 6 joints
   simultaneously): **`±0.00353` rad**, keeping the constraint within
   `0.01` tolerance. Validated against 64 corners + 2000 random interior
   points.
4. **Null-space refinement** (your suggestion - "why not use null-space
   projection instead of nlp_sampling"): computed the null space of the
   active constraint Jacobian (`Jh` plus near-active rows of `Jg`) at
   `q0`. Found one additional, non-trivial free direction (a specific
   mix of all 6 arm joints, not aligned with any single one) beyond the
   already-known-trivial wrist/finger/cap directions. Empirically, this
   direction alone stays within tolerance over a **much larger range**
   than the axis-aligned box's margin (initially found as `~0.87` total,
   before accounting for interaction with the other directions).
5. **Found and fixed a real bug in that refinement**: the direction's
   solo extent and the margin in its 5D orthogonal complement are **not
   independent** - pushing the direction to its own solo limit left zero
   room for any complement margin (verified this failure directly:
   121/3000 random combined points infeasible). Re-derived both jointly
   via a tolerance-budget sweep - final numbers: direction range
   `[-0.204, 0.182]`, complement margin `±0.00334` (5 dims). This gave
   **~41.6x more volume** in the arm-joint subspace than the naive box,
   validated against 5000 random points spanning the whole region (0
   failures).

6. **Resolved**: step 5's "improvement" optimized for maximum volume,
   which was the wrong objective - confirmed by you that the region
   needs to be small, matching the finger joints' own grasp tolerance
   scale (`[-0.025, -0.024]`, width `0.001`). Retargeted to
   `margin = 0.001` rad literally (your explicit choice, despite the
   unit mismatch - fingers are prismatic/meters, arm joints
   revolute/radians). Rebuilt with **two** constructions at this same
   small target, kept side by side rather than picking one:
   - `build_simple_margin_polytope` - the direct answer: wrist axis
     (exact, full range) x `±0.001` rad independently on each of the
     other 6 arm joints. No search, no optimization. **This is the
     answer to #80.**
   - `build_null_space_polytope` - the same null-space direction from
     step 4, sized to the same `0.001` target instead of maximized.
     Kept only for comparison.

   Both validated (0 failures, 3064 points each: 64 corners + 3000
   random). **Comparison at this shared small scale**: identical volume
   (a rotation doesn't change volume - the two boxes are the same size),
   but only **~64-66% overlap** between the two regions - genuinely
   different-shaped/oriented boxes of the same size, not the same region
   in different coordinates. The null-space alignment's value is
   specific to *maximizing* region size (step 4-5); at a fixed small
   scale it doesn't offer an advantage, just a different-shaped region
   of equal volume.

### `q0` is not a chosen configuration

Worth flagging clearly: `q0` throughout Part 2 is whatever the first
random-restart downhill search happened to land on
(`find_grasp_seed`) - not a specific, meaningful grasp pose from the
rest of the pipeline. If the real planner needs the polytope anchored at
a *particular* grasp configuration, this needs rerunning with that seed.

### How to reproduce

```
cd algorithms/nlp_sampling
python wrist_axis_grasp_polytope.py
```
