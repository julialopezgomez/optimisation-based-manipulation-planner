# `iris_zo_cliquecover`

Python reimplementation of Drake's IRIS-ZO and clique-cover algorithms, so
the particle-sampling step inside them can eventually be swapped for
`algorithms/nlp_sampling/nlp_sampling.py`'s constrained sampler instead of
Drake's plain hit-and-run. See #48 for why this is a hand-port rather than
a Python-bindings-only integration or a Drake C++ fork, and #42 for the
umbrella issue this all belongs to.

Ported by hand from Drake's actual C++ source
(`RobotLocomotion/drake`, `planning/iris/*.cc`, `planning/visibility_graph.cc`,
`planning/graph_algorithms/*_via_greedy.cc`), because the parts that
actually needed changing - how points get sampled - aren't exposed to
Python separately from the rest of the algorithm. Everything *around* the
sampling step (collision checking, the max-clique solve) already runs in
fast, parallel C++ via Drake's normal Python bindings and is called
directly, not reimplemented.

## Status

| Issue | What it did |
|---|---|
| [#69](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/69) (closed) | Sanity-checked `nlp_sampling.py` against Drake's own sampler on a plain polytope. See "Reading the validation results" below. |
| [#70](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/70) (closed) | Ported IRIS-ZO itself (`iris_zo.py`) - the algorithm that grows *one* obstacle-free region. |
| [#71](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/71) (closed) | Ported clique-cover (`iris_from_clique_cover.py` + helpers) - the algorithm that covers the *whole* space with several regions, calling #70's code once per region. |
| [#72](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/72) (open) | Not a simple swap after all - see its issue body. Blocked on #45, #74, and (loosely) #46. |
| [#74](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/74) (open) | Drake's own `prog_with_additional_constraints` mechanism can't be combined with clique-cover at all (confirmed - Drake throws unconditionally). Same underlying design question as #72. |
| [#75](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/75) (closed) | Added wall-clock + eval-count timing to all three test/validation scripts (this issue). See "Reading the timing results" below. |
| [#45](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/issues/45) (open) | Not started yet: the MSTS diversity/coverage metric from the NLP Sampling paper, needed to actually compare candidate integration strategies for #72. |

PR: [#73](https://github.com/julialopezgomez/optimisation-based-manipulation-planner/pull/73) (draft, covers #69-#71 and #75).

## Files

```
iris_zo.py                    Grows one obstacle-free convex region ("IRIS-ZO").
visibility_graph.py            Helper: checks which sampled points can "see" each
                                other (straight line between them is obstacle-free).
clique_solvers.py              Helper: groups mutually-visible points into cliques.
iris_from_clique_cover.py      Covers the whole space: samples points, groups them
                                into cliques (via the two files above), grows one
                                region per clique (via iris_zo.py), repeats until
                                enough of the space is covered.
tests/
  test_iris_zo_toy_scene.py                    Regression test for iris_zo.py.
  test_iris_from_clique_cover_toy_scene.py     Regression test for iris_from_clique_cover.py.
  validate_polytope_sampling.py                Sanity check for nlp_sampling.py (see below).
```

Both algorithms currently still sample points the same way Drake's C++
does (calling `HPolyhedron.UniformSample`, which is exposed to Python
directly - no need to reimplement it). That's deliberate: it means the
*port* could be checked against Drake's real output before anything about
the sampling itself changes. #72 is where the sampler actually gets
swapped.

## Running the tests

All three scripts are plain, runnable Python files (no pytest needed) -
run them from anywhere, e.g. from the repo root:

```
python algorithms/iris_zo_cliquecover/tests/test_iris_zo_toy_scene.py
python algorithms/iris_zo_cliquecover/tests/test_iris_from_clique_cover_toy_scene.py
python algorithms/iris_zo_cliquecover/tests/validate_polytope_sampling.py
```

### `test_iris_zo_toy_scene.py` and `test_iris_from_clique_cover_toy_scene.py`

These build a tiny fake robot (a dot that can move in a 2D square, with one
square-shaped obstacle in the middle) and run both the ported code and
Drake's real, built-in version on it, side by side, with the same random
seed. If the port is faithful, both should produce the *same* answer -
same number of regions, same shape, same size - not just a similar-looking
one. The scripts end with plain `assert` statements checking exactly that;
if a script finishes and prints "All checks passed", the port matches
Drake's own implementation on that scene. If an `assert` fails, the port
has drifted from Drake's actual behavior somewhere and needs fixing before
it can be trusted.

**Reading the timing results**: both scripts also print a wall-clock time
for each implementation and a `ported/drake wall-clock ratio`. A ratio
above 1 means the Python orchestration around the port is slower than
Drake's C++ one - expected, since the port's `for` loops (sorting
particles, adding hyperplanes, building cliques) run in plain Python,
while Drake's run in C++. What both implementations call underneath for
the actual expensive work - collision checking - is the *same* parallel
C++ either way (Drake's `CollisionChecker`, called directly, not
reimplemented), so this ratio isolates the pure Python-loop overhead, not
a difference in the core collision-checking cost. On the toy scene here
(200 particles, 2 DOF), that overhead comes out to roughly **1.4x** for
`iris_zo.py` and **1.7x** for `iris_from_clique_cover.py` (clique-cover
has more pure-Python bookkeeping: the greedy clique solve and visibility
graph construction). Take the exact multiplier with a grain of salt at
this toy scale - the real Panda-arm problem has far more particles per
call, where the fixed per-call Python overhead matters proportionally
less, so the real ratio there is likely to be smaller, not larger; this
number is a regression signal to watch (a script that used to report 1.4x
and now reports 4x has probably regressed), not a scaled-up performance
prediction.

### Reading the validation results (`validate_polytope_sampling.py`)

**What is this checking, in plain terms?**

IRIS-ZO grows a region by repeatedly scattering random points inside its
current (shrinking) region and checking which ones hit an obstacle. How
those points get scattered matters. Drake's own version scatters them
using a standard, well-tested method called hit-and-run: pick a random
direction, find the two walls of the region in that direction, land
somewhere between them. Repeated many times, the points end up spread
*uniformly* (equally likely to land anywhere) across the whole region.

We eventually want to scatter points using our own sampler
(`nlp_sampling.py`) instead, because it can also handle a kind of
constraint Drake's sampler has no notion of at all: curved constraints
that only allow a thin sliver of configurations (e.g. "the gripper must be
lined up with the cap to grasp it"). Drake's sampler only ever understands
straight walls (a polytope) - it can't be pointed at a problem like that.

So before trusting our sampler with that harder job (#72), we wanted a
sanity check on the one case where both samplers *can* be compared
head-to-head: a plain polytope, with none of the curved constraints our
sampler is actually built for. **This script hands both samplers the
exact same simple shape (a box, or a box with a few corners sliced off)
and checks whether the points they each produce end up spread out the
same way.** If they don't agree even here, on the easy case, there'd be no
reason to trust our sampler on the hard case either.

This is **not** a test of `nlp_sampling.py`'s actual purpose (sampling
under curved/equality constraints) - there's no equivalent in Drake to
compare that against. That side of the sampler is checked the way it
already was, via `nlp_sampling_standalone_test.py`'s diagnostics against
the real grasp problem.

**How to read the printed output:**

```
dim | drake mean  drake std |   nlp mean    nlp std |  KS stat     KS p
  0 |    -0.0024     0.5778 |    -0.1327     0.5012 |   0.1362   0.0000  <-- check
```

- One row per coordinate (dimension) of the sampled points.
- `mean`/`std`: the average and spread of each sampler's points along that
  coordinate. If both samplers agree, these should be close to each other.
- `KS stat` / `KS p`: a standard statistical test for "do these two sets
  of numbers look like they came from the same distribution?". A small
  `KS p` (below 0.01, marked `<-- check`) means the test thinks they
  probably didn't. Caveat printed under the table: because both samplers
  produce a *chain* of points where each one depends on the last (not
  independent draws), this test is more trigger-happy than usual - treat
  a flag as "worth a look", not "definitely broken".
- A second table (`within-chain std` vs. `across-seed std`) catches a
  specific failure mode: a sampler that looks fine when you pool several
  runs together, only because each run got randomly dropped in a
  different spot and never actually wandered from there. If the
  "within-chain" number is much smaller than the "across-seed" number,
  that's what's happening.

**What we actually found, and what it means:**

- Given a big enough step size, `nlp_sampling.py` matches Drake's sampler
  well - same mean, same spread, most dimensions pass. In other words:
  no bias. It's the same target distribution, just reached differently.
- At `nlp_sampling.py`'s *normal* default step size (deliberately small,
  so it stays safe next to the thin curved constraints it's actually
  designed for), it explores a wide-open plain polytope much more slowly
  than Drake's sampler and needs many more steps to catch up.
- This is not a bug. It's the cost of the caution that lets it work near
  constraints Drake's sampler can't handle at all. The practical
  takeaway for #72: don't reuse that cautious default step size as-is
  once the sampler is driving IRIS-ZO's region growth - a freshly-started
  region is wide open (much like this test), so a larger step size early
  on (tightened only once the region gets small) will likely be needed to
  keep pace with Drake's own particle budget.

**Timing**: the script also prints a `timing:` line per polytope, e.g.

```
timing: drake=0.071s (127337 samples/sec) | nlp_sampling=0.366s (24600 samples/sec, 12000 g-evals total, 1.3 g-evals/sample) | ratio=5.2x
```

`g-evals` counts calls to the polytope's own inequality function - the
one piece of "constraint work" this simplified test case has (there's no
`h` here, remember, since Drake's sampler has no equality-constraint
notion to compare against). At only ~1.3 evaluations per recorded sample,
`nlp_sampling.py` isn't spending its time re-evaluating constraints here -
the roughly **5x** slower wall-clock (on this toy polytope, default step
size) comes from its own per-step bookkeeping (the Gauss-Newton cleanup
step, slack tracking, etc.), which is real Python overhead on top of the
mixing-time cost already discussed above, not a separate finding to chase
down here - just worth having visible before #72 has to reason about
sampler cost at IRIS-ZO's actual particle-count scale.

## Deliberately not ported

Matching Drake's actual usage in this repo (`experiments/grasping_space.ipynb`
only ever calls the plain `IrisZoOptions()`/`IrisFromCliqueCoverOptions()`
paths) - each of these is a separable extra feature in Drake's C++, not a
simplification of the core algorithm, so they're left out rather than
guessed at. Two of them turned out to matter more than "just unused" on
review, though, and now have their own issues rather than being closed
questions:

- **`prog_with_additional_constraints`** - not just unused, actively
  interesting. This is Drake's own existing mechanism for filtering
  IrisZo/IrisNp2 particles against an arbitrary extra `MathematicalProgram`
  (every particle failing a constraint is treated as infeasible, same as a
  collision) - a natural-looking hook for grasp-manifold constraints. But
  Drake's own clique-cover code refuses to combine it with clique-cover at
  all (throws unconditionally, for every IRIS variant - confirmed by
  reading `iris_from_clique_cover.cc`'s precondition check). Tracked in
  #74, alongside the real question it raises: is this the right mechanism
  to lean on once `nlp_sampling.py` is driving particle generation, or a
  distraction from it?
- **`parameterization`** (growing regions in a reparameterized space) -
  relevant to the TC-space/rational-FK hypothesis in #46: parameterizing
  by TC-space instead of regular joint angles might change how efficiently
  regions can be grown near the grasp manifold. Not evaluable yet without
  #45's metric.
- `containment_points`, meshcat debug visualization, `MaxCliqueSolverViaMip`
  (needs a proprietary solver license this project doesn't have) - genuinely
  just unused, no open question attached.
