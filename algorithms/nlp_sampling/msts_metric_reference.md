# Minimum Spanning Tree Score (MSTS): Reference

Reference for `minimum_spanning_tree_score`, `minimum_spanning_tree_score_curve`,
and `plot_msts_curve` in `nlp_sampling.py` (#45). Ported from Section 4.1 of

> Toussaint, M., Braun, C.V., & Ortiz-Haro, J. (2024). NLP Sampling: Combining
> MCMC and NLP Methods for Diverse Constrained Sampling. ArXiv, [abs/2407.03035](https://arxiv.org/abs/2407.03035).

Run `python msts_demo.py` for a live demo against `nlp_sampling.py`'s actual
`restarting_nhr_sample` (not just raw point clouds - see the synthetic
grid/cluster checks below for those): a "double-well" feasible set
(`g(x) = (x0^2-a^2)^2 + x1^2 - b^2 <= 0`, two disjoint disk-like blobs at
x0=-3 and x0=+3) sampled with the real restart+interior-sampling machinery.
`MSTS_2(D_n)` jumps from ~0.1 to ~33 right around the sample count where
restarts first land in *both* wells, then stays flat - visually and
numerically exactly the "found a new mode / stopped finding new modes"
signal this metric is for. Saves a two-panel plot (samples + MSTS curve) to
`artifacts/msts_demo.png`.

## The problem this solves

`per_restart_spread` (mixing diagnostic) and a mean/std/KS-test comparison
against a known-correct sampler (`algorithms/iris_zo_cliquecover/tests/validate_polytope_sampling.py`,
#69) both answer some version of the same question: **once the sampler has
run for a while, did it converge to the right thing?** That's correctness,
checked after the fact.

For this project's actual use case - drawing particles to grow an IRIS-ZO
region - a different question matters more: **how diverse is the sample set
after only a few hundred draws, and did it find every disconnected piece of
the feasible space, or just the one nearest the start?** A sampler that's
asymptotically correct but explores slowly, or that gets stuck in one mode of
a multi-modal feasible set, can be useless in practice even though it would
eventually pass a mean/std/KS check given enough time. Also, in the paper's
setting - and in most of ours except the polytope-only case #69 exploited -
there's no known-correct reference distribution to compare against at all,
so a KS-style check isn't even available.

MSTS is built for that different question, and doesn't need a reference
distribution to compare against - it's computed from the sample set alone.

## How it works

Given a set of `n` sampled points `D = {x_1, ..., x_n}`:

1. Build the **Euclidean minimum spanning tree** over `D` - the smallest set
   of edges connecting all `n` points into one tree, no cycles.
2. Sum the edge costs, where each edge's cost is `|x - x'|^p` for a chosen
   exponent `p`. That sum is `MSTS_p(D)`.

Two regimes, controlled by `p`:

- **`p = 1`** (plain Euclidean length): `MSTS_1(D)` strictly increases as
  points are added - a spanning tree over more points can only get longer.
  The *rate* of increase carries a dimensionality signal: grid-sampling `n`
  points from a `d`-dimensional box gives roughly `MSTS_1 ~ n^(1 - 1/d)`
  (this project's own check: an 8x8 2D grid gives `MSTS_1 = 9.0` against a
  predicted `8.0`; an 8x8x8 3D grid gives `73.0` against a predicted `64.0` -
  right order of magnitude, not exact, since the paper's formula is an
  asymptotic approximation, not an identity).
- **`p > 1`**: `MSTS_p(D)` is *not* monotonic in `n`. As more points land
  inside an already-populated mode, the tree's edges there shrink toward
  zero and, raised to a power `p > 1`, contribute almost nothing new. But
  the edge(s) connecting genuinely *separated* modes stay large - so for
  large `n`, `MSTS_p` ends up dominated by inter-mode distances. In effect,
  `p > 1` turns the same statistic into a **mode-separation / coverage**
  score instead of a density score.

This project's implementation confirms that split concretely: two Gaussian
clusters 10 units apart, 200 points each, sampled in the order
`cluster_a, cluster_b` (so a growing prefix eventually crosses from one into
the other) -

```
n:            2     3     6     11    21    37    68    123   222    400
MSTS_1(D_n):  0.03  0.07  0.20  0.36  0.49  0.80  1.11  1.66  12.59  13.87
MSTS_2(D_n):  0.00  0.00  0.01  0.01  0.02  0.03  0.02  0.04  95.9   93.5
```

`MSTS_1` grows steadily throughout. `MSTS_2` stays near zero until the
prefix first includes points from *both* clusters (between n=222 and n=400,
where the split from `cluster_a` into `cluster_b` happens at n=200), then
jumps by three orders of magnitude - the single edge connecting the two
modes now dominates the whole sum. Run against a single, unsplit cluster of
the same total size, `MSTS_2` instead grows smoothly and stays small
(0.001 to 0.04) the whole way - confirming the jump above is really about
mode separation, not just "more points."

## Reading `minimum_spanning_tree_score_curve` / `plot_msts_curve`

Because sampling here happens sequentially (restarts, then interior steps
per restart - see `restarting_nhr_sample`), it's natural to compute
`MSTS_p(D_n)` for growing prefixes `D_n = D[:n]` **in generation order**, and
plot the score against `n`. That plot is the actual point of the metric:

- A curve that keeps climbing steadily (at `p=1`) means the sampler is still
  finding new territory; a curve that flattens means it's stopped adding
  anything new (mixed, or stuck).
- A curve at `p>1` with sudden jumps means new, previously-unfound modes were
  reached at those points in the sampling process; a flat `p>1` curve with
  occasional density elsewhere means only one mode is being explored, no
  matter how many samples come in - the "stuck in one mode" failure mode
  this metric exists to catch.
- The paper also plots against **number of constraint evaluations**, not
  just `n` - two samplers that reach the same MSTS after the same `n` aren't
  equally good if one needed 10x the constraint evaluations to get there.
  `minimum_spanning_tree_score_curve` returns `(ns, scores)`; pass a
  cumulative per-sample evaluation-count array (built the same way #75's
  `CallCounter` wrapper builds one) as `plot_msts_curve`'s `x_values` to plot
  against that axis instead of raw `n`.

`minimum_spanning_tree_score_curve` only evaluates at `num_checkpoints`
log-spaced points, not every single `n` - recomputing an MST from scratch is
`O(n^2)`, so scoring every prefix would be `O(n^3)` overall. Log-spacing also
happens to be what you'd want for a readable plot anyway.

## Comparing MSTS to the mean/std/KS approach (#69)

| | mean/std/KS (`validate_polytope_sampling.py`) | MSTS |
|---|---|---|
| Answers | Did the sampler converge to the *right* distribution? | How diverse is the sample set, and did it find every mode, as a function of how much sampling has been done? |
| Needs a reference distribution? | Yes - only possible here because Drake's own sampler happens to solve the identical polytope-only problem | No - self-contained, works even when nothing else can solve the same problem to compare against |
| Sensitive to multi-modality? | Not really - a lucky bimodal sample set can still have roughly the "right" mean | Directly, via `p > 1` |
| Says anything about early/rate behaviour? | No - it's a statement about the pooled, final sample set | Yes - that's the point; plot vs. `n` (or eval count) |
| What #69 actually used it for | The one case (plain polytope, no `h`) where a ground-truth sampler exists to compare against at all | N/A yet - not run against `nlp_sampling.py`'s outputs yet, tracked for follow-up once there's a concrete sampling strategy to evaluate (#72) |

They're not competing tools - #69's check is the right one for "does the
sampler even behave correctly in the one case where correctness is
checkable," and MSTS is the right one for "how good is this sampler's actual
job (diverse constrained sampling) doing," which is a question #69's
approach structurally can't answer at all.

## Limitations

- **No absolute "good" value.** Unlike a KS test's p-value, there's no
  built-in pass/fail threshold - MSTS is a comparison tool (this sampler
  configuration vs. that one; this point in the sampling run vs. an earlier
  one), not a certificate.
- **`p=1`'s dimensionality reading is an asymptotic approximation**, not an
  exact formula - confirmed above (`9.0` vs. predicted `8.0`,  `73.0` vs.
  `64.0`), and needs enough points to be a meaningful signal at all.
- **Order-dependent, on purpose.** `minimum_spanning_tree_score_curve`
  assumes `points` is already in the order it was generated - shuffling
  first would make the "vs. n" curve meaningless as a rate-of-exploration
  signal (it would just describe a random pooled sample set, not the
  sampling process).
- **Can't distinguish a real mode from a burn-in transient.** An early
  cluster of un-mixed, still-converging samples can look like a separate
  "mode" to a `p>1` MSTS curve, the same way it would to a human eyeballing
  a scatter plot. Use alongside `per_restart_spread` and the existing
  residual/feasibility diagnostics, not as a sole signal.
- **No single right `p`.** `p=1` and `p=2` answer different questions
  (density/dimensionality vs. mode-separation); `plot_msts_curve` defaults
  to showing both rather than picking one.
