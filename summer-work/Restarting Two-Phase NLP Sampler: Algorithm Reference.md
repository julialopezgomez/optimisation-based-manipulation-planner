# Restarting Two-Phase NLP Sampler. Algorithm Guide

This document summarises the algorithm and implementation choices for the NHR / mRRT sampler, as implemented in `summer-work/nhr.py` and exercised in `summer-work/nhr_standalone_test.py`. It is intended to be a quick reference for how the algorithm maps to the code, what options are available, and how to run it.

The implementation is based on the two-phase sampling algorithm described in the following paper:  

> Toussaint, M., Braun, C.V., & Haro, J.O. (2024). NLP Sampling: Combining MCMC and NLP Methods for Diverse Constrained Sampling. ArXiv, [abs/2407.03035](https://arxiv.org/abs/2407.03035).

and its accompanying codebase: https://github.com/MarcToussaint/24-NLP-Sampling.

### Implementation Choices for Full Algorithm

```
RESTARTING TWO-PHASE NLP SAMPLER
│
├── 0. CHOOSE A RESTART SEED x
│      uniform / distance-based / direction-based
│
├── 1. PHASE I: FIND ONE FEASIBLE POINT
│      operate on F₀₁(x) = s(x)ᵀs(x)
│
│      Section 3.1 toolbox:
│      ├── choose downhill direction
│      │     ├── gradient
│      │     └── Gauss–Newton
│      ├── optionally add noise
│      │     ├── none
│      │     ├── isotropic
│      │     └── covariant / Riemannian
│      └── optionally reject steps
│            ├── always accept
│            ├── Armijo
│            └── MH
│
│      Typical choice in experiments:
│      └── Gauss–Newton downhill
│
├── 2. PHASE II: EXPLORE FROM THAT FEASIBLE POINT
│
│      A. Relaxed-energy sampling
│      │     operate on F₁μ(x)
│      │
│      │     Same Section 3.1 toolbox:
│      │     ├── Langevin
│      │     ├── MALA
│      │     ├── Riemannian Langevin
│      │     └── other MCMC combinations
│      │
│      B. Explicit constrained sampling
│            Section 3.2:
│            ├── NHR  → inequalities / feasible interior
│            └── mRRT → equalities / manifolds
│
└── 3. RESTART
       choose another seed and repeat
       → chance to discover another disconnected component
```

> **What this project actually implements**. \
> The above diagram is the paper's full design space. `nhr.py` only implements **1 (Gauss-Newton downhill)** and **2B (NHR / mRRT)** — none of Phase II-A's relaxed-energy MCMC family (Langevin/MALA/Riemannian Langevin) exists in this codebase. See `nhr_code_walkthrough.md` for exactly what was built and how it maps to the code.


## How to Run the NHR / mRRT Sampler

Maps the algorithm above (restart seed → Phase I → Phase II → repeat) to the actual code in `summer-work/nhr.py` (the library) and `summer-work/nhr_standalone_test.py` (a runnable harness against the real Panda+cap grasp problem, with no meshcat/IRIS/GCS overhead).

### Algorithm → code map

| Algorithm step | Function |
|---|---|
| 0. Choose restart seed | `choose_restart_seed` (uniform / distance / direction) |
| 1. Phase I: find one feasible point | `run_downhill_phase1` (Gauss–Newton downhill, ports Algorithm 1) |
| 2B. Phase II: explicit constrained sampling | `run_interior`, dispatching to `step_hit_and_run` (NHR) or `mrrt_step` (mRRT) |
| 3. Restart | the outer loop inside `restarting_nhr_sample` |

### Quick start

- `cd summer-work`
- Smoke test (fast, just checks nothing is broken):
  `python nhr_standalone_test.py --num-samples 20 --burn-in 10 --num-restarts 2 --quiet --no-plots`
- Real run with NHR (default) and plots:
  `python nhr_standalone_test.py --num-restarts 30 --num-samples 500`
- Same, but with mRRT instead:
  `python nhr_standalone_test.py --interior-method mRRT --num-restarts 30 --num-samples 500`
- Output goes to `summer-work/joint_samples_plots/<timestamp>/`: one PNG histogram per joint, plus `info.json` with every diagnostic.

### CLI flags (`nhr_standalone_test.py`)

| Flag | Default | Meaning |
|---|---|---|
| `--num-samples` | 500 | samples to collect **per restart chain**, after burn-in |
| `--burn-in` | 100 | interior-sampling steps discarded before recording starts |
| `--num-restarts` | 30 | number of independent chains (each with its own Phase I seed) |
| `--restart-strategy` | `distance` | how seeds are chosen: `uniform` (random), `distance` (farthest from samples so far), `direction` (least aligned with existing samples) |
| `--interior-method` | `HR` | `HR` = Nonlinear Hit-and-Run; `mRRT` = Manifold-RRT (see below) |
| `--interior-noise-sigma` | 0.1 | mRRT's fixed step length per move (ignored by HR) |
| `--equality-tol` | 1e-3 | final `\|h(x)\|` tolerance used only for the last accept/reject filter |
| `--seed` | 0 | RNG seed, for reproducibility |
| `--quiet` | off | suppress per-step progress prints |
| `--no-plots` | off | skip saving histograms (useful for quick iteration) |
| `--output-root` | `summer-work/joint_samples_plots` | where plots/`info.json` are written |

### `NHROptions` fields (the sampler's internals)

| Field                                                                  | Default                             | Meaning                                                                                                                                                            |
| ---------------------------------------------------------------------- | ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `num_samples`, `burn_in`, `thinning`                                   | 1000, 100, 1                        | as above; `thinning` keeps every k-th sample                                                                                                                       |
| `good_err_tol`                                                         | 0.01                                | a point is "good enough to record" if its total slack (violations of `g`, `h`) is below this                                                                       |
| `constraint_tol`                                                       | `None` → defaults to `good_err_tol` | tolerance for the **final** feasibility filter (`is_feasible`) — set explicitly if you want a stricter/looser final cutoff than the internal recording gate        |
| `walk_margin`                                                          | 0.1                                 | HR only: how loose the equality tube is *while walking* (looser than `equality_tol`, which governs final sample quality)                                           |
| `eps_slack_increase`                                                   | 0.05                                | HR only: how much a step's slack is allowed to get *worse* and still be accepted (the "lenient accept" from the reference code, not the paper's strict pseudocode) |
| `hit_and_run_inner_tries`                                              | 10                                  | HR only: retries per step before giving up (`exhausted_tries`)                                                                                                     |
| `interior_method`                                                      | `"HR"`                              | `"HR"` or `"mRRT"` — see decision guide below                                                                                                                      |
| `interior_noise_sigma`                                                 | 0.1                                 | mRRT only: fixed step size per move                                                                                                                                |
| `penalty_mu`, `slack_step_alpha`, `slack_max_step`, `slack_reg_lambda` | 1.0, 1.0, 0.1, 1e-2                 | Gauss-Newton cleanup step tuning (penalty weight, step size, trust-region cap, damping) — used both by Phase I and the post-step cleanup                           |
| `downhill_max_steps`                                                   | 50                                  | Phase I: max Gauss-Newton iterations before declaring failure                                                                                                      |
| `finite_difference_step`                                               | 1e-6                                | step size for numerical Jacobians, only used if you don't pass an analytic `Jg`/`Jh`                                                                               |
| `random_seed`, `verbose`                                               | 0, True                             |                                                                                                                                                                    |

### `RestartOptions` fields

| Field | Default | Meaning |
|---|---|---|
| `num_restarts` | 10 | independent chains to run |
| `strategy` | `"uniform"` | seed choice strategy (see CLI table above) |
| `candidates_per_restart` | 50 | candidate seeds tried per restart for `distance`/`direction` strategies |
| `keep_failed_phase1_seeds` | False | currently informational only |

### Extra args to `restarting_nhr_sample` (not on `NHROptions`)

| Arg | Meaning |
|---|---|
| `equality_tol` | final `\|h(x)\|` tolerance (separate from the walk's own `walk_margin`) |
| `project_samples_to_manifold` | if `True`, polish every recorded sample toward exact feasibility (both `g` and `h`) before the final filter — recommended, on by default in the standalone script |
| `projection_iters` | how many Gauss-Newton polish steps to spend per sample |

### Choosing HR vs mRRT

- **HR** (default): general-purpose, cheaper per step. Good when constraints are mostly inequalities or loosely-binding equalities.
- **mRRT**: use when you see the mixing-collapse symptom — a coordinate's within-chain spread is tiny while its spread *across restarts* is large (i.e. the pooled histogram looks fine only because restarts landed in different places, not because any one chain actually explored). This happens when a few razor-thin constraints (tight equalities/inequalities) throttle every coordinate's shared step size, even ones that are otherwise free. On the real grasp problem this was exactly the wrist/cap symptom — switching to mRRT fixed it.
- Check this yourself with `per_restart_spread(samples, restart_info, joint_index)`: compare `median within-chain std` to `across-restart std of means` for the joint you care about — they should be roughly the same order of magnitude if mixing is healthy.

### Reading the output

- Console: restart success counts, reason-code histogram (`accepted`/`mh_reject`/`empty_interval`/`exhausted_tries` for HR, `mrrt_step` for mRRT), final residual summary (`max |h|`, `max g` — should be tiny, ~1e-10 scale, if `project_samples_to_manifold=True`), per-joint spread table, wall-clock and Drake-eval-count/sample.
- `info.json` per run: full per-joint stats, options used, and every step's diagnostics — useful for re-plotting or comparing runs later.
- PNGs: one histogram per joint with mean/std/limits marked.

### Using `nhr.py` directly in your own script

```python
import nhr

opts = nhr.NHROptions(num_samples=500, burn_in=100, interior_method="mRRT", verbose=False)
restart_opts = nhr.RestartOptions(num_restarts=30, strategy="distance")

phase1 = lambda seed: nhr.run_downhill_phase1(
    seed, g=g, Jg=Jg, lower=lower, upper=upper, options=opts, h=h, Jh=Jh
)

samples, restart_info = nhr.restarting_nhr_sample(
    phase1=phase1, g=g, lower=lower, upper=upper, Jg=Jg,
    nhr_options=opts, restart_options=restart_opts,
    h=h, Jh=Jh, equality_tol=1e-3,
    project_samples_to_manifold=True, projection_iters=10,
)
```


### Relevant Repos for Original Implementation:
- https://github.com/MarcToussaint/24-NLP-Sampling
- https://github.com/MarcToussaint/rai
- https://github.com/MarcToussaint/rai-robotModels
