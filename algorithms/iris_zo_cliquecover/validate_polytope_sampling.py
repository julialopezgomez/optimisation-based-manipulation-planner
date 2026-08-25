"""
Validation harness for issue #69: sanity-check nlp_sampling.py's polytope-
only path (g = Ax - b, h = None) against Drake's actual hit-and-run sampler,
`HPolyhedron.UniformSample` (the same routine IrisZo calls internally via
`PopulateParticlesByUniformSampling` in planning/iris/iris_common.cc).

This is a narrow sanity check, not a validation of nlp_sampling.py's actual
purpose. Drake's sampler only ever knows about linear inequalities
(Ax <= b) - it has no equality/manifold support at all, so this script can
only exercise the degenerate case where nlp_sampling.py is given no h. The
part of nlp_sampling.py that samples on nonlinear equality manifolds (e.g.
grasp/cap-orientation constraints) has no Drake counterpart to compare
against; that keeps being validated the way it already is, via
nlp_sampling_standalone_test.py's mixing diagnostics against the real grasp
problem.

Both samplers are seeded from the same feasible point (the polytope's
Chebyshev center, matching Drake's own no-previous-sample convention) and
run as a single continued Markov chain, mirroring how
PopulateParticlesByUniformSampling chains particles. Comparison is by
per-dimension mean/std and a per-dimension two-sample KS test. The KS
p-values are a coarse red-flag signal, not a rigorous guarantee - both
chains are autocorrelated, not i.i.d., which makes the test anti-conservative.

Note on burn_in: nlp_sampling's step_hit_and_run takes locally-bounded
steps (~2*slack_max_step per move, default 0.1), unlike Drake's
UniformSample, which takes an exact full-chord step every call. That makes
nlp_sampling's walk a much slower (diffusive) mixer on a plain polytope -
it needs a burn_in well past its default (200) to reach the same stationary
distribution here. This isn't a correctness gap: it's expected, since the
small-step design is what lets it stay safely inside thin nonlinear
equality tubes elsewhere, which a large exact-chord jump could not do.

Usage:
    python validate_polytope_sampling.py
    python validate_polytope_sampling.py --polytope random --dim 6 --num-samples 4000
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.stats import ks_2samp

from pydrake.common import RandomGenerator
from pydrake.geometry.optimization import HPolyhedron

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nlp_sampling"))
import nlp_sampling

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


# -----------------------------------------------------------------------------
# Test polytopes
# -----------------------------------------------------------------------------

def make_box_polytope(dim: int) -> tuple[np.ndarray, np.ndarray]:
    """A simple axis-aligned box [-1, 1]^dim, as (A, b) with A @ x <= b."""
    A = np.vstack([np.eye(dim), -np.eye(dim)])
    b = np.ones(2 * dim)
    return A, b


def make_random_bounded_polytope(dim: int, seed: int, num_extra_faces: int = 6) -> tuple[np.ndarray, np.ndarray]:
    """
    A non-box bounded polytope: the [-1, 1]^dim box with a handful of random
    halfspaces cut through it, so hit-and-run actually has to deal with
    direction-dependent geometry instead of a trivially-separable box.
    Boundedness is guaranteed by keeping the box's own faces in (A, b).
    """
    rng = np.random.default_rng(seed)
    A_box, b_box = make_box_polytope(dim)

    normals = rng.normal(size=(num_extra_faces, dim))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    # Cut each face partway through the box (fraction of the box's half-diagonal),
    # so it removes a real chunk of volume without leaving an empty set.
    offsets = rng.uniform(0.3, 0.7, size=num_extra_faces) * np.sqrt(dim)

    A = np.vstack([A_box, normals])
    b = np.concatenate([b_box, offsets])
    return A, b


# -----------------------------------------------------------------------------
# Sampling: Drake's HPolyhedron.UniformSample, chained like
# PopulateParticlesByUniformSampling (iris_common.cc)
# -----------------------------------------------------------------------------

def sample_drake_hit_and_run(
    A: np.ndarray, b: np.ndarray, num_samples: int, mixing_steps: int, seed: int,
) -> np.ndarray:
    polyhedron = HPolyhedron(A, b)
    generator = RandomGenerator(seed)

    samples = np.empty((num_samples, A.shape[1]))
    samples[0] = polyhedron.UniformSample(generator, mixing_steps)
    for j in range(1, num_samples):
        samples[j] = polyhedron.UniformSample(generator, samples[j - 1], mixing_steps)
    return samples


def sample_drake_hit_and_run_multi_seed(
    A: np.ndarray, b: np.ndarray, num_samples_per_seed: int, mixing_steps: int, seeds: list[int],
) -> np.ndarray:
    chunks = [
        sample_drake_hit_and_run(A, b, num_samples_per_seed, mixing_steps, seed)
        for seed in seeds
    ]
    return np.vstack(chunks)


# -----------------------------------------------------------------------------
# Sampling: nlp_sampling.py restricted to g = Ax - b, h = None
# -----------------------------------------------------------------------------

def sample_nlp_sampling(
    A: np.ndarray, b: np.ndarray, num_samples: int, burn_in: int, seed: int,
    thinning: int = 1, slack_max_step: float | None = None,
) -> np.ndarray:
    dim = A.shape[1]
    polyhedron = HPolyhedron(A, b)
    x0 = polyhedron.ChebyshevCenter()

    # Box bounds are a hard requirement of nlp_sampling's API, used in
    # addition to g. Make them wide enough that they never bind - only the
    # polytope's own inequalities (g) should ever constrain the walk, to
    # match what HPolyhedron.UniformSample sees.
    radius = np.linalg.norm(b, ord=np.inf) + np.linalg.norm(A, ord=np.inf) + 10.0
    lower = x0 - radius
    upper = x0 + radius

    def g(x):
        return A @ x - b

    def Jg(_x):
        return A

    kwargs = {}
    if slack_max_step is not None:
        kwargs["slack_max_step"] = slack_max_step
        kwargs["delta_max"] = slack_max_step

    options = nlp_sampling.NHROptions(
        num_samples=num_samples, burn_in=burn_in, thinning=thinning,
        random_seed=seed, verbose=False, **kwargs,
    )
    samples, _diagnostics = nlp_sampling.nhr_sample(
        x0=x0, g=g, lower=lower, upper=upper, Jg=Jg, options=options,
    )
    return samples


def sample_nlp_sampling_multi_seed(
    A: np.ndarray, b: np.ndarray, num_samples_per_seed: int, burn_in: int,
    seeds: list[int], thinning: int = 1, slack_max_step: float | None = None,
) -> np.ndarray:
    """Pool independent chains across seeds - reduces single-chain
    autocorrelation noise in the comparison, and surfaces run-to-run
    variance (e.g. a chain that hasn't mixed will disagree seed-to-seed,
    not just against Drake)."""
    chunks = [
        sample_nlp_sampling(A, b, num_samples_per_seed, burn_in, seed, thinning, slack_max_step)
        for seed in seeds
    ]
    return np.vstack(chunks)


# -----------------------------------------------------------------------------
# Comparison
# -----------------------------------------------------------------------------

def compare(drake_samples: np.ndarray, nlp_samples: np.ndarray) -> list[dict]:
    dim = drake_samples.shape[1]
    rows = []
    for d in range(dim):
        drake_col = drake_samples[:, d]
        nlp_col = nlp_samples[:, d]
        ks_stat, ks_p = ks_2samp(drake_col, nlp_col)
        rows.append({
            "dim": d,
            "drake_mean": float(np.mean(drake_col)), "drake_std": float(np.std(drake_col)),
            "nlp_mean": float(np.mean(nlp_col)), "nlp_std": float(np.std(nlp_col)),
            "ks_stat": float(ks_stat), "ks_p": float(ks_p),
        })
    return rows


def print_report(rows: list[dict]) -> None:
    print(f"{'dim':>3} | {'drake mean':>10} {'drake std':>10} | {'nlp mean':>10} {'nlp std':>10} "
          f"| {'KS stat':>8} {'KS p':>8}")
    for row in rows:
        flag = "  <-- check" if row["ks_p"] < 0.01 else ""
        print(f"{row['dim']:>3} | {row['drake_mean']:>10.4f} {row['drake_std']:>10.4f} "
              f"| {row['nlp_mean']:>10.4f} {row['nlp_std']:>10.4f} "
              f"| {row['ks_stat']:>8.4f} {row['ks_p']:>8.4f}{flag}")
    num_flagged = sum(1 for row in rows if row["ks_p"] < 0.01)
    print(f"\n{num_flagged}/{len(rows)} dimensions flagged (KS p < 0.01). "
          "Chains are autocorrelated, not i.i.d., so treat this as a coarse "
          "red-flag signal, not a rigorous test.")


def save_plots(drake_samples: np.ndarray, nlp_samples: np.ndarray, output_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dim = drake_samples.shape[1]
    output_dir.mkdir(parents=True, exist_ok=True)
    ncols = min(dim, 4)
    nrows = int(np.ceil(dim / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3 * nrows), squeeze=False)
    for d in range(dim):
        ax = axes[d // ncols][d % ncols]
        ax.hist(drake_samples[:, d], bins=40, alpha=0.5, density=True, label="Drake HR", color="#4c78a8")
        ax.hist(nlp_samples[:, d], bins=40, alpha=0.5, density=True, label="nlp_sampling", color="#f58518")
        ax.set_title(f"dim {d}")
        ax.legend(fontsize=8)
    for d in range(dim, nrows * ncols):
        axes[d // ncols][d % ncols].axis("off")
    fig.tight_layout()
    fig.savefig(output_dir / "comparison.png", dpi=150)
    plt.close(fig)
    print(f"Saved comparison plot to {output_dir / 'comparison.png'}")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def per_seed_spread(pooled_samples: np.ndarray, num_seeds: int, dim_idx: int) -> tuple[float, float]:
    """Median within-chain std vs. across-chain std-of-means for one
    dimension, mirroring nlp_sampling.per_restart_spread's mixing
    diagnostic: if a chain hasn't mixed, within-chain std is small even
    though the pooled histogram looks fine (because seeds landed in
    different places), not because any one chain actually explored."""
    chains = np.array_split(pooled_samples[:, dim_idx], num_seeds)
    within_std = np.median([np.std(c) for c in chains])
    across_std = np.std([np.mean(c) for c in chains])
    return float(within_std), float(across_std)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--polytope", choices=["box", "random", "both"], default="both")
    parser.add_argument("--dim", type=int, default=4)
    parser.add_argument("--num-samples", type=int, default=3000, help="per seed")
    parser.add_argument("--mixing-steps", type=int, default=50, help="Drake HPolyhedron.UniformSample's mixing_steps")
    parser.add_argument("--burn-in", type=int, default=1000, help="nlp_sampling's burn_in, per seed")
    parser.add_argument("--thinning", type=int, default=1, help="nlp_sampling's thinning")
    parser.add_argument("--slack-max-step", type=float, default=None,
                         help="Override nlp_sampling's step size (default 0.1). Larger values mix faster on "
                              "plain polytopes but would be unsafe near thin equality manifolds elsewhere.")
    parser.add_argument("--num-seeds", type=int, default=3, help="independent chains pooled per sampler")
    parser.add_argument("--seed", type=int, default=0, help="base seed; chains use seed, seed+1, ...")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--output-root", type=str, default="artifacts/iris_zo_validation")
    args = parser.parse_args()

    polytopes = {}
    if args.polytope in ("box", "both"):
        polytopes["box"] = make_box_polytope(args.dim)
    if args.polytope in ("random", "both"):
        polytopes["random"] = make_random_bounded_polytope(args.dim, seed=args.seed)

    seeds = [args.seed + i for i in range(args.num_seeds)]
    run_dir = PROJECT_ROOT / args.output_root / datetime.now().strftime("%Y%m%d_%H%M%S")

    for name, (A, b) in polytopes.items():
        print(f"\n=== polytope: {name} (dim={args.dim}, {args.num_seeds} seeds pooled) ===")
        drake_samples = sample_drake_hit_and_run_multi_seed(
            A, b, num_samples_per_seed=args.num_samples, mixing_steps=args.mixing_steps, seeds=seeds,
        )
        nlp_samples = sample_nlp_sampling_multi_seed(
            A, b, num_samples_per_seed=args.num_samples, burn_in=args.burn_in, seeds=seeds,
            thinning=args.thinning, slack_max_step=args.slack_max_step,
        )
        rows = compare(drake_samples, nlp_samples)
        print_report(rows)

        print(f"\n{'dim':>3} | {'nlp within-chain std':>21} | {'nlp across-seed std-of-means':>29}")
        for d in range(args.dim):
            within_std, across_std = per_seed_spread(nlp_samples, args.num_seeds, d)
            flag = "  <-- not mixing (within << across)" if within_std < 0.5 * across_std else ""
            print(f"{d:>3} | {within_std:>21.4f} | {across_std:>29.4f}{flag}")

        if not args.no_plots:
            save_plots(drake_samples, nlp_samples, run_dir / name)


if __name__ == "__main__":
    main()
