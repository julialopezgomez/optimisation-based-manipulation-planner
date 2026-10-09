"""
Demo for #45: runs the actual restarting_nhr_sample machinery (not just raw
point clouds, see msts_metric_reference.md's synthetic sanity checks) on a
small two-mode constrained problem, and plots the resulting MSTS curve.

The feasible set is g(x) = (x0^2 - a^2)^2 + x1^2 - b^2 <= 0: a classic
"double-well" shape with two disjoint, disk-like feasible blobs centered at
(-a, 0) and (a, 0) when b is small relative to a. Smooth and differentiable
everywhere (unlike a literal union of two disks), so nlp_sampling.py's
Gauss-Newton downhill Phase 1 and interior sampling handle it natively, no
special-casing needed - a real (if small) instance of the "did the sampler
find every mode" question MSTS exists to answer.

Usage:
    python msts_demo.py
    python msts_demo.py --num-restarts 60 --restart-strategy uniform
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


def g(x, a=3.0, b=1.0):
    return np.array([(x[0] ** 2 - a ** 2) ** 2 + x[1] ** 2 - b ** 2])


def Jg(x, a=3.0):
    return np.array([[4.0 * x[0] * (x[0] ** 2 - a ** 2), 2.0 * x[1]]])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-restarts", type=int, default=40)
    parser.add_argument("--restart-strategy", choices=["uniform", "distance", "direction"], default="distance")
    parser.add_argument("--num-samples", type=int, default=50, help="per restart")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    lower, upper = np.array([-5.0, -3.0]), np.array([5.0, 3.0])

    nhr_options = nlp_sampling.NHROptions(
        num_samples=args.num_samples, burn_in=20, random_seed=args.seed, verbose=False)
    restart_options = nlp_sampling.RestartOptions(
        num_restarts=args.num_restarts, strategy=args.restart_strategy, random_seed=args.seed)

    phase1 = lambda seed: nlp_sampling.run_downhill_phase1(
        seed, g=g, Jg=Jg, lower=lower, upper=upper, options=nhr_options)

    samples, restart_info = nlp_sampling.restarting_nhr_sample(
        phase1=phase1, g=g, lower=lower, upper=upper, Jg=Jg,
        nhr_options=nhr_options, restart_options=restart_options,
    )

    print(f"{len(samples)} samples from {args.num_restarts} restarts (strategy={args.restart_strategy})")
    found_left = np.any(samples[:, 0] < 0)
    found_right = np.any(samples[:, 0] > 0)
    print(f"found left well (x0<0): {found_left}   found right well (x0>0): {found_right}")

    ns, scores_p1 = nlp_sampling.minimum_spanning_tree_score_curve(samples, p=1.0, num_checkpoints=15)
    _, scores_p2 = nlp_sampling.minimum_spanning_tree_score_curve(samples, p=2.0, num_checkpoints=15)
    print("\nn:          ", ns)
    print("MSTS_1(D_n):", np.round(scores_p1, 2))
    print("MSTS_2(D_n):", np.round(scores_p2, 2))
    if found_left and found_right:
        jump_idx = int(np.argmax(np.diff(scores_p2))) + 1
        print(f"\nMSTS_2 jumps sharply at n={ns[jump_idx]} - both wells were being explored "
              f"before this many samples were pooled, consistent with 'found_left and found_right' above.")

    if not args.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        axes[0].scatter(samples[:, 0], samples[:, 1], s=8, alpha=0.6)
        axes[0].set_title("Samples (double-well feasible set)")
        axes[0].set_xlabel("x0")
        axes[0].set_ylabel("x1")
        axes[0].grid(True, alpha=0.25)

        nlp_sampling.plot_msts_curve(axes[1], samples, p_values=(1.0, 2.0), num_checkpoints=15)

        fig.tight_layout()
        out_path = PROJECT_ROOT / "artifacts" / "msts_demo.png"
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=150)
        print(f"\nSaved plot to {out_path}")


if __name__ == "__main__":
    main()
