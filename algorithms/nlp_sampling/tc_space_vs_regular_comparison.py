"""
The actual #46 comparison: regular-FK sampling (#78,
tc_space_collision_scenario.py) vs. TC-space rational-FK sampling (#79,
tc_space_collision_scenario_rational.py) on the identical physical
scenario (same panda_link5-vs-sphere obstacle constraint, cross-validated
to agree exactly - see tc_space_collision_scenario_rational.py's own
validation step).

Both sample sets are compared in a COMMON space: q-space. This matters -
s and q are different coordinate scales (s = tan((q-q_star)/2)), so a
raw MSTS comparison of s-space samples against q-space samples would be
comparing different units, not a real finding about which sampler covers
more of the actual, physical configuration space. The TC-space run's
s-space samples are mapped back to q via RationalForwardKinematics.
ComputeQValue before scoring - the WALK still happens in s-space (that's
the actual hypothesis under test), only the EVALUATION metric is put on
common footing.

Usage:
    python tc_space_vs_regular_comparison.py
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling
from tc_space_collision_scenario import build_scene as build_regular_scene
from tc_space_collision_scenario import make_collision_constraint as make_regular_constraint
from tc_space_collision_scenario_rational import build_rational_scene, make_rational_collision_constraint

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


class CallCounter:
    """Same pattern as #75/nlp_sampling_standalone_test.py's CallCounter."""
    def __init__(self, fn):
        self.fn = fn
        self.count = 0

    def __call__(self, x):
        self.count += 1
        return self.fn(x)


def run_sampler(g, Jg, lower, upper, num_restarts, num_samples, seed):
    g_counted = CallCounter(g)
    nhr_options = nlp_sampling.NHROptions(num_samples=num_samples, burn_in=20, random_seed=seed, verbose=False)
    restart_options = nlp_sampling.RestartOptions(num_restarts=num_restarts, strategy="distance", random_seed=seed)
    phase1 = lambda s: nlp_sampling.run_downhill_phase1(
        s, g=g_counted, Jg=Jg, lower=lower, upper=upper, options=nhr_options)

    t0 = time.perf_counter()
    samples, restart_info = nlp_sampling.restarting_nhr_sample(
        phase1=phase1, g=g_counted, lower=lower, upper=upper, Jg=Jg,
        nhr_options=nhr_options, restart_options=restart_options,
    )
    elapsed = time.perf_counter() - t0
    return samples, elapsed, g_counted.count


def main():
    num_restarts, num_samples, seed = 60, 50, 0

    from tc_space_collision_scenario_rational import _validate_against_regular_fk
    max_err = _validate_against_regular_fk(build_rational_scene(), build_regular_scene())
    assert max_err < 1e-8, "TC-space constraint no longer matches regular FK - fix before trusting anything below"
    print(f"Cross-validation OK (max error {max_err:.1e}). Running both samplers.\n")

    print("=== Regular FK (q-space) ===")
    regular_scene = build_regular_scene()
    g_reg, Jg_reg, lower_reg, upper_reg, embed = make_regular_constraint(regular_scene)
    q_samples_regular, elapsed_regular, evals_regular = run_sampler(
        g_reg, Jg_reg, lower_reg, upper_reg, num_restarts, num_samples, seed)
    print(f"{len(q_samples_regular)} samples, {elapsed_regular:.3f}s "
          f"({len(q_samples_regular) / elapsed_regular:.0f} samples/sec), "
          f"{evals_regular} g-evals ({evals_regular / max(len(q_samples_regular), 1):.1f}/sample)")

    print("\n=== TC-space rational FK (s-space) ===")
    rational_scene = build_rational_scene()
    g_tc, Jg_tc, lower_tc, upper_tc = make_rational_collision_constraint(rational_scene)
    s_samples_tc, elapsed_tc, evals_tc = run_sampler(
        g_tc, Jg_tc, lower_tc, upper_tc, num_restarts, num_samples, seed)
    print(f"{len(s_samples_tc)} samples, {elapsed_tc:.3f}s "
          f"({len(s_samples_tc) / elapsed_tc:.0f} samples/sec), "
          f"{evals_tc} g-evals ({evals_tc / max(len(s_samples_tc), 1):.1f}/sample)")

    print(f"\ntiming ratio (TC-space / regular): {elapsed_tc / elapsed_regular:.2f}x "
          f"wall-clock, {evals_tc / max(evals_regular, 1):.2f}x g-evals")

    from scipy.cluster.vq import kmeans2
    for name, samples in [("regular", q_samples_regular), ("TC-space", s_samples_tc)]:
        _, labels = kmeans2(samples, 2, seed=0, minit="++")
        n0, n1 = int(np.sum(labels == 0)), int(np.sum(labels == 1))
        assert min(n0, n1) > 0.1 * len(samples), f"{name} sampler missed one of the two disjoint regions ({n0}/{n1})"

    if len(s_samples_tc) != len(q_samples_regular):
        print(f"\nNote: unequal final sample counts ({len(q_samples_regular)} regular vs. {len(s_samples_tc)} "
              "TC-space, same restart budget) - the MSTS comparison below is only apples-to-apples up to "
              "min(n), the TC-space run's own last row. Don't read anything into which curve is higher past "
              "that point - one series simply stops.")

    # Map TC-space samples back to q-space for a common-units MSTS comparison.
    rat_fk, q_star = rational_scene["rat_fk"], rational_scene["q_star"]
    idx_j2, idx_j4 = rational_scene["idx_j2"], rational_scene["idx_j4"]
    idx_s_j2, idx_s_j4 = rational_scene["idx_s_j2"], rational_scene["idx_s_j4"]

    q_samples_from_tc = np.empty_like(s_samples_tc)
    for i, s_free in enumerate(s_samples_tc):
        s_full = np.zeros(len(rat_fk.s()))
        s_full[idx_s_j2], s_full[idx_s_j4] = s_free[0], s_free[1]
        q_full = rat_fk.ComputeQValue(s_full, q_star)
        q_samples_from_tc[i] = [q_full[idx_j2], q_full[idx_j4]]

    print("\n=== MSTS_1(D_n), both in q-space (common units) ===")
    ns_reg, scores_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=1.0, num_checkpoints=12)
    ns_tc, scores_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=1.0, num_checkpoints=12)
    print(f"{'n':>6} | {'regular':>10} | {'TC-space':>10}")
    for n_r, s_r, n_t, s_t in zip(ns_reg, scores_reg, ns_tc, scores_tc):
        print(f"{n_r:>6} | {s_r:>10.3f} | {s_t:>10.3f}  (n_tc={n_t})")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].scatter(q_samples_regular[:, 0], q_samples_regular[:, 1], s=8, alpha=0.5, label="regular FK", color="#4c78a8")
    axes[0].scatter(q_samples_from_tc[:, 0], q_samples_from_tc[:, 1], s=8, alpha=0.5, label="TC-space (mapped to q)", color="#f58518")
    axes[0].set_xlabel("joint2 (q)")
    axes[0].set_ylabel("joint4 (q)")
    axes[0].set_title("Samples, both mapped to q-space")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(ns_reg, scores_reg, marker="o", markersize=3, label="regular FK", color="#4c78a8")
    axes[1].plot(ns_tc, scores_tc, marker="o", markersize=3, label="TC-space", color="#f58518")
    axes[1].set_xlabel("number of samples (n)")
    axes[1].set_ylabel("MSTS_1 (q-space)")
    axes[1].set_title("Coverage rate, common q-space units")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    fig.tight_layout()
    out_path = PROJECT_ROOT / "artifacts" / "tc_space_vs_regular_comparison.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved plot to {out_path}")


if __name__ == "__main__":
    main()
