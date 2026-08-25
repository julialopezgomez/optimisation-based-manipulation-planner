"""
The actual #46 comparison: regular-FK sampling (#78,
tc_space_collision_scenario.py) vs. TC-space rational-FK sampling (#79,
tc_space_collision_scenario_fast.py - the fast numeric evaluator, not the
Drake-symbolic one in tc_space_collision_scenario_rational.py; the latter
is ~230x slower per call purely from Drake's per-call symbolic
Polynomial/RationalFunction.Evaluate overhead, not anything intrinsic to
rational algebra - validated to agree with it to 1e-15, see
tc_space_collision_scenario_fast.py's own benchmark/validation) on the
identical physical scenario (same panda_link5-vs-sphere obstacle
constraint, cross-validated to agree exactly with the regular-FK
constraint too).

Methodology, all raised in review and each changed the result at some
point - see msts_metric_reference.md and #46 for the full history:

1. Both sample sets are compared in a COMMON space: q-space. s and q are
   different coordinate scales (s = tan((q-q_star)/2)), so a raw MSTS
   comparison of s-space samples against q-space samples would compare
   different units, not a real finding. TC-space's s-space samples are
   mapped back to q via RationalForwardKinematics.ComputeQValue before
   scoring - the WALK still happens in s-space (that's the actual
   hypothesis under test), only the EVALUATION metric is put on common
   footing.
2. Step size is normalized to the same FRACTION of each space's own
   domain diagonal, not the same raw number - the same absolute step
   would cover a different fraction of each (differently-sized) domain,
   an unintended advantage for whichever domain happens to be smaller.
3. MSTS is reported at BOTH p=1 and p=2 - p=1 is a plain diversity/growth
   signal, p=2 is what actually isolates mode-coverage (cross-mode edges
   dominate over within-mode filling only for p>1). Report both rather
   than picking one, since (per review discussion) MST-length metrics can
   in principle be inflated by poor within-mode mixing (several small,
   restart-seeded clusters bridged by medium-length edges can score
   similarly to genuine spread) - neither p alone is a complete story,
   and this is flagged explicitly in the output rather than glossed over.

Runs multiple seeds (not just one) so the early-jump-then-convergence
pattern from earlier single-seed runs can be checked for consistency
rather than trusted from n=1.

Saves every run to its own timestamped directory under
artifacts/tc_space_experiments/ - nothing gets overwritten between runs.

Usage:
    python tc_space_vs_regular_comparison.py
    python tc_space_vs_regular_comparison.py --num-seeds 5 --num-restarts 60 --num-samples 50
"""
import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling
from tc_space_collision_scenario import build_scene as build_regular_scene
from tc_space_collision_scenario import make_collision_constraint as make_regular_constraint
from tc_space_collision_scenario_fast import make_fast_rational_collision_constraint
from tc_space_collision_scenario_rational import build_rational_scene, _validate_against_regular_fk

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


class CallCounter:
    """Same pattern as #75/nlp_sampling_standalone_test.py's CallCounter."""
    def __init__(self, fn):
        self.fn = fn
        self.count = 0

    def __call__(self, x):
        self.count += 1
        return self.fn(x)


def run_sampler(g, Jg, lower, upper, num_restarts, num_samples, seed, slack_max_step):
    g_counted = CallCounter(g)
    nhr_options = nlp_sampling.NHROptions(
        num_samples=num_samples, burn_in=20, random_seed=seed, verbose=False,
        slack_max_step=slack_max_step, delta_max=slack_max_step,
    )
    restart_options = nlp_sampling.RestartOptions(num_restarts=num_restarts, strategy="distance", random_seed=seed)
    phase1 = lambda s: nlp_sampling.run_downhill_phase1(
        s, g=g_counted, Jg=Jg, lower=lower, upper=upper, options=nhr_options)

    t0 = time.perf_counter()
    samples, restart_info = nlp_sampling.restarting_nhr_sample(
        phase1=phase1, g=g_counted, lower=lower, upper=upper, Jg=Jg,
        nhr_options=nhr_options, restart_options=restart_options,
    )
    elapsed = time.perf_counter() - t0
    return samples, elapsed, g_counted.count, restart_info


def map_tc_samples_to_q(s_samples, rational_scene):
    rat_fk, q_star = rational_scene["rat_fk"], rational_scene["q_star"]
    idx_j2, idx_j4 = rational_scene["idx_j2"], rational_scene["idx_j4"]
    idx_s_j2, idx_s_j4 = rational_scene["idx_s_j2"], rational_scene["idx_s_j4"]
    q_samples = np.empty_like(s_samples)
    for i, s_free in enumerate(s_samples):
        s_full = np.zeros(len(rat_fk.s()))
        s_full[idx_s_j2], s_full[idx_s_j4] = s_free[0], s_free[1]
        q_full = rat_fk.ComputeQValue(s_full, q_star)
        q_samples[i] = [q_full[idx_j2], q_full[idx_j4]]
    return q_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-restarts", type=int, default=60)
    parser.add_argument("--num-samples", type=int, default=50, help="per restart")
    parser.add_argument("--num-seeds", type=int, default=5)
    parser.add_argument("--base-seed", type=int, default=0)
    args = parser.parse_args()

    run_dir = PROJECT_ROOT / "artifacts" / "tc_space_experiments" / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving all output to {run_dir}\n")

    max_err = _validate_against_regular_fk(build_rational_scene(), build_regular_scene())
    assert max_err < 1e-8, "TC-space constraint doesn't match regular FK - fix before trusting anything below"
    print(f"Cross-validation OK (max error {max_err:.1e}).\n")

    regular_scene = build_regular_scene()
    g_reg, Jg_reg, lower_reg, upper_reg, embed = make_regular_constraint(regular_scene)
    rational_scene = build_rational_scene()
    g_tc, Jg_tc, lower_tc, upper_tc = make_fast_rational_collision_constraint(rational_scene)

    diag_reg = float(np.linalg.norm(upper_reg - lower_reg))
    diag_tc = float(np.linalg.norm(upper_tc - lower_tc))
    step_fraction = 0.1 / diag_reg
    step_reg = step_fraction * diag_reg
    step_tc = step_fraction * diag_tc
    print(f"domain diagonals: q-space={diag_reg:.3f}, s-space={diag_tc:.3f} (ratio {diag_reg / diag_tc:.2f}x)")
    print(f"step sizes normalized accordingly: q-space={step_reg:.4f}, s-space={step_tc:.4f}\n")

    seeds = [args.base_seed + i for i in range(args.num_seeds)]
    per_seed_results = []

    for seed in seeds:
        print(f"--- seed {seed} ---")
        q_samples_regular, elapsed_regular, evals_regular, restart_info_reg = run_sampler(
            g_reg, Jg_reg, lower_reg, upper_reg, args.num_restarts, args.num_samples, seed, step_reg)
        s_samples_tc, elapsed_tc, evals_tc, restart_info_tc = run_sampler(
            g_tc, Jg_tc, lower_tc, upper_tc, args.num_restarts, args.num_samples, seed, step_tc)
        q_samples_from_tc = map_tc_samples_to_q(s_samples_tc, rational_scene)

        print(f"  regular: {len(q_samples_regular)} samples, {elapsed_regular:.3f}s, {evals_regular} g-evals")
        print(f"  TC-space: {len(s_samples_tc)} samples, {elapsed_tc:.3f}s, {evals_tc} g-evals "
              f"(wall-clock ratio {elapsed_tc / elapsed_regular:.2f}x)")

        from scipy.cluster.vq import kmeans2
        both_found = {}
        for name, samples in [("regular", q_samples_regular), ("TC-space", s_samples_tc)]:
            _, labels = kmeans2(samples, 2, seed=0, minit="++")
            n0, n1 = int(np.sum(labels == 0)), int(np.sum(labels == 1))
            both_found[name] = min(n0, n1) > 0.1 * len(samples)
            if not both_found[name]:
                print(f"  WARNING: {name} sampler may have missed a region this seed ({n0}/{n1})")

        ns_reg, msts2_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=2.0, num_checkpoints=12)
        ns_tc, msts2_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=2.0, num_checkpoints=12)
        _, msts1_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=1.0, num_checkpoints=12)
        _, msts1_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=1.0, num_checkpoints=12)

        per_seed_results.append(dict(
            seed=seed,
            q_samples_regular=q_samples_regular, q_samples_from_tc=q_samples_from_tc,
            restart_info_reg=restart_info_reg, restart_info_tc=restart_info_tc,
            elapsed_regular=elapsed_regular, elapsed_tc=elapsed_tc,
            evals_regular=evals_regular, evals_tc=evals_tc,
            ns_reg=ns_reg, msts1_reg=msts1_reg, msts2_reg=msts2_reg,
            ns_tc=ns_tc, msts1_tc=msts1_tc, msts2_tc=msts2_tc,
            both_found=both_found,
        ))

    # Summary across seeds, at the max common checkpoint index reached by
    # both curves' own final row for each seed (see class docstring point 3
    # for why both p are reported).
    print("\n=== Summary across seeds (final MSTS values, both p=1 and p=2) ===")
    print(f"{'seed':>5} | {'reg MSTS_1':>10} | {'tc MSTS_1':>10} | {'reg MSTS_2':>10} | {'tc MSTS_2':>10} | "
          f"{'wall ratio':>10}")
    for r in per_seed_results:
        print(f"{r['seed']:>5} | {r['msts1_reg'][-1]:>10.3f} | {r['msts1_tc'][-1]:>10.3f} | "
              f"{r['msts2_reg'][-1]:>10.3f} | {r['msts2_tc'][-1]:>10.3f} | "
              f"{r['elapsed_tc'] / r['elapsed_regular']:>9.2f}x")

    msts1_reg_finals = [r['msts1_reg'][-1] for r in per_seed_results]
    msts1_tc_finals = [r['msts1_tc'][-1] for r in per_seed_results]
    msts2_reg_finals = [r['msts2_reg'][-1] for r in per_seed_results]
    msts2_tc_finals = [r['msts2_tc'][-1] for r in per_seed_results]
    print(f"\nmedian final MSTS_1: regular={np.median(msts1_reg_finals):.3f}, TC-space={np.median(msts1_tc_finals):.3f}")
    print(f"median final MSTS_2: regular={np.median(msts2_reg_finals):.3f}, TC-space={np.median(msts2_tc_finals):.3f}")
    tc_wins_p2 = sum(1 for r in per_seed_results if r['msts2_tc'][-1] > r['msts2_reg'][-1])
    print(f"TC-space MSTS_2 higher than regular in {tc_wins_p2}/{len(per_seed_results)} seeds")

    # Plots: MSTS_1 and MSTS_2 curves for every seed (thin lines, shows
    # spread across seeds directly rather than hiding it in a mean).
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for r in per_seed_results:
        axes[0].plot(r["ns_reg"], r["msts1_reg"], color="#4c78a8", alpha=0.6, marker="o", markersize=2)
        axes[0].plot(r["ns_tc"], r["msts1_tc"], color="#f58518", alpha=0.6, marker="o", markersize=2)
        axes[1].plot(r["ns_reg"], r["msts2_reg"], color="#4c78a8", alpha=0.6, marker="o", markersize=2)
        axes[1].plot(r["ns_tc"], r["msts2_tc"], color="#f58518", alpha=0.6, marker="o", markersize=2)
    for ax, p in zip(axes, [1, 2]):
        ax.set_xlabel("number of samples (n)")
        ax.set_ylabel(f"MSTS_{p} (q-space)")
        ax.set_title(f"p={p} ({'diversity' if p == 1 else 'mode coverage'}), {len(seeds)} seeds")
        ax.grid(True, alpha=0.25)
        ax.plot([], [], color="#4c78a8", label="regular FK")
        ax.plot([], [], color="#f58518", label="TC-space")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(run_dir / "msts_all_seeds.png", dpi=150)
    print(f"\nSaved {run_dir / 'msts_all_seeds.png'}")

    # Restart-colored diagnostic for the first seed only (illustrative, not
    # meant to be re-derived per seed - see the module docstring).
    r0 = per_seed_results[0]
    restart_ids_reg = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in r0["restart_info_reg"] if info["status"] == "success"])
    restart_ids_tc = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in r0["restart_info_tc"] if info["status"] == "success"])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].scatter(r0["q_samples_regular"][:, 0], r0["q_samples_regular"][:, 1], c=restart_ids_reg, cmap="tab20", s=10)
    axes[0].set_title(f"Regular FK, colored by restart id (seed {r0['seed']})")
    axes[1].scatter(r0["q_samples_from_tc"][:, 0], r0["q_samples_from_tc"][:, 1], c=restart_ids_tc, cmap="tab20", s=10)
    axes[1].set_title(f"TC-space (mapped to q), colored by restart id (seed {r0['seed']})")
    for ax in axes:
        ax.set_xlabel("joint2 (q)")
        ax.set_ylabel("joint4 (q)")
        ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(run_dir / "restart_coloring_seed0.png", dpi=150)
    print(f"Saved {run_dir / 'restart_coloring_seed0.png'}")

    summary = {
        "args": vars(args),
        "seeds": seeds,
        "step_reg": step_reg, "step_tc": step_tc,
        "diag_reg": diag_reg, "diag_tc": diag_tc,
        "per_seed": [
            {k: v for k, v in r.items() if k not in (
                "q_samples_regular", "q_samples_from_tc", "restart_info_reg", "restart_info_tc",
                "ns_reg", "ns_tc", "msts1_reg", "msts1_tc", "msts2_reg", "msts2_tc")}
            | {"msts1_reg_final": float(r["msts1_reg"][-1]), "msts1_tc_final": float(r["msts1_tc"][-1]),
               "msts2_reg_final": float(r["msts2_reg"][-1]), "msts2_tc_final": float(r["msts2_tc"][-1])}
            for r in per_seed_results
        ],
        "median_msts1_regular": float(np.median(msts1_reg_finals)),
        "median_msts1_tc": float(np.median(msts1_tc_finals)),
        "median_msts2_regular": float(np.median(msts2_reg_finals)),
        "median_msts2_tc": float(np.median(msts2_tc_finals)),
    }
    with open(run_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"Saved {run_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
