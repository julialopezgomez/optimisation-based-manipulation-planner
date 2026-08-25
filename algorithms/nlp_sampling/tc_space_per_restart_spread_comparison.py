"""
Separate, more surgical test than tc_space_vs_regular_comparison.py:
isolates within-chain interior-sampling mixing quality specifically,
stripped of the mode-discovery/coverage confound that dominates the MSTS
comparison. Same scenario, same normalized step sizes, same restart/NHR
random seeds for both samplers (regular FK vs. TC-space, #78 vs. #79/
tc_space_collision_scenario_fast.py) - restart index i in each sampler's
own run corresponds to the same position in that seed's PRNG stream, not
necessarily the same physical seed point (the two samplers' restart-seed
selection depends on their own, differently-shaped domains), so this
compares "how far did the i-th restart's own chain wander" between the
two samplers, paired by restart index rather than pooled.

TC-space's per-restart samples are mapped back to q-space
(RationalForwardKinematics.ComputeQValue) before computing spread, so
both are compared in the same physical units - same reasoning as the MSTS
comparison's common-space fix.

This is a paired comparison, not just two pooled distributions: pairing
by restart index controls for "this particular restart happened to land
somewhere easy/hard to explore" the same way a paired t-test controls for
per-subject variation - a real methodological improvement over eyeballing
two separate scatter plots, which is all the mode-discovery comparison
could offer for this specific question.

Usage:
    python tc_space_per_restart_spread_comparison.py
    python tc_space_per_restart_spread_comparison.py --num-seeds 5 --num-restarts 60
"""
import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling
from tc_space_collision_scenario import build_scene as build_regular_scene
from tc_space_collision_scenario import make_collision_constraint as make_regular_constraint
from tc_space_collision_scenario_fast import make_fast_rational_collision_constraint
from tc_space_collision_scenario_rational import build_rational_scene, _validate_against_regular_fk
from tc_space_vs_regular_comparison import map_tc_samples_to_q, run_sampler

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


def per_restart_spread_q(samples: np.ndarray, restart_info: list[dict]) -> dict:
    """Within-chain std (Euclidean norm over both dims) for each
    successful restart, keyed by restart index. Same idea as
    nlp_sampling.per_restart_spread, generalized to a single scalar
    spread per restart (norm of the per-dimension std) instead of one
    dimension at a time, since this scenario only has 2 dimensions and a
    single "how much did this chain explore" number is what's being
    compared here."""
    offset = 0
    spreads = {}
    for info in restart_info:
        if info["status"] != "success":
            continue  # phase1_failed/nhr_failed entries have no "num_samples" key at all
        n = info["num_samples"]
        chain = samples[offset:offset + n]
        offset += n
        if n <= 1:
            continue
        spreads[info["restart"]] = float(np.linalg.norm(np.std(chain, axis=0)))
    return spreads


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-restarts", type=int, default=60)
    parser.add_argument("--num-samples", type=int, default=50, help="per restart")
    parser.add_argument("--num-seeds", type=int, default=5)
    parser.add_argument("--base-seed", type=int, default=0)
    args = parser.parse_args()

    run_dir = PROJECT_ROOT / "artifacts" / "tc_space_experiments" / \
        (datetime.now().strftime("%Y%m%d_%H%M%S") + "_per_restart_spread")
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"Saving all output to {run_dir}\n")

    max_err = _validate_against_regular_fk(build_rational_scene(), build_regular_scene())
    assert max_err < 1e-8, "TC-space constraint doesn't match regular FK - fix before trusting anything below"

    regular_scene = build_regular_scene()
    g_reg, Jg_reg, lower_reg, upper_reg, embed = make_regular_constraint(regular_scene)
    rational_scene = build_rational_scene()
    g_tc, Jg_tc, lower_tc, upper_tc = make_fast_rational_collision_constraint(rational_scene)

    diag_reg = float(np.linalg.norm(upper_reg - lower_reg))
    diag_tc = float(np.linalg.norm(upper_tc - lower_tc))
    step_fraction = 0.1 / diag_reg
    step_reg, step_tc = step_fraction * diag_reg, step_fraction * diag_tc
    print(f"step sizes: q-space={step_reg:.4f}, s-space={step_tc:.4f} (normalized to the same domain fraction)\n")

    seeds = [args.base_seed + i for i in range(args.num_seeds)]
    all_pairs = []  # (seed, restart_index, spread_regular, spread_tc)

    for seed in seeds:
        q_samples_regular, _, _, restart_info_reg = run_sampler(
            g_reg, Jg_reg, lower_reg, upper_reg, args.num_restarts, args.num_samples, seed, step_reg)
        s_samples_tc, _, _, restart_info_tc = run_sampler(
            g_tc, Jg_tc, lower_tc, upper_tc, args.num_restarts, args.num_samples, seed, step_tc)
        q_samples_from_tc = map_tc_samples_to_q(s_samples_tc, rational_scene)

        spread_reg = per_restart_spread_q(q_samples_regular, restart_info_reg)
        spread_tc = per_restart_spread_q(q_samples_from_tc, restart_info_tc)

        matched = sorted(set(spread_reg) & set(spread_tc))
        only_reg = sorted(set(spread_reg) - set(spread_tc))
        only_tc = sorted(set(spread_tc) - set(spread_reg))
        print(f"seed {seed}: {len(matched)} restarts succeeded in both "
              f"({len(only_reg)} regular-only, {len(only_tc)} TC-only, out of {args.num_restarts} attempted)")

        for i in matched:
            all_pairs.append((seed, i, spread_reg[i], spread_tc[i]))

    seeds_arr = np.array([p[0] for p in all_pairs])
    restart_idx_arr = np.array([p[1] for p in all_pairs])
    spread_reg_arr = np.array([p[2] for p in all_pairs])
    spread_tc_arr = np.array([p[3] for p in all_pairs])

    print(f"\n=== Paired per-restart spread, {len(all_pairs)} matched restarts across {len(seeds)} seeds ===")
    print(f"median spread: regular={np.median(spread_reg_arr):.4f}, TC-space={np.median(spread_tc_arr):.4f}")
    ratio = spread_tc_arr / np.maximum(spread_reg_arr, 1e-12)
    print(f"median (TC/regular) ratio per restart: {np.median(ratio):.3f}")
    tc_wider = int(np.sum(spread_tc_arr > spread_reg_arr))
    print(f"TC-space chain spread more than its matched regular-FK chain in {tc_wider}/{len(all_pairs)} restarts")

    from scipy.stats import wilcoxon
    try:
        stat, p_value = wilcoxon(spread_tc_arr, spread_reg_arr)
        print(f"Wilcoxon signed-rank test (paired, TC vs. regular): stat={stat:.1f}, p={p_value:.4f} "
              "(tests whether the paired differences are symmetric around zero - small p means the "
              "direction of the effect is unlikely to be due to chance, not that the effect is large)")
    except ValueError as e:
        print(f"Wilcoxon test not computed: {e}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    lims = [0, max(spread_reg_arr.max(), spread_tc_arr.max()) * 1.05]
    axes[0].scatter(spread_reg_arr, spread_tc_arr, c=seeds_arr, cmap="tab10", s=20, alpha=0.8)
    axes[0].plot(lims, lims, "k--", linewidth=1, label="y=x")
    axes[0].set_xlim(lims); axes[0].set_ylim(lims)
    axes[0].set_xlabel("regular-FK within-chain spread (q-space)")
    axes[0].set_ylabel("TC-space within-chain spread (mapped to q-space)")
    axes[0].set_title("Paired per-restart spread (color = seed)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.25)
    axes[0].set_aspect("equal")

    axes[1].hist(ratio, bins=20, color="#54a24b", alpha=0.8)
    axes[1].axvline(1.0, color="k", linestyle="--", linewidth=1, label="ratio=1 (no difference)")
    axes[1].axvline(np.median(ratio), color="#e45756", linestyle="-", linewidth=2,
                     label=f"median={np.median(ratio):.2f}")
    axes[1].set_xlabel("TC-space / regular-FK spread ratio, per restart")
    axes[1].set_ylabel("count")
    axes[1].set_title("Distribution of the paired ratio")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(run_dir / "per_restart_spread.png", dpi=150)
    print(f"\nSaved {run_dir / 'per_restart_spread.png'}")

    with open(run_dir / "summary.json", "w") as f:
        json.dump({
            "args": vars(args),
            "num_matched_restarts": len(all_pairs),
            "median_spread_regular": float(np.median(spread_reg_arr)),
            "median_spread_tc": float(np.median(spread_tc_arr)),
            "median_ratio_tc_over_regular": float(np.median(ratio)),
            "tc_wider_count": tc_wider,
            "total_pairs": len(all_pairs),
            "pairs": [{"seed": int(s), "restart": int(i), "spread_regular": float(a), "spread_tc": float(b)}
                      for s, i, a, b in all_pairs],
        }, f, indent=2)
    print(f"Saved {run_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
