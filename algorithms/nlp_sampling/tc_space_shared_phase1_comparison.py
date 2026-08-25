"""
The controlled version of the comparison, per review discussion: earlier
scripts passed the same RNG seed to both samplers' RestartOptions, but
that's not the same as starting from the same physical point - Phase 1
(seed selection + downhill) runs independently in each space, and since
q-space and s-space have different shapes, "the same PRNG stream position"
can still land the two samplers at different, not-equally-explorable
starting configurations. That's a real confound for isolating INTERIOR
sampling behavior specifically (as opposed to differences that are really
about phase1/seed-choice).

Fix: run Phase 1 exactly once, in q-space (regular_scene, #78's
constraint), via the normal restarting_nhr_sample call. For each restart,
take the x0 it actually landed on (recorded in restart_info's "x0" field)
and convert it to s via RationalForwardKinematics.ComputeSValue - exact
and lossless, both constraints already validated to agree on this point
to 1e-15 (see tc_space_collision_scenario_rational._validate_against_regular_fk).
Then run ONLY the interior-sampling phase (nlp_sampling.nhr_sample, no
Phase 1 of its own) from that converted point for the TC-space
constraint. Both interior walks now start from the same physical
configuration - any difference in how far/well each one explores is
attributable to interior-sampling dynamics in each space, not to
different phase1 outcomes.

Plots use each space's own NATIVE coordinates (q for regular, s for
TC-space) - not mapped to a common space - restart-colored, matching the
very first individual scenario demos (tc_space_collision_scenario.py /
tc_space_collision_scenario_fast.py's own __main__ blocks) but at a much
larger sample budget and with restart coloring added.

Usage:
    python tc_space_shared_phase1_comparison.py
    python tc_space_shared_phase1_comparison.py --num-restarts 100 --num-samples 300
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

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--num-restarts", type=int, default=100)
    parser.add_argument("--num-samples", type=int, default=300, help="interior samples per restart")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    run_dir = PROJECT_ROOT / "artifacts" / "tc_space_experiments" / \
        (datetime.now().strftime("%Y%m%d_%H%M%S") + "_shared_phase1")
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
    step_reg, step_tc = step_fraction * diag_reg, step_fraction * diag_tc
    print(f"step sizes: q-space={step_reg:.4f}, s-space={step_tc:.4f} (normalized to the same domain fraction)")

    # Step 1: Phase 1 + interior sampling, regular FK only, as normal.
    # This is the ONLY place seeds get chosen / phase1 downhill runs.
    nhr_options_reg = nlp_sampling.NHROptions(
        num_samples=args.num_samples, burn_in=20, random_seed=args.seed, verbose=False,
        slack_max_step=step_reg, delta_max=step_reg,
    )
    restart_options = nlp_sampling.RestartOptions(
        num_restarts=args.num_restarts, strategy="distance", random_seed=args.seed)
    phase1 = lambda s: nlp_sampling.run_downhill_phase1(
        s, g=g_reg, Jg=Jg_reg, lower=lower_reg, upper=upper_reg, options=nhr_options_reg)

    q_samples_regular, restart_info_reg = nlp_sampling.restarting_nhr_sample(
        phase1=phase1, g=g_reg, lower=lower_reg, upper=upper_reg, Jg=Jg_reg,
        nhr_options=nhr_options_reg, restart_options=restart_options,
    )
    print(f"\nRegular FK: {len(q_samples_regular)} samples from "
          f"{sum(1 for i in restart_info_reg if i['status'] == 'success')}/{args.num_restarts} successful restarts")

    # Step 2: for each restart's shared x0 (converted q -> s), run ONLY
    # nlp_sampling.nhr_sample's interior-sampling phase for the TC-space
    # constraint - no phase1 of its own, no independent seed choice.
    rat_fk, q_star = rational_scene["rat_fk"], rational_scene["q_star"]
    idx_j2, idx_j4 = rational_scene["idx_j2"], rational_scene["idx_j4"]
    idx_s_j2, idx_s_j4 = rational_scene["idx_s_j2"], rational_scene["idx_s_j4"]

    nhr_options_tc = nlp_sampling.NHROptions(
        num_samples=args.num_samples, burn_in=20, random_seed=args.seed, verbose=False,
        slack_max_step=step_tc, delta_max=step_tc,
    )

    tc_chunks = []
    tc_restart_info = []
    skipped = 0
    offset = 0
    for info in restart_info_reg:
        if info["status"] != "success":
            continue
        n = info["num_samples"]
        offset += n
        if "x0" not in info:
            continue
        q0 = info["x0"]
        q_full = q_star.copy()
        q_full[idx_j2], q_full[idx_j4] = q0[0], q0[1]
        s_full = rat_fk.ComputeSValue(q_full, q_star)
        s0 = np.array([s_full[idx_s_j2], s_full[idx_s_j4]])

        try:
            samples_r, _ = nlp_sampling.nhr_sample(
                x0=s0, g=g_tc, lower=lower_tc, upper=upper_tc, Jg=Jg_tc, options=nhr_options_tc)
        except ValueError:
            skipped += 1
            continue
        tc_chunks.append(samples_r)
        tc_restart_info.append({"restart": info["restart"], "status": "success", "num_samples": len(samples_r)})

    s_samples_tc_shared = np.vstack(tc_chunks) if tc_chunks else np.empty((0, 2))
    print(f"TC-space (shared phase1): {len(s_samples_tc_shared)} samples from "
          f"{len(tc_chunks)}/{sum(1 for i in restart_info_reg if i['status'] == 'success')} shared starting points "
          f"({skipped} skipped - x0 wasn't feasible enough for the TC-space constraint at nhr_sample's tolerance)")

    # MSTS + spread, both in q-space common units.
    q_samples_from_tc = np.empty_like(s_samples_tc_shared)
    for i, s_free in enumerate(s_samples_tc_shared):
        s_full = np.zeros(len(rat_fk.s()))
        s_full[idx_s_j2], s_full[idx_s_j4] = s_free[0], s_free[1]
        q_full = rat_fk.ComputeQValue(s_full, q_star)
        q_samples_from_tc[i] = [q_full[idx_j2], q_full[idx_j4]]

    ns_reg, msts1_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=1.0, num_checkpoints=15)
    _, msts2_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=2.0, num_checkpoints=15)
    ns_tc, msts1_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=1.0, num_checkpoints=15)
    _, msts2_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=2.0, num_checkpoints=15)
    print(f"\nFinal MSTS_1 (q-space): regular={msts1_reg[-1]:.2f}, TC-space={msts1_tc[-1]:.2f}")
    print(f"Final MSTS_2 (q-space): regular={msts2_reg[-1]:.2f}, TC-space={msts2_tc[-1]:.2f}")

    # Paired per-restart spread, shared starting point.
    offset = 0
    spread_reg_by_restart = {}
    for info in restart_info_reg:
        if info["status"] != "success":
            continue
        n = info["num_samples"]
        chain = q_samples_regular[offset:offset + n]
        offset += n
        if n > 1:
            spread_reg_by_restart[info["restart"]] = float(np.linalg.norm(np.std(chain, axis=0)))

    offset = 0
    spread_tc_by_restart = {}
    for info in tc_restart_info:
        n = info["num_samples"]
        chain = q_samples_from_tc[offset:offset + n]
        offset += n
        if n > 1:
            spread_tc_by_restart[info["restart"]] = float(np.linalg.norm(np.std(chain, axis=0)))

    matched = sorted(set(spread_reg_by_restart) & set(spread_tc_by_restart))
    spread_reg_arr = np.array([spread_reg_by_restart[i] for i in matched])
    spread_tc_arr = np.array([spread_tc_by_restart[i] for i in matched])
    ratio = spread_tc_arr / np.maximum(spread_reg_arr, 1e-12)
    print(f"\nPaired per-restart spread from shared starting points, {len(matched)} restarts:")
    print(f"  median spread: regular={np.median(spread_reg_arr):.4f}, TC-space={np.median(spread_tc_arr):.4f}")
    print(f"  median (TC/regular) ratio: {np.median(ratio):.3f}")
    print(f"  TC-space spread more in {int(np.sum(spread_tc_arr > spread_reg_arr))}/{len(matched)} restarts")
    from scipy.stats import wilcoxon
    if len(matched) > 5:
        stat, p_value = wilcoxon(spread_tc_arr, spread_reg_arr)
        print(f"  Wilcoxon signed-rank test: stat={stat:.1f}, p={p_value:.4f}")

    # Restart-colored plots, NATIVE coordinates for each space (not mapped
    # to a common space) - matching the original individual scenario demos.
    restart_ids_reg = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in restart_info_reg if info["status"] == "success"])
    restart_ids_tc = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in tc_restart_info])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    sc0 = axes[0].scatter(q_samples_regular[:, 0], q_samples_regular[:, 1], c=restart_ids_reg, cmap="tab20", s=6)
    axes[0].set_xlabel("joint2 (q)")
    axes[0].set_ylabel("joint4 (q)")
    axes[0].set_title(f"Regular FK, native q-space, colored by restart id\n({len(q_samples_regular)} samples)")
    axes[0].grid(True, alpha=0.25)

    sc1 = axes[1].scatter(s_samples_tc_shared[:, 0], s_samples_tc_shared[:, 1], c=restart_ids_tc, cmap="tab20", s=6)
    axes[1].set_xlabel("s (joint2)")
    axes[1].set_ylabel("s (joint4)")
    axes[1].set_title(f"TC-space, native s-space, colored by restart id\n({len(s_samples_tc_shared)} samples, "
                       "shared phase1 starting points with the left panel")
    axes[1].grid(True, alpha=0.25)

    # Same TC-space samples as the middle panel, mapped back to q-space
    # (RationalForwardKinematics.ComputeQValue) so it's directly, visually
    # comparable to the left panel in the same coordinate frame - not just
    # via the summary MSTS/spread numbers.
    sc2 = axes[2].scatter(q_samples_from_tc[:, 0], q_samples_from_tc[:, 1], c=restart_ids_tc, cmap="tab20", s=6)
    axes[2].set_xlabel("joint2 (q)")
    axes[2].set_ylabel("joint4 (q)")
    axes[2].set_title(f"TC-space, mapped to q-space, colored by restart id\n({len(q_samples_from_tc)} samples, "
                       "same points as the middle panel")
    axes[2].set_xlim(axes[0].get_xlim())
    axes[2].set_ylim(axes[0].get_ylim())
    axes[2].grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(run_dir / "samples_native_space_restart_colored.png", dpi=150)
    print(f"\nSaved {run_dir / 'samples_native_space_restart_colored.png'}")

    fig2, axes2 = plt.subplots(1, 2, figsize=(11, 4.5))
    axes2[0].plot(ns_reg, msts1_reg, marker="o", markersize=3, label="regular FK", color="#4c78a8")
    axes2[0].plot(ns_tc, msts1_tc, marker="o", markersize=3, label="TC-space", color="#f58518")
    axes2[0].set_xlabel("number of samples (n)")
    axes2[0].set_ylabel("MSTS_1 (q-space)")
    axes2[0].set_title("Diversity (p=1), shared starting points")
    axes2[0].legend(fontsize=8)
    axes2[0].grid(True, alpha=0.25)

    axes2[1].plot(ns_reg, msts2_reg, marker="o", markersize=3, label="regular FK", color="#4c78a8")
    axes2[1].plot(ns_tc, msts2_tc, marker="o", markersize=3, label="TC-space", color="#f58518")
    axes2[1].set_xlabel("number of samples (n)")
    axes2[1].set_ylabel("MSTS_2 (q-space)")
    axes2[1].set_title("Mode coverage (p=2), shared starting points")
    axes2[1].legend(fontsize=8)
    axes2[1].grid(True, alpha=0.25)
    fig2.tight_layout()
    fig2.savefig(run_dir / "msts_shared_phase1.png", dpi=150)
    print(f"Saved {run_dir / 'msts_shared_phase1.png'}")

    with open(run_dir / "summary.json", "w") as f:
        json.dump({
            "args": vars(args),
            "num_samples_regular": len(q_samples_regular),
            "num_samples_tc": len(s_samples_tc_shared),
            "num_shared_starting_points": len(tc_chunks),
            "num_skipped_infeasible_for_tc": skipped,
            "msts1_final": {"regular": float(msts1_reg[-1]), "tc": float(msts1_tc[-1])},
            "msts2_final": {"regular": float(msts2_reg[-1]), "tc": float(msts2_tc[-1])},
            "spread_median": {"regular": float(np.median(spread_reg_arr)), "tc": float(np.median(spread_tc_arr))},
            "spread_median_ratio_tc_over_regular": float(np.median(ratio)),
            "spread_num_matched_restarts": len(matched),
        }, f, indent=2)
    print(f"Saved {run_dir / 'summary.json'}")


if __name__ == "__main__":
    main()
