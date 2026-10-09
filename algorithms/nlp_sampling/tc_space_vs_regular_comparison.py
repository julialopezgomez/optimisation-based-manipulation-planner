"""
Regular-FK sampling (#78, tc_space_collision_scenario.py) vs. TC-space
rational-FK sampling (tc_space_collision_scenario_fast.py, q_star=0 -
the textbook stereographic projection, see
tc_space_collision_scenario_rational.py's module docstring) on the
identical physical scenario, using nlp_sampling.py's PLAIN DEFAULT
settings for both - no per-space step-size tuning or domain
normalization. An earlier version of this comparison normalized step
size to each space's own domain diagonal; that's dropped here, on
purpose, to see the honest, unadjusted effect of q_star=0's domain shape
(s(joint2) in ~[-1.2,1.2], s(joint4) in ~[-28.6,-0.03] - a ~24x size
mismatch between the two dimensions, from the tangent-half-angle map's
genuine asymptotic behavior near panda_joint4's range boundary, not
anything invented). A single fixed step size applied to both dimensions
of a domain this lopsided is expected to behave very differently in each
dimension - that's the actual point of running this, not a confound to
correct away.

Both sample sets are compared in q-space (regular FK's native space):
s and q are different coordinate scales, so a raw MSTS comparison across
them would compare units, not coverage. TC-space's s-space samples are
mapped back to q via RationalForwardKinematics.ComputeQValue before
scoring.

MSTS reported at both p=1 (diversity) and p=2 (mode coverage) - see
msts_metric_reference.md for why p=2 is the relevant one for "did it find
both disjoint regions", and why relying on either alone is an incomplete
picture.

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

    print(f"q-space domain: joint2 in [{lower_reg[0]:.3f},{upper_reg[0]:.3f}], "
          f"joint4 in [{lower_reg[1]:.3f},{upper_reg[1]:.3f}] (diagonal {np.linalg.norm(upper_reg - lower_reg):.3f})")
    print(f"s-space domain: joint2 in [{lower_tc[0]:.3f},{upper_tc[0]:.3f}], "
          f"joint4 in [{lower_tc[1]:.3f},{upper_tc[1]:.3f}] (diagonal {np.linalg.norm(upper_tc - lower_tc):.3f})")
    print("Using nlp_sampling's plain default step size (slack_max_step=0.1) for BOTH, unadjusted.\n")

    seeds = [args.base_seed + i for i in range(args.num_seeds)]
    per_seed_results = []

    for seed in seeds:
        print(f"--- seed {seed} ---")
        q_samples_regular, elapsed_regular, evals_regular, restart_info_reg = run_sampler(
            g_reg, Jg_reg, lower_reg, upper_reg, args.num_restarts, args.num_samples, seed)
        s_samples_tc, elapsed_tc, evals_tc, restart_info_tc = run_sampler(
            g_tc, Jg_tc, lower_tc, upper_tc, args.num_restarts, args.num_samples, seed)
        q_samples_from_tc = map_tc_samples_to_q(s_samples_tc, rational_scene)

        print(f"  regular: {len(q_samples_regular)} samples, {elapsed_regular:.3f}s, {evals_regular} g-evals")
        print(f"  TC-space: {len(s_samples_tc)} samples, {elapsed_tc:.3f}s, {evals_tc} g-evals")

        from scipy.cluster.vq import kmeans2
        for name, samples in [("regular", q_samples_regular), ("TC-space", s_samples_tc)]:
            if len(samples) < 2:
                print(f"  WARNING: {name} sampler produced fewer than 2 samples this seed")
                continue
            _, labels = kmeans2(samples, 2, seed=0, minit="++")
            n0, n1 = int(np.sum(labels == 0)), int(np.sum(labels == 1))
            if min(n0, n1) <= 0.1 * len(samples):
                print(f"  WARNING: {name} sampler may have missed a region this seed ({n0}/{n1})")

        if len(q_samples_regular) < 2 or len(q_samples_from_tc) < 2:
            print("  Skipping MSTS for this seed - too few samples.")
            continue

        ns_reg, msts2_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=2.0, num_checkpoints=12)
        ns_tc, msts2_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=2.0, num_checkpoints=12)
        _, msts1_reg = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_regular, p=1.0, num_checkpoints=12)
        _, msts1_tc = nlp_sampling.minimum_spanning_tree_score_curve(q_samples_from_tc, p=1.0, num_checkpoints=12)

        per_seed_results.append(dict(
            seed=seed,
            q_samples_regular=q_samples_regular, q_samples_from_tc=q_samples_from_tc,
            s_samples_tc=s_samples_tc,
            restart_info_reg=restart_info_reg, restart_info_tc=restart_info_tc,
            elapsed_regular=elapsed_regular, elapsed_tc=elapsed_tc,
            evals_regular=evals_regular, evals_tc=evals_tc,
            ns_reg=ns_reg, msts1_reg=msts1_reg, msts2_reg=msts2_reg,
            ns_tc=ns_tc, msts1_tc=msts1_tc, msts2_tc=msts2_tc,
        ))

    if not per_seed_results:
        print("\nNo seed produced enough samples in both spaces to compare. Stopping here.")
        return

    print("\n=== Summary across seeds (final MSTS values) ===")
    print(f"{'seed':>5} | {'reg MSTS_1':>10} | {'tc MSTS_1':>10} | {'reg MSTS_2':>10} | {'tc MSTS_2':>10}")
    for r in per_seed_results:
        print(f"{r['seed']:>5} | {r['msts1_reg'][-1]:>10.3f} | {r['msts1_tc'][-1]:>10.3f} | "
              f"{r['msts2_reg'][-1]:>10.3f} | {r['msts2_tc'][-1]:>10.3f}")

    msts1_reg_finals = [r['msts1_reg'][-1] for r in per_seed_results]
    msts1_tc_finals = [r['msts1_tc'][-1] for r in per_seed_results]
    msts2_reg_finals = [r['msts2_reg'][-1] for r in per_seed_results]
    msts2_tc_finals = [r['msts2_tc'][-1] for r in per_seed_results]
    print(f"\nmedian final MSTS_1: regular={np.median(msts1_reg_finals):.3f}, TC-space={np.median(msts1_tc_finals):.3f}")
    print(f"median final MSTS_2: regular={np.median(msts2_reg_finals):.3f}, TC-space={np.median(msts2_tc_finals):.3f}")

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
        ax.set_title(f"p={p} ({'diversity' if p == 1 else 'mode coverage'}), {len(per_seed_results)} seeds")
        ax.grid(True, alpha=0.25)
        ax.plot([], [], color="#4c78a8", label="regular FK")
        ax.plot([], [], color="#f58518", label="TC-space")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(run_dir / "msts_all_seeds.png", dpi=150)
    print(f"\nSaved {run_dir / 'msts_all_seeds.png'}")

    r0 = per_seed_results[0]
    restart_ids_reg = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in r0["restart_info_reg"] if info["status"] == "success"])
    restart_ids_tc = np.concatenate([
        np.full(info["num_samples"], info["restart"]) for info in r0["restart_info_tc"] if info["status"] == "success"])

    fig2, axes2 = plt.subplots(1, 3, figsize=(17, 5.5))
    axes2[0].scatter(r0["q_samples_regular"][:, 0], r0["q_samples_regular"][:, 1], c=restart_ids_reg, cmap="tab20", s=8)
    axes2[0].set_xlabel("joint2 (q)"); axes2[0].set_ylabel("joint4 (q)")
    axes2[0].set_title(f"Regular FK, native q-space (seed {r0['seed']})")
    axes2[0].grid(True, alpha=0.25)

    axes2[1].scatter(r0["s_samples_tc"][:, 0], r0["s_samples_tc"][:, 1], c=restart_ids_tc, cmap="tab20", s=8)
    axes2[1].set_xlabel("s (joint2)"); axes2[1].set_ylabel("s (joint4)")
    axes2[1].set_title(f"TC-space, native s-space (seed {r0['seed']})")
    axes2[1].grid(True, alpha=0.25)

    axes2[2].scatter(r0["q_samples_from_tc"][:, 0], r0["q_samples_from_tc"][:, 1], c=restart_ids_tc, cmap="tab20", s=8)
    axes2[2].set_xlabel("joint2 (q)"); axes2[2].set_ylabel("joint4 (q)")
    axes2[2].set_title(f"TC-space, mapped to q-space (seed {r0['seed']})")
    axes2[2].set_xlim(axes2[0].get_xlim())
    axes2[2].set_ylim(axes2[0].get_ylim())
    axes2[2].grid(True, alpha=0.25)

    fig2.tight_layout()
    fig2.savefig(run_dir / "samples_restart_colored.png", dpi=150)
    print(f"Saved {run_dir / 'samples_restart_colored.png'}")

    summary = {
        "args": vars(args),
        "q_star": "zeros",
        "step_size": "default (0.1, unadjusted)",
        "q_domain_diagonal": float(np.linalg.norm(upper_reg - lower_reg)),
        "s_domain_diagonal": float(np.linalg.norm(upper_tc - lower_tc)),
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
