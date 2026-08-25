"""
Standalone harness for validating nlp_sampling.py's samplers against the real
Panda+cap grasp problem, without any of full_arm_nhr.ipynb's IRIS/GCS/
meshcat machinery (CspaceFreePolytope, CIrisPlantVisualizer,
ManipulationPlanner). Builds only what h_grasp_eq/g_grasp_ineq actually
need: a welded arm+hand+cap plant and its AutoDiff companion.

Usage:
    python nlp_sampling_standalone_test.py
    python nlp_sampling_standalone_test.py --interior-method mRRT --num-restarts 30
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.planning import RobotDiagramBuilder
from pydrake.forwarddiff import jacobian as autodiff_jacobian

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


# -----------------------------------------------------------------------------
# Minimal plant (no obstacles, no visualizer/IRIS/GCS - irrelevant to the
# grasp constraints and their Jacobians)
# -----------------------------------------------------------------------------

def build_minimal_plant():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    scene_graph = builder.scene_graph()
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)

    panda_arm = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_arm.urdf")[0]
    panda_hand = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]

    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("panda_link0", panda_arm),
        RigidTransform(),
    )
    X_8H = RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0])
    plant.WeldFrames(
        plant.GetFrameByName("panda_link8", panda_arm),
        plant.GetFrameByName("panda_hand", panda_hand),
        X_8H,
    )
    plant.GetJointByName("panda_finger_joint1", panda_hand).set_default_translation(-0.024)

    cap = parser.AddModels(file_name=str(PROJECT_ROOT / "my_sdfs" / "bottle_cap.sdf"))[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link", cap),
        RigidTransform(RotationMatrix(), [0.5, 0, 0]),
    )

    plant.Finalize()
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(diagram_context)

    return dict(
        diagram=diagram,
        diagram_context=diagram_context,  # keep alive - plant_context aliases into it
        plant=plant,
        plant_context=plant_context,
        panda_arm=panda_arm,
        panda_hand=panda_hand,
        cap=cap,
    )


def build_autodiff_companion(plant, E, Cap):
    """Ports cell 31's plant_ad = plant.ToAutoDiffXd() pattern."""
    plant_ad = plant.ToAutoDiffXd()
    plant_context_ad = plant_ad.CreateDefaultContext()

    def get_autodiff_frame(frame):
        return plant_ad.GetFrameByName(frame.name(), frame.model_instance())

    E_ad = get_autodiff_frame(E)
    Cap_ad = get_autodiff_frame(Cap)
    return plant_ad, plant_context_ad, E_ad, Cap_ad


# -----------------------------------------------------------------------------
# Grasp constraints - verbatim port of notebook cells 30/31/33/34
# -----------------------------------------------------------------------------

def make_grasp_constraints(plant, plant_context, plant_ad, plant_context_ad, E, Cap, E_ad, Cap_ad):
    Z_MIN = 0.105
    Z_MAX = 0.110
    P_CAP_E_DES = np.array([0.0, 0.0, Z_MIN])
    NEGATIVE_Z_THRESHOLD = -0.9

    def calc_X_CapE(q):
        """
        q may be a plain float array (ordinary sampling/checking) or an
        AutoDiffXd array from InitializeAutoDiff(q) (Jacobian evaluation).
        """
        if np.asarray(q).dtype == object:  # AutoDiffXd elements
            plant_ad.SetPositions(plant_context_ad, q)
            return plant_ad.CalcRelativeTransform(plant_context_ad, Cap_ad, E_ad)

        q = np.asarray(q, dtype=float)
        old_q = plant.GetPositions(plant_context).copy()
        plant.SetPositions(plant_context, q)
        try:
            return plant.CalcRelativeTransform(plant_context, Cap, E)
        finally:
            plant.SetPositions(plant_context, old_q)

    def h_grasp_eq(q):
        X_CapE = calc_X_CapE(q)
        p_CapE = X_CapE.translation()
        R_CapE = X_CapE.rotation().matrix()
        z_E_in_Cap = R_CapE[:, 2]
        return np.array([
            p_CapE[0] - P_CAP_E_DES[0],
            p_CapE[1] - P_CAP_E_DES[1],
            z_E_in_Cap[0],
            z_E_in_Cap[1],
        ])

    def g_grasp_ineq(q):
        X_CapE = calc_X_CapE(q)
        p_CapE = X_CapE.translation()
        R_CapE = X_CapE.rotation().matrix()
        z_E_in_Cap = R_CapE[:, 2]
        return np.array([
            p_CapE[2] - Z_MAX,
            Z_MIN - p_CapE[2],
            z_E_in_Cap[2] - NEGATIVE_Z_THRESHOLD,
        ])

    def h_jacobian_eq(q):
        return autodiff_jacobian(h_grasp_eq, np.asarray(q, dtype=float))

    def g_jacobian_ineq(q):
        return autodiff_jacobian(g_grasp_ineq, np.asarray(q, dtype=float))

    return h_grasp_eq, g_grasp_ineq, h_jacobian_eq, g_jacobian_ineq


# -----------------------------------------------------------------------------
# Joint indices / limits - ports cells 7, 27, 40
# -----------------------------------------------------------------------------

def joint_position_index(plant, joint_name, model_instance=None):
    joint = plant.GetJointByName(joint_name) if model_instance is None else plant.GetJointByName(joint_name, model_instance)
    if joint.num_positions() != 1:
        raise ValueError(f"Joint {joint_name} has {joint.num_positions()} positions; expected 1.")
    return joint.position_start()


def make_joint_indices(plant, panda_arm, panda_hand, cap):
    idx_arm = np.array([joint_position_index(plant, f"panda_joint{i}", panda_arm) for i in range(1, 8)])
    idx_wrist = joint_position_index(plant, "panda_joint7", panda_arm)
    idx_f1 = joint_position_index(plant, "panda_finger_joint1", panda_hand)
    idx_f2 = joint_position_index(plant, "panda_finger_joint2", panda_hand)
    idx_cap = joint_position_index(plant, "cap_to_base", cap)
    return dict(idx_arm=idx_arm, idx_wrist=idx_wrist, idx_f1=idx_f1, idx_f2=idx_f2, idx_cap=idx_cap)


def make_joint_limits(plant, idx_wrist, idx_f1, idx_f2):
    """Ports cell 40's narrowing logic directly on the plant's own limits -
    no ManipulationPlanner object needed (planner.lower/upper_joint_limits
    reduce directly to plant.GetPositionLowerLimits()/UpperLimits())."""
    lower = plant.GetPositionLowerLimits().copy()
    upper = plant.GetPositionUpperLimits().copy()

    lower[idx_wrist], upper[idx_wrist] = -1.0, 1.0
    lower[idx_f1], upper[idx_f1] = -0.025, -0.024
    lower[idx_f2], upper[idx_f2] = 0.024, 0.025

    return lower, upper


# -----------------------------------------------------------------------------
# Call-counting wrappers (self-contained Drake-eval-count diagnostic - no
# nlp_sampling.py changes needed, since WalkerState.evals isn't exposed by the
# public sampling entry points)
# -----------------------------------------------------------------------------

class CallCounter:
    def __init__(self, fn):
        self.fn = fn
        self.count = 0

    def __call__(self, x):
        self.count += 1
        return self.fn(x)


# -----------------------------------------------------------------------------
# Diagnostics
# -----------------------------------------------------------------------------

def run_diagnostics(samples, restart_info, restart_options, h_grasp_eq,
                     g_grasp_ineq, elapsed, eval_counts):
    num_restarts = restart_options.num_restarts
    successes = [i for i in restart_info if i["status"] == "success"]
    phase1_failed = [i for i in restart_info if i["status"] == "phase1_failed"]
    nhr_failed = [i for i in restart_info if i["status"] == "nhr_failed"]

    print()
    print("=" * 70)
    print("RESTART SUMMARY")
    print("=" * 70)
    print(f"restarts: {len(successes)}/{num_restarts} succeeded "
          f"({len(phase1_failed)} phase1_failed, {len(nhr_failed)} nhr_failed)")
    print(f"total samples (post-filter): {len(samples)}")
    print(f"wall-clock: {elapsed:.1f}s ({len(samples) / max(elapsed, 1e-9):.1f} samples/sec)")
    for name, counter in eval_counts.items():
        per_sample = counter.count / max(len(samples), 1)
        print(f"  {name}: {counter.count} calls total, {per_sample:.1f} per returned sample")

    # Final residual summary (grasp-constraint-specific - needs h_grasp_eq/g_grasp_ineq,
    # so it stays here rather than in nlp_sampling.save_run_analytics_artifacts).
    if len(samples):
        hvals = np.array([h_grasp_eq(q) for q in samples])
        gvals = np.array([g_grasp_ineq(q) for q in samples])
        print()
        print("final residuals over returned samples:")
        print(f"  max |h|_inf = {np.max(np.abs(hvals)):.3e}   mean |h|_inf = {np.mean(np.abs(hvals)):.3e}")
        print(f"  max g       = {np.max(gvals):.3e}   (want <= 0)")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--num-samples", type=int, default=500)
    p.add_argument("--burn-in", type=int, default=100)
    p.add_argument("--num-restarts", type=int, default=30)
    p.add_argument("--restart-strategy", choices=["uniform", "distance", "direction"], default="distance")
    p.add_argument("--interior-method", choices=["HR", "mRRT"], default="HR")
    p.add_argument("--interior-noise-sigma", type=float, default=0.1)
    p.add_argument("--equality-tol", type=float, default=1e-3)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--quiet", action="store_true")
    p.add_argument("--no-plots", action="store_true")
    p.add_argument("--output-root", default=str(PROJECT_ROOT / "artifacts" / "joint_samples_plots"))
    return p.parse_args()


def main():
    args = parse_args()

    print("Building minimal Drake plant (no meshcat/IRIS/GCS)...")
    scene = build_minimal_plant()
    plant, plant_context = scene["plant"], scene["plant_context"]
    panda_arm, panda_hand, cap = scene["panda_arm"], scene["panda_hand"], scene["cap"]
    print(f"  positions: {plant.num_positions()}")

    E = plant.GetFrameByName("panda_hand", panda_hand)
    Cap = plant.GetFrameByName("base_link", cap)

    plant_ad, plant_context_ad, E_ad, Cap_ad = build_autodiff_companion(plant, E, Cap)
    h_grasp_eq, g_grasp_ineq, h_jacobian_eq, g_jacobian_ineq = make_grasp_constraints(
        plant, plant_context, plant_ad, plant_context_ad, E, Cap, E_ad, Cap_ad,
    )

    joint_idx = make_joint_indices(plant, panda_arm, panda_hand, cap)
    lower, upper = make_joint_limits(plant, joint_idx["idx_wrist"], joint_idx["idx_f1"], joint_idx["idx_f2"])
    joint_names = list(plant.GetPositionNames())

    h_counter = CallCounter(h_grasp_eq)
    g_counter = CallCounter(g_grasp_ineq)
    eval_counts = {"h_grasp_eq": h_counter, "g_grasp_ineq": g_counter}

    nhr_options = nlp_sampling.NHROptions(
        num_samples=args.num_samples,
        burn_in=args.burn_in,
        interior_method=args.interior_method,
        interior_noise_sigma=args.interior_noise_sigma,
        random_seed=args.seed,
        verbose=not args.quiet,
    )
    restart_options = nlp_sampling.RestartOptions(
        num_restarts=args.num_restarts,
        strategy=args.restart_strategy,
        candidates_per_restart=100,
        random_seed=args.seed,
    )

    print(f"Sampling: interior_method={args.interior_method}, "
          f"{args.num_restarts} restarts x {args.num_samples} samples...")

    if args.no_plots:
        # Skips save_run_analytics_artifacts entirely (no PNGs/JSON written),
        # so this path still hand-wires phase1 + restarting_nhr_sample
        # directly rather than going through run_and_report.
        phase1 = lambda seed: nlp_sampling.run_downhill_phase1(
            seed, g=g_counter, Jg=g_jacobian_ineq, lower=lower, upper=upper,
            options=nhr_options, h=h_counter, Jh=h_jacobian_eq,
        )
        t0 = time.time()
        samples, restart_info = nlp_sampling.restarting_nhr_sample(
            phase1=phase1, g=g_counter, lower=lower, upper=upper, Jg=g_jacobian_ineq,
            nhr_options=nhr_options, restart_options=restart_options,
            h=h_counter, Jh=h_jacobian_eq, equality_tol=args.equality_tol,
            project_samples_to_manifold=True, projection_iters=10,
        )
        elapsed = time.time() - t0
    else:
        t0 = time.time()
        samples, restart_info, _ = nlp_sampling.run_and_report(
            g=g_counter, lower=lower, upper=upper, Jg=g_jacobian_ineq,
            h=h_counter, Jh=h_jacobian_eq,
            nhr_options=nhr_options, restart_options=restart_options,
            joint_names=joint_names, output_root=args.output_root,
            note=f"standalone harness, interior_method={args.interior_method}",
            show=False, equality_tol=args.equality_tol,
            project_samples_to_manifold=True, projection_iters=10,
        )
        elapsed = time.time() - t0

    run_diagnostics(
        samples, restart_info, restart_options, h_grasp_eq, g_grasp_ineq,
        elapsed, eval_counts,
    )


if __name__ == "__main__":
    main()
