"""
Test scenario for #78 (part of #46's TC-space comparison, priority one per
2026-08-25 meeting): a real, 2-DOF subproblem of the actual Panda arm
(panda_joint2, panda_joint4 free, the rest fixed at 0), with a spherical
"obstacle" in task space expressed as a smooth distance constraint on
panda_link5's position - not a SceneGraph collision object, so it's
directly differentiable (AutoDiff) and usable as nlp_sampling.py's `g`,
the same way nlp_sampling_standalone_test.py's grasp constraint is.

The obstacle placement/radius (center=(0.25, 0, 0.65), radius=0.35) was
chosen empirically, not derived: panda_link5's reachable set over the full
(joint2, joint4) box was grid-scanned and connected-components-labeled
(scipy.ndimage.label) to confirm the obstacle actually splits the domain
into exactly two disjoint free regions of comparable size (~2772 and ~3852
grid cells out of 10000, at radius=0.35), rather than assuming it would
from the geometry alone - real robot kinematics don't behave like an
idealized 2-link planar arm. See the joint-space plot this produces:
panda_joint2 above ~(a diagonal band) is one region, below is the other.

This module is the regular-FK version of the constraint (plain
plant.CalcRelativeTransform, mirrors make_grasp_constraints's pattern
exactly). #79's TC-space version reuses this same scene/obstacle, built on
Drake's rational forward kinematics instead.

Usage:
    python tc_space_collision_scenario.py
"""
import sys
from pathlib import Path

import numpy as np
from pydrake.forwarddiff import jacobian as autodiff_jacobian
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

sys.path.insert(0, str(Path(__file__).resolve().parent))
import nlp_sampling

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent

OBSTACLE_CENTER = np.array([0.25, 0.0, 0.65])
OBSTACLE_RADIUS = 0.35


def build_scene():
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    panda_arm = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_arm.urdf")[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0", panda_arm), RigidTransform())
    plant.Finalize()
    context = plant.CreateDefaultContext()

    link5 = plant.GetFrameByName("panda_link5", panda_arm)
    idx_j2 = plant.GetJointByName("panda_joint2", panda_arm).position_start()
    idx_j4 = plant.GetJointByName("panda_joint4", panda_arm).position_start()

    plant_ad = plant.ToAutoDiffXd()
    context_ad = plant_ad.CreateDefaultContext()
    link5_ad = plant_ad.GetFrameByName("panda_link5", panda_arm)

    return dict(
        plant=plant, context=context, panda_arm=panda_arm, link5=link5,
        idx_j2=idx_j2, idx_j4=idx_j4,
        plant_ad=plant_ad, context_ad=context_ad, link5_ad=link5_ad,
    )


def make_collision_constraint(scene: dict):
    """
    Returns (g, Jg, lower, upper, q0, embed) for the 2-DOF (joint2, joint4)
    subproblem. `embed(a, b)` builds the full 7-vector q with joint2=a,
    joint4=b, everything else at 0 - useful for visualization/sanity checks
    against the full plant.
    """
    plant, context = scene["plant"], scene["context"]
    plant_ad, context_ad = scene["plant_ad"], scene["context_ad"]
    link5, link5_ad = scene["link5"], scene["link5_ad"]
    idx_j2, idx_j4 = scene["idx_j2"], scene["idx_j4"]

    q0_full = plant.GetPositions(context).copy()

    def embed(x):
        x = np.asarray(x)
        q = q0_full.astype(x.dtype)
        q[idx_j2] = x[0]
        q[idx_j4] = x[1]
        return q

    def link5_position(x):
        if np.asarray(x).dtype == object:  # AutoDiffXd
            q = embed(x)
            plant_ad.SetPositions(context_ad, q)
            return plant_ad.CalcRelativeTransform(context_ad, plant_ad.world_frame(), link5_ad).translation()
        q = embed(np.asarray(x, dtype=float))
        plant.SetPositions(context, q)
        return plant.CalcRelativeTransform(context, plant.world_frame(), link5).translation()

    def g(x):
        p = link5_position(x)
        dist = np.sqrt(np.sum((p - OBSTACLE_CENTER) ** 2))
        return np.array([OBSTACLE_RADIUS - dist])

    def Jg(x):
        return autodiff_jacobian(g, np.asarray(x, dtype=float))

    joint2 = plant.GetJointByName("panda_joint2", scene["panda_arm"])
    joint4 = plant.GetJointByName("panda_joint4", scene["panda_arm"])
    lower = np.array([joint2.position_lower_limits()[0], joint4.position_lower_limits()[0]])
    upper = np.array([joint2.position_upper_limits()[0], joint4.position_upper_limits()[0]])

    return g, Jg, lower, upper, embed


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scene = build_scene()
    g, Jg, lower, upper, embed = make_collision_constraint(scene)

    nhr_options = nlp_sampling.NHROptions(num_samples=50, burn_in=20, random_seed=0, verbose=False)
    restart_options = nlp_sampling.RestartOptions(num_restarts=60, strategy="distance", random_seed=0)

    phase1 = lambda seed: nlp_sampling.run_downhill_phase1(
        seed, g=g, Jg=Jg, lower=lower, upper=upper, options=nhr_options)

    samples, restart_info = nlp_sampling.restarting_nhr_sample(
        phase1=phase1, g=g, lower=lower, upper=upper, Jg=Jg,
        nhr_options=nhr_options, restart_options=restart_options,
    )

    print(f"{len(samples)} samples from {restart_options.num_restarts} restarts")
    # The two regions split roughly along a diagonal in (joint2, joint4);
    # joint2's sign alone isn't a clean separator (the band is diagonal,
    # not vertical), so cluster on the actual samples instead.
    from scipy.cluster.vq import kmeans2
    _, labels = kmeans2(samples, 2, seed=0, minit="++")
    n0, n1 = int(np.sum(labels == 0)), int(np.sum(labels == 1))
    print(f"k-means split: {n0} / {n1} samples in each cluster (both nonzero => both regions found)")
    assert min(n0, n1) > 0.1 * len(samples), (
        f"expected both disjoint regions to be well-represented, got {n0}/{n1} - "
        "the sampler may have missed one region, or the obstacle placement needs revisiting")

    ns, scores_p1 = nlp_sampling.minimum_spanning_tree_score_curve(samples, p=1.0, num_checkpoints=15)
    _, scores_p2 = nlp_sampling.minimum_spanning_tree_score_curve(samples, p=2.0, num_checkpoints=15)
    print("\nn:          ", ns)
    print("MSTS_1(D_n):", np.round(scores_p1, 2))
    print("MSTS_2(D_n):", np.round(scores_p2, 2))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].scatter(samples[:, 0], samples[:, 1], s=8, alpha=0.6)
    axes[0].set_xlabel("joint2")
    axes[0].set_ylabel("joint4")
    axes[0].set_title("Samples (2-DOF Panda collision scenario)")
    axes[0].grid(True, alpha=0.25)

    nlp_sampling.plot_msts_curve(axes[1], samples, p_values=(1.0, 2.0), num_checkpoints=15)

    fig.tight_layout()
    out_path = PROJECT_ROOT / "artifacts" / "tc_space_collision_scenario_baseline.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved plot to {out_path}")


if __name__ == "__main__":
    main()
