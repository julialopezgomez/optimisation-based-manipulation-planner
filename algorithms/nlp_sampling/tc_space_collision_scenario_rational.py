"""
TC-space (rational forward kinematics) version of the same collision
constraint as tc_space_collision_scenario.py, for #79. Same physical
scenario - same panda_link5 obstacle, center=(0.25,0,0.65), radius=0.35 -
but built as a genuine ratio of polynomials in Drake's TC-space "s"
coordinates (RationalForwardKinematics.CalcBodyPoseAsMultilinearPolynomial
+ ConvertMultilinearPolynomialToRationalFunction), not a numerical
q(s)=q_star+2*arctan(s) substitution reusing the trig-based constraint -
see #79 for why that distinction matters (only the rational-polynomial
form is actually the thing the "TC-space is less nonlinear" hypothesis is
about).

Squared distance, not raw distance, is used throughout (radius^2 - dist^2
<= 0, equivalent to dist >= radius since both sides are nonnegative) -
distance itself would need a square root, which isn't a rational
function, so squared distance is the natural rational formulation.

q_star = zeros(7). This matches the textbook stereographic-projection
definition exactly: t_i = tan(theta_i / 2), no offset at all. Drake's API
requires a q_star argument (it generalizes the base formula to
t_i = tan((theta_i - theta*_i)/2) for its own purposes, e.g. growing
certified regions around an arbitrary seed posture), but with q_star=0
that generalization contributes nothing - Drake computes exactly
tan(theta_i/2), matching the reference definition term for term. An
earlier version of this file used a per-joint-recentered q_star (each
joint's own range midpoint) to keep the resulting s-domain bounded and
symmetric - that was not the standard definition, was not sourced from
Drake's own convention or the literature, and was reverted (see git log)
because it suppressed the actual, intended behavior of the tangent-
half-angle map (s diverging toward the joint's range boundary), which is
the exact phenomenon the TC-space hypothesis is about.

Real, honest consequence of q_star=0 on this scenario: panda_joint4's
actual range ([-3.07, -0.07]) sits almost entirely on one side of theta=0,
right up against the tan(theta/2) asymptote at theta=-pi - theta=0 isn't
even inside joint4's mechanical range at all. So s(joint4) spans roughly
[-28.6, -0.03] here: about 800x wider than s(joint2)'s [-1.21, 1.21].
This is not a bug in this file - it is what the textbook formula actually
computes for this joint's actual range. It has real, likely-negative
consequences for how well a fixed-step-size sampler can explore this
domain, independent of anything about TC-space's merits - see the
comparison scripts for how this plays out.

Usage:
    python tc_space_collision_scenario_rational.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tc_space_collision_scenario import OBSTACLE_CENTER, OBSTACLE_RADIUS
from tc_space_collision_scenario import build_scene as build_regular_scene
from tc_space_collision_scenario import make_collision_constraint as make_regular_constraint

from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.rational import RationalForwardKinematics


def build_rational_scene():
    plant = MultibodyPlant(time_step=0.0)
    scene_graph = SceneGraph()
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    panda_arm = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_arm.urdf")[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0", panda_arm), RigidTransform())
    plant.Finalize()

    rat_fk = RationalForwardKinematics(plant)
    idx_j2 = plant.GetJointByName("panda_joint2", panda_arm).position_start()
    idx_j4 = plant.GetJointByName("panda_joint4", panda_arm).position_start()

    q_star = np.zeros(plant.num_positions())  # t_i = tan(theta_i / 2), the textbook definition exactly

    # Find which index of rat_fk.s() corresponds to joint2/joint4 by probing -
    # not assumed from position index, per ComputeSValue's own docstring.
    s_at_star = rat_fk.ComputeSValue(q_star, q_star)
    assert np.allclose(s_at_star, 0.0)

    def probe(idx_q, delta=0.2):
        q = q_star.copy()
        q[idx_q] += delta
        s = rat_fk.ComputeSValue(q, q_star)
        return int(np.argmax(np.abs(s)))

    idx_s_j2 = probe(idx_j2)
    idx_s_j4 = probe(idx_j4)
    assert idx_s_j2 != idx_s_j4

    return dict(
        plant=plant, panda_arm=panda_arm, rat_fk=rat_fk, q_star=q_star,
        idx_j2=idx_j2, idx_j4=idx_j4, idx_s_j2=idx_s_j2, idx_s_j4=idx_s_j4,
    )


def make_rational_collision_constraint(scene: dict):
    """
    Returns (g, Jg, lower, upper) for the TC-space version of the 2-DOF
    collision constraint. g/Jg operate directly on s-space coordinates
    x = (s_j2, s_j4); lower/upper are the corresponding s-space box
    (ComputeSValue applied to the joint limits).
    """
    plant, panda_arm = scene["plant"], scene["panda_arm"]
    rat_fk, q_star = scene["rat_fk"], scene["q_star"]
    idx_j2, idx_j4 = scene["idx_j2"], scene["idx_j4"]
    idx_s_j2, idx_s_j4 = scene["idx_s_j2"], scene["idx_s_j4"]

    s_vars = rat_fk.s()
    link5_body = plant.GetBodyByName("panda_link5", panda_arm)
    world_body = plant.world_body()

    pose = rat_fk.CalcBodyPoseAsMultilinearPolynomial(q_star, link5_body.index(), world_body.index())
    pos_poly = pose.position()
    rf_pos = [rat_fk.ConvertMultilinearPolynomialToRationalFunction(pos_poly[i]) for i in range(3)]

    dist_sq = rf_pos[0] * 0.0
    for i in range(3):
        diff = rf_pos[i] - float(OBSTACLE_CENTER[i])
        dist_sq = dist_sq + diff * diff
    g_rational = float(OBSTACLE_RADIUS ** 2) - dist_sq

    N, D = g_rational.numerator(), g_rational.denominator()
    dN_ds2 = N.Differentiate(s_vars[idx_s_j2])
    dD_ds2 = D.Differentiate(s_vars[idx_s_j2])
    dN_ds4 = N.Differentiate(s_vars[idx_s_j4])
    dD_ds4 = D.Differentiate(s_vars[idx_s_j4])

    def make_env(x):
        env = {s_vars[i]: 0.0 for i in range(len(s_vars))}
        env[s_vars[idx_s_j2]] = float(x[0])
        env[s_vars[idx_s_j4]] = float(x[1])
        return env

    def g(x):
        env = make_env(x)
        return np.array([N.Evaluate(env) / D.Evaluate(env)])

    def Jg(x):
        env = make_env(x)
        n_val, d_val = N.Evaluate(env), D.Evaluate(env)
        dg_ds2 = (dN_ds2.Evaluate(env) * d_val - n_val * dD_ds2.Evaluate(env)) / d_val ** 2
        dg_ds4 = (dN_ds4.Evaluate(env) * d_val - n_val * dD_ds4.Evaluate(env)) / d_val ** 2
        return np.array([[dg_ds2, dg_ds4]])

    joint2 = plant.GetJointByName("panda_joint2", panda_arm)
    joint4 = plant.GetJointByName("panda_joint4", panda_arm)
    q_lower = np.array([joint2.position_lower_limits()[0], joint4.position_lower_limits()[0]])
    q_upper = np.array([joint2.position_upper_limits()[0], joint4.position_upper_limits()[0]])

    def s_bounds(q2, q4):
        q = q_star.copy()
        q[idx_j2], q[idx_j4] = q2, q4
        s = rat_fk.ComputeSValue(q, q_star)
        return s[idx_s_j2], s[idx_s_j4]

    s2_lo, s4_lo = s_bounds(q_lower[0], q_lower[1])
    s2_hi, s4_hi = s_bounds(q_upper[0], q_upper[1])
    lower = np.array([min(s2_lo, s2_hi), min(s4_lo, s4_hi)])
    upper = np.array([max(s2_lo, s2_hi), max(s4_lo, s4_hi)])

    return g, Jg, lower, upper


def _validate_against_regular_fk(rational_scene, regular_scene, num_trials=8, seed=0):
    """Cross-check: at matching q, g_rational(s(q)) must equal g_regular(q)
    exactly (up to floating point) - same physical obstacle, same distance,
    just reached via a different symbolic route."""
    g_reg, _, _, _, embed = make_regular_constraint(regular_scene)
    g_rat, _, _, _ = make_rational_collision_constraint(rational_scene)
    rat_fk, q_star = rational_scene["rat_fk"], rational_scene["q_star"]
    idx_j2, idx_j4 = rational_scene["idx_j2"], rational_scene["idx_j4"]
    idx_s_j2, idx_s_j4 = rational_scene["idx_s_j2"], rational_scene["idx_s_j4"]

    joint2 = regular_scene["plant"].GetJointByName("panda_joint2", regular_scene["panda_arm"])
    joint4 = regular_scene["plant"].GetJointByName("panda_joint4", regular_scene["panda_arm"])
    rng = np.random.default_rng(seed)
    max_err = 0.0
    for _ in range(num_trials):
        q2 = rng.uniform(joint2.position_lower_limits()[0], joint2.position_upper_limits()[0])
        q4 = rng.uniform(joint4.position_lower_limits()[0], joint4.position_upper_limits()[0])

        g_reg_val = g_reg(np.array([q2, q4]))[0]

        q_full = q_star.copy()
        q_full[idx_j2], q_full[idx_j4] = q2, q4
        s_full = rat_fk.ComputeSValue(q_full, q_star)
        g_rat_val = g_rat(np.array([s_full[idx_s_j2], s_full[idx_s_j4]]))[0]

        err = abs(g_reg_val - g_rat_val)
        max_err = max(max_err, err)
    return max_err


def main():
    rational_scene = build_rational_scene()
    regular_scene = build_regular_scene()

    max_err = _validate_against_regular_fk(rational_scene, regular_scene)
    print(f"Cross-check vs. regular FK: max |g_regular - g_rational| over 8 random points = {max_err:.2e}")
    assert max_err < 1e-8, "TC-space constraint doesn't match the regular-FK constraint - do not trust results below"
    print("Validated: TC-space constraint (q_star=0) matches regular FK exactly.\n")

    g, Jg, lower, upper = make_rational_collision_constraint(rational_scene)
    print(f"s-space domain: joint2 in [{lower[0]:.3f}, {upper[0]:.3f}], joint4 in [{lower[1]:.3f}, {upper[1]:.3f}]")
    print(f"domain diagonal: {np.linalg.norm(upper - lower):.3f}")


if __name__ == "__main__":
    main()
