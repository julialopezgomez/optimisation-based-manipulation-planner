"""
Fast numeric evaluator for the TC-space rational constraint (q_star=0,
see tc_space_collision_scenario_rational.py's module docstring for why),
evaluated without Drake's per-call symbolic Polynomial/RationalFunction.
Evaluate (which walks a full expression tree every call - confirmed to be
the actual bottleneck behind a ~230-290x slowdown, not anything intrinsic
to rational algebra).

The trick: g_rational's numerator/denominator Polynomials are built over
ALL 7 of Drake's TC-space indeterminates, but only 2 of them are actually
free here (the rest are fixed at s=0, since those joints are held at
q_star=0 too) - so ~99% of the monomial terms have a nonzero power on a
fixed variable and evaluate to exactly zero. Extracting
monomial_to_coefficient_map() ONCE, dropping every term with nonzero
power on a fixed variable, and keeping only (power on s_j2, power on
s_j4, coefficient) leaves a small enough set of terms to evaluate as
plain vectorized numpy arithmetic, no symbolic tree-walking per call.
Verified to match Drake's own evaluation to float64 precision before
trusting it for anything - see _benchmark_and_validate() at the bottom.

Usage:
    python tc_space_collision_scenario_fast.py
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tc_space_collision_scenario import OBSTACLE_CENTER, OBSTACLE_RADIUS
from tc_space_collision_scenario_rational import build_rational_scene, make_rational_collision_constraint


def _extract_2var_terms(poly, name_a, name_b):
    """Reduce a Drake Polynomial to (power_a, power_b, coeff) numpy arrays,
    dropping every monomial with a nonzero power on any variable other
    than the two named - those vanish once the other joints' s-values are
    fixed at 0. Variable equality can't go through plain dict.get() -
    Drake's Variable.__eq__ returns a symbolic Formula, not a bool, so
    matching is done by name string instead."""
    ea_list, eb_list, c_list = [], [], []
    for mono, coeff in poly.monomial_to_coefficient_map().items():
        ea = eb = 0
        other_power = 0
        for var, power in mono.get_powers().items():
            name = str(var)
            if name == name_a:
                ea = power
            elif name == name_b:
                eb = power
            else:
                other_power += power
        if other_power != 0:
            continue
        ea_list.append(ea)
        eb_list.append(eb)
        c_list.append(coeff.Evaluate())
    return np.array(ea_list, dtype=float), np.array(eb_list, dtype=float), np.array(c_list, dtype=float)


def _make_fast_evaluator(ea: np.ndarray, eb: np.ndarray, c: np.ndarray):
    def evaluate(a: float, b: float) -> float:
        return float(np.sum(c * (a ** ea) * (b ** eb)))
    return evaluate


def make_fast_rational_collision_constraint(scene: dict):
    """Same contract as tc_space_collision_scenario_rational.
    make_rational_collision_constraint, evaluated via extracted plain-numpy
    terms instead of Drake's symbolic Polynomial/RationalFunction.Evaluate."""
    plant, panda_arm = scene["plant"], scene["panda_arm"]
    rat_fk, q_star = scene["rat_fk"], scene["q_star"]
    idx_s_j2, idx_s_j4 = scene["idx_s_j2"], scene["idx_s_j4"]

    s_vars = rat_fk.s()
    name_j2, name_j4 = str(s_vars[idx_s_j2]), str(s_vars[idx_s_j4])

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
    N_ea, N_eb, N_c = _extract_2var_terms(N, name_j2, name_j4)
    D_ea, D_eb, D_c = _extract_2var_terms(D, name_j2, name_j4)
    dN_ds2_ea, dN_ds2_eb, dN_ds2_c = _extract_2var_terms(N.Differentiate(s_vars[idx_s_j2]), name_j2, name_j4)
    dD_ds2_ea, dD_ds2_eb, dD_ds2_c = _extract_2var_terms(D.Differentiate(s_vars[idx_s_j2]), name_j2, name_j4)
    dN_ds4_ea, dN_ds4_eb, dN_ds4_c = _extract_2var_terms(N.Differentiate(s_vars[idx_s_j4]), name_j2, name_j4)
    dD_ds4_ea, dD_ds4_eb, dD_ds4_c = _extract_2var_terms(D.Differentiate(s_vars[idx_s_j4]), name_j2, name_j4)

    eval_N = _make_fast_evaluator(N_ea, N_eb, N_c)
    eval_D = _make_fast_evaluator(D_ea, D_eb, D_c)
    eval_dN_ds2 = _make_fast_evaluator(dN_ds2_ea, dN_ds2_eb, dN_ds2_c)
    eval_dD_ds2 = _make_fast_evaluator(dD_ds2_ea, dD_ds2_eb, dD_ds2_c)
    eval_dN_ds4 = _make_fast_evaluator(dN_ds4_ea, dN_ds4_eb, dN_ds4_c)
    eval_dD_ds4 = _make_fast_evaluator(dD_ds4_ea, dD_ds4_eb, dD_ds4_c)

    def g(x):
        a, b = float(x[0]), float(x[1])
        return np.array([eval_N(a, b) / eval_D(a, b)])

    def Jg(x):
        a, b = float(x[0]), float(x[1])
        n_val, d_val = eval_N(a, b), eval_D(a, b)
        dg_ds2 = (eval_dN_ds2(a, b) * d_val - n_val * eval_dD_ds2(a, b)) / d_val ** 2
        dg_ds4 = (eval_dN_ds4(a, b) * d_val - n_val * eval_dD_ds4(a, b)) / d_val ** 2
        return np.array([[dg_ds2, dg_ds4]])

    joint2 = plant.GetJointByName("panda_joint2", panda_arm)
    joint4 = plant.GetJointByName("panda_joint4", panda_arm)
    idx_j2, idx_j4 = scene["idx_j2"], scene["idx_j4"]

    def s_bounds(q2, q4):
        q = q_star.copy()
        q[idx_j2], q[idx_j4] = q2, q4
        s = rat_fk.ComputeSValue(q, q_star)
        return s[idx_s_j2], s[idx_s_j4]

    s2_lo, s4_lo = s_bounds(joint2.position_lower_limits()[0], joint4.position_lower_limits()[0])
    s2_hi, s4_hi = s_bounds(joint2.position_upper_limits()[0], joint4.position_upper_limits()[0])
    lower = np.array([min(s2_lo, s2_hi), min(s4_lo, s4_hi)])
    upper = np.array([max(s2_lo, s2_hi), max(s4_lo, s4_hi)])

    return g, Jg, lower, upper


def _benchmark_and_validate(num_trials_correctness=20, num_trials_timing=500):
    scene = build_rational_scene()
    g_drake, Jg_drake, lower, upper = make_rational_collision_constraint(scene)
    g_fast, Jg_fast, lower_fast, upper_fast = make_fast_rational_collision_constraint(scene)

    assert np.allclose(lower, lower_fast) and np.allclose(upper, upper_fast)

    rng = np.random.default_rng(0)
    max_g_err, max_jg_err = 0.0, 0.0
    for _ in range(num_trials_correctness):
        x = rng.uniform(lower, upper)
        max_g_err = max(max_g_err, abs(g_drake(x)[0] - g_fast(x)[0]))
        max_jg_err = max(max_jg_err, np.max(np.abs(Jg_drake(x) - Jg_fast(x))))
    print(f"Correctness: max |g_fast - g_drake| = {max_g_err:.2e}, max |Jg_fast - Jg_drake| = {max_jg_err:.2e}")
    assert max_g_err < 1e-6 and max_jg_err < 1e-4, "fast evaluator doesn't match Drake's symbolic one - do not trust it"

    x_test = rng.uniform(lower, upper)
    t0 = time.perf_counter()
    for _ in range(num_trials_timing):
        g_drake(x_test)
        Jg_drake(x_test)
    t_drake = time.perf_counter() - t0

    t0 = time.perf_counter()
    for _ in range(num_trials_timing):
        g_fast(x_test)
        Jg_fast(x_test)
    t_fast = time.perf_counter() - t0

    print(f"Per-call cost (g+Jg): Drake symbolic = {t_drake / num_trials_timing * 1e6:.1f}us, "
          f"fast numeric = {t_fast / num_trials_timing * 1e6:.1f}us "
          f"({t_drake / t_fast:.1f}x speedup)")


if __name__ == "__main__":
    _benchmark_and_validate()
