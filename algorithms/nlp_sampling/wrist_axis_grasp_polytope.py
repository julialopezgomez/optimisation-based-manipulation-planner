"""
#80: construct a grasp-feasible region directly from the wrist joint's
axis and the constraint's null space, instead of general IRIS-ZO/
clique-cover + NLP-sampling region growing - per 2026-08-25 meeting with
supervisor, refined per review (the null-space suggestion below).

Part 1 - exact, zero-search directions (kinematic facts, not approximations):
From a valid grasp configuration q0, four coordinates can move over their
ENTIRE range with zero effect on h_grasp_eq/g_grasp_ineq
(nlp_sampling_standalone_test.py), confirmed by direct sweep (bit-for-bit
identical residuals, not just small):
  - panda_joint7 (the wrist): its rotation axis passes through the
    end-effector frame's own origin, so a pure rotation about it changes
    neither the frame's position (p_CapE) nor its z-axis direction
    (z_E_in_Cap) - the only two things the grasp constraint checks.
  - both finger joints and the cap's own joint: zero Jacobian entries
    (checked directly) - they don't enter the constraint at all.

Part 2 - the null space of the active constraint Jacobian, an idea from
review that improves substantially on a naive axis-aligned box: an
axis-aligned margin on the remaining 6 arm joints is limited by
whichever single joint is most sensitive (joint2/joint3 here, at
~0.98-0.99), even though most DIRECTIONS in that 6D space aren't nearly
that sensitive. The null space of J_active = [Jh; Jg[active rows]] at q0
(rank 5, out of 6 arm-joint dimensions) contains one non-trivial
direction - not aligned with any single joint - that the FIRST-ORDER
Jacobian analysis says is exactly free. Empirically (not just linearized)
it stays within a much looser tolerance over a MUCH larger range: ~0.87
total vs. the axis-aligned box's ~0.007, about two orders of magnitude
more usable extent in that one direction alone.

The two ranges (this direction, and margin in its 5D orthogonal
complement within the arm joints) are NOT independent - checked jointly,
not assumed: pushing the null-space direction to its own solo limit
leaves zero room for any complement margin at all. The final numbers
below were found by jointly bisecting both together (a small tolerance-
budget sweep, see find_null_space_direction_and_margins), not by
independently maximizing each and hoping they compose.

Usage:
    python wrist_axis_grasp_polytope.py
"""
import itertools
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import null_space

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nlp_sampling_standalone_test import (
    build_autodiff_companion, build_minimal_plant, make_grasp_constraints, make_joint_indices, make_joint_limits,
)
import nlp_sampling

ARM_JOINT_INDICES = [0, 1, 2, 3, 4, 5]  # panda_joint1..6 - excludes joint7 (idx 6), the exactly-free wrist axis
CONSTRAINT_TOL = 0.01  # matches this project's usual good_err_tol/constraint_tol scale
ACTIVE_G_THRESHOLD = -0.02  # g rows this close to 0 are treated as active when building J_active


def build_constraints():
    scene = build_minimal_plant()
    plant, plant_context = scene["plant"], scene["plant_context"]
    panda_arm, panda_hand, cap = scene["panda_arm"], scene["panda_hand"], scene["cap"]
    E = plant.GetFrameByName("panda_hand", panda_hand)
    Cap = plant.GetFrameByName("base_link", cap)
    plant_ad, plant_context_ad, E_ad, Cap_ad = build_autodiff_companion(plant, E, Cap)
    h_grasp_eq, g_grasp_ineq, h_jacobian_eq, g_jacobian_ineq = make_grasp_constraints(
        plant, plant_context, plant_ad, plant_context_ad, E, Cap, E_ad, Cap_ad)
    idx = make_joint_indices(plant, panda_arm, panda_hand, cap)
    lower, upper = make_joint_limits(plant, idx["idx_wrist"], idx["idx_f1"], idx["idx_f2"])
    return dict(
        plant=plant, h=h_grasp_eq, g=g_grasp_ineq, Jh=h_jacobian_eq, Jg=g_jacobian_ineq,
        idx_wrist=idx["idx_wrist"], lower=lower, upper=upper,
    )


def find_grasp_seed(constraints: dict, max_trials: int = 100, seed: int = 0):
    nhr_options = nlp_sampling.NHROptions(random_seed=seed, verbose=False)
    rng = np.random.default_rng(seed)
    for _ in range(max_trials):
        probe = nlp_sampling.sample_uniform_box(constraints["lower"], constraints["upper"], rng)
        q0 = nlp_sampling.run_downhill_phase1(
            probe, g=constraints["g"], Jg=constraints["Jg"], lower=constraints["lower"], upper=constraints["upper"],
            options=nhr_options, h=constraints["h"], Jh=constraints["Jh"])
        if q0 is not None:
            return q0
    raise RuntimeError(f"No feasible grasp seed found in {max_trials} trials")


def verify_wrist_axis_is_exactly_free(constraints: dict, q0: np.ndarray, num_points: int = 21) -> float:
    idx_wrist = constraints["idx_wrist"]
    lower, upper = constraints["lower"][idx_wrist], constraints["upper"][idx_wrist]
    h0, g0 = constraints["h"](q0), constraints["g"](q0)
    max_dev = 0.0
    for q7 in np.linspace(lower, upper, num_points):
        q = q0.copy()
        q[idx_wrist] = q7
        max_dev = max(max_dev, np.max(np.abs(constraints["h"](q) - h0)), np.max(np.abs(constraints["g"](q) - g0)))
    return max_dev


def compute_arm_null_space(constraints: dict, q0: np.ndarray):
    """Returns (direction, complement): `direction` is the unit-norm,
    non-trivial null-space direction in the 6 arm-joint coordinates
    (beyond the trivially-zero wrist/finger/cap columns, already handled
    separately); `complement` is an orthonormal basis (6x5) for its
    orthogonal complement within the 6 arm joints."""
    Jh = constraints["Jh"](q0)
    g0 = constraints["g"](q0)
    Jg = constraints["Jg"](q0)
    active_rows = [i for i in range(len(g0)) if g0[i] > ACTIVE_G_THRESHOLD]
    J_active = np.vstack([Jh] + [Jg[i:i + 1] for i in active_rows])
    N = null_space(J_active)
    # Columns of N corresponding to the trivially-zero coordinates (wrist,
    # fingers, cap) appear as their own basis vectors, since a fully-zero
    # column of J_active trivially puts e_i in the null space - the one
    # remaining column mixes only the 6 arm joints.
    arm_cols = [j for j in range(N.shape[1]) if np.allclose(N[6:, j], 0.0) and not np.allclose(N[:6, j], 0.0)]
    assert len(arm_cols) == 1, f"expected exactly one non-trivial arm-joint null-space direction, got {len(arm_cols)}"
    direction = N[:6, arm_cols[0]]
    direction = direction / np.linalg.norm(direction)
    complement = null_space(direction.reshape(1, -1))
    return direction, complement


def find_null_space_direction_and_margins(constraints, q0, direction, complement, alpha: float = 0.2):
    """Jointly sizes (a) the extent along `direction` and (b) a uniform
    margin over `complement`, so that the two compose validly - checked
    together at every step, not assumed independent (an earlier attempt
    at this found independently-maximized extents do NOT combine safely:
    pushing `direction` to its own solo limit left zero room for any
    complement margin). `alpha` allocates a fraction of the tolerance
    budget to the direction sweep (bisected first, at a tighter interim
    tolerance) before bisecting the complement margin against the full
    tolerance, jointly. alpha=0.2 was chosen by a small sweep favoring
    total volume - see git history for the sweep."""
    def q_from_offsets(t_dir, t_complement):
        q = q0.copy()
        q[:6] += t_dir * direction + complement @ t_complement
        return q

    def feasible(q, tol):
        hv, gv = constraints["h"](q), constraints["g"](q)
        return np.max(np.abs(hv)) < tol and np.max(gv) < tol

    def bisect_direction(budget_tol):
        def feas(t):
            return feasible(q_from_offsets(t, np.zeros(5)), budget_tol)
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if feas(mid) else (lo, mid)
        t_hi = lo
        lo, hi = 0.0, -1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if feas(mid) else (lo, mid)
        t_lo = lo
        return t_lo, t_hi

    def jointly_feasible(t_lo, t_hi, eps, n_samples=11):
        for t_dir in np.linspace(t_lo, t_hi, n_samples):
            for signs in itertools.product([-1, 1], repeat=5):
                if not feasible(q_from_offsets(t_dir, eps * np.array(signs)), CONSTRAINT_TOL):
                    return False
        return True

    def bisect_complement(t_lo, t_hi, hi_guess=0.01):
        lo, hi = 0.0, hi_guess
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if jointly_feasible(t_lo, t_hi, mid) else (lo, mid)
        return lo

    t_lo, t_hi = bisect_direction(alpha * CONSTRAINT_TOL)
    eps = bisect_complement(t_lo, t_hi)
    return t_lo, t_hi, eps


def validate(constraints, q0, direction, complement, t_lo, t_hi, eps, idx_wrist, num_random=5000, seed=1):
    rng = np.random.default_rng(seed)
    worst_h, worst_g, n_fail = 0.0, -np.inf, 0
    for _ in range(num_random):
        t_dir = rng.uniform(t_lo, t_hi)
        t_c = rng.uniform(-eps, eps, size=5)
        q7 = rng.uniform(constraints["lower"][idx_wrist], constraints["upper"][idx_wrist])
        q = q0.copy()
        q[:6] += t_dir * direction + complement @ t_c
        q[idx_wrist] = q7
        hv, gv = constraints["h"](q), constraints["g"](q)
        worst_h = max(worst_h, np.max(np.abs(hv)))
        worst_g = max(worst_g, np.max(gv))
        if not (np.max(np.abs(hv)) < CONSTRAINT_TOL and np.max(gv) < CONSTRAINT_TOL):
            n_fail += 1
    return {"worst_h": worst_h, "worst_g": worst_g, "n_fail": n_fail, "num_random": num_random}


def main():
    constraints = build_constraints()
    q0 = find_grasp_seed(constraints)
    print(f"Grasp seed q0 = {np.round(q0, 4)}")
    print(f"h(q0) = {constraints['h'](q0)}, g(q0) = {constraints['g'](q0)}")

    wrist_dev = verify_wrist_axis_is_exactly_free(constraints, q0)
    assert wrist_dev < 1e-9, "wrist joint isn't exactly free here - stop, the claim doesn't hold for this seed"
    print(f"\nWrist-axis sweep: max deviation = {wrist_dev:.2e} - confirmed exactly free.")

    direction, complement = compute_arm_null_space(constraints, q0)
    print(f"\nNull-space direction (arm joints 1-6): {np.round(direction, 4)}")

    t_lo, t_hi, eps = find_null_space_direction_and_margins(constraints, q0, direction, complement)
    print(f"Jointly-validated extents: direction range=[{t_lo:.4f}, {t_hi:.4f}] "
          f"(width {t_hi - t_lo:.4f}), complement margin=+-{eps:.5f} (5 dims)")

    old_margin = 0.00353  # from the original axis-aligned box, for comparison
    vol_old = (2 * old_margin) ** 6
    vol_new = (t_hi - t_lo) * (2 * eps) ** 5
    print(f"\narm-joint-subspace volume: axis-aligned box={vol_old:.3e}, null-space-aligned region={vol_new:.3e} "
          f"({vol_new / vol_old:.1f}x)")

    idx_wrist = constraints["idx_wrist"]
    result = validate(constraints, q0, direction, complement, t_lo, t_hi, eps, idx_wrist)
    print(f"\nValidation ({result['num_random']} random points spanning the full region, including the wrist's "
          f"full range): worst |h|={result['worst_h']:.4f}, worst g={result['worst_g']:.4f}, "
          f"failures={result['n_fail']}")
    assert result["n_fail"] == 0, "validation found infeasible points - region is too loose, do not trust it"
    print("All validated points within tolerance.")

    print("\nRegion summary:")
    print(f"  panda_joint7 (wrist): full range {constraints['lower'][idx_wrist]} to {constraints['upper'][idx_wrist]} (exact)")
    print(f"  fingers + cap joint: fixed at q0 (exact, zero sensitivity)")
    print(f"  arm joints 1-6: q0 + t*direction + complement@c, "
          f"t in [{t_lo:.4f},{t_hi:.4f}], c in [-{eps:.5f},{eps:.5f}]^5")


if __name__ == "__main__":
    main()
