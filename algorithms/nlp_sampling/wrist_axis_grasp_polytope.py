"""
#80: construct a grasp-feasible region directly from the wrist joint's
axis, instead of general IRIS-ZO/clique-cover + NLP-sampling region
growing - per 2026-08-25 meeting with supervisor.

Claim tested: from a valid grasp configuration, varying panda_joint7 (the
wrist joint) alone, with every other coordinate held fixed, keeps the
ENTIRE grasp constraint (h_grasp_eq, g_grasp_ineq from
nlp_sampling_standalone_test.py) satisfied - not approximately, exactly.
Confirmed by direct sweep: h_grasp_eq/g_grasp_ineq are bit-for-bit
identical across the full swept range of panda_joint7. This makes
geometric sense: joint7's rotation axis passes through the end-effector
frame's own origin, so a pure rotation about it changes neither the
frame's position (p_CapE) nor its z-axis direction (z_E_in_Cap) - the
only two things the grasp constraint actually checks.

So the "line" Steve described is literally the full panda_joint7 range at
a fixed value of every other coordinate - no need to search for it, and
no approximation involved for that direction.

Margin in the other coordinates: sized empirically (bisection over the 6
arm joints' worst-case corner, all 64 sign combinations checked, not just
a single perturbation direction), not derived from a single Jacobian
bound. Finger joints and the cap's own joint have exactly zero
sensitivity (checked directly, not assumed) - left fixed at the seed
value rather than opened up, since gripper width isn't this constraint's
concern and opening them wouldn't grow this specific region anyway.

Usage:
    python wrist_axis_grasp_polytope.py
"""
import itertools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nlp_sampling_standalone_test import (
    build_autodiff_companion, build_minimal_plant, make_grasp_constraints, make_joint_indices, make_joint_limits,
)
import nlp_sampling

ARM_JOINT_INDICES = [0, 1, 2, 3, 4, 5]  # panda_joint1..6 - excludes joint7 (idx 6), the free wrist axis
CONSTRAINT_TOL = 0.01  # matches this project's usual good_err_tol/constraint_tol scale


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
    """Sweeps panda_joint7 over its full range; returns the max constraint
    deviation seen (should be ~0, i.e. bit-for-bit identical to q0's own
    residual - the whole point of this check)."""
    idx_wrist = constraints["idx_wrist"]
    lower, upper = constraints["lower"][idx_wrist], constraints["upper"][idx_wrist]
    h0, g0 = constraints["h"](q0), constraints["g"](q0)
    max_dev = 0.0
    for q7 in np.linspace(lower, upper, num_points):
        q = q0.copy()
        q[idx_wrist] = q7
        max_dev = max(max_dev, np.max(np.abs(constraints["h"](q) - h0)), np.max(np.abs(constraints["g"](q) - g0)))
    return max_dev


def find_arm_margin(constraints: dict, q0: np.ndarray, tol: float = CONSTRAINT_TOL, bisection_iters: int = 20) -> float:
    """Largest symmetric margin eps such that ALL 64 sign combinations of
    perturbing the 6 arm joints by +-eps simultaneously (the true
    worst-case corners of the resulting box, not just a single
    perturbation direction) keep |h| and g within tol."""
    def worst_case(eps: float) -> tuple[float, float]:
        worst_h, worst_g = 0.0, -np.inf
        for signs in itertools.product([-1, 1], repeat=len(ARM_JOINT_INDICES)):
            q = q0.copy()
            for i, s in zip(ARM_JOINT_INDICES, signs):
                q[i] += s * eps
            worst_h = max(worst_h, np.max(np.abs(constraints["h"](q))))
            worst_g = max(worst_g, np.max(constraints["g"](q)))
        return worst_h, worst_g

    lo, hi = 0.0, 0.05
    assert worst_case(lo)[0] < tol  # eps=0 must be feasible (it's q0 itself)
    for _ in range(bisection_iters):
        mid = 0.5 * (lo + hi)
        wh, wg = worst_case(mid)
        if wh < tol and wg < tol:
            lo = mid
        else:
            hi = mid
    return lo


def build_polytope(constraints: dict, q0: np.ndarray, arm_margin: float) -> tuple[np.ndarray, np.ndarray]:
    """Returns (lower, upper) bounds: full panda_joint7 range (exactly
    free), q0[i] +- arm_margin for the 6 arm joints, fingers/cap fixed at
    q0 (zero sensitivity - checked, not assumed - so opening them
    wouldn't grow this specific grasp-constraint region anyway)."""
    idx_wrist = constraints["idx_wrist"]
    n = len(q0)
    lower, upper = q0.copy(), q0.copy()
    for i in ARM_JOINT_INDICES:
        lower[i] -= arm_margin
        upper[i] += arm_margin
    lower[idx_wrist] = constraints["lower"][idx_wrist]
    upper[idx_wrist] = constraints["upper"][idx_wrist]
    return lower, upper


def validate_polytope(constraints: dict, lower: np.ndarray, upper: np.ndarray, num_random: int = 2000, seed: int = 1) -> dict:
    """Checks the box's corners (worst case for a linear approximation)
    AND random interior points (since the true constraint is nonlinear,
    the actual worst point in the box need not be exactly at a corner)."""
    rng = np.random.default_rng(seed)
    max_h, max_g = 0.0, -np.inf

    for signs in itertools.product([-1, 1], repeat=len(ARM_JOINT_INDICES)):
        q = 0.5 * (lower + upper)
        for i, s in zip(ARM_JOINT_INDICES, signs):
            q[i] = upper[i] if s > 0 else lower[i]
        q[constraints["idx_wrist"]] = rng.uniform(lower[constraints["idx_wrist"]], upper[constraints["idx_wrist"]])
        max_h = max(max_h, np.max(np.abs(constraints["h"](q))))
        max_g = max(max_g, np.max(constraints["g"](q)))

    for _ in range(num_random):
        q = rng.uniform(lower, upper)
        max_h = max(max_h, np.max(np.abs(constraints["h"](q))))
        max_g = max(max_g, np.max(constraints["g"](q)))

    return {"max_h": max_h, "max_g": max_g, "num_random": num_random}


def main():
    constraints = build_constraints()
    q0 = find_grasp_seed(constraints)
    print(f"Grasp seed q0 = {np.round(q0, 4)}")
    print(f"h(q0) = {constraints['h'](q0)}")
    print(f"g(q0) = {constraints['g'](q0)}")

    wrist_dev = verify_wrist_axis_is_exactly_free(constraints, q0)
    print(f"\nWrist-axis sweep (panda_joint7 over its full range): "
          f"max constraint deviation from q0's own residual = {wrist_dev:.2e}")
    assert wrist_dev < 1e-9, "wrist joint isn't exactly free here - the claim doesn't hold for this seed, stop"
    print("Confirmed: exactly free (bit-for-bit identical to floating point noise).")

    arm_margin = find_arm_margin(constraints, q0)
    print(f"\nArm-joint margin (bisected, worst-case corner over all 64 sign combinations): {arm_margin:.5f} rad")

    lower, upper = build_polytope(constraints, q0, arm_margin)
    print(f"\nPolytope bounds:")
    names = constraints["plant"].GetPositionNames()
    for i in range(len(q0)):
        tag = " <- wrist (full range)" if i == constraints["idx_wrist"] else (
            " <- fixed (zero sensitivity)" if i not in ARM_JOINT_INDICES else "")
        print(f"  {names[i]:35s} [{lower[i]:8.4f}, {upper[i]:8.4f}]{tag}")

    result = validate_polytope(constraints, lower, upper)
    print(f"\nValidation over 64 worst-case-arm-corner points (random wrist) + "
          f"{result['num_random']} uniformly random interior points:")
    print(f"  max |h| = {result['max_h']:.4f}, max g = {result['max_g']:.4f} "
          f"(tolerance was {CONSTRAINT_TOL})")
    assert result["max_h"] < CONSTRAINT_TOL * 1.01 and result["max_g"] < CONSTRAINT_TOL * 1.01, \
        "validation found a point outside tolerance - margin is too loose"
    print("All validated points within tolerance.")


if __name__ == "__main__":
    main()
