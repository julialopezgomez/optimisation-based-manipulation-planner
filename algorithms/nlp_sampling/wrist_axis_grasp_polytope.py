"""
#80: construct a grasp-feasible region directly from the wrist joint's
axis - per 2026-08-25 meeting with supervisor. Two constructions, built
to the SAME small target margin so they're directly comparable:

1. build_simple_margin_polytope - the direct, primary construction: the
   wrist axis (exactly free, full range - see verify_wrist_axis_is_exactly_free)
   x a small, uniform, fixed margin on each of the other 6 arm joints
   independently. This is the literal "thin line + small margin" concept -
   no optimization, no search for how large the margin can be, just a
   small target picked to match the scale of the finger joints' own grasp
   tolerance ([-0.025,-0.024], width 0.001) - see main() for the exact
   target used.

2. build_null_space_polytope - the null-space-of-the-active-constraint-
   Jacobian construction from earlier review, RETARGETED to the same
   small width instead of being volume-maximized (an earlier version of
   this file maximized it, which was answering a different question than
   "keep it small like the fingers" - see tc_space_investigation_summary.md
   for that history). Kept specifically to compare against (1), not
   because it's known to be better at this small scale.

compare_polytopes() runs both at the same target margin and reports how
they actually differ: volumes, and whether one's box is contained inside
the other's - see main()'s output for the actual numbers, not this
docstring, since these are meant to be regenerated, not read once.

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
ACTIVE_G_THRESHOLD = -0.02


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


def _feasible(constraints, q, tol=CONSTRAINT_TOL):
    hv, gv = constraints["h"](q), constraints["g"](q)
    return np.max(np.abs(hv)) < tol and np.max(gv) < tol


# -----------------------------------------------------------------------------
# 1. Simple construction: wrist axis (exact, full range) x small uniform
#    margin on the other 6 arm joints, no volume optimization.
# -----------------------------------------------------------------------------

def build_simple_margin_polytope(constraints: dict, q0: np.ndarray, margin: float):
    """Returns (lower, upper) - a plain axis-aligned box: full wrist range,
    q0[i] +- margin for the 6 arm joints, fingers/cap fixed at q0."""
    idx_wrist = constraints["idx_wrist"]
    lower, upper = q0.copy(), q0.copy()
    for i in ARM_JOINT_INDICES:
        lower[i] -= margin
        upper[i] += margin
    lower[idx_wrist] = constraints["lower"][idx_wrist]
    upper[idx_wrist] = constraints["upper"][idx_wrist]
    return lower, upper


def validate_box(constraints, lower, upper, num_random=3000, seed=1):
    rng = np.random.default_rng(seed)
    worst_h, worst_g, n_fail = 0.0, -np.inf, 0
    corners = list(itertools.product([-1, 1], repeat=len(ARM_JOINT_INDICES)))
    for signs in corners:
        q = 0.5 * (lower + upper)
        for i, s in zip(ARM_JOINT_INDICES, signs):
            q[i] = upper[i] if s > 0 else lower[i]
        q[constraints["idx_wrist"]] = rng.uniform(lower[constraints["idx_wrist"]], upper[constraints["idx_wrist"]])
        hv, gv = constraints["h"](q), constraints["g"](q)
        worst_h, worst_g = max(worst_h, np.max(np.abs(hv))), max(worst_g, np.max(gv))
        if not _feasible(constraints, q):
            n_fail += 1
    for _ in range(num_random):
        q = rng.uniform(lower, upper)
        hv, gv = constraints["h"](q), constraints["g"](q)
        worst_h, worst_g = max(worst_h, np.max(np.abs(hv))), max(worst_g, np.max(gv))
        if not _feasible(constraints, q):
            n_fail += 1
    return {"worst_h": worst_h, "worst_g": worst_g, "n_fail": n_fail, "n_checked": len(corners) + num_random}


# -----------------------------------------------------------------------------
# 2. Null-space construction, retargeted to the SAME small width as (1) -
#    for comparison only, not because it's assumed better at this scale.
# -----------------------------------------------------------------------------

def compute_arm_null_space_direction(constraints: dict, q0: np.ndarray):
    Jh = constraints["Jh"](q0)
    g0 = constraints["g"](q0)
    Jg = constraints["Jg"](q0)
    active_rows = [i for i in range(len(g0)) if g0[i] > ACTIVE_G_THRESHOLD]
    J_active = np.vstack([Jh] + [Jg[i:i + 1] for i in active_rows])
    N = null_space(J_active)
    arm_cols = [j for j in range(N.shape[1]) if np.allclose(N[6:, j], 0.0) and not np.allclose(N[:6, j], 0.0)]
    assert len(arm_cols) == 1, f"expected exactly one non-trivial arm-joint null-space direction, got {len(arm_cols)}"
    direction = N[:6, arm_cols[0]]
    return direction / np.linalg.norm(direction)


def build_null_space_polytope(constraints: dict, q0: np.ndarray, direction: np.ndarray, margin: float):
    """Same target `margin` as build_simple_margin_polytope, but applied
    in the null-space-aligned frame (direction + its 5D orthogonal
    complement) instead of the raw joint axes. Returns a function
    q(t_dir, t_complement) -> full configuration, plus the box bounds
    [-margin, margin] for each of the 6 coordinates in that frame (all
    equal here, unlike the earlier volume-maximized version)."""
    complement = null_space(direction.reshape(1, -1))  # 6x5

    def q_from_offsets(t_dir, t_complement):
        q = q0.copy()
        q[:6] += t_dir * direction + complement @ t_complement
        q[constraints["idx_wrist"]] = q0[constraints["idx_wrist"]]  # set separately by caller
        return q

    return q_from_offsets, complement


def validate_null_space_box(constraints, q0, direction, complement, margin, num_random=3000, seed=1):
    idx_wrist = constraints["idx_wrist"]
    rng = np.random.default_rng(seed)
    worst_h, worst_g, n_fail = 0.0, -np.inf, 0
    n_checked = 0
    for signs in itertools.product([-1, 1], repeat=6):  # t_dir + 5 complement dims, all at +-margin
        q = q0.copy()
        q[:6] += signs[0] * margin * direction + complement @ (margin * np.array(signs[1:]))
        q[idx_wrist] = rng.uniform(constraints["lower"][idx_wrist], constraints["upper"][idx_wrist])
        hv, gv = constraints["h"](q), constraints["g"](q)
        worst_h, worst_g = max(worst_h, np.max(np.abs(hv))), max(worst_g, np.max(gv))
        if not _feasible(constraints, q):
            n_fail += 1
        n_checked += 1
    for _ in range(num_random):
        t_dir = rng.uniform(-margin, margin)
        t_c = rng.uniform(-margin, margin, size=5)
        q = q0.copy()
        q[:6] += t_dir * direction + complement @ t_c
        q[idx_wrist] = rng.uniform(constraints["lower"][idx_wrist], constraints["upper"][idx_wrist])
        hv, gv = constraints["h"](q), constraints["g"](q)
        worst_h, worst_g = max(worst_h, np.max(np.abs(hv))), max(worst_g, np.max(gv))
        if not _feasible(constraints, q):
            n_fail += 1
        n_checked += 1
    return {"worst_h": worst_h, "worst_g": worst_g, "n_fail": n_fail, "n_checked": n_checked}


# -----------------------------------------------------------------------------
# Comparison
# -----------------------------------------------------------------------------

def is_box_corner_inside_rotated_box(point_offset, direction, complement, margin):
    """Is a raw-joint-space offset (from q0) inside the null-space-frame
    box [-margin,margin]^6? Project onto (direction, complement) and check."""
    t_dir = direction @ point_offset
    t_c = complement.T @ point_offset
    return abs(t_dir) <= margin and np.all(np.abs(t_c) <= margin)


def main():
    constraints = build_constraints()
    q0 = find_grasp_seed(constraints)
    print(f"Grasp seed q0 = {np.round(q0, 4)}")
    print(f"h(q0) = {constraints['h'](q0)}, g(q0) = {constraints['g'](q0)}")

    wrist_dev = verify_wrist_axis_is_exactly_free(constraints, q0)
    assert wrist_dev < 1e-9, "wrist joint isn't exactly free here - stop"
    print(f"\nWrist-axis sweep: max deviation = {wrist_dev:.2e} - confirmed exactly free.\n")

    # Target margin: matches the finger joints' own grasp tolerance width
    # (0.001, [-0.025,-0.024]) literally, in radians, per your direction -
    # not derived/optimized, a fixed small target chosen to match that scale.
    MARGIN = 0.001

    print(f"=== 1. Simple construction: wrist axis x +-{MARGIN} rad on each of the other 6 arm joints ===")
    lower, upper = build_simple_margin_polytope(constraints, q0, MARGIN)
    simple_result = validate_box(constraints, lower, upper)
    vol_simple = (2 * MARGIN) ** 6
    print(f"validated over {simple_result['n_checked']} points (64 corners + random interior): "
          f"worst|h|={simple_result['worst_h']:.5f}, worst g={simple_result['worst_g']:.5f}, "
          f"failures={simple_result['n_fail']}")
    print(f"arm-joint-subspace volume: {vol_simple:.3e}")

    print(f"\n=== 2. Null-space construction, same target margin +-{MARGIN} ===")
    direction = compute_arm_null_space_direction(constraints, q0)
    print(f"null-space direction: {np.round(direction, 4)}")
    _, complement = build_null_space_polytope(constraints, q0, direction, MARGIN)
    ns_result = validate_null_space_box(constraints, q0, direction, complement, MARGIN)
    vol_ns = (2 * MARGIN) ** 6  # same box size, just rotated - volume is basis-independent
    print(f"validated over {ns_result['n_checked']} points: worst|h|={ns_result['worst_h']:.5f}, "
          f"worst g={ns_result['worst_g']:.5f}, failures={ns_result['n_fail']}")
    print(f"arm-joint-subspace volume: {vol_ns:.3e} (same as (1) - a rotation doesn't change volume)")

    print(f"\n=== Comparison ===")
    print("At the SAME small target margin, both constructions have identical volume (a rotation "
          "preserves volume) - the null-space alignment only matters when trying to grow the region "
          "as large as possible (see tc_space_investigation_summary.md's Part 2 step 4-5 history), "
          "not at a small fixed scale like this. The real difference at this scale is orientation:")

    rng = np.random.default_rng(2)
    n_test = 2000
    simple_corner_in_ns = 0
    ns_corner_in_simple = 0
    for _ in range(n_test):
        offset = rng.uniform(-MARGIN, MARGIN, size=6)  # random point in the SIMPLE box
        if is_box_corner_inside_rotated_box(offset, direction, complement, MARGIN):
            simple_corner_in_ns += 1
    for _ in range(n_test):
        t_dir = rng.uniform(-MARGIN, MARGIN)
        t_c = rng.uniform(-MARGIN, MARGIN, size=5)
        offset = t_dir * direction + complement @ t_c  # random point in the NULL-SPACE box, in raw joint coords
        if np.all(np.abs(offset) <= MARGIN):
            ns_corner_in_simple += 1
    print(f"  fraction of the simple box's own random points also inside the null-space box: "
          f"{simple_corner_in_ns / n_test:.1%}")
    print(f"  fraction of the null-space box's own random points also inside the simple box: "
          f"{ns_corner_in_simple / n_test:.1%}")
    print("  (100% for both would mean they're the same region; well below 100% means they're "
          "genuinely different-shaped/oriented regions of the same volume, overlapping partially.)")

    print(f"\nUsing construction (1) - the simple, direct wrist-line + small margin - as the answer to #80.")
    print("Region summary:")
    names = constraints["plant"].GetPositionNames()
    for i in range(len(q0)):
        tag = " <- wrist (full range)" if i == constraints["idx_wrist"] else (
            " <- fixed (zero sensitivity)" if i not in ARM_JOINT_INDICES else "")
        print(f"  {names[i]:35s} [{lower[i]:8.4f}, {upper[i]:8.4f}]{tag}")


if __name__ == "__main__":
    main()
