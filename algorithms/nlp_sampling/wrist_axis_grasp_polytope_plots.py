"""
Plots + Drake HPolyhedron export for wrist_axis_grasp_polytope.py (#80).

Builds the same simple-margin and null-space regions as
wrist_axis_grasp_polytope.py (same q0, same margin=0.001), then:
  1. Plots the wrist-axis exactness sweep.
  2. Plots 2D projections of the simple (axis-aligned) vs. null-space
     (rotated) regions to show what "same volume, ~65% overlap,
     different shape" actually looks like.
  3. Plots the constraint residuals of random samples from each region
     against the tolerance, to show the actual feasibility headroom.
  4. Shows exactly how to turn each region into a Drake HPolyhedron
     (A @ x <= b) - the simple box is MakeBox; the null-space region
     needs explicit (A, b) since it's a box in a rotated frame, not
     axis-aligned. Only over the 7 "live" DOF (6 arm joints + wrist) -
     fingers/cap are held at a fixed value alongside the polytope, not
     folded into it (a polytope needs nonzero volume in every dimension
     it's built over; a pinned coordinate isn't a shape decision, it's a
     separate fact to carry alongside the polytope).

Usage:
    python wrist_axis_grasp_polytope_plots.py
"""
import sys
from pathlib import Path

import numpy as np
from pydrake.geometry.optimization import HPolyhedron

sys.path.insert(0, str(Path(__file__).resolve().parent))
from wrist_axis_grasp_polytope import (
    ARM_JOINT_INDICES, CONSTRAINT_TOL, build_constraints, compute_arm_null_space_direction,
    find_grasp_seed, verify_wrist_axis_is_exactly_free,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MARGIN = 0.001


def build_simple_hpolyhedron(constraints, q0, margin):
    """The 7-DOF (6 arm joints + wrist) simple box as a Drake HPolyhedron.
    Axis-aligned, so this is just MakeBox - no rotation involved."""
    idx_wrist = constraints["idx_wrist"]
    lower7 = np.array([q0[i] - margin for i in ARM_JOINT_INDICES] + [constraints["lower"][idx_wrist]])
    upper7 = np.array([q0[i] + margin for i in ARM_JOINT_INDICES] + [constraints["upper"][idx_wrist]])
    return HPolyhedron.MakeBox(lower7, upper7), lower7, upper7


def build_null_space_hpolyhedron(constraints, q0, direction, complement, margin):
    """The 7-DOF null-space-aligned region as a Drake HPolyhedron. This
    one genuinely needs explicit (A, b): it's a box in a ROTATED frame
    for the 6 arm joints (direction + its 5D orthogonal complement),
    times a plain interval for the wrist (which isn't part of the
    rotation - the wrist axis is already its own, separate exact
    direction). Ordering of the 7 columns matches ARM_JOINT_INDICES + [wrist].

    For each rotated axis v (a unit vector in the 6 arm-joint coordinates),
    the pair of halfspace constraints "-margin <= v . (q_arm - q0_arm) <= margin"
    becomes two rows: [v, 0] . x <= margin + v.q0_arm, and
    [-v, 0] . x <= margin - v.q0_arm (0 in the wrist column - these rows
    don't involve it at all).
    """
    idx_wrist = constraints["idx_wrist"]
    q0_arm = q0[ARM_JOINT_INDICES]
    axes = [direction] + [complement[:, i] for i in range(complement.shape[1])]  # 6 unit vectors, 6-dim each

    A_rows, b_rows = [], []
    for v in axes:
        v7 = np.concatenate([v, [0.0]])  # pad with 0 for the wrist column
        offset = v @ q0_arm
        A_rows.append(v7); b_rows.append(margin + offset)
        A_rows.append(-v7); b_rows.append(margin - offset)
    # wrist bounds (its own column only)
    e_wrist = np.zeros(7); e_wrist[-1] = 1.0
    A_rows.append(e_wrist); b_rows.append(constraints["upper"][idx_wrist])
    A_rows.append(-e_wrist); b_rows.append(-constraints["lower"][idx_wrist])

    A, b = np.array(A_rows), np.array(b_rows)
    return HPolyhedron(A, b), A, b


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    constraints = build_constraints()
    names = constraints["plant"].GetPositionNames()  # cached early - see module notes on plant-state flakiness
    q0 = find_grasp_seed(constraints)
    idx_wrist = constraints["idx_wrist"]
    direction = compute_arm_null_space_direction(constraints, q0)
    from scipy.linalg import null_space
    complement = null_space(direction.reshape(1, -1))

    run_dir = PROJECT_ROOT / "artifacts" / "wrist_axis_polytope"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ---- Plot 1: wrist-axis exactness ----
    lower_w, upper_w = constraints["lower"][idx_wrist], constraints["upper"][idx_wrist]
    q7_vals = np.linspace(lower_w, upper_w, 41)
    h_vals, g_vals = [], []
    for q7 in q7_vals:
        q = q0.copy(); q[idx_wrist] = q7
        h_vals.append(np.max(np.abs(constraints["h"](q))))
        g_vals.append(np.max(constraints["g"](q)))
    fig1, ax1 = plt.subplots(figsize=(7, 4.5))
    ax1.plot(q7_vals, h_vals, marker="o", markersize=3, label="max|h| (equality residual)", color="#4c78a8")
    ax1.plot(q7_vals, g_vals, marker="o", markersize=3, label="max(g) (inequality residual)", color="#f58518")
    ax1.axhline(CONSTRAINT_TOL, color="#e45756", linestyle="--", label=f"tolerance ({CONSTRAINT_TOL})")
    ax1.set_xlabel("panda_joint7 (wrist)")
    ax1.set_ylabel("constraint residual")
    ax1.set_title("Wrist-axis sweep: constraint residual vs. wrist angle\n(flat line = exactly free, not approximately)")
    ax1.set_ylim(-0.001, CONSTRAINT_TOL * 1.5)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.25)
    fig1.tight_layout()
    fig1.savefig(run_dir / "wrist_axis_exactness.png", dpi=150)
    print(f"Saved {run_dir / 'wrist_axis_exactness.png'}")

    # ---- Sample both regions ----
    rng = np.random.default_rng(0)
    n = 3000

    def sample_simple():
        pts = []
        for _ in range(n):
            q = q0.copy()
            for i in ARM_JOINT_INDICES:
                q[i] += rng.uniform(-MARGIN, MARGIN)
            pts.append(q.copy())
        return np.array(pts)

    def sample_null_space():
        pts = []
        for _ in range(n):
            q = q0.copy()
            t_dir = rng.uniform(-MARGIN, MARGIN)
            t_c = rng.uniform(-MARGIN, MARGIN, size=5)
            q[:6] += t_dir * direction + complement @ t_c
            pts.append(q.copy())
        return np.array(pts)

    simple_pts = sample_simple()
    ns_pts = sample_null_space()

    # ---- Plot 2: 2D projections ----
    pairs = [(0, 1), (4, 5)]
    fig2, axes2 = plt.subplots(1, len(pairs), figsize=(6 * len(pairs), 5))
    for ax, (i, j) in zip(axes2, pairs):
        ax.scatter(simple_pts[:, i], simple_pts[:, j], s=4, alpha=0.3, label="simple box", color="#4c78a8")
        ax.scatter(ns_pts[:, i], ns_pts[:, j], s=4, alpha=0.3, label="null-space box", color="#f58518")
        ax.scatter([q0[i]], [q0[j]], s=60, marker="*", color="black", zorder=5, label="q0")
        ax.set_xlabel(names[i]); ax.set_ylabel(names[j])
        ax.set_title(f"{names[i]} vs {names[j]}")
        ax.legend(fontsize=8)
        ax.ticklabel_format(useOffset=False)
    fig2.suptitle("Same volume, different shape: simple axis-aligned box vs. null-space-rotated box\n"
                   "(both at margin=0.001 rad; scale is real, not exaggerated)")
    fig2.tight_layout()
    fig2.savefig(run_dir / "region_comparison_2d.png", dpi=150)
    print(f"Saved {run_dir / 'region_comparison_2d.png'}")

    # ---- Plot 3: residuals ----
    def residuals(pts):
        hs = np.array([np.max(np.abs(constraints["h"](q))) for q in pts])
        gs = np.array([np.max(constraints["g"](q)) for q in pts])
        return hs, gs

    h_simple, g_simple = residuals(simple_pts)
    h_ns, g_ns = residuals(ns_pts)
    fig3, axes3 = plt.subplots(1, 2, figsize=(11, 4.5))
    axes3[0].hist(h_simple, bins=40, alpha=0.6, label="simple box", color="#4c78a8", density=True)
    axes3[0].hist(h_ns, bins=40, alpha=0.6, label="null-space box", color="#f58518", density=True)
    axes3[0].axvline(CONSTRAINT_TOL, color="#e45756", linestyle="--", label="tolerance")
    axes3[0].set_xlabel("max|h| over sample"); axes3[0].set_title("Equality residual distribution")
    axes3[0].legend(fontsize=8)
    axes3[1].hist(g_simple, bins=40, alpha=0.6, label="simple box", color="#4c78a8", density=True)
    axes3[1].hist(g_ns, bins=40, alpha=0.6, label="null-space box", color="#f58518", density=True)
    axes3[1].axvline(CONSTRAINT_TOL, color="#e45756", linestyle="--", label="tolerance")
    axes3[1].set_xlabel("max(g) over sample"); axes3[1].set_title("Inequality residual distribution")
    axes3[1].legend(fontsize=8)
    fig3.tight_layout()
    fig3.savefig(run_dir / "residual_headroom.png", dpi=150)
    print(f"Saved {run_dir / 'residual_headroom.png'}")

    # ---- HPolyhedron construction ----
    print("\n=== Building Drake HPolyhedron objects (7 DOF: 6 arm joints + wrist) ===")
    hp_simple, lower7, upper7 = build_simple_hpolyhedron(constraints, q0, MARGIN)
    print(f"Simple box: HPolyhedron.MakeBox - ambient_dimension={hp_simple.ambient_dimension()}, "
          f"A shape={hp_simple.A().shape}")

    hp_ns, A_ns, b_ns = build_null_space_hpolyhedron(constraints, q0, direction, complement, MARGIN)
    print(f"Null-space box: explicit (A,b) - ambient_dimension={hp_ns.ambient_dimension()}, A shape={A_ns.shape}")

    # sanity: q0's own 7-vector must be inside both
    q0_7 = np.concatenate([q0[ARM_JOINT_INDICES], [q0[idx_wrist]]])
    print(f"\nq0 in simple HPolyhedron: {hp_simple.PointInSet(q0_7)} (should be True)")
    print(f"q0 in null-space HPolyhedron: {hp_ns.PointInSet(q0_7)} (should be True)")

    # sanity: a point well outside (e.g. q0 + 1.0 rad on joint2) must be outside both
    far_point = q0_7.copy(); far_point[1] += 1.0
    print(f"far point in simple HPolyhedron: {hp_simple.PointInSet(far_point)} (should be False)")
    print(f"far point in null-space HPolyhedron: {hp_ns.PointInSet(far_point)} (should be False)")

    print(f"\nFingers/cap held fixed alongside the polytope (not part of it): "
          f"finger1={q0[7]:.4f}, finger2={q0[8]:.4f}, cap={q0[9]:.4f}")

    np.savez(run_dir / "hpolyhedron_export.npz",
             q0=q0, A_simple=hp_simple.A(), b_simple=hp_simple.b(),
             A_null_space=A_ns, b_null_space=b_ns,
             arm_joint_indices=ARM_JOINT_INDICES, idx_wrist=idx_wrist,
             finger1=q0[7], finger2=q0[8], cap=q0[9])
    print(f"Saved {run_dir / 'hpolyhedron_export.npz'} (A/b for both regions, q0, and the fixed finger/cap values)")


if __name__ == "__main__":
    main()
