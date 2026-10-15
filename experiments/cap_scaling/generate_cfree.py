"""Generate C-free for the 8-cap scene (multicap_scene.py), safe around the caps and the obstacles.

C-free is computed in the 9-D arm + finger space, with the caps welded: a cap is a cylinder about its own
joint axis, so no collision depends on a cap angle and the true free set is (9-D free set) x (cap angles).
full_arm_planner_8caps.ipynb loads the regions directly as its 9-D free sets.

Steps:
  1. Grasps: for each cap, the first GRASPS_PER_CAP random seeds (find_grasp_seed) that an NLP refines onto
     the grasp (|h| <= 1e-6, g <= 0) and that are collision-free. Saved next to the regions (*_grasps.npz);
     the notebook uses grasp 0 of each cap, so C-free and the A* are built around the same grasps.
  2. IRIS-NP2 regions grown along each grasp's collision-free wrist runs, from a starting ellipsoid that
     is long along the wrist and thin across it (in IRIS-NP2's metric, collisions beside the grasp line are
     then far, so its planes run parallel to the line). A round sphere left regions only ~0.2 rad long.
  3. Clique cover (IRIS-NP2, relaxed settings) with those regions as the initial sets.
  4. False positives (points inside a region that collide) cut off with separating planes, never cutting
     the collision-free part of a grasp's wrist sweep that the region contains:
       - on the wrist sweeps (obstacles that block part of a cap's wrist range),
       - around the caps (fingers and hand against a cap),
       - uniformly over the joint box (arm against the obstacles).
     Plane: through the closest collision-free configuration q_f to the colliding q_c (an NLP), normal
     q_c - q_f. Around the caps the wrist is fixed in that NLP, so the plane is parallel to the wrist axis.
  5. Validation on fresh samples; regions, grasps and a JSON report are saved.

Run from anywhere with the obmp env (takes ~30-60 min on 32 threads):
    python experiments/cap_scaling/generate_cfree.py
    python experiments/cap_scaling/generate_cfree.py --out data/cfree/my_name.yaml --coverage 0.95
"""
import argparse
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
for p in (REPO_ROOT, REPO_ROOT / "algorithms", REPO_ROOT / "algorithms" / "nlp_sampling"):
    sys.path.insert(0, str(p))

from pydrake.all import (CommonSampledIrisOptions, Hyperellipsoid, IrisFromCliqueCoverOptions,  # noqa: E402
                         IrisInConfigurationSpaceFromCliqueCover, IrisNp2Options, MathematicalProgram,
                         Parallelism, Parser, RandomGenerator, RigidTransform, RobotDiagramBuilder, RollPitchYaw,
                         SceneGraphCollisionChecker, Solve)
from pydrake.geometry.optimization import HPolyhedron, SaveIrisRegionsYamlFile  # noqa: E402
from pydrake.multibody.inverse_kinematics import InverseKinematics  # noqa: E402
from pydrake.planning import IrisNp2  # noqa: E402

import multicap_scene as scene  # noqa: E402
from nlp_sampling_standalone_test import build_autodiff_companion, make_grasp_constraints, make_joint_limits  # noqa: E402
from wrist_axis_grasp_polytope import find_grasp_seed  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--out", default=str(REPO_ROOT / "data" / "cfree" / "cfree_9dof_8caps_3obs_capsafe.yaml"))
ap.add_argument("--coverage", type=float, default=0.98, help="clique cover coverage threshold")
ap.add_argument("--grasps-per-cap", type=int, default=2)
ap.add_argument("--cut-rounds", type=int, default=6)
ap.add_argument("--threads", type=int, default=32)
ARGS = ap.parse_args()
OUT = Path(ARGS.out).resolve()

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", handlers=[logging.StreamHandler(sys.stdout)])
logging.getLogger("drake").setLevel(logging.WARNING)
log = logging.getLogger("generate_cfree").info
PAR = Parallelism(ARGS.threads)

# Plant ordering (same as the notebook's arm + hand): 7 arm joints, then the two fingers.
IDX_WRIST, IDX_F1, IDX_F2 = 6, 7, 8
GRASP_FINGERS = 0.02405      # finger value of the notebook's grasp seeds q0
SWEEP_FINGERS = 0.0245       # finger value of the wrist sweeps (inside the grasp polytope's [0.024, 0.025])
H_TOL, LIMIT_MARGIN = 1e-6, 0.05

# Sampled-IRIS settings (also for the seeded regions).
IRIS = dict(delta=1e-2, epsilon=1e-3, max_iterations=3, max_iterations_separating_planes=20, num_particles=1000,
            termination_threshold=-1, relative_termination_threshold=1e-3, remove_all_collisions_possible=False,
            configuration_space_margin=1e-4, sample_particles_in_parallel=True)


# ------------------------------------------------------------------------------------------------ scene
def build_plant():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    parser = Parser(plant, builder.scene_graph())
    parser.SetAutoRenaming(True)
    arm = parser.AddModels(url=f"file://{REPO_ROOT}/my_sdfs/panda_arm.urdf")[0]
    hand = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0", arm), RigidTransform())
    plant.WeldFrames(plant.GetFrameByName("panda_link8", arm), plant.GetFrameByName("panda_hand", hand),
                     RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0]))
    plant.GetJointByName("panda_finger_joint1", hand).set_default_translation(-0.024)
    caps = scene.add_caps(parser, plant, welded=True)
    scene.add_obstacles(plant)
    plant.Finalize()
    return builder.Build(), plant, arm, hand, caps


diagram, plant, arm, hand, caps = build_plant()
ctx = diagram.CreateDefaultContext()
pctx = plant.GetMyContextFromRoot(ctx)
checker = SceneGraphCollisionChecker(model=diagram, robot_model_instances=[arm, hand], edge_step_size=0.01)
nq = plant.num_positions()
assert nq == 9, f"expected the 9-D arm + finger space, got {nq}"
lo, hi = plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
t_start = time.time()


def free_mask(Q):
    return np.array(checker.CheckConfigsCollisionFree(np.asarray(Q), PAR), dtype=bool)


def inside_any(q, regions):
    return any(R.PointInSet(q) for R in regions)


def iris_options(**overrides):
    s = CommonSampledIrisOptions()
    for k, v in {**IRIS, **overrides}.items():
        setattr(s, k, v)
    s.parallelism = PAR
    o = IrisNp2Options()
    o.sampled_iris_options = s
    return o


# ------------------------------------------------------------------------------------------------ 1. grasps
E = plant.GetFrameByName("panda_hand", hand)
grasp_lower, grasp_upper = make_joint_limits(plant, IDX_WRIST, IDX_F1, IDX_F2)


def refine_grasp_seed(gc, q_seed):
    """min ||q - q_seed||^2 s.t. |h(q)| <= H_TOL, g(q) <= 0, grasp limits, non-wrist arm joints inside their limits."""
    l, u = np.array(gc["lower"], dtype=float), np.array(gc["upper"], dtype=float)
    l[:6] += LIMIT_MARGIN
    u[:6] -= LIMIT_MARGIN
    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(nq, "q")
    prog.AddBoundingBoxConstraint(l, u, q)
    prog.AddConstraint(gc["h"], -H_TOL * np.ones(4), H_TOL * np.ones(4), q)
    prog.AddConstraint(gc["g"], np.full(3, -np.inf), np.zeros(3), q)
    prog.AddQuadraticErrorCost(np.eye(nq), q_seed, q)
    result = Solve(prog, np.clip(q_seed, l, u))
    return result.GetSolution(q), result.is_success()


grasps = []          # grasps[k]: list of 9-D grasps of cap k (wrist -1, fingers +-GRASP_FINGERS)
grasp_seeds = []
for k, cap in enumerate(caps):
    Cap = plant.GetFrameByName("base_link", cap)
    plant_ad, pctx_ad, E_ad, Cap_ad = build_autodiff_companion(plant, E, Cap)
    h, g, Jh, Jg = make_grasp_constraints(plant, pctx, plant_ad, pctx_ad, E, Cap, E_ad, Cap_ad)
    gc = dict(plant=plant, h=h, g=g, Jh=Jh, Jg=Jg, idx_wrist=IDX_WRIST, lower=grasp_lower, upper=grasp_upper,
              _keep=(plant_ad, pctx_ad))
    found, seeds = [], []
    for s in range(50):
        q, ok = refine_grasp_seed(gc, find_grasp_seed(gc, seed=s))
        q[IDX_WRIST], q[IDX_F1], q[IDX_F2] = -1.0, -GRASP_FINGERS, GRASP_FINGERS
        if ok and checker.CheckConfigCollisionFree(q):
            found.append(q)
            seeds.append(s)
        if len(found) == ARGS.grasps_per_cap:
            break
    if not found:
        raise RuntimeError(f"cap {k + 1}: no converged, collision-free grasp in 50 seeds")
    grasps.append(found)
    grasp_seeds.append(seeds)
    log(f"cap {k + 1}: grasps from seeds {seeds}, |h| = {[f'{np.abs(h(q)).max():.0e}' for q in found]}")


def sweep(q, n=201):
    Q = np.repeat(q[None], n, axis=0)
    Q[:, IDX_WRIST] = np.linspace(-1.0, 1.0, n)
    Q[:, IDX_F1], Q[:, IDX_F2] = -SWEEP_FINGERS, SWEEP_FINGERS
    return Q


sweeps = [(sweep(q), free_mask(sweep(q))) for G in grasps for q in G]          # (points, free) per grasp
free_sweep_pts = np.vstack([S[f] for S, f in sweeps])

# ------------------------------------------------------------------------------------------------ 2. seeded regions
domain = HPolyhedron.MakeBox(lo, hi)
seeded = []
seed_opts = iris_options()
SEED_THIN, SEED_EXTRA, MIN_RUN = 1e-4, 1.0, 0.1     # ellipsoid semi-axes across / beyond the wrist run; shortest run seeded


def free_runs(mask):
    """(first, last) index of every run of consecutive True values."""
    runs, start = [], None
    for j, ok in enumerate(list(mask) + [False]):
        if ok and start is None:
            start = j
        elif not ok and start is not None:
            runs.append((start, j - 1))
            start = None
    return runs


for g, (S, f) in enumerate(sweeps):
    for _ in range(3):                                   # re-seed what is still uncovered, at most twice more
        todo = f & ~np.array([inside_any(q, seeded) for q in S])
        runs = [(a, b) for a, b in free_runs(todo) if S[b, IDX_WRIST] - S[a, IDX_WRIST] >= MIN_RUN]
        if not runs:
            break
        for a, b in runs:
            center = 0.5 * (S[a] + S[b])
            if not checker.CheckConfigCollisionFree(center):
                center = S[(a + b) // 2]
            half = 0.5 * (S[b, IDX_WRIST] - S[a, IDX_WRIST])
            radii = np.full(nq, SEED_THIN)
            radii[IDX_WRIST] = half + SEED_EXTRA
            ts = time.time()
            seeded.append(IrisNp2(checker, Hyperellipsoid(np.diag(1.0 / radii), center), domain, seed_opts))
            log(f"seeded region {len(seeded)} (grasp {g}) on wrist run [{S[a, IDX_WRIST]:+.2f}, {S[b, IDX_WRIST]:+.2f}]: "
                f"{seeded[-1].A().shape[0]} faces, {time.time() - ts:.0f}s")
log(f"step 2: {len(seeded)} seeded regions, {(time.time() - t_start) / 60:.1f} min so far")

# ------------------------------------------------------------------------------------------------ 3. clique cover
cc = IrisFromCliqueCoverOptions()
cc.num_points_per_visibility_round = 200
cc.coverage_termination_threshold = ARGS.coverage
cc.parallelism = PAR
cc.iris_options = iris_options()
ts = time.time()
regions = IrisInConfigurationSpaceFromCliqueCover(checker, cc, RandomGenerator(1234), list(seeded))
log(f"step 3: clique cover -> {len(regions)} regions ({len(regions) - len(seeded)} new), {(time.time() - ts) / 60:.1f} min")


# ------------------------------------------------------------------------------------------------ 4. cut false positives
def nlp_plane(q_c, fixed=()):
    """Closest collision-free q_f to q_c (joints in `fixed` held); plane n @ q <= n @ q_f with n = q_c - q_f."""
    ik = InverseKinematics(plant, pctx)
    ik.AddMinimumDistanceLowerBoundConstraint(1e-3)
    q = ik.q()
    prog = ik.get_mutable_prog()
    for i in fixed:
        prog.AddBoundingBoxConstraint(q_c[i], q_c[i], q[i])
    prog.AddQuadraticErrorCost(np.eye(nq), q_c, q)
    result = Solve(prog, q_c)
    if not result.is_success():
        return None
    q_f = result.GetSolution(q)
    n = q_c - q_f
    if np.linalg.norm(n) < 1e-9:
        return None
    n /= np.linalg.norm(n)
    return n, n @ q_f


def keeps_all(plane, keep):
    n, d = plane
    return len(keep) == 0 or np.all(keep @ n <= d + 1e-9)


def longest_free_run_inside(R, S, f):
    """Longest run of consecutive collision-free sweep points inside R (the region's grasp segment)."""
    best, run = [], []
    for q, free in zip(S, f):
        run = run + [q] if (free and R.PointInSet(q)) else []
        if len(run) > len(best):
            best = run
    return np.array(best)


def add_plane(regions, i, plane):
    n, d = plane
    R = regions[i]
    regions[i] = HPolyhedron(np.vstack([R.A(), n]), np.append(R.b(), d))


def cut_sweeps(regions):
    """Sweep points that collide but lie in a region: cut them, keeping the region's longest free run."""
    added = fallback = 0
    for S, f in sweeps:
        for i in range(len(regions)):
            bad = [q for q, free in zip(S, f) if not free and regions[i].PointInSet(q)]
            if not bad:
                continue
            keep = longest_free_run_inside(regions[i], S, f)
            for q_c in bad:
                if not regions[i].PointInSet(q_c):
                    continue
                plane = nlp_plane(q_c)
                if plane is None or not keeps_all(plane, keep):
                    if len(keep) == 0:
                        continue
                    e = np.zeros(nq)                                  # wrist plane at the end of the free run
                    e[IDX_WRIST] = 1.0
                    w_keep = keep[:, IDX_WRIST]
                    plane = (e, w_keep.max()) if q_c[IDX_WRIST] > w_keep.max() else (-e, -w_keep.min())
                    fallback += 1
                add_plane(regions, i, plane)
                added += 1
    return added, fallback


def near_cap_samples(rng, n_per):
    out = []
    for G in grasps:
        for q in G:
            for radius in (0.05, 0.15, 0.3):
                Q = np.repeat(q[None], n_per, axis=0)
                Q[:, :6] += rng.uniform(-radius, radius, (n_per, 6))
                Q[:, IDX_WRIST] = rng.uniform(lo[IDX_WRIST], hi[IDX_WRIST], n_per)
                Q[:, IDX_F1] = rng.uniform(lo[IDX_F1], hi[IDX_F1], n_per)
                Q[:, IDX_F2] = rng.uniform(lo[IDX_F2], hi[IDX_F2], n_per)
                out.append(np.clip(Q, lo, hi))
    return np.vstack(out)


def cut_samples(regions, Q, fixed):
    free = free_mask(Q)
    keep = [free_sweep_pts[[R.PointInSet(q) for q in free_sweep_pts]] for R in regions]
    added = skipped = 0
    for q_c in Q[~free]:
        for i in range(len(regions)):
            if not regions[i].PointInSet(q_c):
                continue
            plane = nlp_plane(q_c, fixed)
            if plane is None or not keeps_all(plane, keep[i]):
                skipped += 1
                continue
            add_plane(regions, i, plane)
            added += 1
    return int((~free).sum()), added, skipped


regions = list(regions)
rng = np.random.default_rng(0)
for rnd in range(ARGS.cut_rounds):
    a_s, fb = cut_sweeps(regions)
    n_c, a_c, s_c = cut_samples(regions, near_cap_samples(rng, 250), fixed=(IDX_WRIST,))
    n_u, a_u, s_u = cut_samples(regions, rng.uniform(lo, hi, (20000, nq)), fixed=())
    log(f"step 4 round {rnd + 1}: sweeps {a_s} planes ({fb} wrist fallback); near caps {n_c} colliding samples, "
        f"{a_c} planes, {s_c} skipped; uniform {n_u} colliding, {a_u} planes, {s_u} skipped")
    if a_s + a_c + a_u == 0:
        break

regions = [R for R in regions if not R.IsEmpty()]


# ------------------------------------------------------------------------------------------------ 5. validation
def validate(regions):
    vr = np.random.default_rng(12345)
    rep = {}
    Q = near_cap_samples(vr, 200)
    f = free_mask(Q)
    ins = np.array([inside_any(q, regions) for q in Q])
    rep["near_caps"] = dict(samples=len(Q), false_positives=int((ins & ~f).sum()), covered_of_free=float((ins & f).sum() / f.sum()))
    U = vr.uniform(lo, hi, (20000, nq))
    f = free_mask(U)
    ins = np.array([inside_any(q, regions) for q in U])
    rep["uniform"] = dict(samples=len(U), inside=int(ins.sum()), false_positives=int((ins & ~f).sum()),
                          covered_of_free=float((ins & f).sum() / f.sum()), free_fraction=float(f.mean()))
    rep["grasp_sweeps"] = []
    for k, G in enumerate(grasps):
        for j, q in enumerate(G):
            S = sweep(q)
            f = free_mask(S)
            ins = np.array([inside_any(s, regions) for s in S])
            rep["grasp_sweeps"].append(dict(cap=k + 1, grasp=j, free=int(f.sum()), covered=int((ins & f).sum()),
                                            false_positives=int((ins & ~f).sum())))
    return rep


report = validate(regions)
report.update(regions=len(regions), faces_mean=float(np.mean([R.A().shape[0] for R in regions])),
              minutes=round((time.time() - t_start) / 60, 1), coverage_threshold=ARGS.coverage,
              grasp_seeds=grasp_seeds, iris_settings=IRIS, cap_poses=scene.CAP_POSES, obstacles=scene.OBSTACLES,
              floor=scene.FLOOR)
log("step 5: validation " + json.dumps({k: report[k] for k in ("near_caps", "uniform")}))
for s in report["grasp_sweeps"]:
    log(f"  cap {s['cap']} grasp {s['grasp']}: wrist sweep covered {s['covered']}/{s['free']} free, false positives {s['false_positives']}")

OUT.parent.mkdir(parents=True, exist_ok=True)
SaveIrisRegionsYamlFile(str(OUT), {f"region_{i:03d}": R for i, R in enumerate(regions)})
np.savez(OUT.with_name(OUT.stem + "_grasps.npz"), grasps=np.array([G[0] for G in grasps]),
         all_grasps=np.array([q for G in grasps for q in G]), cap_of_grasp=np.array([k for k, G in enumerate(grasps) for _ in G]))
OUT.with_name(OUT.stem + "_report.json").write_text(json.dumps(report, indent=1, default=float))
log(f"saved {len(regions)} regions to {OUT} (+ _grasps.npz, _report.json), total {report['minutes']} min")
