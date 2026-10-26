"""Run time of the reachability A* as the number of caps to turn grows (1 to 8), on the 8-cap scene.

One scene (full_arm_planner_8caps_3obs.ipynb: 8 caps, 3 walls, 9-D C-free) and, for each K in --ks, --queries
random queries. A query:
  - picks K of the 8 caps at random (the "active" caps); the A* may grasp only those (the other caps'
    grasp components are left out of the graph), and every cap's angle stays in the A*'s state;
  - turns the active caps by random amounts in [0.5, pi] rad that sum to 2.5 * K rad (the notebook's
    sample_turns), from alpha = 3.0;
  - starts and ends at random arm configurations in C-free.
Every query is solved twice: weighted A* (the notebook's HEURISTIC_WEIGHT, 1.5) and exact A* (weight 1.0,
--exact-time-limit per query; once every exact query of some K times out, larger K skip it).
Timed per query, as in experiments/gcs_comparison: the search (reachability_astar) and placing the
waypoints (build_waypoints), with the peak extra resident memory. Every plan is checked: each move lies in
its convex set, and only the grasped cap turns, by -dwrist on a forward stroke. Not timed: the shared
preprocessing that builds the A*'s graph (Steps 1-4), timed once and reported separately; and C-free.

Writes to --out: summary.json (every query and the per-K statistics), table.tex (paper style),
cap_scaling.html (self-contained page: table, charts, and a render of one K = 8 plan) and render_meshes/.

Run from anywhere, with the obmp conda env:
  python experiments/cap_scaling/cap_scaling.py --out artifacts/cap_scaling
  python experiments/cap_scaling/cap_scaling.py --out /tmp/quick --queries 3 --ks 1,8 --no-render
"""
import argparse
import base64
import contextlib
import io
import json
import os
import platform
import sys
import threading
import time as _t
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--out", required=True, help="output folder (created)")
parser.add_argument("--ks", default="1,2,3,4,5,6,7,8", help="numbers of active caps")
parser.add_argument("--queries", type=int, default=20, help="random queries per K")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--max-expansions", type=int, default=2_000_000)
parser.add_argument("--exact-time-limit", type=float, default=60.0, help="s per query for exact A* (weight 1.0)")
parser.add_argument("--no-exact", action="store_true", help="weighted A* only")
parser.add_argument("--render-k", type=int, default=8, help="K whose first solved query is rendered on the page")
parser.add_argument("--no-render", action="store_true")
parser.add_argument("--verbose", action="store_true", help="print the notebook cells' output")
parser.add_argument("--save-queries", action="store_true", help="also write queries.npz (for experiments/hpp_comparison)")
parser.add_argument("--replay-queries", help="a queries.npz from --save-queries: run exactly those queries (its K values "
                                             "and number per K) instead of drawing new ones, which depend on the sets")
ARGS = parser.parse_args()

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_notebook import REPO_ROOT, code_cell, run_until_step7   # noqa: E402

OUT = Path(ARGS.out).resolve()
OUT.mkdir(parents=True, exist_ok=True)
KS = [int(x) for x in ARGS.ks.split(",")]
FPS, IMG_W, IMG_H, MAX_VIDEO_S = 10, 480, 360, 45

# ---------------------------------------------------------------- the notebook, up to its Step 7
# Its LP_SOLVER cell picks the LP solver ("mosek" with one licence held, or "clp") for the whole run.
print("running the notebook cells (scene, grasp polytopes, C-free, A*)...", flush=True)
run_until_step7(verbose=ARGS.verbose, ns=globals())

import numpy as np                                                       # noqa: E402

# ---------------------------------------------------------------- memory sampler (as in gcs_comparison)
_peak = [0.0]


def rss_gb():
    for line in open('/proc/self/status'):
        if line.startswith('VmRSS'):
            return int(line.split()[1]) / 1e6


def _sample():
    while True:
        _peak[0] = max(_peak[0], rss_gb())
        _t.sleep(0.005)


threading.Thread(target=_sample, daemon=True).start()


def measured(fn):
    """Run fn(); return (result, seconds, peak extra memory in GB over the RSS before the call)."""
    base = rss_gb()
    _peak[0] = base
    t0 = _t.perf_counter()
    out = fn()
    return out, _t.perf_counter() - t0, max(0.0, _peak[0] - base)


# ---------------------------------------------------------------- shared preprocessing (A* Steps 1-4), timed once
PREP_PREFIXES = ("import itertools\nfrom pydrake.solvers import MathematicalProgram, Solve", "def extreme_point(",
                 "L_MIN = 0.05", "n_free = len(free_sets)", "import heapq")
with contextlib.redirect_stdout(io.StringIO()):
    _, t_prep, _ = measured(lambda: [exec(code_cell(p), globals()) for p in PREP_PREFIXES])
all_comps_near = {i: list(cs) for i, cs in comps_near.items()}
graph = dict(free_sets=len(free_sets), components=len(components), transit_edges=len(ff_door) // 2,
             switch_edges=len(fc_links), components_per_cap=[sum(c["cap"] == k for c in components) for k in range(K)],
             L_max_per_cap=[float(x) for x in L_MAX])
print(f"shared preprocessing: {t_prep:.2f}s; LP solver: {LP_SOLVER} (MOSEK enabled: {MosekSolver().enabled()}); "   # noqa: F821
      f"graph {graph}", flush=True)


# ---------------------------------------------------------------- one query
def certified(waypoints, tol=1e-6):
    """Step 6b: both ends of every move in its certifying set; only the grasped cap turns, by -dwrist on a stroke."""
    for (qa, _, _), (qb, _, cert) in zip(waypoints[:-1], waypoints[1:]):
        poly = certifying_set(cert)
        if not (poly.PointInSet(to_arm(qa), tol) and poly.PointInSet(to_arm(qb), tol)):
            return False
        dw, dalpha = qb[idx_wrist] - qa[idx_wrist], qb[idx_caps] - qa[idx_caps]
        expected = np.zeros(K)
        if cert[0] == "G" and dw > TOL:
            expected[components[cert[1]]["cap"]] = -dw
        if not np.allclose(dalpha, expected, atol=tol):
            return False
    return True


def plan_metrics(waypoints):
    moves = list(zip(waypoints[:-1], waypoints[1:]))
    strokes = sum(cert[0] == "G" and qb[idx_wrist] - qa[idx_wrist] > TOL for (qa, _, _), (qb, _, cert) in moves)
    certs = [cert for _, (_, _, cert) in moves]
    return dict(grasps=int(strokes), polytope_changes=int(sum(a != b for a, b in zip(certs[:-1], certs[1:]))),
                path_length=float(sum(np.linalg.norm(qb - qa) for (qa, _, _), (qb, _, _) in moves)))


def random_free(rng):
    while True:
        q = rng.uniform(q_lower, q_upper)
        if any(F.PointInSet(to_arm(q)) for F in free_sets):
            return q


rng = np.random.default_rng(ARGS.seed)
REPLAY = np.load(ARGS.replay_queries) if ARGS.replay_queries else None
if REPLAY is not None:
    KS = sorted(set(int(k) for k in REPLAY["k"]))
    ARGS.queries = int(max(REPLAY["query"])) + 1
W_NOTEBOOK = HEURISTIC_WEIGHT
METHODS = [("weighted", W_NOTEBOOK, None)] + ([] if ARGS.no_exact else [("exact", 1.0, ARGS.exact_time_limit)])
LABELS = {"weighted": f"Weighted A* (w = {W_NOTEBOOK:g})", "exact": "Exact A* (w = 1)"}
runs, keep_plans, exact_given_up = [], {}, None
saved_queries = []   # --save-queries: the same queries for experiments/hpp_comparison
for k_active in KS:
    for qi in range(ARGS.queries):
        if REPLAY is None:
            active = sorted(int(c) for c in rng.choice(K, k_active, replace=False))
            turns = sample_turns(k_active, rng)
            alpha_init = np.full(K, 3.0)
            alpha_goal = alpha_init.copy()
            alpha_goal[active] -= turns
            q_init, q_goal = random_free(rng), random_free(rng)
            q_init[idx_caps], q_goal[idx_caps] = alpha_init, alpha_goal
        else:                                       # the saved query (k_active, qi)
            j = int(np.flatnonzero((REPLAY["k"] == k_active) & (REPLAY["query"] == qi))[0])
            active = [int(c) for c in np.flatnonzero(REPLAY["active_mask"][j])]
            turns = REPLAY["turns"][j][active]
            q_init, q_goal = REPLAY["q_init"][j].copy(), REPLAY["q_goal"][j].copy()
            alpha_init, alpha_goal = q_init[idx_caps], q_goal[idx_caps]
        saved_queries.append((k_active, qi, np.isin(np.arange(K), active), alpha_init - alpha_goal,
                              q_init.copy(), q_goal.copy()))
        # the A* may grasp the active caps only: the other caps' components leave the graph
        comps_near = {i: [c for c in cs if components[c]["cap"] in active] for i, cs in all_comps_near.items()}
        base = dict(k=k_active, query=qi, active_caps=[c + 1 for c in active], turns=[float(t) for t in turns],
                    total_turn=float(turns.sum()))
        # a replayed query (drawn from other sets) can start or end outside these sets: no plan, and no search
        outside = not all(any(F.PointInSet(to_arm(q)) for F in free_sets) for q in (q_init, q_goal))
        for method, weight, limit in METHODS:
            if method == "exact" and exact_given_up is not None:
                runs.append(dict(base, method=method, solved=False, skipped=True))
                continue
            if outside:
                runs.append(dict(base, method=method, weight=weight, solved=False, skipped=False, outside_cfree=True))
                print(f"K={k_active} q{qi:02d} {method:<8}: NO PLAN (start or goal outside C-free)", flush=True)
                continue
            HEURISTIC_WEIGHT = weight
            (path, info), t_search, m_search = measured(
                lambda: reachability_astar(q_init, q_goal, max_expansions=ARGS.max_expansions, time_limit=limit))
            row = dict(base, method=method, weight=weight, expanded=int(info["expanded"]), search_s=t_search,
                       search_mem_gb=m_search, timed_out=bool(info.get("timed_out", False)), skipped=False)
            if path is None:
                row.update(solved=False)
            else:
                (wps, _), t_place, m_place = measured(lambda: build_waypoints(path, q_init, q_goal))
                n = count_actions(path)
                row.update(solved=True, place_s=t_place, total_s=t_search + t_place, mem_gb=max(m_search, m_place),
                           grasps_per_cap=[int(n["grasps_per_cap"][c]) for c in active], returns=int(n["returns"]),
                           releases=int(n["releases"]), hops=int(n["hops"]),
                           actions=int(n["grasps"] + n["returns"] + n["releases"]), certified=bool(certified(wps)),
                           reaches_goal=bool(np.allclose(wps[-1][0], q_goal, atol=1e-6)), **plan_metrics(wps))
                if method == "weighted" and k_active == ARGS.render_k and k_active not in keep_plans:
                    keep_plans[k_active] = (wps, q_init, q_goal, active, row)
            runs.append(row)
            print(f"K={k_active} q{qi:02d} {method:<8}: " + (
                f"{row['actions']} actions ({row['grasps']} grasps), {row['expanded']} expanded, search {t_search * 1e3:.1f} ms, "
                f"total {row['total_s'] * 1e3:.1f} ms, certified {row['certified']}" if row["solved"] else
                f"NO PLAN ({row['expanded']} expanded, {t_search:.1f} s{', time limit' if row['timed_out'] else ''})"), flush=True)
        HEURISTIC_WEIGHT = W_NOTEBOOK
    ex = [r for r in runs if r["k"] == k_active and r["method"] == "exact" and not r["skipped"]]
    if ex and exact_given_up is None and not any(r["solved"] for r in ex):
        exact_given_up = k_active
        print(f"exact A*: no query with K = {k_active} solved within {ARGS.exact_time_limit:g} s; skipped for larger K", flush=True)
if ARGS.save_queries:
    k_, n_, m_, t_, qi_, qg_ = map(np.array, zip(*saved_queries))
    np.savez(OUT / "queries.npz", k=k_, query=n_, active_mask=m_, turns=t_, q_init=qi_, q_goal=qg_)
comps_near = all_comps_near


# ---------------------------------------------------------------- statistics
def stats(xs):
    xs = np.asarray(xs, dtype=float)
    if len(xs) == 0:
        return None
    return dict(median=float(np.median(xs)), p25=float(np.percentile(xs, 25)), p75=float(np.percentile(xs, 75)),
                min=float(xs.min()), max=float(xs.max()), mean=float(xs.mean()))


def method_stats(rows):
    ok = [r for r in rows if r["solved"]]
    return dict(queries=len(rows), solved=len(ok), skipped=sum(r.get("skipped", False) for r in rows),
                timed_out=sum(r.get("timed_out", False) for r in rows),
                outside_cfree=sum(r.get("outside_cfree", False) for r in rows), certified=sum(r["certified"] for r in ok),
                reaches_goal=sum(r["reaches_goal"] for r in ok), total_turn=rows[0]["total_turn"] if rows else None,
                actions=stats([r["actions"] for r in ok]), grasps=stats([r["grasps"] for r in ok]),
                expanded=stats([r["expanded"] for r in ok]), search_s=stats([r["search_s"] for r in ok]),
                place_s=stats([r["place_s"] for r in ok]), total_s=stats([r["total_s"] for r in ok]),
                mem_gb=stats([r["mem_gb"] for r in ok]), path_length=stats([r["path_length"] for r in ok]))


per_k = {}
for k_active in KS:
    per_k[k_active] = {m: method_stats([r for r in runs if r["k"] == k_active and r["method"] == m]) for m, _, _ in METHODS}
    if not ARGS.no_exact:   # extra actions of the weighted plan, on queries both methods solved
        pairs = [(w, e) for w in runs for e in runs if w["k"] == e["k"] == k_active and w["query"] == e["query"]
                 and w["method"] == "weighted" and e["method"] == "exact" and w["solved"] and e["solved"]]
        per_k[k_active]["paired"] = dict(n=len(pairs), same_actions=sum(w["actions"] == e["actions"] for w, e in pairs),
                                         extra_actions_max=max([w["actions"] - e["actions"] for w, e in pairs], default=None))


def machine():
    cpu = next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), platform.processor())
    mem = next(int(l.split()[1]) / 1e6 for l in open("/proc/meminfo") if l.startswith("MemTotal"))
    return f"{cpu}, {os.cpu_count()} threads, {mem:.0f} GB"


MACHINE = machine()
summary = dict(machine=MACHINE, shared_preprocessing_s=t_prep, lp_solver=LP_SOLVER, graph=graph,   # noqa: F821
               cfree_file=str(CFREE_FILE.relative_to(REPO_ROOT)),
               queries_per_k=ARGS.queries, seed=ARGS.seed, turn_rule=dict(mean_per_cap=TURN_MEAN, min=TURN_MIN, max=TURN_MAX),
               methods={m: dict(weight=w, time_limit_s=t) for m, w, t in METHODS}, exact_skipped_from_k=exact_given_up,
               per_k=per_k, runs=runs)
(OUT / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
print("per K:", json.dumps({k: {m: dict(solved=v[m]["solved"], median_ms=v[m]["total_s"]["median"] * 1e3 if v[m]["total_s"] else None)
                                for m, _, _ in METHODS} for k, v in per_k.items()}), flush=True)


# ---------------------------------------------------------------- formatting
def fmt_time(s):
    return f"{s * 1e3:.1f} ms" if s < 1 else f"{s:.2f} s"


def fmt_mem(gb):
    return "<1 MB" if gb < 1e-3 else (f"{gb * 1e3:.0f} MB" if gb < 1 else f"{gb:.1f} GB")


def iqr(st, f):
    return f"{f(st['median'])} <span class='muted'>({f(st['p25'])}–{f(st['p75'])})</span>"


# ---------------------------------------------------------------- table.tex (paper style)
tex = [r"\begin{table}[t]", r"\centering",
       r"\caption{Reachability A* on the 8-cap scene (9-DOF C-free, 3 walls, floor) as the number of caps to",
       rf"turn grows. {ARGS.queries} random queries per $K$; each turns $K$ random caps by random amounts in",
       rf"$[0.5,\pi]$\,rad summing to $2.5K$\,rad. Actions: grasps + ungrasps. Median (25th--75th",
       rf"percentile) over solved queries. Exact A* stops after {ARGS.exact_time_limit:g}\,s. Every plan is certified.}}",
       r"\label{tab:cap_scaling}", r"\setlength{\tabcolsep}{3.5pt}", r"\begin{tabular}{@{}r l r r r r@{}}", r"\toprule",
       r"$K$ & Method & Solved & Actions & Nodes expanded & Time [ms]\\", r"\midrule"]
for n_k, (k_active, v) in enumerate(per_k.items()):
    for j_m, (m, w, _) in enumerate(METHODS):
        st = v[m]
        lead = rf"\multirow{{{len(METHODS)}}}{{*}}{{{k_active}}}" if j_m == 0 else ""
        name = rf"Weighted A* ($w={w:g}$)" if m == "weighted" else "Exact A*"
        if st["skipped"] == st["queries"]:
            tex.append(rf"{lead} & {name} & \multicolumn{{4}}{{l}}{{not run (none solved at $K={exact_given_up}$)}}\\")
        elif not st["solved"]:
            tex.append(rf"{lead} & {name} & 0/{st['queries']} & -- & -- & $>${ARGS.exact_time_limit * 1e3:.0f}\\")
        else:
            a, e, t = st["actions"], st["expanded"], st["total_s"]
            tex.append(rf"{lead} & {name} & {st['solved']}/{st['queries']} & {a['median']:.0f} & {e['median']:.0f} ({e['p25']:.0f}--{e['p75']:.0f}) & "
                       rf"{t['median'] * 1e3:.1f} ({t['p25'] * 1e3:.1f}--{t['p75'] * 1e3:.1f})\\")
    if n_k < len(per_k) - 1:
        tex.append(r"\midrule")
tex += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
(OUT / "table.tex").write_text("\n".join(tex) + "\n")

# ---------------------------------------------------------------- render one plan (Drake VTK, as in gcs_comparison)
video_html = ""
if not ARGS.no_render and ARGS.render_k in keep_plans:
    from PIL import Image
    from pydrake.all import (RobotDiagramBuilder, Parser, RigidTransform, RotationMatrix, RollPitchYaw, Box, Mesh, Rgba,
                             Role, MakeRenderEngineVtk, RenderEngineVtkParams, ColorRenderCamera, RenderCameraCore,
                             CameraInfo, ClippingRange)
    import meshcat_plan_markers as mpm

    MESH_DIR = OUT / "render_meshes"
    MESH_DIR.mkdir(exist_ok=True)
    wps, q_init_r, q_goal_r, active_r, row_r = keep_plans[ARGS.render_k]

    def write_obj(path, V, F):      # V: 3xN, F: 3xM (0-based); area-weighted vertex normals
        P, T = V.T, F.T
        face_n = np.cross(P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]])
        N = np.zeros_like(P)
        for j in range(3):
            np.add.at(N, T[:, j], face_n)
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
        with open(path, "w") as f:
            for v in P: f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            for n in N: f.write(f"vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}\n")
            for t in T: f.write(f"f {t[0] + 1}//{t[0] + 1} {t[1] + 1}//{t[1] + 1} {t[2] + 1}//{t[2] + 1}\n")
        return str(path)

    inspector = visualizer.model_inspector
    fk_ctx = plant.CreateDefaultContext()
    hand_meshes = []
    for body in [plant.get_body(i) for i in plant.GetBodyIndices(panda_hand)]:
        for j, gid in enumerate(inspector.GetGeometries(plant.GetBodyFrameIdOrThrow(body.index()), Role.kIllustration)):
            shape = inspector.GetShape(gid)
            if isinstance(shape, Mesh) and shape.extension() == ".gltf":
                V, F = mpm.read_gltf_triangles(shape.source().path())
                hand_meshes.append((body.name(), inspector.GetPoseInFrame(gid),
                                    write_obj(MESH_DIR / f"{body.name()}_{j}.obj", (V * shape.scale3()).T, F.T)))

    rb = RobotDiagramBuilder(time_step=0.0)
    rp, rsg = rb.plant(), rb.scene_graph()
    rsg.AddRenderer("vtk", MakeRenderEngineVtk(RenderEngineVtkParams(default_clear_color=[0.97, 0.97, 0.97])))
    pr = Parser(rp, rsg)
    pr.SetAutoRenaming(True)
    arm_r = pr.AddModels(url=f"file://{REPO_ROOT}/my_sdfs/panda_arm.urdf")[0]
    hand_r = pr.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]
    rp.WeldFrames(rp.world_frame(), rp.GetFrameByName("panda_link0", arm_r), RigidTransform())
    rp.WeldFrames(rp.GetFrameByName("panda_link8", arm_r), rp.GetFrameByName("panda_hand", hand_r),
                  RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0]))
    scene.add_caps(pr, rp)
    scene.add_walls(rp)
    world, n_geom = rp.world_body(), [0]

    def add(body, X, shape, rgba):
        n_geom[0] += 1
        rp.RegisterVisualGeometry(body, X, shape, f"marker_{n_geom[0]}", rgba.rgba)

    for q, rgba in ((q_init_r, mpm.START_RGBA), (q_goal_r, mpm.GOAL_RGBA)):        # ghost hands at start and goal
        plant.SetPositions(fk_ctx, q)
        for name, X_BG, p in hand_meshes:
            add(world, plant.EvalBodyPoseInWorld(fk_ctx, plant.GetBodyByName(name, panda_hand)) @ X_BG, Mesh(p), rgba)
    for c in active_r:                                                           # each turned cap: start/goal ticks + arc
        X_WCap = plant.GetJointByName("cap_to_base", caps[c]).frame_on_parent().CalcPoseInWorld(fk_ctx)
        a0, a1 = float(q_init_r[idx_caps[c]]), float(q_goal_r[idx_caps[c]])
        for a, rgba, dz in ((a0, mpm.START_RGBA, 0.0), (a1, mpm.GOAL_RGBA, 0.0005)):
            r0, r1 = mpm.CAP_TICK
            add(world, X_WCap @ RigidTransform(RotationMatrix.MakeZRotation(a), [(r0 + r1) / 2, 0, mpm.CAP_Z + dz]),
                Box(r1 - r0, 0.003, 0.002), Rgba(rgba.r(), rgba.g(), rgba.b(), 1.0))
        V, F = mpm.annular_sector(*mpm.CAP_ARC, a0, a1, z=mpm.CAP_Z, n=max(16, int(abs(a1 - a0) * 20)), growth=mpm.CAP_ARC_GROWTH)
        add(world, X_WCap, Mesh(write_obj(MESH_DIR / f"cap{c + 1}_arc.obj", V, F)), mpm.ARC_RGBA)
    rp.Finalize()
    rd = rb.Build()
    rctx = rd.CreateDefaultContext()
    rpctx, rsctx = rp.GetMyMutableContextFromRoot(rctx), rsg.GetMyContextFromRoot(rctx)

    def look_at(eye, target):
        eye, target = np.asarray(eye, float), np.asarray(target, float)
        f = target - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        return RigidTransform(RotationMatrix(np.column_stack([r, np.cross(f, r), f])), eye)

    CAM = ColorRenderCamera(RenderCameraCore("vtk", CameraInfo(IMG_W, IMG_H, np.deg2rad(45)), ClippingRange(0.02, 10), RigidTransform()))
    X_WCam = look_at([1.55, -1.2, 1.65], [0.0, 0.0, 0.25])     # front-right, above: caps, walls and arm in view

    length = row_r["path_length"]
    speed = max(1.0, length / MAX_VIDEO_S)                       # config-space units per second
    qs = [wps[0][0]]
    for (qa, _, _), (qb, _, _) in zip(wps[:-1], wps[1:]):
        m = max(1, int(np.ceil(np.linalg.norm(qb - qa) / speed * FPS)))
        qs += [qa + s * (qb - qa) for s in np.linspace(0, 1, m + 1)[1:]]
    qs += [wps[-1][0]] * FPS
    frames = []
    for q in qs:
        rp.SetPositions(rpctx, q)
        img = rsg.get_query_output_port().Eval(rsctx).RenderColorImage(CAM, rsg.world_frame_id(), X_WCam)
        buf = io.BytesIO()
        Image.fromarray(np.asarray(img.data)[:, :, :3]).save(buf, "JPEG", quality=68)
        frames.append(base64.b64encode(buf.getvalue()).decode())
    Image.open(io.BytesIO(base64.b64decode(frames[0]))).save(OUT / "render_first_frame.jpg")
    print(f"RENDERED K={ARGS.render_k}: {len(frames)} frames at {speed:.1f} unit/s", flush=True)
    turns_txt = ", ".join(f"cap {c + 1}: {t:.2f}" for c, t in zip(active_r, row_r["turns"]))
    video_html = f"""
<h2>One plan with K = {ARGS.render_k}</h2>
<p>Query {row_r['query']}: {row_r['grasps']} grasps (strokes), {row_r['returns']} returns, {row_r['releases']} releases,
path length {row_r['path_length']:.1f}, found in {fmt_time(row_r['total_s'])} ({row_r['expanded']} nodes expanded). Turns in rad: {turns_txt}.
Played at {speed:.1f} configuration-space units per second. Green: start (ghost hand, cap ticks); orange: goal; grey arcs: how far
each cap turns.</p>
<div class="controls"><button id="play">Pause</button><button id="restart">Restart</button>
<label>Speed <select id="speed"><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select></label>
<input type="range" id="scrub" min="0" max="1000" value="0" aria-label="Playback position"><span id="clock">0.0 s</span></div>
<figure class="video"><canvas id="c_plan" width="{IMG_W}" height="{IMG_H}"></canvas></figure>
<script>
const FPS = {FPS}, FRAMES = {json.dumps(frames)};
const imgs = FRAMES.map((b, i) => {{ const im = new Image(); if (i === 0) im.onload = () => draw(); im.src = "data:image/jpeg;base64," + b; return im; }});
const ctx = document.getElementById("c_plan").getContext("2d");
let t = 0, playing = true, speed = 1, last = null;
const scrub = document.getElementById("scrub"), clock = document.getElementById("clock"), play = document.getElementById("play");
function draw() {{
  const f = Math.min(Math.floor(t * FPS), imgs.length - 1);
  if (imgs[f].complete) ctx.drawImage(imgs[f], 0, 0);
  scrub.value = Math.round(1000 * f / (imgs.length - 1)); clock.textContent = t.toFixed(1) + " s";
}}
function tick(now) {{
  if (last !== null && playing) {{ t += (now - last) / 1000 * speed; if (t * FPS >= imgs.length - 1) {{ t = (imgs.length - 1) / FPS; playing = false; play.textContent = "Play"; }} }}
  last = now; draw(); requestAnimationFrame(tick);
}}
play.onclick = () => {{ if (t * FPS >= imgs.length - 1) t = 0; playing = !playing; play.textContent = playing ? "Pause" : "Play"; }};
document.getElementById("restart").onclick = () => {{ t = 0; playing = true; play.textContent = "Pause"; }};
document.getElementById("speed").onchange = e => {{ speed = parseFloat(e.target.value); }};
scrub.oninput = () => {{ t = scrub.value / 1000 * (imgs.length - 1) / FPS; draw(); }};
requestAnimationFrame(tick);
</script>"""


# ---------------------------------------------------------------- charts (inline SVG, one measure each, one series per method)
SERIES_CLASS = {"weighted": "s1", "exact": "s2"}


def dot_chart(title, key, unit_fmt, scale=1.0, limit_value=None):
    """Dots: one query each, per method (side by side within a K). Filled: solved; hollow: no plan within the
    time limit (drawn at limit_value when given). Line: median of the solved queries. Log scale."""
    W, H, L, R, T, B = 460, 270, 70, 16, 16, 36
    pts = [(r["k"], r["method"], (r[key] * scale if r["solved"] or limit_value is None else limit_value), r)
           for r in runs if not r.get("skipped") and (r["solved"] or limit_value is not None or key in r)]
    if not pts:
        return ""
    vals = np.array([p[2] for p in pts])
    lo_e = np.floor(np.log10(max(vals.min(), 1e-9)))
    hi_e = max(np.ceil(np.log10(vals.max())), lo_e + 1)
    ticks = [10 ** e for e in np.arange(lo_e, hi_e + 1)]
    y = lambda v: T + (H - T - B) * (1 - (np.log10(max(v, 1e-12)) - lo_e) / (hi_e - lo_e))
    offs = {m: (0 if len(METHODS) == 1 else (-8 if m == "weighted" else 8)) for m, _, _ in METHODS}
    x = lambda k, m: L + (W - L - R) * ((KS.index(k) + 0.5) / len(KS)) + offs[m]
    g = []
    for tv in ticks:
        g.append(f'<line x1="{L}" x2="{W - R}" y1="{y(tv):.1f}" y2="{y(tv):.1f}" class="grid"/>'
                 f'<text x="{L - 6}" y="{y(tv) + 4:.1f}" class="tick" text-anchor="end">{unit_fmt(tv)}</text>')
    if limit_value is not None and not ARGS.no_exact:
        g.append(f'<line x1="{L}" x2="{W - R}" y1="{y(limit_value):.1f}" y2="{y(limit_value):.1f}" class="limit"/>'
                 f'<text x="{W - R}" y="{y(limit_value) - 4:.1f}" class="tick" text-anchor="end">time limit</text>')
    for k_active in KS:
        g.append(f'<text x="{x(k_active, "weighted") - offs["weighted"]:.1f}" y="{H - B + 16}" class="tick" text-anchor="middle">{k_active}</text>')
    g.append(f'<text x="{(L + W - R) / 2:.1f}" y="{H - 4}" class="axis" text-anchor="middle">caps to turn, K</text>')
    jit = np.random.default_rng(1)
    for k_active, m, v, r in pts:
        what = (f"{unit_fmt(v)}, {r['actions']} actions" if r["solved"] else f"no plan ({'time limit' if r['timed_out'] else 'search exhausted'})")
        tip = f"{LABELS[m]}, K = {k_active}, query {r['query']}: {what}, caps {r['active_caps']}"
        cls = f"dot {SERIES_CLASS[m]}" + ("" if r["solved"] else " hollow")
        g.append(f'<circle cx="{x(k_active, m) + jit.uniform(-4, 4):.1f}" cy="{y(v):.1f}" r="4" class="{cls}" data-tip="{tip}"/>')
    for m, _, _ in METHODS:
        med = [(k, np.median([v for kk, mm, v, r in pts if kk == k and mm == m and r["solved"]])) for k in KS
               if any(kk == k and mm == m and r["solved"] for kk, mm, _, r in pts)]
        if med:
            g.append(f'<polyline class="median {SERIES_CLASS[m]}" points="' + " ".join(f"{x(k, m):.1f},{y(v):.1f}" for k, v in med) + '"/>')
            for k, v in med:
                g.append(f'<rect x="{x(k, m) - 7:.1f}" y="{y(v) - 1.5:.1f}" width="14" height="3" rx="1.5" class="medtick {SERIES_CLASS[m]}" '
                         f'data-tip="{LABELS[m]}, K = {k}: median {unit_fmt(v)}"/>')
    legend = "".join(f'<span><i class="sw {SERIES_CLASS[m]}"></i>{LABELS[m]}</span>' for m, _, _ in METHODS)
    if not ARGS.no_exact:
        legend += '<span><i class="sw hollow s2"></i>no plan within the time limit</span>'
    return (f'<figure class="chart"><figcaption>{title}</figcaption><div class="legend">{legend}</div>'
            f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{title}">{"".join(g)}</svg>'
            f'<div class="sub">Dots: one query each. Bar and line: median of the solved queries. Log scale.</div></figure>')


def ms_fmt(v):
    return f"{v:,.0f} ms" if v >= 10 else (f"{v:g} ms" if v >= 1 else f"{v:.2g} ms")


charts = (dot_chart("Run time per query (search + placing the waypoints)", "total_s", ms_fmt, scale=1e3,
                    limit_value=None if ARGS.no_exact else ARGS.exact_time_limit * 1e3)
          + dot_chart("Nodes expanded per query", "expanded", lambda v: f"{v:,.0f}"))

# ---------------------------------------------------------------- page
rows_html = ""
for k_active, v in per_k.items():
    for j_m, (m, w, _) in enumerate(METHODS):
        st = v[m]
        k_cell = f"<td rowspan='{len(METHODS)}'>{k_active}</td><td rowspan='{len(METHODS)}'>{st['total_turn']:.1f}</td>" if j_m == 0 else ""
        name = f"<td class='name'><i class='sw {SERIES_CLASS[m]}'></i>{LABELS[m]}</td>"
        if st["skipped"] == st["queries"]:
            rows_html += f"<tr>{k_cell}{name}<td colspan='8' class='note'>not run: no query with K = {exact_given_up} solved within {ARGS.exact_time_limit:g} s</td></tr>"
            continue
        if not st["solved"]:
            rows_html += f"<tr>{k_cell}{name}<td>0/{st['queries']}</td><td colspan='7' class='note'>no plan within {ARGS.exact_time_limit:g} s ({st['timed_out']} at the time limit)</td></tr>"
            continue
        same = ""
        if m == "weighted" and "paired" in v and v["paired"]["n"]:
            pr = v["paired"]
            same = f" <span class='muted'>(= exact on {pr['same_actions']}/{pr['n']})</span>"
        rows_html += (f"<tr>{k_cell}{name}<td>{st['solved']}/{st['queries']}</td><td>{st['actions']['median']:.0f}{same}</td>"
                      f"<td>{iqr(st['expanded'], lambda z: f'{z:,.0f}')}</td>"
                      f"<td>{iqr(st['search_s'], fmt_time)}</td><td>{fmt_time(st['place_s']['median'])}</td>"
                      f"<td>{iqr(st['total_s'], fmt_time)}</td><td>{fmt_time(st['total_s']['max'])}</td>"
                      f"<td>{fmt_mem(st['mem_gb']['max'])}</td></tr>")

html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>A* runtime vs caps</title>
<style>
:root {{ --bg:#fafafa; --fg:#1d1d1f; --muted:#666; --line:#ddd; --card:#fff; --s1:#2a78d6; --s2:#eb6834; --grid:#e6e6e6; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#141414; --fg:#eee; --muted:#aaa; --line:#333; --card:#1e1e1e; --s1:#3987e5; --s2:#d95926; --grid:#2c2c2c; }} }}
:root[data-theme="dark"] {{ --bg:#141414; --fg:#eee; --muted:#aaa; --line:#333; --card:#1e1e1e; --s1:#3987e5; --s2:#d95926; --grid:#2c2c2c; }}
body {{ margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }}
h1 {{ font-size:20px; margin:0 0 4px; }} h2 {{ font-size:16px; margin:22px 0 4px; }} p {{ margin:4px 0 12px; color:var(--muted); max-width:1100px; }}
.muted {{ color:var(--muted); }}
.tablewrap {{ overflow-x:auto; }}
table {{ border-collapse:collapse; margin:8px 0 16px; background:var(--card); }}
th, td {{ border-bottom:1px solid var(--line); padding:6px 10px; text-align:right; white-space:nowrap; }}
.row {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(320px, 1fr)); gap:12px; max-width:1100px; }}
figure {{ margin:0; background:var(--card); border:1px solid var(--line); border-radius:6px; padding:8px; max-width:100%; }}
figcaption {{ font-weight:600; margin-bottom:6px; }} .sub {{ color:var(--muted); margin-top:4px; font-size:12px; }}
svg {{ width:100%; height:auto; display:block; }}
.grid {{ stroke:var(--grid); stroke-width:1; }} .tick, .axis {{ fill:var(--muted); font-size:11px; }}
.dot {{ fill-opacity:0.6; stroke:var(--card); stroke-width:1.5; }} .dot:hover {{ fill-opacity:1; }}
.dot.s1 {{ fill:var(--s1); }} .dot.s2 {{ fill:var(--s2); }}
.dot.hollow {{ fill:var(--card); fill-opacity:1; stroke-width:2; }} .dot.hollow.s2 {{ stroke:var(--s2); }} .dot.hollow.s1 {{ stroke:var(--s1); }}
.median {{ fill:none; stroke-width:2; }} .median.s1 {{ stroke:var(--s1); }} .median.s2 {{ stroke:var(--s2); }}
.medtick.s1 {{ fill:var(--s1); }} .medtick.s2 {{ fill:var(--s2); }}
.limit {{ stroke:var(--muted); stroke-width:1; stroke-dasharray:4 3; }}
.legend {{ display:flex; flex-wrap:wrap; gap:4px 14px; font-size:12px; color:var(--fg); margin-bottom:4px; }}
.sw {{ display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:5px; vertical-align:-1px; }}
.sw.s1 {{ background:var(--s1); }} .sw.s2 {{ background:var(--s2); }} .sw.hollow {{ background:var(--card); border:2px solid var(--s2); width:8px; height:8px; }}
td.name, th.name {{ text-align:left; }} td.note {{ text-align:left; color:var(--muted); white-space:normal; }}
.video {{ max-width:{IMG_W + 16}px; }} canvas {{ display:block; width:100%; height:auto; border-radius:4px; background:#f7f7f7; }}
.controls {{ display:flex; align-items:center; gap:10px; margin:10px 0; flex-wrap:wrap; }}
button, select {{ font:inherit; padding:4px 12px; border-radius:4px; border:1px solid var(--line); background:var(--card); color:var(--fg); cursor:pointer; }}
input[type=range] {{ width:min(420px, 90vw); }}
#tip {{ position:fixed; pointer-events:none; background:var(--card); color:var(--fg); border:1px solid var(--line); border-radius:4px; padding:4px 8px; font-size:12px; display:none; }}
</style></head><body>
<h1>Reachability A*: run time as the number of caps grows</h1>
<p>One scene: the Panda arm, 8 bottle caps on a 0.5 m arc around it and 3 walls between groups of caps
(<code>multicap_scene.py</code>); C-free is {graph['free_sets']} convex sets in the 9-D arm + finger space
(<code>{CFREE_FILE.name}</code>). For each K, {ARGS.queries} random queries: K random caps turn by random amounts in [0.5, π] rad
summing to 2.5·K rad, between random start and goal configurations. The A* may grasp only those K caps; every cap stays in its state.
Each query is solved by the weighted A* (the notebook's default, w = {W_NOTEBOOK:g}: guaranteed at most {W_NOTEBOOK:g}× the fewest actions)
{"" if ARGS.no_exact else f"and by the exact A* (w = 1: fewest actions), which stops after {ARGS.exact_time_limit:g} s per query"}.
Measured on this machine ({MACHINE}).</p>
<div class="tablewrap"><table><thead><tr><th>K</th><th>Total turn [rad]</th><th class="name">Method</th><th>Solved</th><th>Actions</th>
<th>Nodes expanded</th><th>Search</th><th>Placing waypoints</th><th>Total</th><th>Total, slowest</th><th>Peak memory</th></tr></thead>
<tbody>{rows_html}</tbody></table></div>
<p>Median over the solved queries, with the 25th–75th percentile in brackets. Actions: grasps + ungrasps (the A*'s cost);
"= exact on n/m": queries where the weighted plan needs as many actions as the exact one, out of those both solved.
Every plan found is certified ({sum(r.get("certified", False) for r in runs)}/{sum(r["solved"] for r in runs)}): every straight
move has both ends in one convex set, and only the grasped cap turns, by −Δwrist on a forward stroke. Not in the times: the shared preprocessing that turns the sets into the planning
graph ({graph['components']} grasp components, {graph['transit_edges']} transit and {graph['switch_edges']} switch edges),
{fmt_time(t_prep)} once per scene; C-free itself (offline, <code>experiments/cap_scaling/generate_cfree.py</code>).</p>
<div class="row">{charts}</div>
{video_html}
<div id="tip"></div>
<script>
const tip = document.getElementById("tip");
document.querySelectorAll("[data-tip]").forEach(el => {{
  el.addEventListener("mousemove", e => {{ tip.textContent = el.dataset.tip; tip.style.display = "block";
    tip.style.left = (e.clientX + 12) + "px"; tip.style.top = (e.clientY + 12) + "px"; }});
  el.addEventListener("mouseleave", () => {{ tip.style.display = "none"; }});
}});
</script></body></html>"""
(OUT / "cap_scaling.html").write_text(html)
print(f"PAGE cap_scaling.html {len(html) / 1e6:.1f} MB", flush=True)
