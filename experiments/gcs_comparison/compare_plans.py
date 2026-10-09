"""Compare the reachability A* with GCS on the same sets (paper Table III, Run A; the plan_comparison pages).

From ONE start and goal, for each turn D:
  - the reachability A*, GCS on the A*'s sequence of sets and, for --full-turns, the full GCS;
  - the compute time and peak extra memory of each. Each full GCS solve runs in a fresh process
    (full_gcs_child.py), so its memory is not hidden by memory an earlier solve left allocated;
  - an offscreen render (Drake VTK) of every plan, with the start/goal and wrist-limit markers as scene
    geometry;
  - for --page-turns, a self-contained side-by-side comparison page (compare_D*.html).
Writes summary.json, the scenario (scenario.npz, cfree_fixed.yaml) and the pages to --out.

All planning code is the notebook's own (full_arm_planner_gcs.ipynb, via run_notebook.py). Not timed for any
method: the shared preprocessing that builds the A*'s graph (transition sets, extreme points, transit and
switch edges), which the GCS graph reuses; it is timed once and reported separately.

Run from anywhere, with the obmp conda env (MOSEK licence: the repo's mosek.lic):
  python experiments/gcs_comparison/compare_plans.py --out artifacts/plan_comparison_new
  # reproduce Run A exactly (its saved start, goal and sets):
  python experiments/gcs_comparison/compare_plans.py --scenario artifacts/plan_comparison --out artifacts/run_a_again
"""
import argparse
import base64
import contextlib
import io
import json
import os
import sys
import threading
import time as _t
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--out", required=True, help="output folder (created)")
parser.add_argument("--scenario", help="folder with scenario.npz and cfree_fixed.yaml to reproduce exactly")
parser.add_argument("--turns", default="0.5,1.5,6", help="cap turns D in rad (A* and GCS on the A* sequence)")
parser.add_argument("--full-turns", default="0.5,1.5", help="turns that also get the full GCS (6 does not fit in 60 GB)")
parser.add_argument("--page-turns", default="0.5,1.5", help="turns that get a comparison page")
parser.add_argument("--memory-limit", type=float, default=28.0, help="GB; stop the run above this resident memory")
parser.add_argument("--verbose", action="store_true", help="print the notebook cells' output")
ARGS = parser.parse_args()

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_notebook import REPO_ROOT, code_cell, run_until_gcs_section   # noqa: E402

os.environ.setdefault("MOSEKLM_LICENSE_FILE", str(REPO_ROOT / "mosek.lic"))
OUT = Path(ARGS.out).resolve()
OUT.mkdir(parents=True, exist_ok=True)
MESH_DIR = OUT / "render_meshes"
MESH_DIR.mkdir(exist_ok=True)
SCENARIO = Path(ARGS.scenario).resolve() if ARGS.scenario else None
TURNS = [float(x) for x in ARGS.turns.split(",")]
FULL_TURNS = {float(x) for x in ARGS.full_turns.split(",") if x}
PAGE_TURNS = {float(x) for x in ARGS.page_turns.split(",") if x}
SPEED, FPS, IMG_W, IMG_H = 1.0, 12, 560, 420     # playback speed (config-space units/s), frame rate, image size

# ---------------------------------------------------------------- the notebook, up to its GCS section
print("running the notebook cells (scene, free-set fixing, A*)...", flush=True)
run_until_gcs_section(verbose=ARGS.verbose, ns=globals())

import numpy as np                                                       # noqa: E402
from PIL import Image                                                    # noqa: E402
from pydrake.all import (RobotDiagramBuilder, Parser, RigidTransform, RotationMatrix, RollPitchYaw, Box,  # noqa: E402
                         Mesh, Rgba, Role, MakeRenderEngineVtk, RenderEngineVtkParams, ColorRenderCamera,
                         RenderCameraCore, CameraInfo, ClippingRange)
from pydrake.geometry.optimization import LoadIrisRegionsYamlFile, SaveIrisRegionsYamlFile  # noqa: E402
from pydrake.solvers import MosekSolver                                  # noqa: E402
import meshcat_plan_markers as mpm                                       # noqa: E402

exec(code_cell("from pydrake.planning import GcsTrajectoryOptimization"), globals())
exec(code_cell("def astar_vertex_sequence(built, path):").split("\n\nastar_seq =")[0], globals())

# ---------------------------------------------------------------- memory sampler
_peak = [0.0]


def rss_gb():
    for line in open('/proc/self/status'):
        if line.startswith('VmRSS'):
            return int(line.split()[1]) / 1e6


def _sample():
    while True:
        r = rss_gb()
        _peak[0] = max(_peak[0], r)
        if r > ARGS.memory_limit:
            print(f"memory limit: {r:.1f} GB > {ARGS.memory_limit} GB, stopping", flush=True)
            os._exit(3)
        _t.sleep(0.05)


threading.Thread(target=_sample, daemon=True).start()


def measured(fn):
    """Run fn(); return (result, seconds, peak extra memory in GB over the RSS before the call)."""
    base = rss_gb()
    _peak[0] = base
    t0 = _t.perf_counter()
    out = fn()
    return out, _t.perf_counter() - t0, max(0.0, _peak[0] - base)


# ---------------------------------------------------------------- scenario and shared preprocessing
if SCENARIO:   # reuse a saved start, goal and sets: the A* steps below rebuild everything from them
    sc = np.load(SCENARIO / "scenario.npz")
    planner.cs_free = list(LoadIrisRegionsYamlFile(str(SCENARIO / "cfree_fixed.yaml")).values())
    q_init_astar, q_goal_astar = sc["q_init"].copy(), sc["q_goal_base"].copy()
alpha_init = float(q_init_astar[idx_cap])

# A* Steps 1-4 (cap-dropped sets, transition sets and their extreme points, components, transit/switch
# edges): used by every method. Re-run here to time it (and, with --scenario, to rebuild it).
PREP_PREFIXES = ("import itertools\nfrom pydrake.solvers import MathematicalProgram, Solve", "def extreme_point(",
                 "L_MIN = 0.05", "n_free = len(free_sets)")
# One MOSEK licence is held for the whole run (the notebook's cell acquires it; this is a fallback), so no
# method pays a licence checkout per solve. Each full GCS child process holds its own.
licence = MosekSolver.AcquireLicense() if MosekSolver().enabled() else None
with contextlib.redirect_stdout(io.StringIO()):
    _, t_prep, _ = measured(lambda: [exec(code_cell(p), globals()) for p in PREP_PREFIXES])
print(f"shared preprocessing: {t_prep:.2f}s; MOSEK enabled: {MosekSolver().enabled()}", flush=True)
np.savez(OUT / "scenario.npz", q_init=q_init_astar, q_goal_base=q_goal_astar, idx_cap=idx_cap, idx_wrist=idx_wrist)
SaveIrisRegionsYamlFile(str(OUT / "cfree_fixed.yaml"), {f"region_{i:03d}": r for i, r in enumerate(planner.cs_free)})

# ---------------------------------------------------------------- plans
plans, rows = {}, {}
for D in TURNS:
    q_goal = q_goal_astar.copy(); q_goal[idx_cap] = alpha_init - D
    tag = f"D{str(D).replace('.', 'p')}"

    def run_astar():
        p, _ = reachability_astar(q_init_astar, q_goal)
        w, _ = build_waypoints(p, q_init_astar, q_goal)
        return p, w
    (apath, awps), t_a, m_a = measured(run_astar)
    plans[(D, "astar")] = awps
    rows[(D, "astar")] = dict(plan_metrics(awps), valid=bool(check_waypoints(awps, verbose=False)), time=t_a, mem=m_a)

    N = int(np.ceil(D / L_MAX)) + 1
    b, t_b, m_b = measured(lambda: build_layered_gcs(q_init_astar, q_goal, N, keep={n.set_id for n in apath if n.kind == "F"}))
    g = b["gcs"].graph_of_convex_sets()
    size = dict(vertices=g.num_vertices(), edges=g.num_edges(), layers=N, build_time=t_b)
    o = GraphOfConvexSetsOptions(); o.max_rounded_paths = 10

    seq = astar_vertex_sequence(b, apath)
    (_, rres), t_r, m_r = measured(lambda: b["gcs"].SolveConvexRestriction(seq, o))
    if rres.is_success():
        rw = gcs_waypoints(b, rres, seq); plans[(D, "gcs_on_astar")] = rw
        rows[(D, "gcs_on_astar")] = dict(plan_metrics(rw), valid=bool(check_waypoints(rw, verbose=False)),
                                         time=t_a + t_b + t_r, time_parts=dict(astar=t_a, build=t_b, restriction=t_r),
                                         mem=max(m_a, m_b, m_r), sets=len(seq) - 2)
    else:
        print(f"{tag}: restriction failed {rres.get_solution_result()}", flush=True)

    del b
    if D in FULL_TURNS:
        # A fresh process per full solve (it reloads the scenario saved above): clean memory, unaffected by
        # earlier solves, and MOSEK stays multithreaded (a forked child ran the same solve ~4x slower).
        import subprocess
        result_file = OUT / f"_full_gcs_{tag}.json"
        subprocess.run([sys.executable, str(Path(__file__).resolve().parent / "full_gcs_child.py"),
                        "--scenario", str(OUT), "--turn", str(D), "--result", str(result_file)],
                       check=True, cwd=REPO_ROOT)
        res = json.loads(result_file.read_text())
        result_file.unlink()
        if "waypoints" in res:
            res["waypoints"] = [(np.array(q), label, tuple(cert)) for q, label, cert in res["waypoints"]]
        size = dict(vertices=res["vertices"], edges=res["edges"], layers=res["layers"], build_time=res["time_parts"]["build"])
        tp = res["time_parts"]; mem = res["peak"] - res["base"]
        if "waypoints" in res:
            fw = res["waypoints"]; plans[(D, "gcs_full")] = fw
            rows[(D, "gcs_full")] = dict(plan_metrics(fw), valid=bool(check_waypoints(fw, verbose=False)),
                                         time=tp["build"] + tp["solve"], time_parts=tp, mem=mem,
                                         peak_process=res["peak"], cost=res["cost"], **size)
        else:
            rows[(D, "gcs_full")] = dict(failed=res["failed"], time=tp["build"] + tp["solve"], mem=mem, **size)
    print("results", tag, json.dumps({k[1]: v for k, v in rows.items() if k[0] == D}, default=float), flush=True)

(OUT / "summary.json").write_text(json.dumps(dict(shared_preprocessing_s=t_prep, **{f"D{k[0]}_{k[1]}": v for k, v in rows.items()}), indent=1, default=float))

# ---------------------------------------------------------------- render scene with markers as geometry
def write_obj(path, V, F):           # V: 3xN, F: 3xM (0-based); area-weighted vertex normals (the renderer needs them)
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
hand_bodies = [plant.get_body(i) for i in plant.GetBodyIndices(panda_hand)]
wrist_joint = plant.GetJointByName("panda_joint7", panda_arm)
cap_joint = plant.GetJointByName("cap_to_base", cap)
ctx = plant.CreateDefaultContext()
X_CH = plant.CalcRelativeTransform(ctx, wrist_joint.frame_on_child(), plant.GetBodyByName("panda_hand", panda_hand).body_frame())
v = X_CH.rotation().matrix() @ np.array([0.0, 1.0, 0.0]); phi = float(np.arctan2(v[1], v[0]))
X_6F = wrist_joint.frame_on_parent().GetFixedPoseInBodyFrame()
X_7C = wrist_joint.frame_on_child().GetFixedPoseInBodyFrame()
X_WCap = cap_joint.frame_on_parent().CalcPoseInWorld(ctx)
lower, upper = wrist_joint.position_lower_limits()[0] + phi, wrist_joint.position_upper_limits()[0] + phi

hand_meshes = []     # (body name, X_BG, obj path)
for body in hand_bodies:
    for k, gid in enumerate(inspector.GetGeometries(plant.GetBodyFrameIdOrThrow(body.index()), Role.kIllustration)):
        shape = inspector.GetShape(gid)
        if isinstance(shape, Mesh) and shape.extension() == ".gltf":
            V, F = mpm.read_gltf_triangles(shape.source().path())
            hand_meshes.append((body.name(), inspector.GetPoseInFrame(gid),
                                write_obj(MESH_DIR / f"{body.name()}_{k}.obj", (V * shape.scale3()).T, F.T)))

def build_render_scene(q_start, q_goal):
    rb = RobotDiagramBuilder(time_step=0.0); rp = rb.plant(); rsg = rb.scene_graph()
    rsg.AddRenderer("vtk", MakeRenderEngineVtk(RenderEngineVtkParams(default_clear_color=[0.97, 0.97, 0.97])))
    pr = Parser(rp, rsg); pr.SetAutoRenaming(True)
    arm = pr.AddModels(url=f"file://{REPO_ROOT}/my_sdfs/panda_arm.urdf")[0]
    hnd = pr.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]
    rp.WeldFrames(rp.world_frame(), rp.GetFrameByName("panda_link0", arm), RigidTransform())
    rp.WeldFrames(rp.GetFrameByName("panda_link8", arm), rp.GetFrameByName("panda_hand", hnd),
                  RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0]))
    cp = pr.AddModels(file_name="my_sdfs/bottle_cap.sdf")[0]
    o1 = pr.AddModels("my_sdfs/obstacle.sdf")[0]; o3 = pr.AddModels("my_sdfs/obstacle.sdf")[0]
    rp.WeldFrames(rp.world_frame(), rp.GetFrameByName("base_link", cp), RigidTransform(RotationMatrix(), [0.5, 0, 0]))
    rp.WeldFrames(rp.world_frame(), rp.GetFrameByName("obstacle_link", o1), RigidTransform(RotationMatrix(), [0.51, 0.031, 0.01]))
    rp.WeldFrames(rp.world_frame(), rp.GetFrameByName("obstacle_link", o3), RigidTransform(RotationMatrix(), [0.467, -0.005, 0.01]))
    world = rp.world_body(); n = [0]
    def add(body, X, shape, rgba):
        n[0] += 1; rp.RegisterVisualGeometry(body, X, shape, f"marker_{n[0]}", rgba.rgba)
    # ghost grippers (real hand meshes) at the start and goal configurations
    for q, rgba in ((q_start, mpm.START_RGBA), (q_goal, mpm.GOAL_RGBA)):
        plant.SetPositions(ctx, q)
        for name, X_BG, path in hand_meshes:
            add(world, plant.EvalBodyPoseInWorld(ctx, plant.GetBodyByName(name, panda_hand)) @ X_BG, Mesh(path), rgba)
    # cap: ticks at the start/goal angles + spiral arc with arrowhead
    a0, a1 = float(q_start[idx_cap]), float(q_goal[idx_cap])
    for a, rgba, dz in ((a0, mpm.START_RGBA, 0.0), (a1, mpm.GOAL_RGBA, 0.0005)):
        r0, r1 = mpm.CAP_TICK
        add(world, X_WCap @ RigidTransform(RotationMatrix.MakeZRotation(a), [(r0 + r1) / 2, 0, mpm.CAP_Z + dz]),
            Box(r1 - r0, 0.003, 0.002), Rgba(rgba.r(), rgba.g(), rgba.b(), 1.0))
    if abs(a1 - a0) > 1e-6:
        V, F = mpm.annular_sector(*mpm.CAP_ARC, a0, a1, z=mpm.CAP_Z, n=max(16, int(abs(a1 - a0) * 20)), growth=mpm.CAP_ARC_GROWTH)
        add(world, X_WCap, Mesh(write_obj(MESH_DIR / "cap_arc.obj", V, F)), mpm.ARC_RGBA)
    # wrist dial on link 6 (allowed / forbidden bands, start/goal ticks) and the needle on link 7
    link6, link7 = rp.GetBodyByName("panda_link6", arm), rp.GetBodyByName("panda_link7", arm)
    V, F = mpm.annular_sector(*mpm.DIAL_R, lower, upper, z=mpm.DIAL_Z)
    add(link6, X_6F, Mesh(write_obj(MESH_DIR / "dial_allowed.obj", V, F)), Rgba(0.35, 0.35, 0.35, 0.6))
    V, F = mpm.annular_sector(*mpm.DIAL_R, upper, lower + 2 * np.pi, z=mpm.DIAL_Z)
    add(link6, X_6F, Mesh(write_obj(MESH_DIR / "dial_forbidden.obj", V, F)), Rgba(0.9, 0.1, 0.1, 0.45))
    r0, r1 = mpm.DIAL_R[0] - 0.005, mpm.DIAL_R[1] + 0.005
    for w, rgba in ((float(q_start[idx_wrist]), mpm.START_RGBA), (float(q_goal[idx_wrist]), mpm.GOAL_RGBA)):
        add(link6, X_6F @ RigidTransform(RotationMatrix.MakeZRotation(w + phi), [(r0 + r1) / 2, 0, mpm.DIAL_Z + 0.001]),
            Box(r1 - r0, 0.005, 0.003), Rgba(rgba.r(), rgba.g(), rgba.b(), 1.0))
    L = mpm.DIAL_R[1] + 0.01
    add(link7, X_7C @ RigidTransform(RotationMatrix.MakeZRotation(phi), [L / 2, 0, mpm.DIAL_Z + 0.002]),
        Box(L, 0.005, 0.004), mpm.NEEDLE_RGBA)
    rp.Finalize()
    d = rb.Build(); dctx = d.CreateDefaultContext()
    return rp, rsg, d, dctx

def look_at(eye, target):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye; f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
    dn = np.cross(f, r)
    return RigidTransform(RotationMatrix(np.column_stack([r, dn, f])), eye)

CAM = ColorRenderCamera(RenderCameraCore("vtk", CameraInfo(IMG_W, IMG_H, np.deg2rad(45)), ClippingRange(0.02, 10), RigidTransform()))
VIEW_DIR = np.array([1.0, -1.1, 0.75]) / np.linalg.norm([1.0, -1.1, 0.75])   # camera sits on this side of the scene

def frame_camera(waypoint_lists):
    """Look-at camera that fits the arm links, hand and cap over every waypoint of the given plans."""
    pts = [X_WCap.translation()]
    bodies = [plant.GetBodyByName(n, panda_arm) for n in ("panda_link2", "panda_link4", "panda_link6", "panda_link7")]
    bodies.append(plant.GetBodyByName("panda_hand", panda_hand))
    for wps in waypoint_lists:
        for q, _, _ in wps:
            plant.SetPositions(ctx, q)
            pts += [plant.EvalBodyPoseInWorld(ctx, b).translation() for b in bodies]
    pts = np.array(pts)
    center = 0.5 * (pts.min(0) + pts.max(0))
    radius = np.max(np.linalg.norm(pts - center, axis=1)) + 0.08
    dist = radius / np.sin(np.deg2rad(45) / 2) * 0.85
    return look_at(center + dist * VIEW_DIR, center)

def render_frames(waypoints, scene, X_WCam):
    rp, rsg, d, dctx = scene
    pctx = rp.GetMyMutableContextFromRoot(dctx); sctx = rsg.GetMyContextFromRoot(dctx)
    qs = [waypoints[0][0]]
    for (qa, _, _), (qb, _, _) in zip(waypoints[:-1], waypoints[1:]):
        k = max(1, int(np.ceil(np.linalg.norm(qb - qa) / SPEED * FPS)))
        qs += [qa + s * (qb - qa) for s in np.linspace(0, 1, k + 1)[1:]]
    qs += [waypoints[-1][0]] * FPS
    frames = []
    for q in qs:
        rp.SetPositions(pctx, q)
        img = rsg.get_query_output_port().Eval(sctx).RenderColorImage(CAM, rsg.world_frame_id(), X_WCam)
        buf = io.BytesIO(); Image.fromarray(np.asarray(img.data)[:, :, :3]).save(buf, "JPEG", quality=72)
        frames.append(base64.b64encode(buf.getvalue()).decode())
    return frames

# ---------------------------------------------------------------- comparison pages
NAMES = {"astar": "Reachability A*", "gcs_on_astar": "GCS on the A* sequence", "gcs_full": "Full GCS"}
NOTES = {"astar": "search + placing the waypoints",
         "gcs_on_astar": "A* search + building the layered GCS graph + one convex program on the A*'s sets",
         "gcs_full": "building the layered GCS graph + convex relaxation + rounding to ≤10 candidate paths (MOSEK)"}

def fmt_time(s):
    return f"{s * 1e3:.1f} ms" if s < 1 else f"{s:.1f} s"

def fmt_mem(gb):
    return f"{gb * 1e3:.0f} MB" if gb < 1 else f"{gb:.1f} GB"

for D in TURNS:
    if D not in PAGE_TURNS:
        continue
    q_goal = q_goal_astar.copy(); q_goal[idx_cap] = alpha_init - D
    scene = build_render_scene(q_init_astar, q_goal)
    methods = [m for m in ("astar", "gcs_on_astar", "gcs_full") if (D, m) in plans]
    X_WCam = frame_camera([plans[(D, m)] for m in methods])
    videos = {m: render_frames(plans[(D, m)], scene, X_WCam) for m in methods}
    print(f"RENDERED D={D}: " + ", ".join(f"{m} {len(f)} frames" for m, f in videos.items()), flush=True)
    table = "".join(
        f"<tr><td>{NAMES[m]}</td><td>{int(rows[(D, m)]['grasps'])}</td><td>{rows[(D, m)]['path_length']:.2f}</td>"
        f"<td>{fmt_time(rows[(D, m)]['time'])}</td><td>{fmt_mem(rows[(D, m)]['mem'])}</td>"
        f"<td>{'yes' if rows[(D, m)]['valid'] else 'NO'}</td><td class='note'>{NOTES[m]}</td></tr>" for m in methods)
    full = rows.get((D, "gcs_full"), {})
    graph = (f"Full GCS graph: {full.get('layers')} grasp layers, {full.get('vertices')} vertices, {full.get('edges')} edges."
             if full else "")
    panels = "".join(f'<figure><figcaption>{NAMES[m]}</figcaption><canvas id="c_{m}" width="{IMG_W}" height="{IMG_H}"></canvas>'
                     f'<div class="sub">{int(rows[(D, m)]["grasps"])} grasp(s) · length {rows[(D, m)]["path_length"]:.2f} · '
                     f'{len(videos[m]) / FPS:.1f} s</div></figure>' for m in methods)
    frames_js = json.dumps({m: videos[m] for m in methods})
    html = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Plan comparison, turn {D} rad</title>
<style>
:root {{ --bg:#fafafa; --fg:#1d1d1f; --muted:#666; --line:#ddd; --card:#fff; --accent:#2b6cb0; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#141414; --fg:#eee; --muted:#aaa; --line:#333; --card:#1e1e1e; --accent:#7fb2ff; }} }}
body {{ margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px/1.45 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }}
h1 {{ font-size:20px; margin:0 0 4px; }} p {{ margin:4px 0 12px; color:var(--muted); max-width:1100px; }}
table {{ border-collapse:collapse; margin:8px 0 16px; background:var(--card); }}
th, td {{ border-bottom:1px solid var(--line); padding:6px 10px; text-align:right; white-space:nowrap; }}
th:first-child, td:first-child, td.note, th:last-child {{ text-align:left; }} td.note {{ color:var(--muted); white-space:normal; }}
.row {{ display:grid; grid-template-columns:repeat(auto-fit, minmax(340px, 1fr)); gap:12px; }}
figure {{ margin:0; background:var(--card); border:1px solid var(--line); border-radius:6px; padding:8px; max-width:100%; }}
figcaption {{ font-weight:600; margin-bottom:6px; }} canvas {{ display:block; width:100%; height:auto; border-radius:4px; background:#f7f7f7; }}
.sub {{ color:var(--muted); margin-top:4px; }}
.controls {{ display:flex; align-items:center; gap:10px; margin:14px 0; flex-wrap:wrap; }}
button, select {{ font:inherit; padding:4px 12px; border-radius:4px; border:1px solid var(--line); background:var(--card); color:var(--fg); cursor:pointer; }}
input[type=range] {{ width:min(520px, 90vw); }}
.legend span {{ display:inline-block; margin-right:14px; }}
.sw {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:5px; vertical-align:-1px; }}
</style></head><body>
<h1>Turn {D} rad: reachability A* vs GCS</h1>
<p>Same 10-DOF scene (two obstacles, 42 free sets, validated transition sets, L<sub>max</sub> = 0.81 rad) and the same start
and goal for every plan. All plans play at the same speed in configuration space ({SPEED:g} unit/s), so a longer clip means a longer path.
Time and memory are measured on this machine (AMD Ryzen 9 9950X, 60 GB); memory is the peak extra resident memory of the computation.</p>
<table><thead><tr><th>Method</th><th>Grasps</th><th>Path length</th><th>Compute time</th><th>Peak memory</th><th>Moves certified</th><th>What the time covers</th></tr></thead>
<tbody>{table}</tbody></table>
<p>Not in the times above, because every method uses it: the shared preprocessing that turns the free sets into the
planning graph (transition sets, their extreme wrist points, transit and switch edges), {fmt_time(t_prep)} once per scene.
The free sets themselves (IRIS) are computed offline and loaded. {graph}
"Moves certified": every straight move has both ends in one validated convex set, and the cap turns only during a stroke.</p>
<div class="legend"><span><i class="sw" style="background:#33cc33"></i>start (ghost hand, cap tick, wrist tick)</span>
<span><i class="sw" style="background:#ff8c00"></i>goal</span><span><i class="sw" style="background:#595959"></i>wrist range</span>
<span><i class="sw" style="background:#e61a1a"></i>beyond the wrist limit</span><span><i class="sw" style="background:#1a1a1a"></i>needle = current wrist angle</span></div>
<div class="controls"><button id="play">Pause</button><button id="restart">Restart</button>
<label>Speed <select id="speed"><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select></label>
<input type="range" id="scrub" min="0" max="1000" value="0"><span id="clock">0.0 s</span></div>
<div class="row">{panels}</div>
<script>
const FPS = {FPS}, FRAMES = {frames_js};
const imgs = {{}}, ctxs = {{}};
let maxLen = 0;
for (const [m, list] of Object.entries(FRAMES)) {{
  imgs[m] = list.map((b, i) => {{ const im = new Image(); if (i === 0) im.onload = () => draw(); im.src = "data:image/jpeg;base64," + b; return im; }});
  ctxs[m] = document.getElementById("c_" + m).getContext("2d");
  maxLen = Math.max(maxLen, list.length);
}}
let t = 0, playing = true, speed = 1, last = null;
const scrub = document.getElementById("scrub"), clock = document.getElementById("clock"), play = document.getElementById("play");
function draw() {{
  const f = Math.floor(t * FPS);
  for (const m in imgs) {{ const list = imgs[m], im = list[Math.min(f, list.length - 1)]; if (im.complete) ctxs[m].drawImage(im, 0, 0); }}
  scrub.value = Math.round(1000 * Math.min(1, f / (maxLen - 1))); clock.textContent = (t).toFixed(1) + " s";
}}
function tick(now) {{
  if (last !== null && playing) {{ t += (now - last) / 1000 * speed; if (t * FPS >= maxLen - 1) {{ t = (maxLen - 1) / FPS; playing = false; play.textContent = "Play"; }} }}
  last = now; draw(); requestAnimationFrame(tick);
}}
play.onclick = () => {{ if (t * FPS >= maxLen - 1) t = 0; playing = !playing; play.textContent = playing ? "Pause" : "Play"; }};
document.getElementById("restart").onclick = () => {{ t = 0; playing = true; play.textContent = "Pause"; }};
document.getElementById("speed").onchange = e => {{ speed = parseFloat(e.target.value); }};
scrub.oninput = () => {{ t = scrub.value / 1000 * (maxLen - 1) / FPS; draw(); }};
requestAnimationFrame(tick);
</script></body></html>"""
    name = f"compare_D{str(D).replace('.', 'p')}.html"
    (OUT / name).write_text(html)
    print(f"PAGE {name} {len(html) / 1e6:.1f} MB", flush=True)
