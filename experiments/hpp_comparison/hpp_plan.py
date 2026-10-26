"""Manipulation RRT (HPP) on the cap scene that export_scene.py writes. Runs in the hpp:obmp container (env/).

One step per tutorial section (see GUIDE.md). --stop-after stops after a step; with python -i you are then left
in an interactive shell with the viewer (the address viser prints, usually http://localhost:8000), as with the tutorials' `python -i init.py`:
  cd /home/user/obmp
  python -i experiments/hpp_comparison/hpp_plan.py --stop-after robot          # robot, env, cap, graph, waypoints
  python -i experiments/hpp_comparison/hpp_plan.py --stop-after solve --query D0p5 --seed 0
  python experiments/hpp_comparison/hpp_plan.py --stop-after batch --seeds 10  # every query, writes raw/*.npz
"""
import argparse
import ctypes
import json
import threading
import time
from pathlib import Path

import numpy as np
from pinocchio import SE3, neutral
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint
from pyhpp.core import Dichotomy
from pyhpp.manipulation import Device, Graph, ManipulationPlanner, Problem, ProgressiveProjector, urdf
from pyhpp.manipulation.constraint_graph_factory import ConstraintGraphFactory, Rule
from pyhpp.manipulation.security_margins import SecurityMargins

STEPS = ["robot", "env", "cap", "graph", "waypoints", "solve", "batch"]
ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--scene-dir", default="/home/user/obmp/artifacts/hpp_comparison/1cap_2obs", help="export_scene.py's output")
ap.add_argument("--stop-after", choices=STEPS, default="solve") 
ap.add_argument("--no-viewer", action="store_true")
ap.add_argument("--query", default="D0p5", help="solve: the query")
ap.add_argument("--seed", type=int, default=0, help="solve: the random seed")
ap.add_argument("--queries", help="batch: comma-separated query names (default: all)")
ap.add_argument("--seeds", type=int, default=10, help="batch: seeds 0..N-1 per query")
ap.add_argument("--time-limit", type=float, default=60.0, help="s per run")
ARGS = ap.parse_args()
D = Path(ARGS.scene_dir).resolve()          # loadModel needs absolute paths
META = json.loads((D / "scene.json").read_text())

GRIPPER = "panda/gripper"
HANDLE_MASK = [True, True, True, True, True, False]   # x y z rx ry rz of the handle: rotation about z (the cap axis) free
GRASP_FINGERS = {"panda/panda_finger_joint1": -0.02405, "panda/panda_finger_joint2": 0.02405}   # the notebooks' grasp width


def upto(step):
    """True if --stop-after is this step or a later one."""
    return STEPS.index(step) <= STEPS.index(ARGS.stop_after)


class Converter:
    """Drake position order (export_scene.py) <-> HPP configuration, matched by joint name. Joints of models
    not loaded yet are skipped."""

    def __init__(self, robot, names):
        have = set(robot.getJointNames())
        self.robot, self.n = robot, len(names)
        self.pairs = [(i, robot.rankInConfiguration[n]) for i, n in enumerate(names) if n in have]

    def to_hpp(self, q):
        x = neutral(self.robot.model())
        for i, r in self.pairs:
            x[r] = q[i]
        return x

    def to_drake(self, x):
        q = np.zeros(self.n)
        for i, r in self.pairs:
            q[i] = x[r]
        return q


def load_model(prefix, name, pose=SE3.Identity()):
    """Tutorial 2's urdf.loadModel(robot, 0, prefix, "anchor", urdf, srdf, pose), on export_scene.py's files. They
    refer to their visual meshes as @SCENE_DIR@/meshes/..., filled in here, so the same files work on the host
    and in the container (hence loadModelFromString, which takes the files' text)."""
    urdf.loadModelFromString(robot, 0, prefix, "anchor", (D / f"{name}.urdf").read_text().replace("@SCENE_DIR@", str(D)),
                             (D / f"{name}.srdf").read_text(), pose)


def start_configuration():
    """The first query's start configuration, to show in the viewer (all zeros is not a valid Panda pose)."""
    return conv.to_hpp(np.array(json.loads((D / "queries.json").read_text())[0]["q_init"]))


# ---------------------------------------------------------------- step 1: the robot (tutorial 2)
robot = Device("cap-scene")
load_model("panda", "panda")
conv = Converter(robot, META["hpp_joint_names"])
q_show = start_configuration()


def check_fk():
    """Step 1 checkpoint: the hand pose in HPP vs Drake, on export_scene.py's reference configurations."""
    z = np.load(D / "consistency.npz")
    fid = robot.model().getFrameId("panda/panda_hand")
    err = 0.0
    for q, X in zip(z["q"], z["X_WE"]):
        robot.currentConfiguration(conv.to_hpp(q))
        robot.computeFramesForwardKinematics()
        err = max(err, float(np.abs(robot.data().oMf[fid].homogeneous - X).max()))
    print(f"hand pose, HPP vs Drake: max difference {err:.1e} over {len(z['q'])} configurations")


# ---------------------------------------------------------------- step 2: the obstacles (tutorial 2)
if upto("env"):
    load_model("env", "env")


# ---------------------------------------------------------------- step 3: the cap (tutorial 2, "Adding an object")
def new_problem():
    """A Problem whose isConfigValid checks joint bounds and collisions (a new one checks nothing), and whose
    motions are checked continuously. Each transition of the graph builds its own path validation (with its
    collision filters and security margins) from the problem's factory, by default discretized at a 0.05 step,
    which let a finger pass 4 mm into a post between two samples. Dichotomy with tolerance 0 bounds how far each
    body can move along a segment, so the whole segment is certified. (Setting problem.pathValidation instead
    replaces the graph's per-transition dispatch, and then no grasp is valid.)"""
    problem = Problem(robot)
    problem.clearConfigValidations()
    problem.addConfigValidation("JointBoundValidation")
    problem.addConfigValidation("CollisionValidation")
    problem.pathValidationFactory(Dichotomy(robot, 0.0))   # before build_graph: transitions copy it at initialize()
    # a motion in a constrained state (e.g. turning the cap) is a straight line projected onto the constraints;
    # the projector cuts it into short pieces (0.01) that stay close to their projection, which continuous
    # validation needs (without it, evaluating the path could jump; as in HPP's ur3-spheres M-RRT example)
    problem.pathProjector(ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.01))
    return problem


def check_collisions():
    """Step 3 checkpoint: HPP's vs Drake's collision verdict on export_scene.py's reference configurations."""
    z = np.load(D / "consistency.npz")
    bad = [i for i, (q, c) in enumerate(zip(z["q"], z["colliding"]))
           if bool(problem.isConfigValid(conv.to_hpp(q))[0]) == bool(c)]
    print(f"collisions, HPP vs Drake: {len(z['q']) - len(bad)}/{len(z['q'])} agree; disagreements: {bad[:10]}")


if upto("cap"):
    for c in META["caps"]:
        load_model(c["prefix"], "cap", SE3(np.array(c["rotation"]), np.array(c["position"])))
        robot.handles()[c["prefix"] + "/handle"].mask = HANDLE_MASK
    conv = Converter(robot, META["hpp_joint_names"])
    q_show = start_configuration()
    problem = new_problem()


# ---------------------------------------------------------------- step 4: the constraint graph (tutorial 2)
def build_graph(problem, active_caps):
    """Tutorial 2's graph for the caps: one gripper, one handle per cap, no contact surfaces (a cap's joint is
    locked outside a grasp, at whatever angle it has), fingers at the grasp width everywhere, and only the
    caps in active_caps may be grasped."""
    graph = Graph("graph", robot, problem)
    graph.maxIterations(40)
    graph.errorThreshold(1e-5)
    objects = [c["prefix"] for c in META["caps"]]
    factory = ConstraintGraphFactory(graph)
    factory.setGrippers([GRIPPER])
    factory.setObjects(objects, [[o + "/handle"] for o in objects], [[] for _ in objects])
    factory.setRules([Rule([GRIPPER], ["^$"], True)]                     # holding nothing: the free state
                     + [Rule([GRIPPER], [f"^cap{k}/handle$"], True) for k in active_caps])
    factory.generate()
    fixed = ComparisonTypes()
    fixed[:] = [ComparisonType.EqualToZero]
    graph.addNumericalConstraintsToGraph([LockedJoint(robot, n, np.array([v]), fixed) for n, v in GRASP_FINGERS.items()])
    # step 5 (tutorial 3, security margins): the fingers close around the cap with ~0.05 mm to spare, so the
    # last approach (pre-grasp -> grasp) and the first retreat may touch the cap
    sm = SecurityMargins(problem, factory, ["panda"] + objects, robot)
    sm.apply()
    for name in graph.getTransitionNames():
        if name.endswith(("_12", "_21", "_12_ls", "_21_ls")):
            k = name.split("cap")[1].split("/")[0]
            for j in ("panda/panda_joint7", "panda/panda_finger_joint1", "panda/panda_finger_joint2"):
                graph.setSecurityMarginForTransition(graph.getTransition(name), j, f"cap{k}/cap_to_base", float("-inf"))
    graph.initialize()
    problem.constraintGraph(graph)
    return graph


if upto("graph"):
    graph = build_graph(problem, active_caps=range(len(META["caps"])))
    print("states:", graph.getStateNames())


# ---------------------------------------------------------------- step 5: waypoints (tutorial 3)
if upto("waypoints"):
    QUERIES = json.loads((D / "queries.json").read_text())
    q_init = conv.to_hpp(np.array(QUERIES[0]["q_init"]))
    print("q_init valid:", problem.isConfigValid(q_init)[0])
    shooter = problem.configurationShooter()
    f01 = graph.getTransition("panda/gripper > cap0/handle | f_01")
    f12 = graph.getTransition("panda/gripper > cap0/handle | f_12")
    qpg = qg = None
    for i in range(1000):                                   # tutorial 3: a pre-grasp, then a grasp from it
        res, q1, err = graph.generateTargetConfig(f01, q_init, shooter.shoot())
        if not res or not f01.pathValidation().validateConfiguration(q1)[0]:
            continue
        res, q2, err = graph.generateTargetConfig(f12, q1, q1)
        if res and f12.pathValidation().validateConfiguration(q2)[0]:
            qpg, qg = q1, q2
            break
    print(f"pre-grasp and grasp: {'found' if qg is not None else 'NOT found'} after {i + 1} tries")
    # the last approach, as the planner checks it (with the margins)
    res = f12.pathValidation().validate(problem.steeringMethod()(qpg, qg), False)
    print("approach pre-grasp -> grasp valid:", res[0], "" if res[0] else res[-1])
    # the coupling: in a grasp, the cap must turn by minus the wrist (the paper's convention)
    rw, rc = robot.rankInConfiguration["panda/panda_joint7"], robot.rankInConfiguration["cap0/cap_to_base"]
    dw = 0.3 if qg[rw] < 0.7 else -0.3
    for sign in (-1, 1):
        q = qg.copy()
        q[rw] += dw
        q[rc] += sign * dw
        err, same_leaf = graph.getConfigErrorForTransitionLeaf(graph.getTransition("Loop | 0-0"), qg, q)
        print(f"d(cap) = {sign:+d} * d(wrist): stays in the grasp = {same_leaf}")
    q_show = qg


# ---------------------------------------------------------------- step 6: solve one query (tutorial 2, "Solving")
def solve(query, seed, time_limit):
    """One Manipulation RRT run, on a fresh problem and graph (the active caps depend on the query)."""
    problem = new_problem()
    graph = build_graph(problem, query["active_caps"])
    problem.initConfig(conv.to_hpp(np.array(query["q_init"])))
    problem.addGoalConfig(conv.to_hpp(np.array(query["q_goal"])))
    planner = ManipulationPlanner(problem)
    planner.maxIterations(10 ** 9)
    planner.timeOut(time_limit)               # tutorial 2 stops at maxIterations(500); we stop on time
    ctypes.CDLL("libc.so.6").srand(seed)      # HPP samples with std::rand: seed it, or every run is the same
    t0 = time.perf_counter()
    try:
        path, error = planner.solve(), None
    except Exception as e:                    # reaching the time limit raises
        path, error = None, str(e).strip()[:300]
    return dict(path=path, problem=problem, graph=graph, success=path is not None, error=error,
                time_s=time.perf_counter() - t0, roadmap_nodes=len(planner.roadmap().nodes()))


if ARGS.stop_after == "solve":
    query = next(q for q in QUERIES if q["name"] == ARGS.query)
    run = solve(query, ARGS.seed, ARGS.time_limit)
    problem, graph, path = run["problem"], run["graph"], run["path"]
    print(f"{query['name']} seed {ARGS.seed}: "
          + (f"solved in {run['time_s']:.1f} s" if path is not None else f"failed ({run['error']})")
          + f", {run['roadmap_nodes']} roadmap nodes")
    if path is not None:
        tr = path.timeRange()
        states = [graph.getStateFromConfiguration(np.asarray(path.eval(t)[0]))
                  for t in np.linspace(tr.first, tr.second, 400)]
        print("states along the path:", [s for i, s in enumerate(states) if i == 0 or s != states[i - 1]])


# ---------------------------------------------------------------- step 7: batch runs
_peak = [0.0]


def rss_gb():
    for line in open("/proc/self/status"):
        if line.startswith("VmRSS"):
            return int(line.split()[1]) / 1e6


def _sample_memory():                         # as in experiments/gcs_comparison
    while True:
        _peak[0] = max(_peak[0], rss_gb())
        time.sleep(0.005)


def sample_path(path, graph, dt=0.002):
    """The path every dt of its parameter, in Drake position order, with each sample's graph state and whether
    HPP could evaluate it (a constrained path is projected when evaluated, and the projection can fail)."""
    tr = path.timeRange()
    Q, S, OK = [], [], []
    for t in np.append(np.arange(tr.first, tr.second, dt), tr.second):
        x, ok = path.eval(t)
        x = np.asarray(x)
        Q.append(conv.to_drake(x))
        S.append(graph.getStateFromConfiguration(x))
        OK.append(bool(ok))
    return np.array(Q), np.array(S), np.array(OK)


if upto("batch"):
    threading.Thread(target=_sample_memory, daemon=True).start()
    out = D / "raw"
    out.mkdir(exist_ok=True)
    names = ARGS.queries.split(",") if ARGS.queries else [q["name"] for q in QUERIES]
    for query in [q for q in QUERIES if q["name"] in names]:
        if query["q_init_in_collision"] or query["q_goal_in_collision"]:   # export_scene.py's Drake check
            print(f"{query['name']}: skipped, its start or goal is in collision", flush=True)
            continue
        for seed in range(ARGS.seeds):
            if (out / f"{query['name']}_s{seed}.npz").exists():   # resume: memory grows over many runs in one
                continue                                          # process, so long batches run per query
            base = rss_gb()
            _peak[0] = base
            run = solve(query, seed, ARGS.time_limit)
            rec = dict(query=query["name"], seed=seed, time_limit=ARGS.time_limit, success=run["success"],
                       error=run["error"] or "", time_s=run["time_s"], roadmap_nodes=run["roadmap_nodes"],
                       mem_gb=max(0.0, _peak[0] - base))
            if run["success"]:
                rec["path_q"], rec["path_states"], rec["path_eval_ok"] = sample_path(run["path"], run["graph"])
            np.savez(out / f"{query['name']}_s{seed}.npz", **rec)
            print(f"{query['name']} seed {seed}: "
                  + (f"solved in {run['time_s']:.1f} s" if run["success"] else "failed")
                  + f", {run['roadmap_nodes']} nodes", flush=True)


# ---------------------------------------------------------------- viewer (tutorial 2)
if not ARGS.no_viewer and ARGS.stop_after != "batch":
    from pyhpp_viser import Viewer
    v = Viewer(robot)
    v.initViewer(open=False, loadModel=True)
    if "graph" in globals():
        v.setProblem(problem)
        v.setGraph(graph)
    if globals().get("path") is not None:
        v.loadPath(path)          # then press Play in the view controller
    v(q_show)
    print("viewer: the HTTP address in the viser box above (usually http://localhost:8000)")
