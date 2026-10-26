"""Write a Drake scene as URDF/SRDF for HPP, plus the queries and a Drake reference set (hpp_comparison).

The scene is built in Drake exactly as the notebooks build it, and every file HPP reads is generated from that
plant, so the two cannot drift. Run with the obmp conda env, from the repo root:
  python experiments/hpp_comparison/export_scene.py --scene 1cap_2obs
  python experiments/hpp_comparison/export_scene.py --scene 8caps_3obs --queries artifacts/cap_scaling_hpp/queries.npz
Writes to artifacts/hpp_comparison/<scene>/: panda/env/cap .urdf/.srdf, scene.json, queries.json, consistency.npz.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path
from xml.sax.saxutils import quoteattr

import numpy as np
from pydrake.all import (BodyIndex, Box, Cylinder, Mesh, Parser, PrismaticJoint, RevoluteJoint, RigidTransform,
                         RobotDiagramBuilder, Role, RollPitchYaw, Sphere, WeldJoint)

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

MIN_DIST = 1e-5                        # collision: signed distance below this (check_collision_q_by_ik)
GRASP_HEIGHT = 0.5 * (0.105 + 0.110)   # hand origin above the cap base: middle of the grasp band [z_min, z_max]
HANDLE_CLEARANCE = 0.05                # pre-grasp: the hand this far above the grasp
FINGER_WIDTH = 0.02405                 # |finger joint| at a grasp (the notebooks' q0)
RUN_A = REPO_ROOT / "artifacts" / "plan_comparison" / "scenario.npz"   # 1cap_2obs start/goal (gcs_comparison, Run A)
RUN_A_TURNS = (0.5, 1.5, 6.0)
MESHES = "@SCENE_DIR@/meshes"          # visual meshes are copied to <out>/meshes; hpp_plan.py fills in the directory
Y_UP_TO_Z_UP = RigidTransform(RollPitchYaw(np.pi / 2, 0.0, 0.0), [0.0, 0.0, 0.0])   # what Drake applies to glTF


def build_scene(name):
    """The scene as the notebooks build it (multicap_scene.SCENES): '1cap_2obs' or '8caps_3obs'."""
    import multicap_scene
    builder = RobotDiagramBuilder(time_step=0.0)
    plant, scene_graph = builder.plant(), builder.scene_graph()
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    arm, hand = multicap_scene.add_panda(parser, plant)
    caps, _ = multicap_scene.add_scene(parser, plant, name)
    plant.Finalize()
    return dict(diagram=builder.Build(), plant=plant, scene_graph=scene_graph,
                models=dict(arm=arm, hand=hand, caps=caps))


class DrakeChecker:
    """The project's collision rule (check_collision_q_by_ik): inside the joint limits, and every unfiltered
    pair of geometries at signed distance >= MIN_DIST."""

    def __init__(self, scene):
        self.plant, self.scene_graph = scene["plant"], scene["scene_graph"]
        self.context = scene["diagram"].CreateDefaultContext()
        self.plant_context = self.plant.GetMyContextFromRoot(self.context)
        self.sg_context = self.scene_graph.GetMyContextFromRoot(self.context)
        self.lo, self.hi = self.plant.GetPositionLowerLimits(), self.plant.GetPositionUpperLimits()

    def colliding_bodies(self, q):
        """[(body A, body B)] for every pair closer than MIN_DIST at q."""
        self.plant.SetPositions(self.plant_context, q)
        qo = self.scene_graph.get_query_output_port().Eval(self.sg_context)
        insp = qo.inspector()
        body = lambda g: self.plant.GetBodyFromFrameId(insp.GetFrameId(g)).name()
        return [(body(p.id_A), body(p.id_B)) for p in qo.ComputeSignedDistancePairwiseClosestPoints(MIN_DIST)
                if p.distance < MIN_DIST]

    def in_collision(self, q, tol=1e-9):
        return bool(np.any(q < self.lo - tol) or np.any(q > self.hi + tol) or self.colliding_bodies(q))


# ---------------------------------------------------------------- URDF/SRDF writing
def _origin(X):
    xyz = " ".join(f"{v:.9g}" for v in X.translation())
    rpy = " ".join(f"{v:.9g}" for v in RollPitchYaw(X.rotation()).vector())
    return f'<origin xyz="{xyz}" rpy="{rpy}"/>'


def _shape(shape, out=None):
    """URDF geometry of a Drake shape. A mesh (visual only) is copied, with its folder (glTF buffers, textures),
    to out/meshes."""
    if isinstance(shape, Mesh):
        src = Path(shape.source().path())
        for f in src.parent.iterdir():
            if f.is_file() and not (out / "meshes" / f.name).exists():
                (out / "meshes").mkdir(exist_ok=True)
                shutil.copy(f, out / "meshes" / f.name)
        return f'<mesh filename="{MESHES}/{src.name}" scale="{" ".join(f"{v:.9g}" for v in shape.scale3())}"/>'
    if isinstance(shape, Sphere):
        return f'<sphere radius="{shape.radius():.9g}"/>'
    if isinstance(shape, Cylinder):
        return f'<cylinder radius="{shape.radius():.9g}" length="{shape.length():.9g}"/>'
    if isinstance(shape, Box):
        return f'<box size="{shape.width():.9g} {shape.depth():.9g} {shape.height():.9g}"/>'
    raise NotImplementedError(f"shape {type(shape).__name__}: add it to _shape")


def _link(name, collision, visual, out):
    """A URDF link: collision [(shape, pose in the link)], Drake's proximity shapes; visual [(shape, pose, rgba)],
    Drake's illustration shapes (the Franka meshes, the cap's and obstacles' primitives)."""
    lines = [f'  <link name={quoteattr(name)}>',
             '    <inertial><mass value="0.1"/><inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/></inertial>']
    for shape, X in collision:
        lines.append(f'    <collision>{_origin(X)}<geometry>{_shape(shape)}</geometry></collision>')
    for k, (shape, X, rgba) in enumerate(visual):
        if isinstance(shape, Mesh) and shape.extension() == ".gltf":
            X = X @ Y_UP_TO_Z_UP     # glTF is y-up; Drake turns it z-up when it loads it, a URDF viewer does not
        color = "" if rgba is None else (f'<material name="{name}_{k}"><color rgba="'
                                         + " ".join(f"{v:.3g}" for v in rgba.rgba) + '"/></material>')
        lines.append(f'    <visual>{_origin(X)}<geometry>{_shape(shape, out)}</geometry>{color}</visual>')
    return lines + ['  </link>']


def _geometries(plant, scene_graph, body, role=Role.kProximity):
    """[(geometry id, shape, pose in the body, rgba or None)] of a body, for a role."""
    insp = scene_graph.model_inspector()
    fid = plant.GetBodyFrameIdOrThrow(body.index())
    out = []
    for g in insp.GetGeometries(fid, role):
        props = insp.GetIllustrationProperties(g) if role == Role.kIllustration else None
        rgba = props.GetPropertyOrDefault("phong", "diffuse", None) if props is not None else None
        out.append((g, insp.GetShape(g), insp.GetPoseInFrame(g), rgba))
    return out


def _proximity_geometries(plant, scene_graph, body):
    return [(g, s, X) for g, s, X, _ in _geometries(plant, scene_graph, body)]


def write_model(path, name, plant, scene_graph, bodies, context, move_geometry=None):
    """URDF of a tree of Drake bodies: one link per body (its shapes), one joint per Drake joint
    between two of them, with Drake's limits; welds become fixed joints. Each movable joint's child frame is
    its child body's frame (true for every URDF/SDF joint here), so an HPP link frame = the Drake body frame.
    move_geometry {from body: to body}: write a body's shapes on another link instead (pose taken at context)."""
    names = {b.index() for b in bodies}
    move_geometry = move_geometry or {}
    collision, visual = {b.name(): [] for b in bodies}, {b.name(): [] for b in bodies}
    for b in bodies:
        to = plant.GetBodyByName(move_geometry.get(b.name(), b.name()), b.model_instance())
        X_TB = plant.CalcRelativeTransform(context, to.body_frame(), b.body_frame())
        collision[to.name()] += [(s, X_TB @ X) for _, s, X in _proximity_geometries(plant, scene_graph, b)]
        visual[to.name()] += [(s, X_TB @ X, c) for _, s, X, c in _geometries(plant, scene_graph, b, Role.kIllustration)]
    lines = ['<?xml version="1.0"?>', f'<robot name={quoteattr(name)}>']
    for b in bodies:
        lines += _link(b.name(), collision[b.name()], visual[b.name()], Path(path).parent)
    for i in plant.GetJointIndices():
        j = plant.get_joint(i)
        P, C = j.parent_body(), j.child_body()
        if P.index() not in names or C.index() not in names:
            continue
        head = (f'  <joint name={quoteattr(j.name())} type="%s">'
                f'<parent link={quoteattr(P.name())}/><child link={quoteattr(C.name())}/>')
        if isinstance(j, WeldJoint):
            X_PC = plant.CalcRelativeTransform(context, P.body_frame(), C.body_frame())
            lines.append(head % "fixed" + _origin(X_PC) + '</joint>')
            continue
        assert j.frame_on_child().GetFixedPoseInBodyFrame().IsExactlyIdentity(), j.name()
        if isinstance(j, RevoluteJoint):
            kind, axis = "revolute", j.revolute_axis()
        elif isinstance(j, PrismaticJoint):
            kind, axis = "prismatic", j.translation_axis()
        else:
            raise NotImplementedError(type(j).__name__)
        lo, hi = j.position_lower_limits()[0], j.position_upper_limits()[0]
        lines.append(head % kind + _origin(j.frame_on_parent().GetFixedPoseInBodyFrame())
                     + f'<axis xyz="{" ".join(f"{v:.9g}" for v in axis)}"/>'
                     + f'<limit lower="{lo:.9g}" upper="{hi:.9g}" effort="100" velocity="2"/></joint>')
    Path(path).write_text("\n".join(lines + ['</robot>']) + "\n")


def filtered_link_pairs(plant, scene_graph, bodies):
    """Pairs of bodies whose every pair of geometries Drake's collision filter excludes."""
    insp = scene_graph.model_inspector()
    geoms = {b.index(): [g for g, _, _ in _proximity_geometries(plant, scene_graph, b)] for b in bodies}
    pairs = []
    for i, a in enumerate(bodies):
        for b in bodies[i + 1:]:
            filt = [insp.CollisionFiltered(x, y) for x in geoms[a.index()] for y in geoms[b.index()]]
            if filt and all(filt):
                pairs.append((a.name(), b.name()))
            elif any(filt):
                raise RuntimeError(f"{a.name()}/{b.name()}: only some geometry pairs are filtered")
    return pairs


def write_srdf(path, name, disabled, extra=()):
    lines = ['<?xml version="1.0"?>', f'<robot name={quoteattr(name)}>']
    lines += [f'  <disable_collisions link1={quoteattr(a)} link2={quoteattr(b)} reason="Drake filter"/>'
              for a, b in disabled]
    Path(path).write_text("\n".join(lines + list(extra) + ['</robot>']) + "\n")


def hpp_joint_names(scene):
    """HPP name of the joint of each Drake position: panda/<joint> or cap<k>/<joint>."""
    plant, models = scene["plant"], scene["models"]
    names = [None] * plant.num_positions()
    for i in plant.GetJointIndices():
        j = plant.get_joint(i)
        if j.num_positions() != 1:
            continue
        if j.model_instance() in (models["arm"], models["hand"]):
            names[j.position_start()] = f"panda/{j.name()}"
        else:
            names[j.position_start()] = f"cap{models['caps'].index(j.model_instance())}/{j.name()}"
    return names


# ---------------------------------------------------------------- step 1: the robot (tutorial 2)
def export_robot(scene, out, meta):
    plant, sg, models = scene["plant"], scene["scene_graph"], scene["models"]
    bodies = [plant.get_body(b) for m in (models["arm"], models["hand"]) for b in plant.GetBodyIndices(m)]
    write_model(out / "panda.urdf", "panda", plant, sg, bodies, plant.CreateDefaultContext())
    gripper = ['  <gripper name="gripper" clearance="0">',          # the panda_hand frame: your h(q)'s frame
               '    <position xyz="0 0 0" wxyz="1 0 0 0"/>',
               '    <link name="panda_hand"/>', '  </gripper>']
    write_srdf(out / "panda.srdf", "panda", filtered_link_pairs(plant, sg, bodies), gripper)
    meta["hpp_joint_names"] = hpp_joint_names(scene)


def export_consistency(scene, out, meta):
    """Drake's reference for the HPP checkpoints: random configurations, the hand pose, the collision verdict."""
    plant, models = scene["plant"], scene["models"]
    checker = DrakeChecker(scene)
    E = plant.GetFrameByName("panda_hand", models["hand"])
    Q = np.random.default_rng(0).uniform(plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits(),
                                         size=(500, plant.num_positions()))
    X_WE, colliding = [], []
    for q in Q:
        colliding.append(checker.in_collision(q))
        plant.SetPositions(checker.plant_context, q)
        X_WE.append(plant.CalcRelativeTransform(checker.plant_context, plant.world_frame(), E).GetAsMatrix4())
    np.savez(out / "consistency.npz", q=Q, X_WE=np.array(X_WE), colliding=np.array(colliding))
    print(f"  {len(Q)} reference configurations, {sum(colliding)} in collision (Drake)")


# ---------------------------------------------------------------- step 2: the obstacles (tutorial 2)
def export_env(scene, out, meta):
    """Every proximity geometry not on the robot or a cap, as one link fixed to the world (like tutorial 2's ground)."""
    plant, sg, models = scene["plant"], scene["scene_graph"], scene["models"]
    context = plant.CreateDefaultContext()
    taken = {b for m in (models["arm"], models["hand"], *models["caps"]) for b in plant.GetBodyIndices(m)}
    geoms, visual = [], []
    for i in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(i))
        if body.index() in taken:
            continue
        X_WB = plant.EvalBodyPoseInWorld(context, body)
        geoms += [(s, X_WB @ X) for _, s, X in _proximity_geometries(plant, sg, body)]
        visual += [(s, X_WB @ X, c) for _, s, X, c in _geometries(plant, sg, body, Role.kIllustration)]
    (out / "env.urdf").write_text("\n".join(['<?xml version="1.0"?>', '<robot name="env">']
                                            + _link("env_link", geoms, visual, out) + ['</robot>']) + "\n")
    write_srdf(out / "env.srdf", "env", [])
    print(f"  {len(geoms)} obstacle shapes")


# ---------------------------------------------------------------- step 3: the cap (tutorial 2, "Adding an object")
def export_cap(scene, out, meta):
    """cap.urdf/.srdf, loaded once per cap by hpp_plan.py at the poses in scene.json. No contact surface: a cap
    is never placed (the constraint graph locks its joint outside a grasp instead).
    The base's cylinder is written on cap_link: it is coaxial with the cap joint, so it occupies the same space at
    every cap angle, and the security margins of hpp_plan.py (on the joint cap<k>/cap_to_base) then cover the
    whole cap. On base_link (fixed to the world) no joint margin could reach it, and the fingers graze its rim on
    the last millimetres of the approach."""
    plant, sg, models = scene["plant"], scene["scene_graph"], scene["models"]
    context = plant.CreateDefaultContext()
    cap = models["caps"][0]
    bodies = [plant.GetBodyByName("base_link", cap), plant.GetBodyByName("cap_link", cap)]
    write_model(out / "cap.urdf", "cap", plant, sg, bodies, context, move_geometry={"base_link": "cap_link"})
    handle = [f'  <handle name="handle" clearance="{HANDLE_CLEARANCE}" approaching_direction="0 0 1">',
              f'    <position xyz="0 0 {GRASP_HEIGHT:.9g}" wxyz="0 0 1 0"/>',   # z pointing down, as tutorial 2's box
              '    <link name="cap_link"/>', '  </handle>']
    write_srdf(out / "cap.srdf", "cap", filtered_link_pairs(plant, sg, bodies), handle)
    poses = [plant.EvalBodyPoseInWorld(context, plant.GetBodyByName("base_link", c)) for c in models["caps"]]
    meta["caps"] = [dict(prefix=f"cap{k}", position=X.translation().tolist(), rotation=X.rotation().matrix().tolist())
                    for k, X in enumerate(poses)]
    meta["idx_caps"] = [plant.GetJointByName("cap_to_base", c).position_start() for c in models["caps"]]
    meta["idx_wrist"] = plant.GetJointByName("panda_joint7", models["arm"]).position_start()


# ---------------------------------------------------------------- step 5: the queries
def export_queries(scene, out, meta):
    """1cap_2obs: Run A's start and goal, the cap turned from 3 to 3 - D. 8caps_3obs: cap_scaling's queries. The fingers are
    set to the grasp width, at which the constraint graph holds them."""
    plant, models = scene["plant"], scene["models"]
    if meta["scene"] == "1cap_2obs":
        sc = np.load(RUN_A)
        i_cap = int(sc["idx_cap"])
        queries = []
        for turn in RUN_A_TURNS:
            q_goal = sc["q_goal_base"].copy()
            q_goal[i_cap] = sc["q_init"][i_cap] - turn
            queries.append(dict(name=f"D{turn:g}".replace(".", "p"), turn=turn, active_caps=[0],
                                q_init=sc["q_init"].copy(), q_goal=q_goal))
    else:
        z = np.load(REPO_ROOT / meta["queries_file"])
        queries = [dict(name=f"K{int(k)}_q{int(n):02d}", k=int(k), active_caps=np.flatnonzero(m).tolist(),
                        turns=t.tolist(), q_init=qi.copy(), q_goal=qg.copy())
                   for k, n, m, t, qi, qg in zip(z["k"], z["query"], z["active_mask"], z["turns"], z["q_init"], z["q_goal"])]
    fingers = {plant.GetJointByName(n, models["hand"]).position_start(): v
               for n, v in (("panda_finger_joint1", -FINGER_WIDTH), ("panda_finger_joint2", FINGER_WIDTH))}
    checker = DrakeChecker(scene)
    for qr in queries:
        for key in ("q_init", "q_goal"):
            for i, v in fingers.items():
                qr[key][i] = v
            qr[key + "_in_collision"] = checker.in_collision(qr[key])
            qr[key] = qr[key].tolist()
    (out / "queries.json").write_text(json.dumps(queries, indent=1))
    print(f"  {len(queries)} queries; start or goal in collision (Drake): "
          f"{sum(q['q_init_in_collision'] or q['q_goal_in_collision'] for q in queries)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scene", choices=("1cap_2obs", "8caps_3obs"), default="1cap_2obs")
    ap.add_argument("--out", help="default: artifacts/hpp_comparison/<scene>")
    ap.add_argument("--queries", help="8caps_3obs: the queries.npz written by cap_scaling.py --save-queries")
    a = ap.parse_args()
    out = Path(a.out).resolve() if a.out else REPO_ROOT / "artifacts" / "hpp_comparison" / a.scene
    out.mkdir(parents=True, exist_ok=True)
    scene, meta = build_scene(a.scene), dict(scene=a.scene, queries_file=a.queries)
    for f in (export_robot, export_consistency, export_env, export_cap, export_queries):
        print(f.__name__)
        f(scene, out, meta)
    (out / "scene.json").write_text(json.dumps(meta, indent=1))
    print(f"wrote {out}")
