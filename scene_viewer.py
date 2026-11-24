"""Local web page for the scene generator in multicap_scene.py.

Run:  python scene_viewer.py   then open the printed link (forward the two ports if you work over SSH).
The page changes random_scene's settings, generates scenes, lets you set any object to a specific value, and shows
the result in Meshcat. multicap_scene.py is not modified: its settings are set in this process for one call at a time.
"""
import argparse
import contextlib
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import numpy as np
from pydrake.all import (Box, CollisionFilterDeclaration, GeometrySet, Meshcat, MeshcatParams, MeshcatVisualizer,
                         Parser, RigidTransform, RobotDiagramBuilder, Rgba, Role, Sphere)

import multicap_scene as sc

PAGE = Path(__file__).resolve().parent / "scene_viewer.html"
HOME = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]  # arm pose shown in the viewer

# The settings of random_scene the page can change (names of constants in multicap_scene.py), with a help text.
SETTINGS = {
    "CAP_ANGLE_RANGE": "where caps go around the robot, from +x (deg)",
    "CAP_RADIUS_RANGE": "cap distance from the robot base (m)",
    "CAP_Z_RANGE": "cap height (m)",
    "MIN_CAP_GAP": "smallest angle between two caps (deg)",
    "SMALL_OBS_RING": "small obstacle distance from its cap centre (m)",
    "MIN_SMALL_OBS_GAP": "smallest angle between two small obstacles of one cap (deg)",
    "WALL_SIZE": "wall start and end distance from the base, and thickness (m)",
    "WALL_HEIGHT_RANGE": "wall height (m)",
    "WALL_CAP_DIST": "wall centre plane to its cap centre (m)",
    "BIG_OBS_RADIUS_RANGE": "big obstacle radius (m)",
    "BIG_OBS_HEIGHT_RANGE": "big obstacle height (m)",
    "BIG_OBS_DIST_RANGE": "big obstacle distance from the robot base (m)",
    "BIG_OBS_CAP_DIST": "big obstacle edge to any cap centre, at least (m)",
    "MARGIN": "free space kept between objects (m)",
    "MAX_TRIES": "attempts per object before giving up",
}
DEFAULTS = {name: getattr(sc, name) for name in SETTINGS}  # the values written in multicap_scene.py


@contextlib.contextmanager
def settings(values):
    """Use the page's settings for one random_scene call, then restore the values from multicap_scene.py."""
    for name, v in values.items():
        if name in SETTINGS:
            setattr(sc, name, tuple(v) if isinstance(v, list) else v)
    try:
        yield
    finally:
        for name, v in DEFAULTS.items():
            setattr(sc, name, v)


def to_model(scene):
    """Scene dict -> the page's object table, in the units the page edits: caps by angle, distance and height;
    small obstacles and walls relative to their nearest cap; big obstacles by angle and distance from the base."""
    caps = np.array(scene["caps"], dtype=float).reshape(-1, 3)
    cap_angle = np.rad2deg(np.arctan2(caps[:, 1], caps[:, 0]))
    cap_dist = np.hypot(caps[:, 0], caps[:, 1])
    model = dict(caps=[dict(angle=a, radius=r, z=z) for a, r, z in zip(cap_angle, cap_dist, caps[:, 2])],
                 small_obs=[], walls=[], big_obs=[], floor=bool(scene["floor"]))
    for x, y, z in scene["small_obs"]:
        k = int(np.argmin(np.hypot(caps[:, 0] - x, caps[:, 1] - y)))
        model["small_obs"].append(dict(cap=k, angle=np.rad2deg(np.arctan2(y - caps[k, 1], x - caps[k, 0])),
                                       dist=np.hypot(x - caps[k, 0], y - caps[k, 1]), dz=z - caps[k, 2]))
    for w in scene["walls"]:
        wall = dict(cap=None, side=1, dist=0.0, angle=w["angle_deg"], height=w["height"], r_in=w["r_in"],
                    r_out=w["r_out"], thickness=w["thickness"])
        delta = (w["angle_deg"] - cap_angle + 180) % 360 - 180   # wall angle minus cap angle (deg)
        dist = cap_dist * np.abs(np.sin(np.deg2rad(delta)))      # cap centre to the wall's centre plane
        if (np.abs(delta) < 90).any():                            # caps on the wall's side of the base
            k = int(np.argmin(np.where(np.abs(delta) < 90, dist, np.inf)))
            wall.update(cap=k, side=1 if delta[k] >= 0 else -1, dist=dist[k])
        model["walls"].append(wall)
    for o in scene["big_obs"]:
        x, y, _ = o["pos"]
        model["big_obs"].append(dict(angle=np.rad2deg(np.arctan2(y, x)), dist=np.hypot(x, y),
                                     radius=o["radius"], height=o["height"]))
    return model


def to_scene(model):
    """The page's object table -> scene dict (the inverse of to_model). A wall tied to a cap is placed with the
    same formula as random_scene: cap angle + side * asin(distance / cap distance from the base)."""
    caps = [[c["radius"] * np.cos(np.deg2rad(c["angle"])), c["radius"] * np.sin(np.deg2rad(c["angle"])), c["z"]]
            for c in model["caps"]]
    small_obs = []
    for o in model["small_obs"]:
        cx, cy, cz = caps[o["cap"]]
        a = np.deg2rad(o["angle"])
        small_obs.append([cx + o["dist"] * np.cos(a), cy + o["dist"] * np.sin(a), cz + o.get("dz", 0.01)])
    walls = []
    for w in model["walls"]:
        angle = w["angle"]
        if w["cap"] is not None:
            c = model["caps"][w["cap"]]
            if abs(w["dist"]) > c["radius"]:
                raise ValueError(f"a wall cannot be {w['dist']} m from a cap that is {c['radius']} m from the base")
            angle = c["angle"] + w["side"] * np.rad2deg(np.arcsin(w["dist"] / c["radius"]))
        walls.append(dict(angle_deg=angle, r_in=w["r_in"], r_out=w["r_out"], height=w["height"],
                          thickness=w["thickness"]))
    big_obs = [dict(pos=[o["dist"] * np.cos(np.deg2rad(o["angle"])), o["dist"] * np.sin(np.deg2rad(o["angle"])),
                         o["height"] / 2], radius=o["radius"], height=o["height"]) for o in model["big_obs"]]
    return dict(caps=caps, small_obs=small_obs, big_obs=big_obs, walls=walls, floor=bool(model["floor"]))


def render(scene):
    """Show the scene in Meshcat (caps jointed, as the planner uses them; arm at HOME). Returns the pairs of
    objects that overlap. Drake skips pairs of bodies welded to the world, so every pair is allowed first."""
    meshcat.Delete()
    builder = RobotDiagramBuilder(time_step=0.0)
    plant, scene_graph = builder.plant(), builder.scene_graph()
    parser = Parser(plant, scene_graph)
    parser.SetAutoRenaming(True)
    arm, hand, caps = sc.add_scene(parser, plant, scene)
    plant.Finalize()
    MeshcatVisualizer.AddToBuilder(builder.builder(), scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant.SetPositions(plant.GetMyMutableContextFromRoot(context), arm, HOME)
    diagram.ForcedPublish(context)

    # Name of each scene model, by model instance index (add_scene adds them in this order after the hand).
    names = ([f"cap {i}" for i in range(1, len(scene["caps"]) + 1)]
             + [f"small obstacle {i}" for i in range(1, len(scene["small_obs"]) + 1)]
             + [f"big obstacle {i}" for i in range(1, len(scene["big_obs"]) + 1)]
             + [f"wall {i}" for i in range(1, len(scene["walls"]) + 1)] + ["floor"] * bool(scene["floor"]))
    names = {int(hand) + 1 + i: name for i, name in enumerate(names)}
    sg_context = scene_graph.GetMyMutableContextFromRoot(context)
    inspector = scene_graph.model_inspector()
    ids = inspector.GetAllGeometryIds(Role.kProximity)
    scene_graph.collision_filter_manager(sg_context).Apply(CollisionFilterDeclaration().AllowWithin(GeometrySet(ids)))
    found = set()
    for pair in scene_graph.get_query_output_port().Eval(sg_context).ComputePointPairPenetration():
        a, b = [plant.GetBodyFromFrameId(inspector.GetFrameId(g)) for g in (pair.id_A, pair.id_B)]
        a, b = ["robot base" if x.name() == "panda_link0" else names.get(int(x.model_instance())) for x in (a, b)]
        # Left out: the arm's moving links (its pose is arbitrary), and the base's collision spheres reaching 1 cm
        # into the floor (both are fixed to the world, so the planner never checks that pair).
        if a and b and a != b and {a, b} != {"robot base", "floor"} and pair.depth > 1e-9:
            found.add(tuple(sorted((a, b))))
    return [f"{a} overlaps {b}" for a, b in sorted(found)]


def range_notes(model, values):
    """Values outside random_scene's settings. Notes only: a specific value may be outside on purpose."""
    v = {**DEFAULTS, **values}
    checks = []   # (object, what, value, setting)
    for i, c in enumerate(model["caps"], 1):
        checks += [(f"cap {i}", "angle", c["angle"], "CAP_ANGLE_RANGE"), (f"cap {i}", "distance", c["radius"],
                   "CAP_RADIUS_RANGE"), (f"cap {i}", "height", c["z"], "CAP_Z_RANGE")]
    for i, o in enumerate(model["small_obs"], 1):
        checks.append((f"small obstacle {i}", "distance from cap", o["dist"], "SMALL_OBS_RING"))
    for i, w in enumerate(model["walls"], 1):
        checks.append((f"wall {i}", "height", w["height"], "WALL_HEIGHT_RANGE"))
        if w["cap"] is not None:
            checks.append((f"wall {i}", "distance to cap", w["dist"], "WALL_CAP_DIST"))
    for i, o in enumerate(model["big_obs"], 1):
        checks += [(f"big obstacle {i}", "radius", o["radius"], "BIG_OBS_RADIUS_RANGE"), (f"big obstacle {i}",
                   "height", o["height"], "BIG_OBS_HEIGHT_RANGE"), (f"big obstacle {i}", "distance from base",
                   o["dist"], "BIG_OBS_DIST_RANGE")]
    notes = [f"{obj}: {what} {x:.3f} is outside {name} {list(v[name])}" for obj, what, x, name in checks
             if not v[name][0] - 1e-9 <= x <= v[name][1] + 1e-9]

    gap = lambda a, b: abs((a - b + 180) % 360 - 180)
    angles = [c["angle"] for c in model["caps"]]
    notes += [f"caps {i + 1} and {j + 1}: {gap(a, b):.1f} deg apart, less than MIN_CAP_GAP {v['MIN_CAP_GAP']}"
              for i, a in enumerate(angles) for j, b in enumerate(angles) if j > i and gap(a, b) < v["MIN_CAP_GAP"]]
    for k in range(len(model["caps"])):
        mine = [(i + 1, o["angle"]) for i, o in enumerate(model["small_obs"]) if o["cap"] == k]
        if len(mine) > 4:
            notes.append(f"cap {k + 1}: {len(mine)} small obstacles, random_scene allows at most 4")
        notes += [f"small obstacles {i} and {j}: {gap(a, b):.1f} deg apart, less than MIN_SMALL_OBS_GAP "
                  f"{v['MIN_SMALL_OBS_GAP']}" for n, (i, a) in enumerate(mine) for j, b in mine[n + 1:]
                  if gap(a, b) < v["MIN_SMALL_OBS_GAP"]]
    caps_xy = [(c["radius"] * np.cos(np.deg2rad(c["angle"])), c["radius"] * np.sin(np.deg2rad(c["angle"])))
               for c in model["caps"]]
    for i, o in enumerate(model["big_obs"], 1):
        x, y = o["dist"] * np.cos(np.deg2rad(o["angle"])), o["dist"] * np.sin(np.deg2rad(o["angle"]))
        notes += [f"big obstacle {i}: edge {np.hypot(x - cx, y - cy) - o['radius']:.3f} m from cap {k}, "
                  f"less than BIG_OBS_CAP_DIST {v['BIG_OBS_CAP_DIST']}" for k, (cx, cy) in enumerate(caps_xy, 1)
                  if np.hypot(x - cx, y - cy) < o["radius"] + v["BIG_OBS_CAP_DIST"]]
    return notes


def highlight(scene, kind, i):
    """Draw a yellow marker on one object, so a table row can be found in the 3D view."""
    meshcat.Delete("/highlight")
    yellow = Rgba(1.0, 0.85, 0.0, 0.5)
    if kind == "walls":
        w = scene["walls"][i]
        _, pose = sc.make_wall(w)
        meshcat.SetObject("/highlight", Box(w["r_out"] - w["r_in"] + 0.02, w["thickness"] + 0.02,
                                            w["height"] + 0.02), yellow)
    elif kind == "big_obs":
        o = scene["big_obs"][i]
        pose = RigidTransform(o["pos"])
        meshcat.SetObject("/highlight", Sphere(max(o["radius"], o["height"] / 2) + 0.02), yellow)
    else:
        pose = RigidTransform(scene[kind][i])
        meshcat.SetObject("/highlight", Sphere(0.045 if kind == "caps" else 0.015), yellow)
    meshcat.SetTransform("/highlight", pose)
    return {}


def show(scene, values):
    """Render a scene and return everything the page displays."""
    overlaps = render(scene)
    model = to_model(scene)
    return dict(scene=scene, model=model, overlaps=overlaps, notes=range_notes(model, values))


def generate(body):
    with settings(body["settings"]):
        scene = sc.random_scene(body["n_caps"], body["n_small_obs"], body["n_big_obs"], body["n_walls"],
                                body["seed"], body["small_obs_angles_deg"])
    return show(scene, body["settings"])


def apply(body):
    if "preset" in body:
        scene = json.loads(json.dumps(sc.SCENES[body["preset"]], default=float))
    elif "model" in body:
        scene = to_scene(body["model"])
    else:
        scene = body["scene"]
    return show(scene, body.get("settings", {}))


ROUTES = {"/api/generate": generate, "/api/apply": apply,
          "/api/highlight": lambda body: highlight(body["scene"], body["kind"], body["index"])}


class Handler(BaseHTTPRequestHandler):
    def send(self, text, content_type="application/json"):
        data = text.encode()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self.send(PAGE.read_text(), "text/html; charset=utf-8")
        elif self.path == "/api/config":
            port = int(meshcat.web_url().rsplit(":", 1)[1])
            self.send(json.dumps(dict(meshcat_port=port, presets=list(sc.SCENES),
                                      settings={n: dict(value=DEFAULTS[n], help=h) for n, h in SETTINGS.items()})))
        else:
            self.send_error(404)

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        try:
            reply = ROUTES[self.path](body)
        except Exception as e:  # shown on the page instead of stopping the server
            reply = {"error": f"{type(e).__name__}: {e}"}
        self.send(json.dumps(reply, default=float))

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8050, help="port of the page")
    ap.add_argument("--meshcat-port", type=int, default=8051, help="port of the 3D view (not 7000: macOS AirPlay uses it)")
    args = ap.parse_args()
    meshcat = Meshcat(MeshcatParams(port=args.meshcat_port))
    print(f"Scene viewer: http://localhost:{args.port}   3D view: {meshcat.web_url()}   (forward both ports)", flush=True)
    HTTPServer(("127.0.0.1", args.port), Handler).serve_forever()
