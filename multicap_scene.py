"""Planning scenes. A scene is a dict (see SCENES). add_scene puts the Panda and a scene into a plant;
random_scene makes new scene dicts. Used by the planner notebooks and by experiments/cap_scaling/generate_cfree.py."""
from pathlib import Path

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw

SDFS = Path(__file__).resolve().parent / "my_sdfs"
CAP_SDF = SDFS / "bottle_cap.sdf"                 # revolute cap joint
CAP_SDF_WELDED = SDFS / "bottle_cap_welded.sdf"   # same geometry, fixed joint
SMALL_OBS_SDF = SDFS / "obstacle.sdf"             # small cylinder
FLOOR_SDF = SDFS / "floor.sdf"

# A scene: 
#   caps and small_obs are [x, y, z] positions; 
#   big_obs are dict(pos, radius, height);
#   walls are dict(angle_deg, r_in, r_out, height, thickness), radial from the robot base; 
#   floor is True/False.
SCENES = {
    "1cap_2obs": dict(
        caps=[[0.5, 0.0, 0.0]],
        small_obs=[[0.51, 0.031, 0.01], [0.467, -0.005, 0.01]],
        big_obs=[], walls=[], floor=True),
    "2caps_0obs": dict(
        caps=[[0.5, 0.0, 0.0], [0.5, 0.25, 0.0]],
        small_obs=[], big_obs=[], walls=[], floor=True),
    "8caps_3obs": dict(
        caps=[[0.5 * np.cos(np.deg2rad(a)), 0.5 * np.sin(np.deg2rad(a)), 0.0]
              for a in [-120, -85, -50, -15, 15, 50, 85, 120]],
        small_obs=[], big_obs=[],
        walls=[dict(angle_deg=0, r_in=0.30, r_out=0.75, height=0.35, thickness=0.04),
               dict(angle_deg=102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04),
               dict(angle_deg=-102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04)],
        floor=True),
}

# Settings of random_scene (metres and degrees).
CAP_ANGLE_RANGE = (-120, 120)    # where caps go around the robot, measured from +x
CAP_RADIUS_RANGE = (0.45, 0.55)  # distance of a cap from the robot base
CAP_Z_RANGE = (0.0, 0.15)        # cap height: not below the floor, at most 15 cm up
MIN_CAP_GAP = 25                 # smallest angle between two caps, so none sits above another
SMALL_OBS_RING = (0.031, 0.036)  # distance from the cap centre (0.029 would touch the cap)
MIN_SMALL_OBS_GAP = 45           # smallest angle between two small obstacles of one cap
WALL_SIZE = dict(r_in=0.30, r_out=0.75, thickness=0.04)
WALL_HEIGHT_RANGE = (0.35, 0.50)
WALL_CAP_DIST = (0.10, 0.125)    # wall to cap centre: restrains the wrist range of a 1-cap grasp without blocking it
BIG_OBS_RADIUS_RANGE = (0.04, 0.06)
BIG_OBS_HEIGHT_RANGE = (0.20, 0.40)
BIG_OBS_DIST_RANGE = (0.35, 0.75)  # distance of a big obstacle from the robot base
BIG_OBS_CAP_DIST = 0.20          # big obstacle to cap: free space between their edges
MARGIN = 0.01                    # free space kept between objects
MAX_TRIES = 1000


def add_panda(parser, plant):
    """The Panda arm welded at the origin, its hand welded to panda_link8. Returns (arm, hand)."""
    arm = parser.AddModels(file_name=str(SDFS / "panda_arm.urdf"))[0] # we use this and not drake's to limit the joint range of the wrist
    hand = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0", arm), RigidTransform()) # weld arm to world
    plant.WeldFrames(plant.GetFrameByName("panda_link8", arm), plant.GetFrameByName("panda_hand", hand), # weld hand to arm
                     RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0]))
    plant.GetJointByName("panda_finger_joint1", hand).set_default_translation(-0.0245)
    plant.GetJointByName("panda_finger_joint2", hand).set_default_translation(0.0245)
    return arm, hand


def add_caps(parser, plant, poses, welded=False):
    """One bottle cap per pose, its base welded to the world. Returns the model instances.
    
    welded=True removes the cap revolute joint, so that they can be ignored during c-free polytope generation.
    """
    caps = []
    for p in poses:
        cap = parser.AddModels(file_name=str(CAP_SDF_WELDED if welded else CAP_SDF))[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link", cap), RigidTransform(p))
        caps.append(cap)
    return caps


def make_wall(wall):
    """SDF text and world pose of one radial wall (a box)."""
    a = np.deg2rad(wall["angle_deg"]) # angle of the wall's centre line, measured from +x
    r_mid = 0.5 * (wall["r_in"] + wall["r_out"]) # distance of the wall's centre line from the robot base
    size = f"{wall['r_out'] - wall['r_in']} {wall['thickness']} {wall['height']}" # depth, thickness, height
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="wall">
    <link name="wall_link">
      <visual name="wall_visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material><diffuse>0.55 0.45 0.35 0.6</diffuse></material>
      </visual>
      <collision name="wall_collision"><geometry><box><size>{size}</size></box></geometry></collision>
    </link>
  </model>
</sdf>"""
    pose = RigidTransform(RollPitchYaw(0.0, 0.0, a), [r_mid * np.cos(a), r_mid * np.sin(a), wall["height"] / 2])
    return sdf, pose


def make_big_obs(radius, height):
    """SDF text of one big cylinder obstacle, centred on its link origin."""
    shape = f"<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>"
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="big_obs">
    <link name="big_obs_link">
      <visual name="big_obs_visual">
        <geometry>{shape}</geometry>
        <material><diffuse>0.8 0.2 0.2 1.0</diffuse></material>
      </visual>
      <collision name="big_obs_collision"><geometry>{shape}</geometry></collision>
    </link>
  </model>
</sdf>"""


def add_obstacles(parser, plant, small_obs, big_obs, walls, floor=True):
    """Weld every obstacle to the world: small cylinders, big cylinders, walls and the floor, each from an SDF."""
    models = [(SMALL_OBS_SDF.read_text(), "obstacle_link", RigidTransform(p)) for p in small_obs]
    models += [(make_big_obs(o["radius"], o["height"]), "big_obs_link", RigidTransform(o["pos"])) for o in big_obs]
    models += [(sdf, "wall_link", pose) for sdf, pose in map(make_wall, walls)]
    if floor:
        models.append((FLOOR_SDF.read_text(), "floor_link", RigidTransform()))
    for sdf, link, pose in models:
        model = parser.AddModelsFromString(sdf, "sdf")[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(link, model), pose)


def add_scene(parser, plant, scene, welded=False):
    """Add the Panda and a scene (a name in SCENES, or a scene dict) to plant, before Finalize. The parser needs
    SetAutoRenaming(True). welded=True welds the caps (their angles are no DOF). Returns (arm, hand, caps)."""
    s = SCENES[scene] if isinstance(scene, str) else scene
    arm, hand = add_panda(parser, plant)
    caps = add_caps(parser, plant, s["caps"], welded)
    add_obstacles(parser, plant, s["small_obs"], s["big_obs"], s["walls"], s["floor"])
    return arm, hand, caps


def random_scene(n_caps, n_small_obs=0, n_big_obs=0, n_walls=0, seed=0, small_obs_angles_deg=None):
    """A random scene dict, ready for add_scene. 
    
        Ranges and margins are the constants above.
        n_small_obs: small obstacles per cap (0 to 4), an int or one count per cap.
        small_obs_angles_deg: optional preset angles around each cap, one list per cap (replaces n_small_obs).
        Big obstacles and walls keep MARGIN free space from everything placed before them (caps, small obstacles,
    each other); 
        each wall stands next to one cap, at WALL_CAP_DIST from it, to restrain that cap's wrist range,
    so there is room for at most about 2 walls per cap."""
    rng = np.random.default_rng(seed)
    discs = []  # (x, y, radius) footprint of everything placed
    clear = lambda x, y, r: all(np.hypot(x - a, y - b) - r - c >= MARGIN for a, b, c in discs) # True if (x, y, r) is clear of all discs
    gap = lambda a, b: abs((a - b + 180) % 360 - 180)  # angle between two directions, in degrees

    # Caps: random angles around the robot, at least MIN_CAP_GAP apart. The free angle left after the gaps is
    # split at sorted random points, so the spacing is exact and uniform.
    free = CAP_ANGLE_RANGE[1] - CAP_ANGLE_RANGE[0] - (n_caps - 1) * MIN_CAP_GAP
    if free < 0:
        raise ValueError(f"{n_caps} caps do not fit {MIN_CAP_GAP} deg apart in {CAP_ANGLE_RANGE}")
    cap_angles = CAP_ANGLE_RANGE[0] + np.sort(rng.uniform(0, free, n_caps)) + MIN_CAP_GAP * np.arange(n_caps)
    cap_radii = rng.uniform(*CAP_RADIUS_RANGE, n_caps)
    cap_z = rng.uniform(*CAP_Z_RANGE, n_caps)
    caps = [[r * np.cos(np.deg2rad(a)), r * np.sin(np.deg2rad(a)), z] for a, r, z in zip(cap_angles, cap_radii, cap_z)]
    discs += [(x, y, 0.025) for x, y, _ in caps]

    # Small obstacles: in a ring around each cap, at least MIN_SMALL_OBS_GAP apart. The ring and the cap
    # spacing keep them clear of the caps and of each other, so no check against the discs is needed.
    counts = [n_small_obs] * n_caps if isinstance(n_small_obs, int) else list(n_small_obs)
    small_obs = []
    for k, (cx, cy, cz) in enumerate(caps):
        obs_angles = [] if small_obs_angles_deg is None else list(small_obs_angles_deg[k])
        for _ in range(counts[k] if small_obs_angles_deg is None else 0):
            for _ in range(MAX_TRIES):
                a = rng.uniform(0, 360)
                if all(gap(a, b) >= MIN_SMALL_OBS_GAP for b in obs_angles):
                    obs_angles.append(a)
                    break
            else:
                raise RuntimeError(f"No room for {counts[k]} small obstacles {MIN_SMALL_OBS_GAP} deg apart")
        assert len(obs_angles) <= 4, "at most 4 small obstacles per cap"
        assert all(gap(a, b) >= MIN_SMALL_OBS_GAP for i, a in enumerate(obs_angles) for b in obs_angles[:i]), \
            f"preset angles of cap {k} are closer than {MIN_SMALL_OBS_GAP} deg"
        for a in obs_angles:
            d = rng.uniform(*SMALL_OBS_RING)
            x, y = cx + d * np.cos(np.deg2rad(a)), cy + d * np.sin(np.deg2rad(a))
            small_obs.append([x, y, cz + 0.01])
            discs.append((x, y, 0.005))

    # Walls: next to a random cap, on a random side, at a random distance WALL_CAP_DIST from its centre.
    walls = []
    for _ in range(n_walls):
        for _ in range(MAX_TRIES):
            k = rng.integers(n_caps)
            side = rng.choice([-1, 1])
            angle = cap_angles[k] + side * np.rad2deg(np.arcsin(rng.uniform(*WALL_CAP_DIST) / cap_radii[k]))
            along = np.arange(WALL_SIZE["r_in"], WALL_SIZE["r_out"] + 1e-9, WALL_SIZE["thickness"])
            wall_discs = [(r * np.cos(np.deg2rad(angle)), r * np.sin(np.deg2rad(angle)), WALL_SIZE["thickness"] / 2)
                          for r in along]
            if all(clear(*d) for d in wall_discs):
                walls.append(dict(angle_deg=angle, height=rng.uniform(*WALL_HEIGHT_RANGE), **WALL_SIZE))
                discs += wall_discs
                break
        else:
            raise RuntimeError(f"No free place for wall {len(walls) + 1} of {n_walls} next to {n_caps} caps "
                               f"(a cap has two sides: use at most about 2 walls per cap)")

    # Big obstacles: anywhere in the distance range, away from the caps.
    big_obs = []
    for _ in range(n_big_obs):
        for _ in range(MAX_TRIES):
            radius, height = rng.uniform(*BIG_OBS_RADIUS_RANGE), rng.uniform(*BIG_OBS_HEIGHT_RANGE)
            d, a = rng.uniform(*BIG_OBS_DIST_RANGE), rng.uniform(-180, 180)
            x, y = d * np.cos(np.deg2rad(a)), d * np.sin(np.deg2rad(a))
            if clear(x, y, radius) and all(np.hypot(x - cx, y - cy) >= radius + BIG_OBS_CAP_DIST for cx, cy, _ in caps):
                big_obs.append(dict(pos=[x, y, height / 2], radius=radius, height=height))
                discs.append((x, y, radius))
                break
        else:
            raise RuntimeError("No free place for a big obstacle")

    as_floats = lambda v: [float(t) for t in v]
    return dict(caps=[as_floats(c) for c in caps], small_obs=[as_floats(o) for o in small_obs],
                big_obs=[dict(pos=as_floats(o["pos"]), radius=float(o["radius"]), height=float(o["height"]))
                         for o in big_obs],
                walls=[{k: float(v) for k, v in w.items()} for w in walls], floor=True)
