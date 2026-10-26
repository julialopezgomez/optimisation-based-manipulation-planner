"""The planning scenes: where the caps and the obstacles are, and how to add them (and the Panda) to a plant.

Every scene is built from here, by the planner notebooks (caps jointed: each cap angle is a DOF), by
experiments/cap_scaling/generate_cfree.py (caps welded: C-free is computed in the 9-D arm + finger space,
which is exact because a cap's collision geometry is a cylinder about its own joint axis) and by
experiments/hpp_comparison/export_scene.py, so they all see the same geometry.

The scenes (SCENES, add_scene), each with a floor just under the caps and the robot base:
  1cap_2obs:  one cap at (0.5, 0, 0) and two obstacles (obstacle.sdf: small cylinders) beside it.
  2caps_0obs: two caps, 0.25 m apart, no obstacles.
  8caps_3obs: 8 caps on an arc and 3 radial walls between groups of caps (add_walls adds the walls and the floor).
"""
from pathlib import Path

import numpy as np
from pydrake.all import Box, RigidTransform, RollPitchYaw, RotationMatrix

REPO_ROOT = Path(__file__).resolve().parent
CAP_SDF = REPO_ROOT / "my_sdfs" / "bottle_cap.sdf"                 # revolute cap joint
CAP_SDF_WELDED = REPO_ROOT / "my_sdfs" / "bottle_cap_welded.sdf"   # same geometry, fixed joint
OBSTACLE_SDF = REPO_ROOT / "my_sdfs" / "obstacle.sdf"             

# Caps on an arc of radius 0.5 m around the robot base, at floor height (same height as the single-cap
# scene's cap at (0.5, 0, 0)). Angles in degrees from +x; the two outermost caps are behind the base.
CAP_RADIUS = 0.5                                            # radius/distance of the caps from the robot base
CAP_ANGLES_DEG = [-120, -85, -50, -15, 15, 50, 85, 120]     # angles around the robot where caps are placed
CAP_POSES = [[CAP_RADIUS * np.cos(np.deg2rad(a)), CAP_RADIUS * np.sin(np.deg2rad(a)), 0.0] for a in CAP_ANGLES_DEG]

# Walls (boxes fixed to the world): radial walls between groups of caps, so moving the arm from one group
# to the next means going over or around a wall. Each wall starts 0.3 m from the base, which leaves the
# arm's lower links room to turn. The front wall is close to its two neighbouring caps (0.13 m), so it also
# blocks part of their wrist range.
WALLS = [
    dict(name="front_wall", angle_deg=0, r_in=0.30, r_out=0.75, height=0.35, thickness=0.04),
    dict(name="left_wall", angle_deg=102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04),
    dict(name="right_wall", angle_deg=-102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04),
]
WALL_RGBA = [0.55, 0.45, 0.35, 0.6]

# Floor: a slab whose top is just under the caps and the robot base, so no plan moves the arm below the
# surface (e.g. under a wall instead of over it).
FLOOR = dict(name="floor", top_z=-0.01, size_xy=3.0, thickness=0.1)
FLOOR_RGBA = [0.85, 0.85, 0.82, 0.35]


def wall_box(o):
    """(Box shape, pose in world) of one radial wall."""
    a = np.deg2rad(o["angle_deg"])
    r_mid = 0.5 * (o["r_in"] + o["r_out"])
    X = RigidTransform(RollPitchYaw(0.0, 0.0, a), [r_mid * np.cos(a), r_mid * np.sin(a), 0.5 * o["height"]])
    return Box(o["r_out"] - o["r_in"], o["thickness"], o["height"]), X


def add_caps(parser, plant, welded=False, poses=CAP_POSES):
    """Add one bottle cap per pose, its base welded to the world. Returns the model instances."""
    caps = []
    for p in poses:
        cap = parser.AddModels(file_name=str(CAP_SDF_WELDED if welded else CAP_SDF))[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link", cap),
                         RigidTransform(RotationMatrix(), p))
        caps.append(cap)
    return caps


def add_walls(plant, walls=WALLS, floor=FLOOR):
    """Register each wall and the floor on the world body (collision + illustration). Call before Finalize()."""
    from pydrake.all import CoulombFriction
    boxes = [(o["name"], *wall_box(o), WALL_RGBA) for o in walls]
    if floor is not None:
        boxes.append((floor["name"], Box(floor["size_xy"], floor["size_xy"], floor["thickness"]),
                      RigidTransform([0.0, 0.0, floor["top_z"] - 0.5 * floor["thickness"]]), FLOOR_RGBA))
    for name, shape, X, rgba in boxes:
        plant.RegisterCollisionGeometry(plant.world_body(), X, shape, name, CoulombFriction(0.9, 0.8))
        plant.RegisterVisualGeometry(plant.world_body(), X, shape, name + "_visual", np.array(rgba))


# The names the original full_arm_planner_8caps.ipynb uses (until it moves to legacy/)
OBSTACLES = WALLS
add_obstacles = add_walls


# ---------------------------------------------------------------- the scenes
# Obstacles: obstacle.sdf cylinders welded at a position. Walls: boxes on the world body (WALLS).
SCENES = {
    "1cap_2obs": dict(caps=[[0.5, 0.0, 0.0]], obstacles=[[0.51, 0.031, 0.01], [0.467, -0.005, 0.01]],
                      walls=[], floor=FLOOR),
    "2caps_0obs": dict(caps=[[0.5, 0.0, 0.0], [0.5, 0.25, 0.0]], obstacles=[], walls=[], floor=FLOOR),
    "8caps_3obs": dict(caps=CAP_POSES, obstacles=[], walls=WALLS, floor=FLOOR),
}


def add_panda(parser, plant):
    """The Panda arm welded at the origin and its hand welded to panda_link8, as in every notebook. Returns
    (arm, hand)."""
    from pydrake.all import RigidTransform, RollPitchYaw
    arm = parser.AddModels(file_name=str(REPO_ROOT / "my_sdfs" / "panda_arm.urdf"))[0]
    hand = parser.AddModels(url="package://drake_models/franka_description/urdf/panda_hand.urdf")[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0", arm), RigidTransform())
    plant.WeldFrames(plant.GetFrameByName("panda_link8", arm), plant.GetFrameByName("panda_hand", hand),
                     RigidTransform(RollPitchYaw(0.0, 0.0, -np.deg2rad(45.0)), [0.0, 0.0, 0.0]))
    plant.GetJointByName("panda_finger_joint1", hand).set_default_translation(-0.024)
    return arm, hand


def add_scene(parser, plant, name, welded=False):
    """Add scene `name`'s caps, obstacles, walls and floor to plant (before Finalize; the parser needs
    SetAutoRenaming(True) for several caps or obstacles). Returns (caps, obstacles): their model instances."""
    s = SCENES[name]
    caps = add_caps(parser, plant, welded=welded, poses=s["caps"])
    obstacles = []
    for p in s["obstacles"]:
        obstacle = parser.AddModels(file_name=str(OBSTACLE_SDF))[0]
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("obstacle_link", obstacle),
                         RigidTransform(RotationMatrix(), p))
        obstacles.append(obstacle)
    if s["walls"] or s["floor"] is not None:
        add_walls(plant, s["walls"], s["floor"])
    return caps, obstacles
