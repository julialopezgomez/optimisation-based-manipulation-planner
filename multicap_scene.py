"""The 8-cap scene: where the caps and the obstacle boxes are, and how to add them to a plant.

Shared by full_arm_planner_8caps.ipynb (caps jointed: each cap angle is a DOF) and by
experiments/cap_scaling/generate_cfree.py (caps welded: C-free is computed in the 9-D arm + finger space,
which is exact because a cap's collision geometry is a cylinder about its own joint axis).

The scene: 8 caps on an arc, 3 radial walls between groups of caps, and a floor just under the caps and the
robot base (add_obstacles adds the walls and the floor).
"""
from pathlib import Path

import numpy as np
from pydrake.all import Box, RigidTransform, RollPitchYaw, RotationMatrix

REPO_ROOT = Path(__file__).resolve().parent
CAP_SDF = REPO_ROOT / "my_sdfs" / "bottle_cap.sdf"                 # revolute cap joint
CAP_SDF_WELDED = REPO_ROOT / "my_sdfs" / "bottle_cap_welded.sdf"   # same geometry, fixed joint

# Caps on an arc of radius 0.5 m around the robot base, at floor height (same height as the single-cap
# scene's cap at (0.5, 0, 0)). Angles in degrees from +x; the two outermost caps are behind the base.
CAP_RADIUS = 0.5
CAP_ANGLES_DEG = [-120, -85, -50, -15, 15, 50, 85, 120]
CAP_POSES = [[CAP_RADIUS * np.cos(np.deg2rad(a)), CAP_RADIUS * np.sin(np.deg2rad(a)), 0.0] for a in CAP_ANGLES_DEG]

# Obstacle boxes, fixed to the world: radial walls between groups of caps, so moving the arm from one group
# to the next means going over or around a wall. Each wall starts 0.3 m from the base, which leaves the
# arm's lower links room to turn. The front wall is close to its two neighbouring caps (0.13 m), so it also
# blocks part of their wrist range.
OBSTACLES = [
    dict(name="front_wall", angle_deg=0, r_in=0.30, r_out=0.75, height=0.35, thickness=0.04),
    dict(name="left_wall", angle_deg=102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04),
    dict(name="right_wall", angle_deg=-102, r_in=0.30, r_out=0.75, height=0.50, thickness=0.04),
]
OBSTACLE_RGBA = [0.55, 0.45, 0.35, 0.6]

# Floor: a slab whose top is just under the caps and the robot base, so no plan moves the arm below the
# surface (e.g. under a wall instead of over it).
FLOOR = dict(name="floor", top_z=-0.01, size_xy=3.0, thickness=0.1)
FLOOR_RGBA = [0.85, 0.85, 0.82, 0.35]


def obstacle_box(o):
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


def add_obstacles(plant, obstacles=OBSTACLES, floor=FLOOR):
    """Register each obstacle box and the floor on the world body (collision + illustration). Call before Finalize()."""
    from pydrake.all import CoulombFriction
    boxes = [(o["name"], *obstacle_box(o), OBSTACLE_RGBA) for o in obstacles]
    if floor is not None:
        boxes.append((floor["name"], Box(floor["size_xy"], floor["size_xy"], floor["thickness"]),
                      RigidTransform([0.0, 0.0, floor["top_z"] - 0.5 * floor["thickness"]]), FLOOR_RGBA))
    for name, shape, X, rgba in boxes:
        plant.RegisterCollisionGeometry(plant.world_body(), X, shape, name, CoulombFriction(0.9, 0.8))
        plant.RegisterVisualGeometry(plant.world_body(), X, shape, name + "_visual", np.array(rgba))
