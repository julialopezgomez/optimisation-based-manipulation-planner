"""Meshcat markers for a planned manipulation: where it starts and ends, and the wrist limits.

PlanMarkers draws, under one Meshcat prefix:
- a ghost of the real gripper (its illustration meshes, in one translucent colour) at the start
  (green) and goal (orange) configurations;
- on top of the cap, a green tick at the start angle and an orange tick at the goal angle (read
  them against the cap's own red line), plus a grey spiral arc with an arrowhead showing the
  direction and amount of turning;
- a dial around the wrist joint axis: allowed range (grey), forbidden range (red), green/orange
  ticks at the start/goal wrist values and a needle that turns with the wrist.

The dial is drawn in the wrist joint's parent-side frame, so it follows the arm but not the
wrist; call update(plant_context) after every SetPositions during playback.
"""
import json
import os

import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import Box, Mesh, Rgba, Role
from pydrake.math import RigidTransform, RotationMatrix

START_RGBA = Rgba(0.2, 0.8, 0.2, 0.6)
GOAL_RGBA = Rgba(1.0, 0.55, 0.0, 0.6)
ARC_RGBA = Rgba(0.4, 0.4, 0.4, 0.9)
ALLOWED_RGBA = Rgba(0.3, 0.3, 0.3, 0.35)
FORBIDDEN_RGBA = Rgba(0.9, 0.1, 0.1, 0.3)
NEEDLE_RGBA = Rgba(0.1, 0.1, 0.1, 1.0)

# Cap markers, in the cap joint frame (sizes from my_sdfs/bottle_cap.sdf: radius 0.025, top at z=0.0075)
CAP_Z = 0.0075 + 0.001
CAP_TICK = (0.018, 0.036)        # radial extent of the start/goal ticks
CAP_ARC = (0.029, 0.032)         # radial extent of the turning arc at its start
CAP_ARC_GROWTH = 0.001           # radius added per rad turned, so turns >= 2*pi don't overlap

# Wrist dial, in the wrist joint's parent-side frame (z = wrist axis)
DIAL_Z = 0.06
DIAL_R = (0.075, 0.09)


def annular_sector(r_in, r_out, a0, a1, z=0.0, n=64, growth=0.0):
    """Flat ring band from angle a0 to a1 (either direction) at height z.

    growth: radius added per rad travelled from a0 (a spiral band).
    Returns (vertices 3xN, faces 3xM) for Meshcat.SetTriangleMesh.
    """
    t = np.linspace(0.0, 1.0, n + 1)
    a = a0 + t * (a1 - a0)
    dr = growth * np.abs(a - a0)
    inner = np.stack([(r_in + dr) * np.cos(a), (r_in + dr) * np.sin(a), np.full_like(a, z)])
    outer = np.stack([(r_out + dr) * np.cos(a), (r_out + dr) * np.sin(a), np.full_like(a, z)])
    V = np.hstack([inner, outer])
    i = np.arange(n)
    F = np.hstack([np.stack([i, i + 1, n + 1 + i]), np.stack([i + 1, n + 2 + i, n + 1 + i])])
    return V, F.astype(np.int32)


_GLTF_DTYPES = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_GLTF_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_gltf_triangles(filename):
    """Triangles of a .gltf file (external or embedded-free .bin buffers), in Drake's mesh frame.

    Applies the node transforms and, like Drake, rotates glTF's y-up into z-up (+90 deg about x).
    Meshcat ignores the rgba of a textured glTF, so this is what lets the ghost have a flat colour.
    Returns (vertices Nx3, faces Mx3).
    """
    with open(filename) as f:
        g = json.load(f)
    buffers = [np.fromfile(os.path.join(os.path.dirname(filename), b["uri"]), dtype=np.uint8)
               for b in g["buffers"]]

    def accessor(i):
        a = g["accessors"][i]
        view = g["bufferViews"][a["bufferView"]]
        dtype, width = np.dtype(_GLTF_DTYPES[a["componentType"]]), _GLTF_WIDTHS[a["type"]]
        item = dtype.itemsize * width
        stride = view.get("byteStride", item)
        start = view.get("byteOffset", 0) + a.get("byteOffset", 0)
        raw = buffers[view["buffer"]][start:start + stride * (a["count"] - 1) + item]
        rows = np.lib.stride_tricks.as_strided(raw, (a["count"], item), (stride, 1))
        return rows.copy().view(dtype).reshape(a["count"], width)

    def node_transform(node):
        if "matrix" in node:
            return np.array(node["matrix"]).reshape(4, 4).T
        T = np.eye(4)
        x, y, z, w = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
        T[:3, :3] = RotationMatrix(Quaternion(w, x, y, z)).matrix() * np.array(node.get("scale", [1.0, 1.0, 1.0]))
        T[:3, 3] = node.get("translation", [0.0, 0.0, 0.0])
        return T

    vertices, faces = [], []

    def visit(i, T_parent):
        node = g["nodes"][i]
        T = T_parent @ node_transform(node)
        for prim in g["meshes"][node["mesh"]]["primitives"] if "mesh" in node else []:
            V = accessor(prim["attributes"]["POSITION"]).astype(float)
            F = (accessor(prim["indices"]).reshape(-1, 3) if "indices" in prim
                 else np.arange(len(V)).reshape(-1, 3))
            faces.append(F + sum(len(v) for v in vertices))
            vertices.append(V @ T[:3, :3].T + T[:3, 3])
        for child in node.get("children", []):
            visit(child, T)

    for i in g["scenes"][g.get("scene", 0)]["nodes"]:
        visit(i, np.eye(4))
    V = np.vstack(vertices) @ RotationMatrix.MakeXRotation(np.pi / 2).matrix().T
    return V, np.vstack(faces).astype(np.int32)


class PlanMarkers:
    def __init__(self, meshcat, plant, inspector, hand_instance, wrist_joint, cap_joint,
                 prefix="/plan_markers"):
        self.meshcat = meshcat
        self.plant = plant
        self.inspector = inspector
        self.wrist_joint = wrist_joint
        self.cap_joint = cap_joint
        self.prefix = prefix
        self.context = plant.CreateDefaultContext()   # FK here never touches the playback context

        self.hand = plant.GetBodyByName("panda_hand", hand_instance)
        self.hand_bodies = [plant.get_body(i) for i in plant.GetBodyIndices(hand_instance)]
        self._mesh_cache = {}

        # phi: direction of the hand's +y (finger opening) in the wrist's child frame, so the
        # needle lines up with the fingers. The hand is welded to that frame, so phi is constant.
        X_CH = plant.CalcRelativeTransform(self.context, wrist_joint.frame_on_child(),
                                           self.hand.body_frame())
        v = X_CH.rotation().matrix() @ np.array([0.0, 1.0, 0.0])
        self.phi = float(np.arctan2(v[1], v[0]))

    # ------------------------------------------------------------------
    def set_start_goal(self, q_start, q_goal):
        self.clear()
        cap_angles, wrist_angles = [], []
        for q, name, rgba in ((q_start, "start", START_RGBA), (q_goal, "goal", GOAL_RGBA)):
            self.plant.SetPositions(self.context, q)
            self._draw_ghost_gripper(f"{self.prefix}/{name}_gripper", rgba)
            cap_angles.append(self.cap_joint.GetOnePosition(self.context))
            wrist_angles.append(self.wrist_joint.GetOnePosition(self.context))

        self._draw_cap(*cap_angles)
        self._draw_dial(*wrist_angles)
        self.plant.SetPositions(self.context, q_start)
        self.update(self.context)

    def update(self, plant_context):
        """Move the dial with the arm and turn the needle with the wrist."""
        X_WF = self.wrist_joint.frame_on_parent().CalcPoseInWorld(plant_context)
        self.meshcat.SetTransform(f"{self.prefix}/wrist_dial", X_WF)
        w = self.wrist_joint.GetOnePosition(plant_context)
        self.meshcat.SetTransform(f"{self.prefix}/wrist_dial/needle",
                                  RigidTransform(RotationMatrix.MakeZRotation(w + self.phi)))

    def clear(self):
        self.meshcat.Delete(self.prefix)

    # ------------------------------------------------------------------
    def _box(self, path, size, p, rgba):
        self.meshcat.SetObject(path, Box(*size), rgba)
        self.meshcat.SetTransform(path, RigidTransform(np.asarray(p, dtype=float)))

    def _radial_bar(self, path, r0, r1, angle, z, rgba, width=0.003, thickness=0.002):
        # Bar from radius r0 to r1 along direction `angle` in the parent's xy-plane, at height z.
        self._box(f"{path}/bar", (r1 - r0, width, thickness), [(r0 + r1) / 2, 0.0, z], rgba)
        self.meshcat.SetTransform(path, RigidTransform(RotationMatrix.MakeZRotation(angle)))

    def _mesh_triangles(self, mesh):
        key = (mesh.source().path(), tuple(mesh.scale3()))
        if key not in self._mesh_cache:
            V, F = read_gltf_triangles(key[0])
            self._mesh_cache[key] = ((V * mesh.scale3()).T, F.T)
        return self._mesh_cache[key]

    def _draw_ghost_gripper(self, path, rgba):
        # Every illustration geometry of the hand's bodies (hand + both fingers), at their poses
        # in self.context, in one flat translucent colour.
        for body in self.hand_bodies:
            body_path = f"{path}/{body.name()}"
            self.meshcat.SetTransform(body_path, self.plant.EvalBodyPoseInWorld(self.context, body))
            frame_id = self.plant.GetBodyFrameIdOrThrow(body.index())
            for k, geometry_id in enumerate(self.inspector.GetGeometries(frame_id, Role.kIllustration)):
                shape, geom_path = self.inspector.GetShape(geometry_id), f"{body_path}/{k}"
                if isinstance(shape, Mesh) and shape.extension() == ".gltf":
                    V, F = self._mesh_triangles(shape)
                    self.meshcat.SetTriangleMesh(geom_path, V, F, rgba)
                else:
                    self.meshcat.SetObject(geom_path, shape, rgba)
                self.meshcat.SetTransform(geom_path, self.inspector.GetPoseInFrame(geometry_id))

    def _draw_cap(self, alpha_start, alpha_goal):
        path = f"{self.prefix}/cap"
        self.meshcat.SetTransform(path, self.cap_joint.frame_on_parent().CalcPoseInWorld(self.context))
        self._radial_bar(f"{path}/start", *CAP_TICK, alpha_start, CAP_Z, START_RGBA)
        self._radial_bar(f"{path}/goal", *CAP_TICK, alpha_goal, CAP_Z + 0.0005, GOAL_RGBA)

        turn = alpha_goal - alpha_start
        if abs(turn) < 1e-6:
            return
        V, F = annular_sector(*CAP_ARC, alpha_start, alpha_goal, z=CAP_Z,
                              n=max(16, int(abs(turn) * 20)), growth=CAP_ARC_GROWTH)
        self.meshcat.SetTriangleMesh(f"{path}/arc", V, F, ARC_RGBA)

        # Arrowhead at the goal end, pointing along the turning direction
        r_mid = np.mean(CAP_ARC) + CAP_ARC_GROWTH * abs(turn)
        a_tip = alpha_goal + np.sign(turn) * 0.25
        polar = [(r_mid - 0.004, alpha_goal), (r_mid + 0.004, alpha_goal), (r_mid, a_tip)]
        V = np.array([[r * np.cos(a), r * np.sin(a), CAP_Z] for r, a in polar]).T
        self.meshcat.SetTriangleMesh(f"{path}/arrow", V, np.array([[0], [1], [2]], dtype=np.int32),
                                     ARC_RGBA)

    def _draw_dial(self, wrist_start, wrist_goal):
        path = f"{self.prefix}/wrist_dial"
        lower = self.wrist_joint.position_lower_limits()[0] + self.phi
        upper = self.wrist_joint.position_upper_limits()[0] + self.phi

        V, F = annular_sector(*DIAL_R, lower, upper, z=DIAL_Z)
        self.meshcat.SetTriangleMesh(f"{path}/allowed", V, F, ALLOWED_RGBA)
        if upper - lower < 2 * np.pi:
            V, F = annular_sector(*DIAL_R, upper, lower + 2 * np.pi, z=DIAL_Z)
            self.meshcat.SetTriangleMesh(f"{path}/forbidden", V, F, FORBIDDEN_RGBA)

        r0, r1 = DIAL_R[0] - 0.005, DIAL_R[1] + 0.005
        self._radial_bar(f"{path}/start", r0, r1, wrist_start + self.phi, DIAL_Z + 0.001, START_RGBA,
                         width=0.004)
        self._radial_bar(f"{path}/goal", r0, r1, wrist_goal + self.phi, DIAL_Z + 0.0015, GOAL_RGBA,
                         width=0.004)
        # needle: drawn along +x of its own frame; update() rotates it to the current wrist angle
        self._box(f"{path}/needle/bar", (DIAL_R[1] + 0.01, 0.004, 0.003),
                  [(DIAL_R[1] + 0.01) / 2, 0.0, DIAL_Z + 0.002], NEEDLE_RGBA)
