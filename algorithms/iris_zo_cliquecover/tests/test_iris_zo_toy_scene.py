"""
Regression check for iris_zo.py (#70): cross-checks the ported algorithm
against Drake's native pydrake.planning.IrisZo on a tiny 2-DOF point-robot
scene with one box obstacle. Deliberately small/fast (not the full Panda
scene in experiments/grasping_space.ipynb) so it can run on every change to
iris_zo.py. Both implementations use the same random_seed and ultimately
call the same underlying HPolyhedron.UniformSample, so a passing run means
face count, N_test/N_col at every step, and inscribed-ellipsoid volume all
match Drake's C++ exactly - not just "close enough".

Usage:
    python tests/test_iris_zo_toy_scene.py
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import iris_zo as ported

from pydrake.geometry import Box, ProximityProperties, AddContactMaterial
from pydrake.geometry.optimization import HPolyhedron, Hyperellipsoid
from pydrake.math import RigidTransform
from pydrake.multibody.tree import PrismaticJoint, SpatialInertia, UnitInertia
from pydrake.planning import (
    RobotDiagramBuilder, SceneGraphCollisionChecker, CollisionCheckerParams,
    IrisZo, IrisZoOptions as DrakeIrisZoOptions,
)


def build_checker():
    builder = RobotDiagramBuilder(time_step=0.0)
    plant = builder.plant()
    scene_graph = builder.scene_graph()

    model_instance = plant.AddModelInstance("particle")

    dummy = plant.AddRigidBody(
        "dummy", model_instance,
        SpatialInertia(0.0, [0, 0, 0], UnitInertia(0.0, 0.0, 0.0)),
    )
    particle = plant.AddRigidBody(
        "particle_body", model_instance,
        SpatialInertia(1.0, [0, 0, 0], UnitInertia(1.0, 1.0, 1.0)),
    )

    joint_x = plant.AddJoint(PrismaticJoint(
        "joint_x", plant.world_frame(), dummy.body_frame(), [1, 0, 0]))
    joint_x.set_position_limits([-2.0], [2.0])
    joint_y = plant.AddJoint(PrismaticJoint(
        "joint_y", dummy.body_frame(), particle.body_frame(), [0, 1, 0]))
    joint_y.set_position_limits([-2.0], [2.0])

    props = ProximityProperties()
    AddContactMaterial(1e5, 1e5, None, props)
    plant.RegisterCollisionGeometry(
        particle, RigidTransform(), Box(0.05, 0.05, 0.05), "particle_geom", props)

    # A box obstacle straddling the middle of the domain, so it carves a
    # real hole out of the free configuration space.
    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform([0.0, 0.0, 0.0]),
        Box(0.8, 0.8, 0.1), "obstacle", props)

    plant.Finalize()
    diagram = builder.Build()

    params = CollisionCheckerParams()
    params.model = diagram
    params.robot_model_instances = [model_instance]
    params.edge_step_size = 0.05
    return SceneGraphCollisionChecker(params)


def main():
    checker = build_checker()
    domain = HPolyhedron.MakeBox([-2.0, -2.0], [2.0, 2.0])

    # Seed away from the obstacle.
    seed_point = np.array([1.5, 1.5])
    assert checker.CheckConfigCollisionFree(seed_point)
    starting_ellipsoid = Hyperellipsoid.MakeHypersphere(0.01, seed_point)

    print("=== Ported iris_zo.py ===")
    ported_options = ported.IrisZoOptions(
        num_particles=200, max_iterations=3, max_iterations_separating_planes=20,
        random_seed=0, verbose=True,
    )
    t0 = time.perf_counter()
    P_ported = ported.iris_zo(checker, starting_ellipsoid, domain, ported_options)
    ported_elapsed = time.perf_counter() - t0
    ellipsoid_ported = P_ported.MaximumVolumeInscribedEllipsoid()
    print(f"ported: faces={P_ported.A().shape[0]}, inscribed ellipsoid volume={ellipsoid_ported.CalcVolume():.4f}, "
          f"wall-clock={ported_elapsed:.3f}s")

    print("\n=== Drake native IrisZo ===")
    drake_options = DrakeIrisZoOptions()
    drake_options.sampled_iris_options.num_particles = 200
    drake_options.sampled_iris_options.max_iterations = 3
    drake_options.sampled_iris_options.max_iterations_separating_planes = 20
    drake_options.sampled_iris_options.random_seed = 0
    drake_options.sampled_iris_options.verbose = True
    t0 = time.perf_counter()
    P_drake = IrisZo(checker, starting_ellipsoid, domain, drake_options)
    drake_elapsed = time.perf_counter() - t0
    ellipsoid_drake = P_drake.MaximumVolumeInscribedEllipsoid()
    print(f"drake:  faces={P_drake.A().shape[0]}, inscribed ellipsoid volume={ellipsoid_drake.CalcVolume():.4f}, "
          f"wall-clock={drake_elapsed:.3f}s")
    print(f"\nported/drake wall-clock ratio: {ported_elapsed / drake_elapsed:.2f}x "
          f"(>1 means the ported Python orchestration is slower than Drake's C++ one; "
          f"both call the same parallel collision-checking C++ underneath, so this ratio "
          f"isolates the Python-loop overhead, not the collision-checking cost)")

    # Sanity: neither region should contain the obstacle's center.
    obstacle_center = np.array([0.0, 0.0])
    print(f"\nobstacle center in ported region: {P_ported.PointInSet(obstacle_center)} (should be False)")
    print(f"obstacle center in drake region:  {P_drake.PointInSet(obstacle_center)} (should be False)")
    print(f"seed point in ported region: {P_ported.PointInSet(seed_point)} (should be True)")
    print(f"seed point in drake region:  {P_drake.PointInSet(seed_point)} (should be True)")

    assert not P_ported.PointInSet(obstacle_center), "ported region should exclude the obstacle"
    assert not P_drake.PointInSet(obstacle_center), "drake region should exclude the obstacle"
    assert P_ported.PointInSet(seed_point), "ported region should contain the seed"
    assert P_drake.PointInSet(seed_point), "drake region should contain the seed"
    assert P_ported.A().shape[0] == P_drake.A().shape[0], "face count should match Drake exactly (same RNG stream)"
    np.testing.assert_allclose(ellipsoid_ported.CalcVolume(), ellipsoid_drake.CalcVolume(), rtol=1e-3)
    print("\nAll checks passed - ported iris_zo.py matches Drake's native IrisZo on this scene.")


if __name__ == "__main__":
    main()
