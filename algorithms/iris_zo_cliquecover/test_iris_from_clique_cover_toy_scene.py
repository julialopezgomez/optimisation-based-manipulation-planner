"""
Regression check for iris_from_clique_cover.py (#71): cross-checks the
ported algorithm against Drake's native
pydrake.planning.IrisInConfigurationSpaceFromCliqueCover, on the same
2-DOF point-robot-with-one-obstacle scene as test_iris_zo_toy_scene.py. The
square obstacle in the middle of the domain makes the free configuration
space non-convex (an annulus-like region around it), so reaching the
default 70% coverage threshold genuinely requires multiple IRIS regions -
this exercises the clique-cover loop itself, not just a single IrisZo call.

Usage:
    python test_iris_from_clique_cover_toy_scene.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_iris_zo_toy_scene import build_checker
import iris_from_clique_cover as ported
from clique_solvers import max_clique_greedy

from pydrake.common import RandomGenerator
from pydrake.planning import (
    IrisInConfigurationSpaceFromCliqueCover,
    IrisFromCliqueCoverOptions as DrakeIrisFromCliqueCoverOptions,
    IrisZoOptions as DrakeIrisZoOptions,
    MaxCliqueSolverViaGreedy,
)


def main():
    checker = build_checker()

    print("=== Ported iris_from_clique_cover.py ===")
    ported_options = ported.IrisFromCliqueCoverOptions(
        coverage_termination_threshold=0.7,
        iteration_limit=5,
        minimum_clique_size=3,
        num_points_per_visibility_round=100,
        num_points_per_coverage_check=500,
    )
    ported_options.iris_zo_options.num_particles = 200
    ported_options.iris_zo_options.max_iterations = 3
    ported_options.iris_zo_options.random_seed = 0
    ported_options.iris_zo_options.verbose = False

    sets_ported: list = []
    ported.iris_from_clique_cover(
        checker, ported_options, RandomGenerator(0), sets_ported, max_clique_greedy,
    )
    volumes_ported = sorted(s.MaximumVolumeInscribedEllipsoid().CalcVolume() for s in sets_ported)
    print(f"ported: {len(sets_ported)} sets, inscribed-ellipsoid volumes={[f'{v:.3f}' for v in volumes_ported]}")

    print("\n=== Drake native IrisInConfigurationSpaceFromCliqueCover ===")
    drake_options = DrakeIrisFromCliqueCoverOptions()
    drake_options.coverage_termination_threshold = 0.7
    drake_options.iteration_limit = 5
    drake_options.minimum_clique_size = 3
    drake_options.num_points_per_visibility_round = 100
    drake_options.num_points_per_coverage_check = 500
    drake_iris_zo_options = DrakeIrisZoOptions()
    drake_iris_zo_options.sampled_iris_options.num_particles = 200
    drake_iris_zo_options.sampled_iris_options.max_iterations = 3
    drake_iris_zo_options.sampled_iris_options.random_seed = 0
    drake_iris_zo_options.sampled_iris_options.verbose = False
    drake_options.iris_options = drake_iris_zo_options

    sets_drake = IrisInConfigurationSpaceFromCliqueCover(
        checker, drake_options, RandomGenerator(0), [], MaxCliqueSolverViaGreedy(),
    )
    volumes_drake = sorted(s.MaximumVolumeInscribedEllipsoid().CalcVolume() for s in sets_drake)
    print(f"drake:  {len(sets_drake)} sets, inscribed-ellipsoid volumes={[f'{v:.3f}' for v in volumes_drake]}")

    # Both covers should need >1 region (the free space isn't convex) and
    # should agree on how many regions the greedy cover needed.
    assert len(sets_ported) > 1, "expected multiple regions for a non-convex free space"
    assert len(sets_ported) == len(sets_drake), (
        f"ported found {len(sets_ported)} sets, drake found {len(sets_drake)}")
    np.testing.assert_allclose(volumes_ported, volumes_drake, rtol=1e-2)

    obstacle_center = np.array([0.0, 0.0])
    assert not any(s.PointInSet(obstacle_center) for s in sets_ported)
    assert not any(s.PointInSet(obstacle_center) for s in sets_drake)

    print("\nAll checks passed - ported iris_from_clique_cover.py matches "
          "Drake's native implementation on this scene.")


if __name__ == "__main__":
    main()
