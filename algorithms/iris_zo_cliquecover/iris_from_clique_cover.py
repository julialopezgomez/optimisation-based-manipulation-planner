"""
Python port of Drake's IrisInConfigurationSpaceFromCliqueCover
(planning/iris/iris_from_clique_cover.cc), for issue #71. Restricted to the
IrisZo variant (this project's only iris_options variant, per #70) - the
IrisOptions/IrisNp2Options branches, and everything gated behind them
(parameterization checks, bounding_region override, meshcat visualization)
are not ported, matching #70's scoping.

Concurrency: Drake's C++ pipelines clique-finding and region-growing across
threads via a producer/consumer queue (AsyncQueue) - but for the IrisZo
variant specifically, Drake itself only ever uses *one* builder thread
(`num_builder_threads` is hardcoded to 1 for IrisZoOptions in the source,
since IrisZo already parallelizes its own collision checking internally).
So for this variant the concurrency machinery reduces to a plain sequential
loop over cliques even in Drake's own C++ - ported here as exactly that,
with no AsyncQueue/thread-pool equivalent needed.

Per-clique region growing calls this project's own iris_zo() (#70,
iris_zo.py) directly, seeded by the clique's minimum-volume circumscribing
ellipsoid (the Loewner-John ellipsoid) - the actual trick from the Werner
et al. "Clique Covers" paper that makes seeding from visibility cliques
better than growing from a generic small ball.
"""

from dataclasses import dataclass, field

import numpy as np

from pydrake.common import RandomGenerator
from pydrake.geometry.optimization import HPolyhedron, Hyperellipsoid
from pydrake.planning import CollisionChecker

from clique_solvers import MaxCliqueSolver, compute_greedy_truncated_clique_cover, max_clique_greedy
from iris_zo import IrisZoOptions, iris_zo
from visibility_graph import visibility_graph

Array = np.ndarray


@dataclass
class IrisFromCliqueCoverOptions:
    """Subset of pydrake.planning.IrisFromCliqueCoverOptions actually used
    here (IrisZo variant only). Field names/defaults match the original."""
    coverage_termination_threshold: float = 0.7
    iteration_limit: int = 100
    minimum_clique_size: int = 3
    num_points_per_coverage_check: int = 1000
    num_points_per_visibility_round: int = 200
    point_in_set_tol: float = 1e-6
    rank_tol_for_minimum_volume_circumscribed_ellipsoid: float = 1e-6
    iris_zo_options: IrisZoOptions = field(default_factory=IrisZoOptions)


def approximately_compute_coverage(
    domain: HPolyhedron, sets: list[HPolyhedron], checker: CollisionChecker,
    num_samples: int, point_in_set_tol: float, generator: RandomGenerator,
    last_polytope_sample: Array,
) -> tuple[float, Array]:
    """
    Ports ApproximatelyComputeCoverage. Returns (fraction_covered,
    updated last_polytope_sample) - the sample chain is threaded through
    explicitly since Python has no output-reference-parameter equivalent.
    """
    if not sets:
        return 0.0, last_polytope_sample

    sample = last_polytope_sample
    num_in_sets = 0
    for _ in range(num_samples):
        while True:
            sample = domain.UniformSample(generator, sample)
            if checker.CheckConfigCollisionFree(sample):
                break
        if any(s.PointInSet(sample, point_in_set_tol) for s in sets):
            num_in_sets += 1

    return num_in_sets / num_samples, sample


def iris_from_clique_cover(
    checker: CollisionChecker,
    options: IrisFromCliqueCoverOptions,
    generator: RandomGenerator,
    sets: list[HPolyhedron],
    max_clique_solver: MaxCliqueSolver = max_clique_greedy,
) -> list[HPolyhedron]:
    """
    Ports IrisInConfigurationSpaceFromCliqueCover (IrisZo variant only).
    `sets` is mutated in place (existing regions count toward coverage and
    are avoided when sampling new visibility points) and also returned, to
    match pydrake's own binding ergonomics.
    """
    if options.coverage_termination_threshold <= 0:
        raise ValueError("coverage_termination_threshold must be > 0.")
    if options.iteration_limit <= 0:
        raise ValueError("iteration_limit must be > 0.")

    domain = HPolyhedron.MakeBox(
        checker.plant().GetPositionLowerLimits(),
        checker.plant().GetPositionUpperLimits(),
    )
    last_polytope_sample = domain.UniformSample(generator)

    num_positions = checker.plant().num_positions()
    minimum_clique_size = max(options.minimum_clique_size, num_positions + 1)
    num_points_per_visibility_round = max(
        options.num_points_per_visibility_round, 2 * minimum_clique_size)

    num_iterations = 0
    while num_iterations < options.iteration_limit:
        coverage, last_polytope_sample = approximately_compute_coverage(
            domain, sets, checker, options.num_points_per_coverage_check,
            options.point_in_set_tol, generator, last_polytope_sample,
        )
        if options.iris_zo_options.verbose:
            print(f"IrisFromCliqueCover coverage: {coverage:.3f}")
        if coverage >= options.coverage_termination_threshold:
            break

        if options.iris_zo_options.verbose:
            print(f"IrisFromCliqueCover Iteration {num_iterations + 1}/{options.iteration_limit}")

        # Sample points that are collision-free and not already covered by
        # an existing set, for this round's visibility graph.
        points = np.empty((domain.ambient_dimension(), num_points_per_visibility_round))
        for i in range(num_points_per_visibility_round):
            while True:
                last_polytope_sample = domain.UniformSample(generator, last_polytope_sample)
                if (checker.CheckConfigCollisionFree(last_polytope_sample)
                        and not any(s.PointInSet(last_polytope_sample) for s in sets)):
                    break
            points[:, i] = last_polytope_sample

        adjacency = visibility_graph(checker, points)
        cliques = compute_greedy_truncated_clique_cover(
            minimum_clique_size, adjacency, max_clique_solver)

        num_new_sets = 0
        for clique_mask in cliques:
            clique_points = points[:, clique_mask]
            try:
                clique_ellipse = Hyperellipsoid.MinimumVolumeCircumscribedEllipsoid(
                    clique_points, options.rank_tol_for_minimum_volume_circumscribed_ellipsoid,
                )
            except RuntimeError:
                continue

            if not checker.CheckConfigCollisionFree(clique_ellipse.center()):
                # The Loewner-John ellipsoid's center need not itself be
                # collision-free (it's a fitted centroid, not one of the
                # sampled points) - fall back to the nearest actual clique
                # point, keeping the fitted shape matrix.
                deltas = clique_points - clique_ellipse.center()[:, None]
                nearest = clique_points[:, np.argmin(np.linalg.norm(deltas, axis=0))]
                clique_ellipse = Hyperellipsoid(clique_ellipse.A(), nearest)

            new_set = iris_zo(checker, clique_ellipse, domain, options.iris_zo_options)
            sets.append(new_set)
            num_new_sets += 1

        if num_new_sets == 0:
            num_points_per_visibility_round *= 2
        num_iterations += 1

    return sets
