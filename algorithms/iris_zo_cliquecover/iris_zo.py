"""
Python port of Drake's IRIS-ZO algorithm (planning/iris/iris_zo.cc and
iris_common.cc, RobotLocomotion/drake), for issue #70. Ported by hand
because the outer loop isn't separately exposed to Python - Drake exposes
IrisZo() only as one opaque C++ call (pydrake.planning.IrisZo).

Reference: P. Werner, T. Cohn, R. H. Jiang, T. Seyde, M. Simchowitz,
R. Tedrake, D. Rus, "Faster Algorithms for Growing Collision-Free Convex
Polytopes in Robot Configuration Space."
https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

What's ported: the full outer loop (probabilistic collision-free test,
bisection-to-boundary, ellipsoid-tangent hyperplane fitting, redundancy
pruning, ellipsoid/volume-based termination) - the part that genuinely
isn't bindable. What's deliberately NOT ported (not exercised by this
project's own usage in experiments/grasping_space.ipynb, and each adds
real scope):
  - `parameterization` (growing regions in a reparameterized space Q->C).
    This port always grows regions directly in configuration space.
  - `containment_points` / `prog_with_additional_constraints` (the
    QP-based tangent-to-convex-hull hyperplane variant and the
    additional-constraint particle filter).
  - meshcat debug visualization.
Each of these is a real, separable extension of AddTangentToPolytope /
the particle filter in Drake's C++, not a simplification of the same
logic - add them here only if a caller actually needs them.

Sampling seam: particle generation is factored into
`populate_particles_by_uniform_sampling`, which currently calls Drake's
own HPolyhedron.UniformSample (the same routine IrisZo calls internally),
so this port can be validated against Drake's native IrisZo before #72
swaps it for nlp_sampling.py.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from pydrake.common import RandomGenerator
from pydrake.geometry.optimization import HPolyhedron, Hyperellipsoid
from pydrake.planning import CollisionChecker

Array = np.ndarray


# -----------------------------------------------------------------------------
# Options
# -----------------------------------------------------------------------------

@dataclass
class IrisZoOptions:
    """Subset of CommonSampledIrisOptions/IrisZoOptions actually used here.
    Field names/defaults match pydrake.planning.IrisZoOptions."""
    num_particles: int = 1000
    tau: float = 0.5
    delta: float = 0.05
    epsilon: float = 0.01
    max_iterations: int = 3
    max_iterations_separating_planes: int = 20
    max_separating_planes_per_iteration: int = 10
    verbose: bool = False
    require_sample_point_is_contained: bool = True
    configuration_space_margin: float = 0.01
    relax_margin: bool = False
    termination_threshold: float = 0.02
    relative_termination_threshold: float = 0.001
    remove_all_collisions_possible: bool = True
    random_seed: int = 1234
    mixing_steps: int = 50
    bisection_steps: int = 10


# -----------------------------------------------------------------------------
# Ported helpers (iris_common.cc)
# -----------------------------------------------------------------------------

def unadaptive_test_samples(epsilon: float, delta: float, tau: float) -> int:
    """Ports internal::unadaptive_test_samples."""
    return int(-2 * np.log(delta) / (tau * tau * epsilon) + 0.5)


def calc_delta_min(delta: float, max_iterations: int) -> float:
    """Ports internal::calc_delta_min."""
    return delta * 6 / (np.pi ** 2 * max_iterations ** 2)


def populate_particles_by_uniform_sampling(
    P: HPolyhedron, num_to_sample: int, mixing_steps: int, generator: RandomGenerator,
) -> Array:
    """
    Ports internal::PopulateParticlesByUniformSampling. Chains particles
    (particle j starts its hit-and-run walk from particle j-1, each getting
    `mixing_steps` moves) via Drake's own HPolyhedron.UniformSample - the
    same routine IrisZo calls internally, validated against nlp_sampling.py
    in #69. Drake parallelizes this chain-per-thread; not replicated here
    since it's not the bottleneck (collision checking is, and that stays
    parallel via CheckConfigsCollisionFree regardless).
    """
    particles = np.empty((num_to_sample, P.ambient_dimension()))
    particles[0] = P.UniformSample(generator, mixing_steps)
    for j in range(1, num_to_sample):
        particles[j] = P.UniformSample(generator, particles[j - 1], mixing_steps)
    return particles


def add_tangent_to_polytope(
    ellipsoid: Hyperellipsoid,
    point: Array,
    configuration_space_margin: float,
    relax_margin: bool,
    A_rows: list[Array],
    b_rows: list[float],
) -> None:
    """
    Ports internal::AddTangentToPolytope's plain (non-containment-points)
    overload. Appends one row to A_rows/b_rows in place. Drake preallocates
    and conservativeResize()s A/b for C++ heap-allocation avoidance; that's
    a micro-optimization irrelevant in Python, so this just appends to
    plain lists (converted to arrays once, by the caller, when building the
    HPolyhedron).
    """
    ellipsoid_ata = ellipsoid.A().T @ ellipsoid.A()
    a_face = ellipsoid_ata @ (point - ellipsoid.center())
    a_face = a_face / np.linalg.norm(a_face)

    b_point = a_face @ point
    b_face = b_point - configuration_space_margin
    b_center = a_face @ ellipsoid.center()

    if b_center > b_face:
        if relax_margin:
            # Hyperplane would cut off the ellipsoid's own center - split
            # the difference instead, matching Drake's relaxation.
            b_face = (b_point + b_center) / 2.0
        else:
            raise ValueError(
                "The current center of the IRIS region is within "
                "configuration_space_margin of being infeasible. Check your "
                "sample point and/or constraints - the configuration space "
                "surrounding the sample point must have an interior."
            )

    A_rows.append(a_face)
    b_rows.append(b_face)


# -----------------------------------------------------------------------------
# Main algorithm (iris_zo.cc)
# -----------------------------------------------------------------------------

def iris_zo(
    checker: CollisionChecker,
    starting_ellipsoid: Hyperellipsoid,
    domain: HPolyhedron,
    options: Optional[IrisZoOptions] = None,
) -> HPolyhedron:
    """Ports IrisZo(). See module docstring for what's intentionally omitted."""
    if options is None:
        options = IrisZoOptions()

    ambient_dimension = domain.ambient_dimension()
    if starting_ellipsoid.ambient_dimension() != ambient_dimension:
        raise ValueError("starting_ellipsoid and domain must share ambient dimension.")
    if not domain.IsBounded():
        raise ValueError("domain must be bounded.")
    if not domain.PointInSet(starting_ellipsoid.center()):
        raise ValueError("starting_ellipsoid center must be inside domain.")
    if options.max_iterations_separating_planes <= 0:
        raise ValueError("max_iterations_separating_planes must be > 0.")

    starting_center = starting_ellipsoid.center()
    if not checker.CheckConfigCollisionFree(starting_center):
        raise ValueError("starting_ellipsoid center is in collision.")

    generator = RandomGenerator(options.random_seed)

    current_ellipsoid = starting_ellipsoid
    current_ellipsoid_center = starting_ellipsoid.center()
    previous_volume = 0.0

    domain_A, domain_b = domain.A(), domain.b()
    P = domain
    P_prev = domain

    iteration = 0
    while True:
        if options.verbose:
            print(f"IrisZo outer iteration {iteration}")

        A_rows = [row for row in domain_A]
        b_rows = [val for val in domain_b]

        if options.max_iterations == 1:
            outer_delta = options.delta
        else:
            outer_delta = options.delta * 6 / (np.pi ** 2 * (iteration + 1) ** 2)

        num_iterations_separating_planes = 0
        probabilistic_test_passed = False

        while num_iterations_separating_planes < options.max_iterations_separating_planes:
            k_squared = (num_iterations_separating_planes + 1) ** 2
            delta_k = outer_delta * 6 / (np.pi ** 2 * k_squared)
            N_test = unadaptive_test_samples(options.epsilon, delta_k, options.tau)
            N_samples_to_draw = max(N_test, options.num_particles)

            particles = populate_particles_by_uniform_sampling(
                P, N_samples_to_draw, options.mixing_steps, generator,
            )

            particle_col_free = checker.CheckConfigsCollisionFree(list(particles), True)

            particles_in_collision = []
            number_particles_in_collision_unadaptive_test = 0
            for i, is_free in enumerate(particle_col_free):
                if not is_free:
                    if len(particles_in_collision) < options.num_particles:
                        particles_in_collision.append(particles[i])
                    if i < N_test:
                        number_particles_in_collision_unadaptive_test += 1

            threshold = (1 - options.tau) * options.epsilon * N_test
            probabilistic_test_passed = number_particles_in_collision_unadaptive_test <= threshold

            if options.verbose:
                print(f"IrisZo N_test {N_test}, N_col {number_particles_in_collision_unadaptive_test}, "
                      f"thresh {threshold}")

            if not options.remove_all_collisions_possible and probabilistic_test_passed:
                break

            # Bisection: push each colliding particle toward the region
            # boundary between the (known collision-free) ellipsoid center
            # and the particle. current_point tracks the *last query found
            # in collision*, not the midpoint - this lands the hyperplane
            # close to the true boundary rather than deep in the obstacle,
            # matching Drake's exact bisection logic.
            particles_in_collision_updated = []
            for particle in particles_in_collision:
                curr_pt_lower = current_ellipsoid_center
                if not checker.CheckConfigCollisionFree(curr_pt_lower):
                    current_point = curr_pt_lower
                else:
                    curr_pt_upper = particle
                    current_point = particle
                    for _ in range(options.bisection_steps):
                        query = 0.5 * (curr_pt_upper + curr_pt_lower)
                        if checker.CheckConfigCollisionFree(query):
                            curr_pt_lower = query
                        else:
                            curr_pt_upper = query
                            current_point = query
                particles_in_collision_updated.append(current_point)

            # Sort by ellipsoid-metric distance to center (ascending) -
            # nearest obstacle points get separating hyperplanes first.
            ata = current_ellipsoid.A().T @ current_ellipsoid.A()
            distances = [
                (p - current_ellipsoid_center) @ ata @ (p - current_ellipsoid_center)
                for p in particles_in_collision_updated
            ]
            indices_sorted = np.argsort(distances)

            num_particles_in_collision = len(particles_in_collision_updated)
            particle_is_redundant = [False] * num_particles_in_collision
            hyperplanes_added = 0

            for i in indices_sorted:
                if particle_is_redundant[i]:
                    continue
                add_tangent_to_polytope(
                    current_ellipsoid, particles_in_collision_updated[i],
                    options.configuration_space_margin, options.relax_margin,
                    A_rows, b_rows,
                )
                hyperplanes_added += 1

                if (options.max_separating_planes_per_iteration > 0
                        and hyperplanes_added == options.max_separating_planes_per_iteration):
                    break

                particle_is_redundant[i] = True
                latest_a, latest_b = A_rows[-1], b_rows[-1]
                for j in range(num_particles_in_collision):
                    if not particle_is_redundant[j]:
                        margin = latest_a @ particles_in_collision_updated[j] - latest_b
                        if margin >= 0:
                            particle_is_redundant[j] = True

            P = HPolyhedron(np.array(A_rows), np.array(b_rows))

            if probabilistic_test_passed:
                break
            num_iterations_separating_planes += 1

        current_ellipsoid = P.MaximumVolumeInscribedEllipsoid()
        current_ellipsoid_center = current_ellipsoid.center()
        volume = current_ellipsoid.CalcVolume()
        delta_volume = volume - previous_volume

        if options.verbose:
            print(f"IrisZo delta vol {delta_volume}")

        if delta_volume <= options.termination_threshold:
            break
        if delta_volume / (previous_volume + 1e-6) <= options.relative_termination_threshold:
            break
        iteration += 1
        if iteration >= options.max_iterations:
            break
        if not checker.CheckConfigCollisionFree(current_ellipsoid_center):
            if options.verbose:
                print("IrisZo terminating early: new ellipsoid center is in collision.")
            break
        if options.require_sample_point_is_contained:
            if not P.PointInSet(starting_center):
                if options.verbose:
                    print("IrisZo ERROR: initial seed point not contained in region.")
                return P_prev

        previous_volume = volume
        P_prev = P
        P = domain

    return P
