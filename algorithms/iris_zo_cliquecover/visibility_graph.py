"""
Python port of Drake's VisibilityGraph (planning/visibility_graph.cc), for
issue #71.

Ported to a dense numpy boolean array rather than Drake's
Eigen::SparseMatrix<bool> + custom triplet iterator: the point counts here
(num_points_per_visibility_round, a couple hundred by default) make a dense
matrix trivial memory-wise, and Python has no equivalent reason to hand-roll
a triplet iterator to dodge a temporary copy. This is a representation
change only - the graph (including the self-loop on every collision-free
vertex's diagonal, which Drake's C++ also sets) is identical.
"""

import numpy as np

from pydrake.planning import CollisionChecker

Array = np.ndarray


def visibility_graph(checker: CollisionChecker, points: Array) -> Array:
    """
    Ports VisibilityGraph(). `points` is (dim, num_points) - one column per
    configuration, matching Drake's Eigen convention. Returns a symmetric
    (num_points, num_points) boolean adjacency matrix; row/col i's diagonal
    entry is True iff point i is collision-free (matching Drake's self-loop
    convention, relied on by the clique solvers - a collision-free point
    forms a valid clique of size 1 with itself).
    """
    num_points = points.shape[1]
    points_free = np.array(
        [checker.CheckConfigCollisionFree(points[:, i]) for i in range(num_points)]
    )

    adjacency = np.zeros((num_points, num_points), dtype=bool)
    for i in range(num_points):
        if not points_free[i]:
            continue
        adjacency[i, i] = True
        for j in range(i + 1, num_points):
            if points_free[j] and checker.CheckEdgeCollisionFree(points[:, i], points[:, j]):
                adjacency[i, j] = True
                adjacency[j, i] = True
    return adjacency
