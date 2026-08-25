"""
Python port of Drake's greedy clique/clique-cover solvers
(planning/graph_algorithms/max_clique_solver_via_greedy.cc,
min_clique_cover_solver_via_greedy.cc) plus the bespoke truncated-cover
routine iris_from_clique_cover.cc actually uses internally
(ComputeGreedyTruncatedCliqueCover - distinct from
MinCliqueCoverSolverViaGreedy: no minimum-clique-size override, no
singleton fallback, and it always fully removes a covered vertex from the
graph rather than supporting a `partition` toggle). For issue #71.

Adjacency matrices are dense numpy boolean arrays here rather than Drake's
Eigen::SparseMatrix<bool> (see visibility_graph.py's docstring for why) -
same graphs, different representation. MaxCliqueSolverViaMip (Gurobi/Mosek)
and the exact MinCliqueCoverSolverBase machinery are not ported: this
project only ever uses the default greedy heuristics (see
experiments/grasping_space.ipynb), and MaxCliqueSolverViaMip needs a
proprietary solver license this project doesn't have.
"""

from typing import Callable

import numpy as np

Array = np.ndarray
MaxCliqueSolver = Callable[[Array], Array]


def max_clique_greedy(adjacency_matrix: Array) -> Array:
    """
    Ports MaxCliqueSolverViaGreedy::DoSolveMaxClique. Greedily adds the
    highest-degree remaining candidate to the clique (by the *original*
    degree ordering, computed once up front - not recomputed after each
    addition), keeping only candidates still adjacent to every added
    vertex. Returns a boolean membership mask.
    """
    n = adjacency_matrix.shape[0]
    degrees = adjacency_matrix.sum(axis=0)
    available = list(np.argsort(-degrees, kind="stable"))

    clique_members = []
    while available:
        point_to_add = available.pop(0)
        clique_members.append(point_to_add)
        available = [
            v for v in available
            if v != point_to_add and adjacency_matrix[v, point_to_add]
        ]

    is_clique_member = np.zeros(n, dtype=bool)
    is_clique_member[clique_members] = True
    return is_clique_member


def min_clique_cover_greedy(
    adjacency_matrix: Array,
    min_clique_size: int = 1,
    partition: bool = False,
    max_clique_solver: MaxCliqueSolver = max_clique_greedy,
) -> list[set[int]]:
    """
    Ports MinCliqueCoverSolverViaGreedy::DoSolveMinCliqueCover. Repeatedly
    takes the max clique from the (shrinking) graph and adds it to the
    cover, until the max clique found is smaller than min_clique_size. If
    min_clique_size == 1, any vertex never covered (can happen once its row/
    column has been zeroed by an earlier clique) is appended as its own
    singleton clique at the end, matching Drake's edge-case handling.
    """
    adjacency_matrix = adjacency_matrix.copy()
    n = adjacency_matrix.shape[0]
    cover: list[set[int]] = []
    covered = np.zeros(n, dtype=bool)

    while adjacency_matrix.sum() > 0:
        max_clique = max_clique_solver(adjacency_matrix)
        if max_clique.sum() < min_clique_size:
            break
        clique_indices = set(np.flatnonzero(max_clique).tolist())
        covered[list(clique_indices)] = True
        cover.append(clique_indices)

        if partition:
            adjacency_matrix[max_clique, :] = False
            adjacency_matrix[:, max_clique] = False
        else:
            mask_idx = np.flatnonzero(max_clique)
            adjacency_matrix[np.ix_(mask_idx, mask_idx)] = False

    if min_clique_size == 1:
        for i in range(n):
            if not covered[i]:
                cover.append({i})
    return cover


def compute_greedy_truncated_clique_cover(
    minimum_clique_size: int,
    adjacency_matrix: Array,
    max_clique_solver: MaxCliqueSolver = max_clique_greedy,
) -> list[Array]:
    """
    Ports ComputeGreedyTruncatedCliqueCover (iris_from_clique_cover.cc,
    anonymous namespace). Operates on its own copy of adjacency_matrix.
    Stops as soon as the largest remaining clique is <= minimum_clique_size
    OR too few points are left to beat it - deliberately not a full cover
    (points left uncovered here get resampled in the next visibility round
    by the caller, unlike min_clique_cover_greedy's singleton fallback).
    Returns a list of boolean membership masks, in the order found.
    """
    adjacency_matrix = adjacency_matrix.copy()
    num_points_left = adjacency_matrix.shape[0]
    last_clique_size = np.iinfo(np.int64).max

    cliques: list[Array] = []
    while last_clique_size > minimum_clique_size and num_points_left > minimum_clique_size:
        max_clique = max_clique_solver(adjacency_matrix)
        last_clique_size = int(max_clique.sum())
        num_points_left -= last_clique_size

        if last_clique_size >= minimum_clique_size:
            cliques.append(max_clique)
            mask_idx = np.flatnonzero(max_clique)
            adjacency_matrix[np.ix_(mask_idx, mask_idx)] = False

    return cliques
