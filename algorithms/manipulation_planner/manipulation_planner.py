"""
Shared toy WSG-gripper `ManipulationPlanner`, deduplicated from
`manipulation_planner_3dof.ipynb` and `manipulation_planner_4dof.ipynb`
(both notebooks previously defined their own copy of this class, which had
drifted apart in a few places - see issue #60).

Solves for a minimum-grasp-count path through a mixed-integer program over
collision-free polytopes, then stitches a GCS trajectory through it.
"""

import time

import numpy as np
from pydrake.all import (
    IrisFromCliqueCoverOptions, IrisInConfigurationSpaceFromCliqueCover, ModelInstanceIndex,
    RandomGenerator, SceneGraphCollisionChecker,
)
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, HPolyhedron, Point
from pydrake.planning import GcsTrajectoryOptimization
from pydrake.solvers import MathematicalProgram, MosekSolver
from pydrake.trajectories import CompositeTrajectory
from scipy.spatial import ConvexHull

from ciris_plant_visualizer import CIrisPlantVisualizer


def convex_hull_to_hpolyhedron(hull: ConvexHull) -> HPolyhedron:
    A = hull.equations[:, :-1]
    b = -hull.equations[:, -1]
    return HPolyhedron(A, b)


class ManipulationPlanner():

    def __init__(self,
            visualizer: CIrisPlantVisualizer,
            CP: HPolyhedron,
            CG: HPolyhedron,
            gripper: ModelInstanceIndex,
            cap: ModelInstanceIndex,
            max_grasps: int = 20,
        ):
        self.builder = visualizer.builder
        self.plant = visualizer.plant
        self.plant_context = visualizer.plant_context
        self.scene_graph = visualizer.scene_graph
        self.q_star = visualizer.q_star
        self.rat_fk = visualizer.rat_forward_kin
        self.inspector = visualizer.model_inspector
        self.diagram = visualizer.task_space_diagram
        self.diagram_context = visualizer.task_space_diagram_context
        self.lower_joint_limits = visualizer.q_lower_limits
        self.upper_joint_limits = visualizer.q_upper_limits
        self.visualize_cspace = visualizer.visualize_collision_constraint
        self.cspace_free_polytope = visualizer.cspace_free_polytope
        self.visualizer = visualizer

        self.CP = CP
        self.CG = CG
        self.gripper = gripper
        self.cap = cap
        self.max_grasps = max_grasps

        self.q_dim = self.plant.num_positions()

        self.cs_free = self._generate_cfree()

    def set_max_grasps(self, max_grasps: int):
        self.max_grasps = max_grasps


    def compute_trajectory(self, x_init, x_goal, display=True):
        print("Finding path for minimum grasps")
        path = self._find_minimum_grasp_path(
                x_init = x_init,
                x_goal = x_goal,
                P = self.CP,
                G = self.CG,
                lower_joint_limits = self.lower_joint_limits,
                upper_joint_limits = self.upper_joint_limits,
                c_free_polytopes = self.cs_free
            )

        print("Path found. Generating trajectory")
        traj = self._generate_trajectory(path)

        print("Trajectory generated. Starting display")
        if display:
            self.display_trajectory(traj)

        return path, traj

    def display_trajectory(self, traj, meshcat=True, plotly=False):
        if meshcat:
            num_points = int((traj.end_time() - traj.start_time()) * 4000)
            for t in np.linspace(traj.start_time(), traj.end_time(), num_points):
                q = traj.value(t)
                self.plant.GetJointByName("left_finger_sliding_joint", self.gripper).set_translation(self.plant_context, q[2])
                if self.q_dim > 3:
                    self.plant.GetJointByName("right_finger_sliding_joint", self.gripper).set_translation(self.plant_context, q[3])
                self.plant.GetJointByName("base_revolute_joint", self.gripper).set_angle(self.plant_context, q[1])
                self.plant.GetJointByName("cap_to_base", self.cap).set_angle(self.plant_context, -q[0])
                self.diagram.ForcedPublish(self.diagram_context)
                time.sleep(0.01)
        path = [traj.value(t) for t in np.linspace(traj.start_time(), traj.end_time(), 100)]
        path_q = [q.ravel() for q in path] # Convert to list of numpy arrays
        if plotly:
            self.visualize_cspace(factor=1, num_points=30, paths=[path_q], filled_polytopes=self.cs_free)
        return path_q

    def _generate_trajectory(self, path):
        trajs = []
        print("Generating trajectory from:")
        for q0, q1 in zip(path[:-1], path[1:]):
            print("From\t{}\tto\t{}".format(q0, q1))
            traj, result = self._generate_edge_trajectory(q0, q1)
            if not result.is_success():
                print("failed to generate edge trajectory from {} to {}".format(q0, q1))
                return None
            trajs.append(traj)
        return CompositeTrajectory.AlignAndConcatenate(trajs)


    def _generate_edge_trajectory(self, x_init, x_goal):
        trajopt = GcsTrajectoryOptimization(self.plant.num_positions())
        gcs_regions = trajopt.AddRegions(self.cs_free, order=1, h_min=0.01)
        source = trajopt.AddRegions([Point(x_init)], order=0)
        target = trajopt.AddRegions([Point(x_goal)], order=0)
        trajopt.AddEdges(source, gcs_regions)
        trajopt.AddEdges(gcs_regions, target)
        trajopt.AddPathLengthCost()
        options = GraphOfConvexSetsOptions()
        [traj, result] = trajopt.SolvePath(source, target, options)
        print(f"result.is_success() = {result.is_success()}")
        return traj, result


    def _find_minimum_grasp_path(
            self,
            x_init: np.ndarray,
            x_goal: np.ndarray,
            P: HPolyhedron,
            G: HPolyhedron,
            lower_joint_limits: np.ndarray,
            upper_joint_limits: np.ndarray,
            c_free_polytopes: list,
            ) -> np.ndarray:

        for i in range(self.max_grasps):
            if i % 10 == 0 and i > 0:
                print(f"Trying with {i} grasps")
            path = self._solve_for_n_grasps_CC(
                i,
                x_init,
                x_goal,
                P,
                G,
                lower_joint_limits,
                upper_joint_limits,
                c_free_polytopes
            )

            if path is not None:
                print(f"Found a solution with {i} grasps")
                return path
        print(f"No solution found for less than {self.max_grasps} grasps")


    def _solve_for_n_grasps_CC(
            self,
            n_grasps: int,
            x_init: np.ndarray,
            x_goal: np.ndarray,
            P: HPolyhedron,
            G: HPolyhedron,
            lower_joint_limits: np.ndarray,
            upper_joint_limits: np.ndarray,
            c_free_polytopes: list,
            ) -> np.ndarray:
        # Initialize the program
        prog = MathematicalProgram()
        n_points = 2 * n_grasps + 2
        n_vars = self.q_dim * n_points

        # Create decision variables
        x = prog.NewContinuousVariables(n_vars, "x")

        # 1. Cost function: Minimize sum of squared distances between consecutive points

        c = np.zeros((n_vars - self.q_dim, 1))
        C = np.zeros((n_vars - self.q_dim, n_vars))
        C[:, self.q_dim:] = np.eye(n_vars - self.q_dim)
        C[:, :-self.q_dim] -= np.eye(n_vars - self.q_dim)

        Q, b = self._to_quadratic_form(C, c)
        Q += np.eye(Q.shape[1]) * 1e-6  # Add a small value to the diagonal to make Q positive definite

        prog.AddQuadraticCost(Q=Q, b=b, vars=x)

        # 2. Initial and goal constraints
        prog.AddLinearEqualityConstraint(np.eye(self.q_dim), x_init.flatten(), x[:self.q_dim])
        prog.AddLinearEqualityConstraint(np.eye(self.q_dim), x_goal.flatten(), x[-self.q_dim:])


        ###### Placement and Grasping Constraints should be adapted task specific #####

        # 3. Placement constraints (equality) -> Cap remains constant in transit paths
        for i in range(n_grasps + 1):
            idx1 = 2*self.q_dim*i
            idx2 = 2*self.q_dim*i + self.q_dim
            prog.AddLinearEqualityConstraint(x[idx1] - x[idx2] == 0)

        # 4. Grasp constraints (equality between gripper orientations)
        for i in range(n_grasps):
            idx = 2*self.q_dim*(i+1) - 1
            prog.AddLinearEqualityConstraint(x[idx-self.q_dim+1] - x[idx-self.q_dim+2] - x[idx+1] + x[idx+2] == 0)

        ###### end of manual constraints


        # 5. Inequality Constraints (Placement hull)
        for i in range(n_points):
            prog.AddLinearConstraint(
                P.A(),  # Coefficient matrix
                -np.inf * np.ones_like(P.b()),  # Lower bound
                P.b(),  # Upper bound
                x[self.q_dim*i:self.q_dim*(i+1)]
            )
        # 6. Inequality Constraints (Grasp hull)
        # for i in range(n_points - 2):
        #     prog.AddLinearConstraint(
        #         G.A(),
        #         -np.inf * np.ones_like(G.b()),
        #         G.b(),
        #         x[self.q_dim*(i+1):self.q_dim*(i+2)]
        #     )

        # 7. Joint limits (inequality)
        prog.AddBoundingBoxConstraint(
            np.tile(lower_joint_limits, n_points),
            np.tile(upper_joint_limits, n_points),
            x
        )

        # 7. Collision-free polytope constraints (MIP)
        # Ensure all points are contained in at least one polytope

        M = 1e6  # Big-M constant (adjust based on problem scale)
        for i in range(n_points):
            x_i = x[self.q_dim*i:self.q_dim*(i+1)]

            # Creates one bineary 0-1 variable for each polytope
            z_i = prog.NewBinaryVariables(len(c_free_polytopes), name=f"z_{i}")

            # Each point x_i must be contained in at least one polytope
            prog.AddLinearConstraint(sum(z_i) >= 1)

            for j, poly in enumerate(c_free_polytopes):
                A_j = poly.A()
                b_j = poly.b()
                z_j = z_i[j]

                # Add constraints for each row of the polytope
                for k in range(A_j.shape[0]):
                    # Construct coefficient matrix [A_j_row | M]
                    coeffs = np.hstack([A_j[k], M])
                    # Combine x_i and z_j into variable vector
                    vars = np.hstack([x_i, [z_j]])
                    # A_j @ x_i + M * z[j] <= b_j + M
                    prog.AddLinearConstraint(
                        coeffs,
                        -np.inf,
                        b_j[k] + M,
                        vars
                    )

        # 8. Connected components constraints
        connected_components = self._compute_connected_components(P, G, c_free_polytopes)
        K = len(connected_components)

        # create binary variables for component membership (excluding init ang goal)
        z = {}
        for i in range (1, n_points-1): # exclude init and goal
            z[i] = prog.NewBinaryVariables(K, f"z_{i}")
            # Each point must belong to exactly one component
            prog.AddLinearConstraint(sum(z[i]) == 1)

        # Consecutive points must belong to the same component, ignoring init and goal
        for i in range(1, n_points-2, 2):
            for k in range(K):
                prog.AddLinearConstraint(z[i][k] == z[i+1][k])

        # Enforce that each component contains at least one point
        for i in range(1, n_points-1):
            x_i = x[self.q_dim*i:self.q_dim*(i+1)]
            z_i = z[i]

            for k, comp in enumerate(connected_components):
                A_k = comp.A()
                b_k = comp.b()
                z_k = z_i[k]

                for j in range(A_k.shape[0]):
                    coeffs = np.hstack([A_k[j], M])
                    vars = np.hstack([x_i, [z_k]])
                    prog.AddLinearConstraint(coeffs, -np.inf, b_k[j] + M, vars)

        # Solve the problem
        solver = MosekSolver()
        result = solver.Solve(prog)

        if not result.is_success():
            return None

        return result.GetSolution(x).reshape(-1, self.q_dim)

    @staticmethod
    def _to_quadratic_form(C, c):
        return np.dot(C.T, C), -np.dot(C.T, c).flatten()

    def _compute_connected_components(
        self,
        placement: HPolyhedron,
        grasp: HPolyhedron,
        c_free: list,
    ) -> list:
        """
        Computes convex connected components as:
        {placement ∩ grasp ∩ (union of c_free)}.
        Returns convex hulls of connected regions.
        """
        components = []

        # Compute intersections with each c_free polytope
        for poly in c_free:
            intersection = placement.Intersection(grasp).Intersection(poly)
            if not intersection.IsEmpty():
                components.append(intersection)

        # Merge overlapping components
        merged = []
        for comp in components:
            add_to_merged = False
            for m in merged:
                if comp.Intersection(m).IsEmpty():
                    continue
                # Merge overlapping components
                vertices_comp = self.visualizer.get_polytope_vertices(comp)
                vertices_m = self.visualizer.get_polytope_vertices(m)
                vertices = np.vstack([vertices_comp, vertices_m])
                merged_hull = ConvexHull(vertices)
                merged_poly = convex_hull_to_hpolyhedron(merged_hull)
                merged.remove(m)
                merged.append(merged_poly)
                add_to_merged = True
                break
            if not add_to_merged:
                merged.append(comp)

        return merged


    def _generate_cfree(self):
        generator = RandomGenerator(1234)
        checker = SceneGraphCollisionChecker(
            model=self.diagram,
            robot_model_instances=[self.plant.GetModelInstanceByName("robot")],
            edge_step_size=0.01,
        )
        options = IrisFromCliqueCoverOptions()
        options.num_points_per_visibility_round = 200
        options.coverage_termination_threshold = 0.99
        options.iris_options.configuration_space_margin = 0.00001
        # See https://github.com/RobotLocomotion/drake/issues/21343 -> If getting this issue reduce c-space margin above
        regions = IrisInConfigurationSpaceFromCliqueCover(checker, options, generator, [])
        return regions
