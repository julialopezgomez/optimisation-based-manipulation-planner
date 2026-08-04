import numpy as np
from pydrake.all import (HPolyhedron, AngleAxis,
                         VPolytope, Sphere, Ellipsoid, InverseKinematics,
                         RationalForwardKinematics, GeometrySet, Role,
                         RigidTransform, RotationMatrix,
                         Hyperellipsoid, Simulator, Box)
import mcubes

import visualization_utils as viz_utils

import pydrake.symbolic as sym
from pydrake.all import MeshcatVisualizer, StartMeshcat, DiagramBuilder, \
    AddMultibodyPlantSceneGraph, TriangleSurfaceMesh, Rgba, SurfaceTriangle, Sphere
from scipy.linalg import null_space
from scipy.spatial import ConvexHull
import time

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from ipywidgets import FloatSlider, VBox, HBox, Output
from IPython.display import display
import colorsys


class CIrisPlantVisualizer:
    def __init__(
            self,
            plant,
            builder,
            scene_graph,
            cspace_free_polytope,
            **kwargs):
        if plant.num_positions() > 3:
            if kwargs.get('allow_plus_3dof', False):
                print("Visualisations won't work properly. Can't visualize the TC-Space of plants with more than 3-DOF. The first 3 DOF from the plant will be visualized")
            else:
                raise ValueError(
                    "Can't visualize the TC-Space of plants with more than 3-DOF")
        
        

        # Create a meshcat visualizer for the tc-space
        # self.meshcat_cspace = StartMeshcat()
        # self.meshcat_cspace.Delete()
        # builder_cspace = DiagramBuilder()
        # plant_cspace, scene_graph_cspace = AddMultibodyPlantSceneGraph(
        #     builder_cspace, time_step=0.0)
        # plant_cspace.Finalize()

        # self.visualizer_cspace = MeshcatVisualizer.AddToBuilder(
        #     builder_cspace, scene_graph_cspace, self.meshcat_cspace)

        # Set up the plant, builder, scene_graph, and visualizer
        self.plant = plant
        # self.robot_diagram = builder.Build()
        self.builder = builder.builder()
        self.scene_graph = scene_graph
        self.viz_role = kwargs.get('viz_role', Role.kIllustration)
        
        # Create a meshcat visualizer for the task space
        self.meshcat_task_space = StartMeshcat()
        self.meshcat_task_space.Delete()
        self.visualizer_task_space = MeshcatVisualizer.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat_task_space)

        # Create the task space diagram and context
        # - a diagram in drake is a collection of connected systems
        # - the context is the state of the diagram, used in the meshcat to update the visualization
        self.task_space_diagram = builder.Build()
        self.task_space_diagram_context = self.task_space_diagram.CreateDefaultContext()

        # Create the c-space diagram and context
        # self.cspace_diagram = builder_cspace.Build()
        # self.cspace_diagram_context = self.cspace_diagram.CreateDefaultContext()


        self.plant_context = plant.GetMyMutableContextFromRoot(
            self.task_space_diagram_context)
        self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)
        self.simulator = Simulator(
            self.task_space_diagram,
            self.task_space_diagram_context)
        self.simulator.Initialize()

        self.cspace_free_polytope = cspace_free_polytope

        # SceneGraph inspectors for highlighting geometry pairs.
        self.model_inspector = self.scene_graph.model_inspector()
        self.query = self.scene_graph.get_query_output_port().Eval(
            self.scene_graph.GetMyContextFromRoot(self.task_space_diagram_context))

        # Construct Rational Forward Kinematics for easy conversions.
        self.rat_forward_kin = RationalForwardKinematics(plant)
        self.s_variables = sym.Variables(self.rat_forward_kin.s())
        self.s_array = self.rat_forward_kin.s()
        self.num_joints = self.plant.num_positions()

        # the point around which we construct the stereographic projection.
        self.q_star = kwargs.get('q_star', np.zeros(self.num_joints))

        # The lower and upper limits of the joint positions in the plant.
        self.q_lower_limits = plant.GetPositionLowerLimits()
        self.s_lower_limits = self.rat_forward_kin.ComputeSValue(
            self.q_lower_limits, self.q_star)
        self.q_upper_limits = plant.GetPositionUpperLimits()
        self.s_upper_limits = self.rat_forward_kin.ComputeSValue(
            self.q_upper_limits, self.q_star)

        # A dictionary mapping str -> (HPolyhedron, SearchResult, Color) where
        # SearchResult can be None. This is used for visualizing cspace regions
        # and their certificates in task space.
        self.region_certificate_groups = {}

        # Set up the IK object to enable visualization of the collision
        # constraint.
        self.ik = InverseKinematics(plant, self.plant_context)
        min_dist = 1e-5
        self.collision_constraint = self.ik.AddMinimumDistanceLowerBoundConstraint(
            min_dist, 1e-5)

        # The plane numbers which we wish to visualize.
        self._plane_indices_of_interest = []
        self.plane_indices = np.arange(
            0, len(cspace_free_polytope.separating_planes()))


    def clear_plane_indices_of_interest(self):
        self._plane_indices_of_interest = []
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    def add_plane_indices_of_interest(self, *elts):
        for e in elts:
            if e not in self._plane_indices_of_interest:
                self._plane_indices_of_interest.append(e)
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    def remove_plane_indices_of_interest(self, *elts):
        self._plane_indices_of_interest[:] = (
            e for e in self._plane_indices_of_interest if e not in elts)
        cur_q = self.plant.GetPositions(self.plant_context)
        self.show_res_q(cur_q)

    #     visualizer.update_certificates(s)

    def show_res_q(self, q):
        self.plant.SetPositions(self.plant_context, q)
        in_collision = self.check_collision_q_by_ik(q)
        s = self.rat_forward_kin.ComputeSValue(np.array(q), self.q_star)

        color = Rgba(1, 0.72, 0, 1) if in_collision else Rgba(0.24, 1, 0, 1)
        self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)

        self.plot_cspace_points(s, name='/s', color=color, radius=0.05)

        self.update_certificates(s)

    def show_res_s(self, s):
        q = self.rat_forward_kin.ComputeQValue(np.array(s), self.q_star)
        self.show_res_q(q)

    '''
    Check if the configuration q is in collision.
    '''
    def check_collision_q_by_ik(self, q, min_dist=1e-5):
        if np.all(q >= self.q_lower_limits) and \
                np.all(q <= self.q_upper_limits):
            return 1 - 1 * \
                float(self.collision_constraint.evaluator().CheckSatisfied(q, min_dist))
        else:
            return 1

    def check_collision_s_by_ik(self, s, min_dist=1e-5, third=None, fourth=None):
        s = np.array(s)
        q = self.rat_forward_kin.ComputeQValue(s, self.q_star)
        return self.check_collision_q_by_ik(q, min_dist)

    # def visualize_collision_constraint(self, **kwargs):
    #     if self.plant.num_positions() >= 3:
    #         self._visualize_collision_constraint3d(**kwargs)
    #     else:
    #         self._visualize_collision_constraint2d(**kwargs)
    

    def visualize_collision_constraint(self, **kwargs):
        if self.plant.num_positions() >= 3:
            self._visualize_collision_constraint3d_plotly(**kwargs)
        else:
            self._visualize_collision_constraint2d_plotly(**kwargs)

    def _visualize_collision_constraint3d_plotly(
            self,
            num_points=50,
            factor=2,
            qs: list[np.ndarray] = None,  # List of configurations
            paths: list[list[np.ndarray]] = None,  # List of paths (each path is a list of configurations)
            filled_polytopes: list[HPolyhedron] = None,
            filled_s_polytopes: list[HPolyhedron] = None,
            wireframe_polytopes: list[HPolyhedron] = None):
        """
        Visualize the collision constraint in 3D using Plotly.
        :param num_points: Density of the marching cubes grid. Runtime scales cubically in N.
        :param factor: Scaling factor for the grid limits.
        :param qs: List of configurations to plot. Each configuration should be a numpy array of joint positions.
        :param paths: List of paths to plot. Each path is a list of configurations.
        :param filled_polytopes: List of polytopes to plot as filled volumes.
        :param wireframe_polytopes: List of polytopes to plot as wireframes.
        :return: Plotly figure object.
        """
        # Assert that all configurations have the correct number of joints
        if qs is not None:
            for q in qs:
                assert len(q) == 3, f"Configuration must have 3 joints."
                assert np.all(q >= self.q_lower_limits) \
                    and np.all(q <= self.q_upper_limits), \
                    f"Configuration {q} is out of bounds."
                    
        if paths is not None:
            for path in paths:
                for q in path:
                    assert len(q) == 3, f"All configurations in path must have 3 joints."
                    assert np.all(q >= self.q_lower_limits) \
                        and np.all(q <= self.q_upper_limits), \
                        f"Configuration {q} is out of bounds. Path: {path}"

        # Generate the grid for q visualization (3D)
        q0 = np.linspace(
            factor * self.q_lower_limits[0],
            factor * self.q_upper_limits[0],
            num_points)
        q1 = np.linspace(
            factor * self.q_lower_limits[1],
            factor * self.q_upper_limits[1],
            num_points)
        q2 = np.linspace(
            factor * self.q_lower_limits[2],
            factor * self.q_upper_limits[2],
            num_points)

        Q0, Q1, Q2 = np.meshgrid(q0, q1, q2, indexing="ij")
        Z_q = np.zeros_like(Q0)

        # Populate the collision grid for q-space
        for i in range(num_points):
            for j in range(num_points):
                for k in range(num_points):
                    q = np.array([Q0[i, j, k], Q1[i, j, k], Q2[i, j, k]])
                    Z_q[i, j, k] = self.check_collision_q_by_ik(q)

        # Generate the grid for s visualization (3D)
        s0 = np.linspace(
            factor * self.s_lower_limits[0],
            factor * self.s_upper_limits[0],
            num_points)
        s1 = np.linspace(
            factor * self.s_lower_limits[1],
            factor * self.s_upper_limits[1],
            num_points)
        s2 = np.linspace(
            factor * self.s_lower_limits[2],
            factor * self.s_upper_limits[2],
            num_points)

        S0, S1, S2 = np.meshgrid(s2, s0, s1, indexing="ij")
        Z_s = np.zeros_like(S0)  # Initialize the collision grid for s-space

        # Populate the collision grid for s-space
        for i in range(num_points):
            for j in range(num_points):
                for k in range(num_points):
                    s = np.array([S1[i, j, k], S2[i, j, k], S0[i, j, k]])
                    Z_s[i, j, k] = self.check_collision_s_by_ik(s)

        # Create the Plotly 3D surface plot
        # Create the q-space plot
        fig_q = go.Figure(
            data=[
                go.Volume(
                    x=Q0.flatten(),
                    y=Q1.flatten(),
                    z=Q2.flatten(),
                    value=Z_q.flatten(),
                    opacity=0.8,  # Adjust transparency for better visibility
                    colorscale=[
                        [0, "rgba(255,255,255,0)"],  # Free space
                        [1, "red"]     # Collision space
                    ],
                    showscale=False
                )
            ]
        )

        # Create the s-space plot
        fig_s = go.Figure(
            data=[
                go.Volume(
                    x=S0.flatten(),
                    y=S1.flatten(),
                    z=S2.flatten(),
                    value=Z_s.flatten(),
                    opacity=0.8,
                    colorscale=[
                        [0, "rgba(255,255,255,0)"],  # Free space
                        [1, "red"]     # Collision space
                    ],
                    showscale=False
                )
            ]
        )

        # If configurations are provided, plot them in both spaces
        if qs is not None:
            for i, q in enumerate(qs):
                # Check if the configuration is in collision
                in_collision = self.check_collision_q_by_ik(q)
                marker_color = "orange" if in_collision else "green"

                # Compute s values from the configuration
                s_values = self.rat_forward_kin.ComputeSValue(q, self.q_star)

                # Add marker to q-space plot
                fig_q.add_trace(go.Scatter3d(
                    x=[q[0]],
                    y=[q[1]],
                    z=[q[2]],
                    mode="markers",
                    marker=dict(size=8, color=marker_color),
                    name=f"Configuration {i}"
                ))

                # Add marker to s-space plot
                fig_s.add_trace(go.Scatter3d(
                    x=[s_values[2]],  # s2
                    y=[s_values[0]],  # s0
                    z=[s_values[1]],  # s1
                    mode="markers",
                    marker=dict(size=8, color=marker_color),
                    name=f"Configuration {i}"
                ))

        # If paths are provided, plot them in both spaces
        if paths is not None:
            for i, path in enumerate(paths):
                # Extract q and s values for the path
                q_path = np.array(path)
                s_path = np.array([self.rat_forward_kin.ComputeSValue(q, self.q_star) for q in path])

                # Add path to q-space plot
                fig_q.add_trace(go.Scatter3d(
                    x=q_path[:, 0],
                    y=q_path[:, 1],
                    z=q_path[:, 2],
                    mode="lines",
                    line=dict(color=f"hsl({(i+1) * 60}, 100%, 50%)", width=4),
                    marker=dict(size=4),
                    name=f"Path {i}"
                ))

                # Add path to s-space plot
                fig_s.add_trace(go.Scatter3d(
                    x=s_path[:, 2],  # s2
                    y=s_path[:, 0],  # s0
                    z=s_path[:, 1],  # s1
                    mode="lines",
                    line=dict(color=f"hsl({(i+1) * 60}, 100%, 50%)", width=4),
                    marker=dict(size=4),
                    name=f"Path {i}"
                ))

        # Update layout for better readability
        for fig in [fig_q, fig_s]:
            fig.update_layout(
                autosize=True,
                margin=dict(l=50, r=50, b=50, t=50)
            )

        # Add filled polytopes to the plot
        if filled_polytopes is not None:
            for i, polytope in enumerate(filled_polytopes):
                self.plot_polytope_3d(polytope, fig_q, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {i}")
                self.plot_polytope_3d(polytope, fig_s, isS=True, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {i}")

        if filled_s_polytopes is not None:
            for j, polytope in enumerate(filled_s_polytopes):
                i += j
                self.plot_s_polytope_3d(polytope, fig_q, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {j}")
                self.plot_s_polytope_3d(polytope, fig_s, isS=True, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {j}")
            
        # Add wireframe polytopes to the plot
        if wireframe_polytopes is not None:
            for i, polytope in enumerate(wireframe_polytopes):
                self.plot_polytope_3d(polytope, fig_q, wireframe=True, color=f"hsl({(i+3) * 75}, 100%, 50%)", name=f"Wireframe Polytope {i}")
                self.plot_polytope_3d(polytope, fig_s, isS=True, wireframe=True, color=f"hsl({(i+3) * 75}, 100%, 50%)", name=f"Wireframe Polytope {i}")

        # Combine the figures and show
        fig = make_subplots(rows=1, cols=2, specs=[[{'type': 'scene'}, {'type': 'scene'}]],
                            subplot_titles=("C-Space Collision Constraint", "TC-Space Collision Constraint"))
        for trace in fig_q.data:
            fig.add_trace(trace, row=1, col=1)
        for trace in fig_s.data:
            fig.add_trace(trace, row=1, col=2)

        # Update layout for the first subplot (C-Space)
        fig.update_scenes(
            xaxis_title="q0", yaxis_title="q1", zaxis_title="q2",
            row=1, col=1
        )

        # Update layout for the second subplot (TC-Space)
        fig.update_scenes(
            xaxis_title="s0", yaxis_title="s1", zaxis_title="s2",
            row=1, col=2
        )

        # Update the overall layout
        fig.update_layout(
            title_text="Collision Constraint Visualization",
            showlegend=False,
            width=1000,  # Adjust width to fit both plots
            height=500   # Adjust height as needed
        )

        # Show the combined figure
        fig.show()

    def _visualize_collision_constraint2d_plotly(
            self, 
            factor=2, 
            num_points=20, 
            config=None,
            wireframe_polytopes: list[HPolyhedron] = None,
            filled_polytopes: list[HPolyhedron] = None,
            filled_s_polytopes: list[HPolyhedron] = None
            ):
        """
        Visualize the 2D collision constraint in both C-space and TC-space with interactive sliders.
        :param factor: Scaling factor for the grid limits.
        :param num_points: Number of points along each axis.
        :param config: Optional configuration to plot. Should be a numpy array of joint positions.
        """
        # Assert that the configuration has the correct number of joints
        if config is not None:
            assert len(config) == 2, \
                f"Configuration must have 2 joints."

        # Generate the grid for q visualization
        q0 = np.linspace(
            factor * self.q_lower_limits[0],
            factor * self.q_upper_limits[0],
            num_points)
        q1 = np.linspace(
            factor * self.q_lower_limits[1],
            factor * self.q_upper_limits[1],
            num_points)
        X_q, Y_q = np.meshgrid(q0, q1)
        Z_q = np.zeros_like(X_q)

        # Populate the collision grid for q
        for i in range(num_points):
            for j in range(num_points):
                Z_q[i, j] = self.check_collision_q_by_ik(
                    np.array([X_q[i, j], Y_q[i, j]]))

        # Generate the grid for s visualization
        s0 = np.linspace(
            factor * self.s_lower_limits[0],
            factor * self.s_upper_limits[0],
            num_points)
        s1 = np.linspace(
            factor * self.s_lower_limits[1],
            factor * self.s_upper_limits[1],
            num_points)
        X_s, Y_s = np.meshgrid(s1, s0)  # Invert the axes for s-space
        Z_s = np.zeros_like(X_s)

        # Populate the collision grid for s
        for i in range(num_points):
            for j in range(num_points):
                Z_s[i, j] = self.check_collision_s_by_ik(
                    np.array([Y_s[i, j], X_s[i, j]]))  # Note the inversion of s0 and s1

        # Create the q-space plot
        fig_q = go.Figure(
            data=[
                go.Heatmap(
                    z=Z_q,
                    x=q0,
                    y=q1,
                    colorscale=[
                        [0, 'white'],
                        [1, 'red']
                    ],
                    showscale=False
                )
            ]
        )

        # Create the s-space plot
        fig_s = go.Figure(
            data=[
                go.Heatmap(
                    z=Z_s,
                    x=s1,  # Horizontal axis for s1
                    y=s0,  # Vertical axis for s0
                    colorscale=[
                        [0, 'white'],
                        [1, 'red']
                    ],
                    showscale=False
                )
            ]
        )

        # If a configuration is provided, plot the point/marker in both plots
        if config is not None:
            # Check if the configuration is in collision
            in_collision = self.check_collision_q_by_ik(config)
            marker_color = "orange" if in_collision else "green"

            # Compute s values from the configuration
            s_values = self.rat_forward_kin.ComputeSValue(config, self.q_star)

            # Add marker to q-space plot
            fig_q.add_trace(go.Scatter(
                x=[config[0]],
                y=[config[1]],
                mode="markers",
                marker=dict(size=12, color=marker_color),
                name="q"
            ))

            # Add marker to s-space plot
            fig_s.add_trace(go.Scatter(
                x=[s_values[1]],  # s1
                y=[s_values[0]],  # s0
                mode="markers",
                marker=dict(size=12, color=marker_color),
                name="s"
            ))

        # Update layout for better readability
        for fig in [fig_q, fig_s]:
            fig.update_layout(
                autosize=True,
                margin=dict(l=50, r=50, b=50, t=50)
            )
        
        # Add filled polytopes to the plot
        if filled_polytopes is not None:
            for i, polytope in enumerate(filled_polytopes):
                self.plot_polytope_2d(polytope, fig_q, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {i}")
                self.plot_polytope_2d(polytope, fig_s, isS=True, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {i}")
        if filled_s_polytopes is not None:
            for j, polytope in enumerate(filled_s_polytopes):
                i += j
                self.plot_s_polytope_2d(polytope, fig_q, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {j}")
                self.plot_s_polytope_2d(polytope, fig_s, isS=True, wireframe=False, color=f"hsl({(i+1) * 60}, 100%, 50%)", name=f"Filled Polytope {j}")

        # Add wireframe polytopes to the plot
        if wireframe_polytopes is not None:
            for i, polytope in enumerate(wireframe_polytopes):
                self.plot_polytope_2d(polytope, fig_q, wireframe=True, color=f"hsl({(i+3) * 60}, 100%, 50%)", name=f"Wireframe Polytope {i}")
                self.plot_polytope_2d(polytope, fig_s, isS=True, wireframe=True, color=f"hsl({(i+3) * 60}, 100%, 50%)", name=f"Wireframe Polytope {i}")
        # Combine the two figures into a single figure with subplots
        fig = make_subplots(rows=1, cols=2, subplot_titles=("C-Space Collision Constraint", "TC-Space Collision Constraint"))

        # Add traces from fig_q and fig_s to the combined figure
        for trace in fig_q.data:
            fig.add_trace(trace, row=1, col=1)
        for trace in fig_s.data:
            fig.add_trace(trace, row=1, col=2)

        # Update layout for the first subplot (C-Space)
        fig.update_xaxes(title_text="q0", row=1, col=1)
        fig.update_yaxes(title_text="q1", row=1, col=1)

        # Update layout for the second subplot (TC-Space)
        fig.update_xaxes(title_text="s0", row=1, col=2)
        fig.update_yaxes(title_text="s1", row=1, col=2)

        # Update the overall layout
        fig.update_layout(
            title_text="Collision Constraint Visualization",
            showlegend=False,
            width=1000,  # Adjust width to fit both plots
            height=500   # Adjust height as needed
        )

        # Show the combined figure
        fig.show()
        
    def plot_polytope_2d(
            self, 
            polytope: HPolyhedron, 
            fig: go.Figure, 
            isS: bool = False,
            wireframe: bool = False,
            color: str = 'blue', 
            name: str = 'Polytope'):
        
        # Get vertices and edges
        vertices = self.get_polytope_vertices(polytope)
        
        if isS:
            # turn all vertices coordinates to TC-space
            vertices = [self.rat_forward_kin.ComputeSValue(v, self.q_star) for v in vertices]
            vertices = np.array(vertices)
            vertices = np.array([vertices[:,1], vertices[:,0]]).T
        
        
        if wireframe:
            edges = self.get_polytope_edges(vertices)
            
            # Extract x, y, z coordinates for the edges
            x_lines = []
            y_lines = []
            for edge in edges:
                x_lines.extend([vertices[edge[0]][0], vertices[edge[1]][0], None])
                y_lines.extend([vertices[edge[0]][1], vertices[edge[1]][1], None])
            
            # Add the wireframe to the figure
            fig.add_trace(go.Scatter(
                x=x_lines,
                y=y_lines,
                mode="lines",
                line=dict(color=color, width=2),
                name=name
            ))
        else:
            
            hull = ConvexHull(vertices)
            vertices = vertices[hull.vertices]
            # Close the polygon by repeating the first vertex
            x = vertices[:, 0].tolist() + [vertices[0, 0]]
            y = vertices[:, 1].tolist() + [vertices[0, 1]]
        
            h, s, l = map(float, color[4:-1].replace("%", "").split(","))
            r, g, b = colorsys.hls_to_rgb(h / 360, l / 100, s / 100)
            r, g, b = int(r * 255), int(g * 255), int(b * 255)
            color = f"rgba({r}, {g}, {b}, {0.2})"
            
            # Add the filled polygon to the figure
            fig.add_trace(go.Scatter(
                x=x,
                y=y,
                mode="lines",
                fill="toself",  # Fill the polygon
                line=dict(color=color, width=2),
                fillcolor=color,  # Set fill color with opacity
                name=name
            ))
            
    def plot_polytope_3d(
            self,
            polytope: HPolyhedron,
            fig: go.Figure,
            isS: bool = False,
            wireframe: bool = False,
            color: str = 'blue',
            name: str = 'Polytope'):
        
        # Get vertices and edges
        vertices = self.get_polytope_vertices(polytope)
        
        if isS:
            # turn all vertices coordinates to TC-space
            vertices = [self.rat_forward_kin.ComputeSValue(v, self.q_star) for v in vertices]
            vertices = np.array(vertices)
            vertices = np.array([vertices[:,2], vertices[:,0], vertices[:,1]]).T
            
            #swap columns 0,1,2 to 2,0,1
        
        if wireframe:
        
            edges = self.get_polytope_edges(vertices)
            
            # Extract x, y, z coordinates for the edges
            x_lines = []
            y_lines = []
            z_lines = []
            for edge in edges:
                x_lines.extend([vertices[edge[0]][0], vertices[edge[1]][0], None])
                y_lines.extend([vertices[edge[0]][1], vertices[edge[1]][1], None])
                z_lines.extend([vertices[edge[0]][2], vertices[edge[1]][2], None])
                
            # Add the wireframe to the figure
            fig.add_trace(go.Scatter3d(
                x=x_lines,
                y=y_lines,
                z=z_lines,
                mode="lines",
                line=dict(color=color, width=2),
                name=name
            ))
            
        else:
            
            # Get faces for volume rendering
            hull = ConvexHull(vertices)
            faces = hull.simplices  # Faces are represented as triangles (indices of vertices)
            
            # Extract x, y, z coordinates of the vertices
            x = vertices[:, 0]
            y = vertices[:, 1]
            z = vertices[:, 2]
            
            # Add the volume to the figure
            fig.add_trace(go.Mesh3d(
                x=x,
                y=y,
                z=z,
                i=faces[:, 0],  # Indices of the first vertex of each triangle
                j=faces[:, 1],  # Indices of the second vertex of each triangle
                k=faces[:, 2],  # Indices of the third vertex of each triangle
                opacity=0.2,  # Set opacity for the volume
                color=color,      # Set color for the volume
                name=name
            ))
    
    def plot_s_polytope_2d(
            self, 
            polytope: HPolyhedron, 
            fig: go.Figure, 
            isS: bool = False,
            wireframe: bool = False,
            color: str = 'blue', 
            name: str = 'Polytope'):
        
        # Get vertices and edges
        vertices = self.get_polytope_vertices(polytope)
        
        if not isS:
            # turn all vertices coordinates to TC-space
            vertices = [self.rat_forward_kin.ComputeQValue(v, self.q_star) for v in vertices]
            vertices = np.array(vertices)
            # vertices = np.array([vertices[:,1], vertices[:,0]]).T
        
        else:
            vertices = np.array([vertices[:,1], vertices[:,0]]).T
        
        
        if wireframe:
            edges = self.get_polytope_edges(vertices)
            
            # Extract x, y, z coordinates for the edges
            x_lines = []
            y_lines = []
            for edge in edges:
                x_lines.extend([vertices[edge[0]][0], vertices[edge[1]][0], None])
                y_lines.extend([vertices[edge[0]][1], vertices[edge[1]][1], None])
            
            # Add the wireframe to the figure
            fig.add_trace(go.Scatter(
                x=x_lines,
                y=y_lines,
                mode="lines",
                line=dict(color=color, width=2),
                name=name
            ))
        else:
            
            hull = ConvexHull(vertices)
            vertices = vertices[hull.vertices]
            # Close the polygon by repeating the first vertex
            x = vertices[:, 0].tolist() + [vertices[0, 0]]
            y = vertices[:, 1].tolist() + [vertices[0, 1]]
        
            h, s, l = map(float, color[4:-1].replace("%", "").split(","))
            r, g, b = colorsys.hls_to_rgb(h / 360, l / 100, s / 100)
            r, g, b = int(r * 255), int(g * 255), int(b * 255)
            color = f"rgba({r}, {g}, {b}, {0.2})"
            
            # Add the filled polygon to the figure
            fig.add_trace(go.Scatter(
                x=x,
                y=y,
                mode="lines",
                fill="toself",  # Fill the polygon
                line=dict(color=color, width=2),
                fillcolor=color,  # Set fill color with opacity
                name=name
            ))
            
    def plot_s_polytope_3d(
            self,
            polytope: HPolyhedron,
            fig: go.Figure,
            isS: bool = False,
            wireframe: bool = False,
            color: str = 'blue',
            name: str = 'Polytope'):
        
        # Get vertices and edges
        vertices = self.get_polytope_vertices(polytope)
        
        if not isS:
            # turn all vertices coordinates to TC-space
            vertices = [self.rat_forward_kin.ComputeQValue(v, self.q_star) for v in vertices]
            vertices = np.array(vertices)
            # vertices = np.array([vertices[:,2], vertices[:,0], vertices[:,1]]).T
            
        else:
            vertices = np.array([vertices[:,2], vertices[:,0], vertices[:,1]]).T
            
            
            #swap columns 0,1,2 to 2,0,1
        
        if wireframe:
        
            edges = self.get_polytope_edges(vertices)
            
            # Extract x, y, z coordinates for the edges
            x_lines = []
            y_lines = []
            z_lines = []
            for edge in edges:
                x_lines.extend([vertices[edge[0]][0], vertices[edge[1]][0], None])
                y_lines.extend([vertices[edge[0]][1], vertices[edge[1]][1], None])
                z_lines.extend([vertices[edge[0]][2], vertices[edge[1]][2], None])
                
            # Add the wireframe to the figure
            fig.add_trace(go.Scatter3d(
                x=x_lines,
                y=y_lines,
                z=z_lines,
                mode="lines",
                line=dict(color=color, width=2),
                name=name
            ))
            
        else:
            
            # Get faces for volume rendering
            hull = ConvexHull(vertices)
            faces = hull.simplices  # Faces are represented as triangles (indices of vertices)
            
            # Extract x, y, z coordinates of the vertices
            x = vertices[:, 0]
            y = vertices[:, 1]
            z = vertices[:, 2]
            
            # Add the volume to the figure
            fig.add_trace(go.Mesh3d(
                x=x,
                y=y,
                z=z,
                i=faces[:, 0],  # Indices of the first vertex of each triangle
                j=faces[:, 1],  # Indices of the second vertex of each triangle
                k=faces[:, 2],  # Indices of the third vertex of each triangle
                opacity=0.2,  # Set opacity for the volume
                color=color,      # Set color for the volume
                name=name
            ))
            
        
    def get_polytope_vertices(self, polytope: HPolyhedron):
        return VPolytope(polytope).vertices().T 
    
    def get_polytope_edges(self, vertices):
        hull = ConvexHull(vertices)
        edges = set()
        for simplex in hull.simplices:
            for i in range(len(simplex)):
                edges.add((simplex[i], simplex[(i + 1) % len(simplex)]))
        return list(edges)
        

    

    def update_region_visualization_by_group_name(self, name, **kwargs):
        region_and_certificates_list = self.region_certificate_groups[name]
        for i, (r, _, color) in enumerate(region_and_certificates_list):
            viz_utils.plot_polytope(r, self.meshcat_cspace, f"/{name}/{i}",
                                    resolution=kwargs.get("resolution", 30),
                                    color=color,
                                    wireframe=kwargs.get("wireframe", True),
                                    random_color_opacity=kwargs.get("random_color_opacity", 0.7),
                                    fill=kwargs.get("fill", True),
                                    line_width=kwargs.get("line_width", 10))

    def update_region_visualization(self, **kwargs):
        for name in self.region_certificate_groups.keys():
            self.update_region_visualization_by_group_name(name, **kwargs)

    def add_group_of_regions_to_visualization(
            self, region_color_tuples, group_name, **kwargs):
        # **kwargs are the ones for viz_utils.plot_polytopes
        self.region_certificate_groups[group_name] = [
            (region, None, color) for (
                region, color) in region_color_tuples]
        self.update_region_visualization_by_group_name(group_name, **kwargs)

    def add_group_of_regions_and_certs_to_visualization(
            self, region_cert_color_tuples, group_name, **kwargs):
        # **kwargs are the ones for viz_utils.plot_polytopes
        # each element of region_and_certs_list is an (HPolyhedron,
        # SearchResult)
        self.region_certificate_groups[group_name] = region_cert_color_tuples
        self.update_region_visualization_by_group_name(group_name, **kwargs)

    def plot_cspace_points(self, points, name, **kwargs):
        if len(points.shape) == 1:
            viz_utils.plot_point(points, self.meshcat_cspace, name, **kwargs)
        else:
            for i, s in enumerate(points):
                viz_utils.plot_point(
                    s, self.meshcat_cspace, name + f"/{i}", **kwargs)

    def highlight_geometry_id(self, geom_id, color, name=None):
        if name is None:
            name = f"/id_{geom_id}"
        shape = self.model_inspector.GetShape(geom_id)
        X_WG = self.get_geom_id_pose_in_world(geom_id)
        self.meshcat_task_space.SetObject(name, shape, color)
        self.meshcat_task_space.SetTransform(name, X_WG)

    def get_geom_id_pose_in_world(self, geom_id):
        frame_id = self.model_inspector.GetFrameId(geom_id)
        X_FG = self.model_inspector.GetPoseInFrame(geom_id)
        X_WF = self.query.GetPoseInWorld(frame_id)
        return X_WF @ X_FG

    def plot_plane_by_index_at_s(
            self,
            s,
            plane_index,
            search_result,
            color,
            name_prefix=""):
        name = name_prefix + f"/plane_{plane_index}"
        sep_plane = self.cspace_free_polytope.separating_planes()[plane_index]

        geom1, geom2 = sep_plane.positive_side_geometry.id(),\
            sep_plane.negative_side_geometry.id()

        # highlight the geometry
        self.highlight_geometry_id(geom1, color, name + f"/{geom1}")
        self.highlight_geometry_id(geom2, color, name + f"/{geom2}")

        env = {var_s: val_s for var_s, val_s in zip(
            self.cspace_free_polytope.rational_rat_forward_kin().s(), s)}

        a = np.array([a_poly.Evaluate(env)
                     for a_poly in search_result.a[plane_index]])
        b = search_result.b[plane_index].Evaluate(env)

        expressed_body = self.plant.get_body(sep_plane.expressed_body)
        X_WE = self.plant.EvalBodyPoseInWorld(
            self.plant_context, expressed_body)
        X_EW = X_WE.inverse()
        X_WG1 = self.get_geom_id_pose_in_world(geom1)
        X_WG2 = self.get_geom_id_pose_in_world(geom2)
        p1 = (X_EW @ X_WG1).translation()
        p2 = (X_EW @ X_WG2).translation()

        mu = -b / (a.T @ (p2 - p1))
        offset = mu * (p2 - p1)
        axis = (a / np.linalg.norm(a))[:, np.newaxis]
        P = null_space(axis.T)
        R = np.hstack([P, axis])
        R = RotationMatrix(R)
        X_E_plane = RigidTransform(R, offset)

        self.meshcat_task_space.SetObject(name + "/plane",
                                          Box(5, 5, 0.02),
                                          Rgba(color.r(), color.g(), color.b(), 0.5))
        self.meshcat_task_space.SetTransform(name + "/plane", X_WE @ X_E_plane)

    def update_certificates(self, s):
        for group_name, region_and_cert_list in self.region_certificate_groups.items():
            for i, (region, search_result, color) in enumerate(
                    region_and_cert_list):
                plane_color = Rgba(color.r(), color.g(), color.b(), 1) if color is not None else None
                name_prefix = f"/{group_name}/region_{i}"
                if region.PointInSet(s) and search_result is not None:
                    for plane_index in self.plane_indices:
                        if plane_index in self._plane_indices_of_interest:
                            self.plot_plane_by_index_at_s(
                                s, plane_index, search_result, plane_color, name_prefix=name_prefix)
                        else:
                            self.meshcat_task_space.Delete(
                                name_prefix + f"/plane_{plane_index}")
                else:
                    self.meshcat_task_space.Delete(name_prefix)

    def animate_traj_s(self, traj, steps, runtime, idx_list = None, sleep_time = 0.1):
        # loop
        idx = 0
        going_fwd = True
        time_points = np.linspace(0, traj.end_time(), steps)
        frame_count = 0
        for _ in range(runtime):
            # print(idx)
            t0 = time.time()
            s = traj.value(time_points[idx])
            self.show_res_s(s)
            self.task_space_diagram_context.SetTime(frame_count * 0.01)
            self.task_space_diagram.ForcedPublish(self.task_space_diagram_context)
            self.cspace_diagram_context.SetTime(frame_count * 0.01)
            self.cspace_diagram.ForcedPublish(self.cspace_diagram_context)
            frame_count += 1
            if going_fwd:
                if idx + 1 < steps:
                    idx += 1
                else:
                    going_fwd = False
                    idx -= 1
            else:
                if idx - 1 >= 0:
                    idx -= 1
                else:
                    going_fwd = True
                    idx += 1
            t1 = time.time()
            pause = sleep_time - (t1 - t0)
            if pause > 0:
                time.sleep(pause)

    def save_meshcats(self, filename_prefix):
        with open(filename_prefix + "_cspace.html", "w") as f:
            f.write(self.meshcat_cspace.StaticHtml())
        with open(filename_prefix + "_task_space.html", "w") as f:
            f.write(self.meshcat_task_space.StaticHtml())