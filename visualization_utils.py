import time
import numpy as np
from functools import partial
from pydrake.all import (MathematicalProgram, le, SnoptSolver,
                         SurfaceTriangle, TriangleSurfaceMesh,
                         VPolytope, HPolyhedron, Sphere, RigidTransform,
                         RotationMatrix, Rgba)
from pydrake.common import RandomGenerator
import mcubes
from scipy.spatial import ConvexHull
from scipy.linalg import block_diag
from fractions import Fraction
import itertools
import random
import colorsys
from pydrake.all import PiecewisePolynomial



def plot_surface(meshcat_instance,
                 path,
                 X,
                 Y,
                 Z,
                 rgba=Rgba(.87, .6, .6, 1.0),
                 wireframe=False,
                 wireframe_line_width=1.0):
    # taken from
    # https://github.com/RussTedrake/manipulation/blob/346038d7fb3b18d439a88be6ed731c6bf19b43de/manipulation/meshcat_cpp_utils.py#L415
    (rows, cols) = Z.shape
    assert (np.array_equal(X.shape, Y.shape))
    assert (np.array_equal(X.shape, Z.shape))

    vertices = np.empty((rows * cols, 3), dtype=np.float32)
    vertices[:, 0] = X.reshape((-1))
    vertices[:, 1] = Y.reshape((-1))
    vertices[:, 2] = Z.reshape((-1))

    # Vectorized faces code from https://stackoverflow.com/questions/44934631/making-grid-triangular-mesh-quickly-with-numpy  # noqa
    faces = np.empty((rows - 1, cols - 1, 2, 3), dtype=np.uint32)
    r = np.arange(rows * cols).reshape(rows, cols)
    faces[:, :, 0, 0] = r[:-1, :-1]
    faces[:, :, 1, 0] = r[:-1, 1:]
    faces[:, :, 0, 1] = r[:-1, 1:]
    faces[:, :, 1, 1] = r[1:, 1:]
    faces[:, :, :, 2] = r[1:, :-1, None]
    faces.shape = (-1, 3)

    # TODO(Russ): support per vertex / Colormap colors.
    meshcat_instance.SetTriangleMesh(
        path,
        vertices.T,
        faces.T,
        rgba,
        wireframe,
        wireframe_line_width)


def plot_point(point, meshcat_instance, name,
               color=Rgba(0.06, 0.0, 0, 1), radius=0.01):
    meshcat_instance.SetObject(name,
                               Sphere(radius),
                               color)
    meshcat_instance.SetTransform(name, RigidTransform(
        RotationMatrix(), stretch_array_to_3d(point)))


def plot_polytope(polytope, meshcat_instance, name,
                  resolution=50, color=None,
                  wireframe=True,
                  random_color_opacity=0.2,
                  fill=True,
                  line_width=10):
    if color is None:
        color = Rgba(*np.random.rand(3), random_color_opacity)
    if polytope.ambient_dimension == 3:
        verts, triangles = get_plot_poly_mesh(polytope,
                                              resolution=resolution)
        meshcat_instance.SetObject(name, TriangleSurfaceMesh(triangles, verts),
                                   color, wireframe=wireframe)

    else:
        plot_hpoly2d(polytope, meshcat_instance, name,
                     color,
                     line_width=line_width,
                     fill=fill,
                     resolution=resolution,
                     wireframe=wireframe)


def plot_hpoly2d(polytope, meshcat_instance, name,
                 color,
                 line_width=8,
                 fill=False,
                 resolution=30,
                 wireframe=True):
    # plot boundary
    vpoly = VPolytope(polytope)
    verts = vpoly.vertices()
    hull = ConvexHull(verts.T)
    inds = np.append(hull.vertices, hull.vertices[0])
    hull_drake = verts.T[inds, :].T
    hull_drake3d = np.vstack([hull_drake, np.zeros(hull_drake.shape[1])])
    color_RGB = Rgba(color.r(), color.g(), color.b(), 1)
    meshcat_instance.SetLine(name, hull_drake3d,
                             line_width=line_width, rgba=color_RGB)
    if fill:
        width = 0.5
        C = block_diag(polytope.A(), np.array([-1, 1])[:, np.newaxis])
        d = np.append(polytope.b(), width * np.ones(2))
        hpoly_3d = HPolyhedron(C, d)
        verts, triangles = get_plot_poly_mesh(hpoly_3d,
                                              resolution=resolution)
        meshcat_instance.SetObject(name + "/fill",
                                   TriangleSurfaceMesh(triangles, verts),
                                   color, wireframe=wireframe)


def get_plot_poly_mesh(polytope, resolution):
    def inpolycheck(q0, q1, q2, A, b):
        q = np.array([q0, q1, q2])
        res = np.min(1.0 * (A @ q - b <= 0))
        return res

    aabb_max, aabb_min = get_AABB_limits(polytope)

    col_hand = partial(inpolycheck, A=polytope.A(), b=polytope.b())
    vertices, triangles = mcubes.marching_cubes_func(tuple(aabb_min),
                                                     tuple(aabb_max),
                                                     resolution,
                                                     resolution,
                                                     resolution,
                                                     col_hand,
                                                     0.5)
    tri_drake = [SurfaceTriangle(*t) for t in triangles]
    return vertices, tri_drake


def get_AABB_limits(hpoly, dim=3):
    max_limits = []
    min_limits = []
    A = hpoly.A()
    b = hpoly.b()

    for idx in range(dim):
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = x[idx]
        aabbprog.AddCost(cost)
        aabbprog.AddConstraint(le(A @ x, b))
        solver = SnoptSolver()
        result = solver.Solve(aabbprog)
        min_limits.append(result.get_optimal_cost() - 0.01)
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = -x[idx]
        aabbprog.AddCost(cost)
        aabbprog.AddConstraint(le(A @ x, b))
        solver = SnoptSolver()
        result = solver.Solve(aabbprog)
        max_limits.append(-result.get_optimal_cost() + 0.01)
    return max_limits, min_limits


def stretch_array_to_3d(arr, val=0.):
    if arr.shape[0] < 3:
        arr = np.append(arr, val * np.ones((3 - arr.shape[0])))
    return arr


def infinite_hues():
    yield Fraction(0)
    for k in itertools.count():
        i = 2**k # zenos_dichotomy
        for j in range(1,i,2):
            yield Fraction(j,i)


def hue_to_hsvs(h: Fraction):
    # tweak values to adjust scheme
    for s in [Fraction(6,10)]:
        for v in [Fraction(6,10), Fraction(9,10)]:
            yield (h, s, v)


def rgb_to_css(rgb) -> str:
    uint8tuple = map(lambda y: int(y*255), rgb)
    return tuple(uint8tuple)


def css_to_html(css):
    return f"<text style=background-color:{css}>&nbsp;&nbsp;&nbsp;&nbsp;</text>"


def n_colors(n=33, rgbs_ret = False):
    hues = infinite_hues()
    hsvs = itertools.chain.from_iterable(hue_to_hsvs(hue) for hue in hues)
    rgbs = (colorsys.hsv_to_rgb(*hsv) for hsv in hsvs)
    csss = (rgb_to_css(rgb) for rgb in rgbs)
    to_ret = list(itertools.islice(csss, n)) if rgbs_ret else list(itertools.islice(csss, n))
    return to_ret

def draw_traj(meshcat_instance, traj, maxit, name = "/trajectory",
              color = Rgba(0,0,0,1), line_width = 3):
    pts = np.squeeze(np.array([traj.value(it * traj.end_time() / maxit) for it in range(maxit)]))
    pts_3d = np.hstack([pts, 0 * np.ones((pts.shape[0], 3 - pts.shape[1]))]).T
    meshcat_instance.SetLine(name, pts_3d, line_width, color)

def generate_walk_around_polytope(h_polytope, num_verts):
    v_polytope = VPolytope(h_polytope)
    verts_to_visit_index = np.random.randint(0, v_polytope.vertices().shape[1], num_verts)
    verts_to_visit = v_polytope.vertices()[:, verts_to_visit_index]
    t_knots = np.linspace(0, 1,  verts_to_visit.shape[1])
    lin_traj = PiecewisePolynomial.FirstOrderHold(t_knots, verts_to_visit)
    return lin_traj


def _resolve_meshcat_target(target, plant_context, diagram, diagram_context,
                            scene_graph, need_scene_graph):
    """Shared duck-typing for walk_polytopes/play_configurations: accepts
    either a bare plant (with the rest passed explicitly) or any object
    exposing .plant/.plant_context and either (.diagram, .diagram_context)
    or (.task_space_diagram, .task_space_diagram_context) - e.g. a
    CIrisPlantVisualizer or ManipulationPlanner instance."""
    plant = getattr(target, "plant", target)
    plant_context = plant_context if plant_context is not None else getattr(target, "plant_context", None)
    diagram = diagram if diagram is not None else (
        getattr(target, "diagram", None) or getattr(target, "task_space_diagram", None))
    diagram_context = diagram_context if diagram_context is not None else (
        getattr(target, "diagram_context", None) or getattr(target, "task_space_diagram_context", None))
    scene_graph = scene_graph if scene_graph is not None else getattr(target, "scene_graph", None)

    if plant_context is None or diagram is None or diagram_context is None:
        raise ValueError(
            "Need plant_context, diagram, and diagram_context - either pass them "
            "explicitly, or pass a visualizer/planner object that exposes them.")
    if need_scene_graph and scene_graph is None:
        raise ValueError(
            "check_collisions=True needs scene_graph - pass it explicitly, or pass "
            "a target object that exposes .scene_graph.")
    return plant, plant_context, diagram, diagram_context, scene_graph


def play_configurations(target, configs, *,
                        plant_context=None, diagram=None, diagram_context=None,
                        scene_graph=None, pause=0.15, check_collisions=True,
                        collision_depth_tol=1e-4, label="samples", verbose=True):
    """Publish a pre-computed sequence of configurations to Meshcat one at a
    time, optionally reporting real Drake collision status at each step -
    the generic playback half of walk_polytopes, usable with configurations
    from any source (HPolyhedron.UniformSample, nlp_sampling.py's
    restarting_nhr_sample_with_equalities, a saved path, ...), not just a
    polytope walk.

    target/plant_context/diagram/diagram_context/scene_graph: see
    walk_polytopes - same duck-typed resolution.
    configs: an (N, nq) array, or a list/iterable of nq-length arrays.

    Returns a list of dicts, one per step:
        {region, step, q, in_collision, num_collision_pairs}.
    """
    plant, plant_context, diagram, diagram_context, scene_graph = _resolve_meshcat_target(
        target, plant_context, diagram, diagram_context, scene_graph, check_collisions)

    query_port = scene_graph.get_query_output_port() if check_collisions else None
    inspector = scene_graph.model_inspector() if check_collisions else None

    configs = list(configs)
    log = []
    for step, q in enumerate(configs):
        q = np.asarray(q, dtype=float)
        plant.SetPositions(plant_context, q)
        diagram.ForcedPublish(diagram_context)

        in_collision, n_pairs, real_pairs = None, None, []
        if check_collisions:
            query_object = query_port.Eval(scene_graph.GetMyContextFromRoot(diagram_context))
            pairs = query_object.ComputePointPairPenetration()
            real_pairs = [p for p in pairs if p.depth > collision_depth_tol]
            n_pairs = len(real_pairs)
            in_collision = n_pairs > 0

        log.append(dict(region=label, step=step, q=q.copy(),
                       in_collision=in_collision, num_collision_pairs=n_pairs))

        if verbose:
            status = ""
            if check_collisions:
                status = " | ok"
                if in_collision:
                    pair_names = sorted({
                        f"{inspector.GetName(p.id_A)}<->{inspector.GetName(p.id_B)}"
                        for p in real_pairs})
                    shown = ", ".join(pair_names[:3]) + ("..." if len(pair_names) > 3 else "")
                    status = f" | COLLIDING ({n_pairs} pairs: {shown})"
            print(f"[{label}] step {step + 1}/{len(configs)}: q={np.round(q, 3)}{status}")

        if pause > 0:
            time.sleep(pause)

    if check_collisions and verbose and log:
        n_bad = sum(1 for entry in log if entry["in_collision"])
        print(f"\n{n_bad}/{len(log)} sampled configurations were in real collision.")

    return log


def walk_polytopes(target, polytopes, *,
                   plant_context=None, diagram=None, diagram_context=None,
                   scene_graph=None, steps_per_region=20, mixing_steps=5,
                   pause=0.15, seed=0, check_collisions=True,
                   collision_depth_tol=1e-4, verbose=True):
    """Randomly walk through one or more HPolyhedron regions - via Drake's
    own hit-and-run sampler, HPolyhedron.UniformSample, chained so each
    step seeds the next - publishing each sampled configuration to Meshcat
    (via play_configurations). A quick way to get a visual/printed sense of
    what a set of polytopes (c-free, grasp, placement, ...) actually
    contains, and to catch configurations that are wrongly included (e.g.
    claimed collision-free but not, by the real scene geometry).

    Note: HPolyhedron.UniformSample's hit-and-run has no notion of any
    underlying nonlinear constraint a polytope like grasp_polytope only
    approximates (e.g. h_grasp_eq/g_grasp_ineq) - on a very anisotropic box
    (tiny margin in most dimensions, full range in a couple), its mixing in
    the tight dimensions can be slow, so consecutive samples can look
    under-varied there. For genuinely constraint-aware sampling instead of
    box sampling, see wrist_axis_grasp_polytope.sample_grasp_configurations
    (algorithms/nlp_sampling/), which walks the real constraint manifold via
    nlp_sampling.py.

    target: a plant, or any object exposing .plant/.plant_context and
        either (.diagram, .diagram_context) or (.task_space_diagram,
        .task_space_diagram_context) - e.g. a CIrisPlantVisualizer or
        ManipulationPlanner instance. Pass plant_context/diagram/
        diagram_context/scene_graph explicitly if target is a bare plant.
    polytopes: an HPolyhedron, a list of them, or a {name: HPolyhedron (or
        list of them)} dict - walked through in order, one after another.
    steps_per_region: hit-and-run steps sampled (and published) per region.
    mixing_steps: hit-and-run steps taken *between* each published sample -
        kept low (as here) so consecutive samples chain into a genuine
        random walk rather than independent draws; see
        HPolyhedron.UniformSample's own docs for the tradeoff.
    check_collisions: if True (needs scene_graph, resolved from `target`
        when possible), reports real Drake collision status at each step
        against the actual scene geometry - independent of whatever the
        polytope itself claims.

    Returns a list of dicts, one per sampled step:
        {region, step, q, in_collision, num_collision_pairs}.
    """
    plant, plant_context, diagram, diagram_context, scene_graph = _resolve_meshcat_target(
        target, plant_context, diagram, diagram_context, scene_graph, check_collisions)
    bundle = dict(plant_context=plant_context, diagram=diagram,
                 diagram_context=diagram_context, scene_graph=scene_graph)

    if isinstance(polytopes, HPolyhedron):
        plan = [("region", polytopes)]
    elif isinstance(polytopes, dict):
        plan = [(name, p) for name, polys in polytopes.items()
               for p in ([polys] if isinstance(polys, HPolyhedron) else polys)]
    else:
        plan = [(f"region_{i}", p) for i, p in enumerate(polytopes)]

    generator = RandomGenerator(seed)
    log = []
    for name, P in plan:
        if P.IsEmpty():
            if verbose:
                print(f"[{name}] skipped: empty polytope")
            continue
        if not P.IsBounded():
            if verbose:
                print(f"[{name}] skipped: unbounded polytope (UniformSample needs boundedness)")
            continue

        q = P.ChebyshevCenter()
        region_qs = []
        for _ in range(steps_per_region):
            q = P.UniformSample(generator, q, mixing_steps=mixing_steps)
            region_qs.append(q.copy())

        log.extend(play_configurations(
            plant, region_qs, check_collisions=check_collisions,
            collision_depth_tol=collision_depth_tol, label=name, verbose=verbose,
            pause=pause, **bundle))

    if check_collisions and verbose and log:
        n_bad = sum(1 for entry in log if entry["in_collision"])
        print(f"\n{n_bad}/{len(log)} sampled configurations across {len(plan)} region(s) "
             "were in real collision.")

    return log