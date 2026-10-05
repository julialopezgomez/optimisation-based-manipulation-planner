"""Interactive 2-D cross-section viewer for a high-dimensional C-space.

Given a plant with N degrees of freedom and one or more labeled groups of
`HPolyhedron` regions (c-free, grasp/placement polytopes, ...) defined over
the full N-dimensional configuration space, `CSpaceSliceVisualizer` shows a
live 2-D plot of the true geometric cross-section through any 2 chosen
joints, with the other N-2 fixed by sliders (or by scrubbing a trajectory's
time). This is a slice, not a projection: the other N-2 coordinates are held
at specific values, not integrated/dropped out.

See `cspace_explorer.ipynb` for a standalone usage example (its own plant,
no running planner needed), and the integration cell near the end of
`full_arm_planner.ipynb` for one wired to a live `ManipulationPlanner` run.

Needs `anywidget` installed (backs plotly's `FigureWidget` as of plotly>=6)
for the interactive class; the slicing math below has no such dependency and
runs standalone (see `_smoke_test` / `python cspace_slice_visualizer.py`).
"""

from __future__ import annotations

import colorsys
import warnings
from dataclasses import dataclass
from functools import partial
from typing import Callable, Optional, Sequence, Union

import numpy as np
from pydrake.geometry.optimization import HPolyhedron, VPolytope
from scipy.spatial import ConvexHull, QhullError

try:
    import plotly.graph_objects as go
except ImportError:  # pragma: no cover
    go = None

try:
    import ipywidgets as widgets
    from IPython.display import display
except ImportError:  # pragma: no cover
    widgets = None
    display = None


PolytopeOrList = Union[HPolyhedron, Sequence[HPolyhedron]]

_AUTO_COLORS = [
    "steelblue", "seagreen", "darkorange", "purple",
    "crimson", "goldenrod", "teal", "slategray",
]


# -----------------------------------------------------------------------------
# Pure slicing math - no widgets, no plotting, unit-testable on its own.
# -----------------------------------------------------------------------------

def slice_polyhedron(P: HPolyhedron, free_axes: Sequence[int], q_full: np.ndarray) -> HPolyhedron:
    """The true geometric cross-section of P at q_full, over free_axes.

    Not a projection: every coordinate NOT in free_axes is fixed to
    q_full's value there, and the returned polyhedron is over just the
    remaining len(free_axes) coordinates, in free_axes order.
    """
    free_axes = list(free_axes)
    n = P.ambient_dimension()
    fixed_axes = [i for i in range(n) if i not in free_axes]
    A, b = P.A(), P.b()
    q_full = np.asarray(q_full, dtype=float)
    A_free = A[:, free_axes]
    b_slice = b - A[:, fixed_axes] @ q_full[fixed_axes]
    return HPolyhedron(A_free, b_slice)


def slice_polygon_vertices(P: HPolyhedron, free_axes: Sequence[int], q_full: np.ndarray,
                            dedup_decimals: int = 9):
    """(vertices, kind) for the 2-D slice of P at q_full.

    kind is one of "empty" / "point" / "segment" / "polygon". vertices is a
    (k, 2) array; for "polygon" it is CCW-ordered (via ConvexHull) and ready
    to draw as a closed, fillable ring.
    """
    S = slice_polyhedron(P, free_axes, q_full)
    if S.IsEmpty():
        return np.empty((0, 2)), "empty"
    try:
        V = VPolytope(S).vertices().T
    except RuntimeError:
        # Unbounded or otherwise degenerate in a way VPolytope can't handle.
        return np.empty((0, 2)), "empty"
    if len(V) == 0:
        return V, "empty"
    # cdd emits ~1e-12-scale noise at coordinates that should be exact zeros;
    # round-and-dedup before deciding how many distinct vertices we actually have.
    V = np.unique(np.round(V, dedup_decimals), axis=0)
    if len(V) == 1:
        return V, "point"
    if len(V) == 2:
        return V, "segment"
    try:
        hull = ConvexHull(V)
    except QhullError:
        # Near-collinear: fall back to the two extreme points along the
        # dominant spread direction rather than failing the whole redraw.
        centered = V - V.mean(axis=0)
        spread_axis = centered[np.argmax(np.linalg.norm(centered, axis=1))]
        norm = np.linalg.norm(spread_axis)
        if norm < 1e-12:
            return V[:1], "point"
        spread_axis = spread_axis / norm
        proj = centered @ spread_axis
        seg = V[[np.argmin(proj), np.argmax(proj)]]
        return seg, "segment"
    return V[hull.vertices], "polygon"


def _fill_rgba(color: Optional[str], alpha: float) -> Optional[str]:
    """Best-effort 'color' -> 'rgba(...)' string at the given alpha.

    Accepts the repo's existing 'hsl(h, s%, l%)' convention (see
    ciris_plant_visualizer.py's plot_polytope_2d) as well as any name/hex
    matplotlib can parse. Returns None if the color can't be parsed, so the
    caller can fall back to a flat trace opacity instead.
    """
    if color is None:
        return None
    c = color.strip()
    if c.startswith("hsl("):
        try:
            h, s, l = (float(x) for x in c[4:-1].replace("%", "").split(","))
            r, g, b = colorsys.hls_to_rgb(h / 360, l / 100, s / 100)
            return f"rgba({int(r * 255)}, {int(g * 255)}, {int(b * 255)}, {alpha})"
        except ValueError:
            pass
    try:
        import matplotlib.colors as mcolors
        r, g, b, _ = mcolors.to_rgba(c)
        return f"rgba({int(r * 255)}, {int(g * 255)}, {int(b * 255)}, {alpha})"
    except Exception:
        return None


@dataclass
class _Group:
    name: str
    polytopes: list
    color: str
    bounded: list  # per-polytope bool, checked once at add time - see _set_group_record
    visible: bool = True
    trace_start: int = -1
    legend_trace: int = -1


# -----------------------------------------------------------------------------
# The interactive tool.
# -----------------------------------------------------------------------------

class CSpaceSliceVisualizer:
    """A live-updating 2-D slice of a high-dimensional C-space.

    Every input is optional at construction time and settable later (via
    add_group/set_trajectory/set_path) - the tool is meant to be usable
    before a planner run has produced a trajectory or even c-free regions.
    """

    def __init__(self, plant, plant_context=None, *,
                 diagram=None, diagram_context=None,
                 groups: Optional[dict] = None,
                 trajectory=None, path=None, path_times=None,
                 q0: Optional[np.ndarray] = None,
                 free_axes=(0, 1),
                 num_traj_samples: int = 200,
                 fill_opacity: float = 0.25,
                 continuous_update: bool = True,
                 figure_height: int = 520,
                 on_config_change: Optional[Callable[[np.ndarray], str]] = None):
        if go is None or widgets is None:
            raise ImportError(
                "CSpaceSliceVisualizer needs plotly and ipywidgets installed."
            )

        self.plant = plant
        self.plant_context = plant_context if plant_context is not None else plant.CreateDefaultContext()
        self.diagram = diagram
        self.diagram_context = diagram_context
        self._sync_meshcat_enabled = diagram is not None and diagram_context is not None
        self.on_config_change = on_config_change

        self.nq = plant.num_positions()
        self.names = list(plant.GetPositionNames())
        self.short_names = [f"q{i} {name}" for i, name in enumerate(self.names)]

        self.lower = np.asarray(plant.GetPositionLowerLimits(), dtype=float).copy()
        self.upper = np.asarray(plant.GetPositionUpperLimits(), dtype=float).copy()
        non_finite = ~np.isfinite(self.lower) | ~np.isfinite(self.upper)
        if np.any(non_finite):
            self.lower[non_finite] = -2 * np.pi
            self.upper[non_finite] = 2 * np.pi

        self._q = np.array(q0, dtype=float).copy() if q0 is not None else (self.lower + self.upper) / 2
        self._override = np.zeros(self.nq, dtype=bool)
        self._suspend = False

        self._groups: dict[str, _Group] = {}
        self._auto_color_idx = 0
        self._ix, self._iy = free_axes
        self.fill_opacity = fill_opacity
        self.continuous_update = continuous_update

        self._traj = None
        self._traj_ts = None
        self._traj_Q = None
        self._traj_line_trace = None
        self._traj_marker_trace = None
        self._config_marker_trace = None

        self._build_widgets(figure_height)

        if groups:
            for name, (polys, color) in groups.items():
                self._set_group_record(name, polys, color, True)
        self._rebuild_traces()
        self._sync_sliders_from_q()
        self._apply_free_axes(self._ix, self._iy)

        if trajectory is not None:
            self.set_trajectory(trajectory, num_traj_samples)
        elif path is not None:
            self.set_path(path, path_times)
        # else: _apply_free_axes above already triggered the initial redraw.

        # _sync_meshcat() is otherwise only ever triggered by a slider/time
        # callback - without this, the plant (and any attached Meshcat view)
        # would silently stay at its default/zero pose instead of q0 until
        # the user first touches a slider.
        self._sync_meshcat()

    @classmethod
    def from_visualizer(cls, visualizer, **kwargs) -> "CSpaceSliceVisualizer":
        """Build from an existing CIrisPlantVisualizer, reusing its plant/
        context/diagram so the Meshcat task-space view stays in sync."""
        return cls(
            visualizer.plant,
            visualizer.plant_context,
            diagram=getattr(visualizer, "task_space_diagram", None),
            diagram_context=getattr(visualizer, "task_space_diagram_context", None),
            **kwargs,
        )

    # -- widget construction --------------------------------------------------

    def _build_widgets(self, figure_height):
        try:
            self.fig = go.FigureWidget()
        except ImportError as exc:
            raise ImportError(
                "plotly.graph_objects.FigureWidget needs the 'anywidget' package: "
                "pip install anywidget"
            ) from exc
        self.fig.update_layout(
            height=figure_height,
            margin=dict(l=50, r=20, t=30, b=40),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, x=0),
        )

        axis_options = [(f"{i}: {name}", i) for i, name in enumerate(self.names)]
        self.dd_x = widgets.Dropdown(options=axis_options, value=self._ix, description="x axis")
        self.dd_y = widgets.Dropdown(options=axis_options, value=self._iy, description="y axis")
        self.dd_x.observe(partial(self._on_axis_dropdown_change, which="x"), names="value")
        self.dd_y.observe(partial(self._on_axis_dropdown_change, which="y"), names="value")

        self.time_slider = widgets.FloatSlider(
            min=0, max=1, value=0, step=0.01, description="t",
            continuous_update=self.continuous_update, disabled=True,
            layout=widgets.Layout(width="480px"))
        self.time_slider.observe(self._on_time_change, names="value")

        self.follow_button = widgets.Button(description="Follow trajectory", disabled=True)
        self.follow_button.on_click(lambda _btn: self.clear_overrides())

        self.joint_sliders = []
        for i in range(self.nq):
            step = max((self.upper[i] - self.lower[i]) / 200, 1e-6)
            slider = widgets.FloatSlider(
                min=self.lower[i], max=self.upper[i], value=float(self._q[i]),
                step=step, description=self.short_names[i],
                continuous_update=self.continuous_update, readout_format=".4f",
                style={"description_width": "230px"},
                layout=widgets.Layout(width="580px"))
            slider.observe(partial(self._on_joint_change, idx=i), names="value")
            self.joint_sliders.append(slider)

        self.group_checkboxes: dict[str, "widgets.Checkbox"] = {}
        self.group_box = widgets.HBox([])
        self.status_out = widgets.Output()

        self.widget_box = widgets.VBox([
            widgets.HBox([self.dd_x, self.dd_y]),
            widgets.HBox([self.time_slider, self.follow_button]),
            self.group_box,
            self.fig,
            widgets.VBox(self.joint_sliders),
            self.status_out,
        ])

    def _rebuild_traces(self):
        traces = []
        for group in self._groups.values():
            group.trace_start = len(traces)
            fill_color = _fill_rgba(group.color, self.fill_opacity)
            for _ in group.polytopes:
                traces.append(go.Scatter(
                    x=[], y=[], mode="lines", fill="toself",
                    line=dict(color=group.color, width=1.5),
                    fillcolor=fill_color,
                    opacity=1.0 if fill_color else self.fill_opacity,
                    showlegend=False, legendgroup=group.name,
                    hoverinfo="skip", visible=False))
            group.legend_trace = len(traces)
            traces.append(go.Scatter(
                x=[None], y=[None], mode="lines",
                line=dict(color=group.color, width=3),
                name=group.name, legendgroup=group.name, showlegend=True,
                visible=group.visible))

        self._traj_line_trace = len(traces)
        traces.append(go.Scatter(x=[], y=[], mode="lines", line=dict(color="black", width=2),
                                  name="trajectory", hoverinfo="skip", visible=False))
        self._traj_marker_trace = len(traces)
        traces.append(go.Scatter(x=[], y=[], mode="markers",
                                  marker=dict(symbol="diamond", size=11, color="black"),
                                  name="t", visible=False))
        self._config_marker_trace = len(traces)
        traces.append(go.Scatter(x=[], y=[], mode="markers",
                                  marker=dict(symbol="star", size=14, color="crimson",
                                              line=dict(color="black", width=1)),
                                  name="current q"))

        self.fig.data = []
        if traces:
            self.fig.add_traces(traces)
        self._rebuild_group_checkboxes()

    def _rebuild_group_checkboxes(self):
        boxes = []
        for name, group in self._groups.items():
            cb = self.group_checkboxes.get(name)
            if cb is None:
                cb = widgets.Checkbox(value=group.visible, description=name, indent=False)
                cb.observe(partial(self._on_group_visibility, name=name), names="value")
                self.group_checkboxes[name] = cb
            boxes.append(cb)
        self.group_box.children = boxes

    # -- group / trajectory mutation -------------------------------------------

    def _set_group_record(self, name, polytopes, color, visible):
        polys = [polytopes] if isinstance(polytopes, HPolyhedron) else list(polytopes)
        bounded = []
        for p in polys:
            if p.ambient_dimension() != self.nq:
                raise ValueError(
                    f"group {name!r}: polytope ambient_dimension {p.ambient_dimension()} "
                    f"!= plant.num_positions() {self.nq}")
            # Checked once here, not per redraw: a slice of a bounded polytope is
            # always bounded too, so this fully determines every future tick.
            # NOTE: VPolytope() on an unbounded HPolyhedron is a hard Drake abort,
            # not a catchable Python exception - _redraw_impl MUST skip these
            # polytopes entirely rather than ever calling slice_polygon_vertices
            # on one, or a single slider tick would take down the whole kernel.
            is_bounded = p.IsBounded()
            bounded.append(is_bounded)
            if not is_bounded:
                warnings.warn(f"group {name!r} contains an unbounded polytope; "
                              "it will be skipped (VPolytope needs boundedness).")
        if color is None:
            color = _AUTO_COLORS[self._auto_color_idx % len(_AUTO_COLORS)]
            self._auto_color_idx += 1
        self._groups[name] = _Group(name=name, polytopes=polys, color=color,
                                    bounded=bounded, visible=visible)

    def add_group(self, name: str, polytopes: PolytopeOrList, color: Optional[str] = None, *,
                  visible: bool = True):
        self._set_group_record(name, polytopes, color, visible)
        self._rebuild_traces()
        self._redraw()

    set_group = add_group  # replacing an existing group is the same operation

    def remove_group(self, name: str):
        del self._groups[name]
        self.group_checkboxes.pop(name, None)
        self._rebuild_traces()
        self._redraw()

    def set_groups(self, mapping: dict):
        for name, (polys, color) in mapping.items():
            self._set_group_record(name, polys, color, True)
        self._rebuild_traces()
        self._redraw()

    def _configure_time_slider(self, ts):
        self._suspend = True
        try:
            self.time_slider.max = max(self.time_slider.max, float(ts[-1]))
            self.time_slider.min = float(ts[0])
            self.time_slider.max = float(ts[-1])
            self.time_slider.step = max((ts[-1] - ts[0]) / 200, 1e-6)
            self.time_slider.value = float(ts[0])
            self.time_slider.disabled = False
            self.follow_button.disabled = False
        finally:
            self._suspend = False

    def set_trajectory(self, traj, num_samples: int = 200):
        self._traj = traj
        ts = np.linspace(traj.start_time(), traj.end_time(), num_samples)
        Q = np.array([np.asarray(traj.value(t)).ravel() for t in ts])
        self._traj_ts, self._traj_Q = ts, Q
        self._configure_time_slider(ts)
        self._apply_time(ts[0])

    def set_path(self, path_q: Sequence[np.ndarray], times: Optional[Sequence[float]] = None):
        Q = np.array([np.asarray(q).ravel() for q in path_q])
        ts = np.asarray(times, dtype=float) if times is not None else np.linspace(0.0, 1.0, len(Q))
        self._traj = None
        self._traj_ts, self._traj_Q = ts, Q
        self._configure_time_slider(ts)
        self._apply_time(ts[0])

    def clear_trajectory(self):
        self._traj = None
        self._traj_ts = self._traj_Q = None
        self._suspend = True
        try:
            self.time_slider.disabled = True
            self.follow_button.disabled = True
        finally:
            self._suspend = False
        self._redraw()

    # -- axis / configuration state --------------------------------------------

    def _relabel_slider(self, idx):
        tag = "  [x]" if idx == self._ix else ("  [y]" if idx == self._iy else "")
        prefix = "* " if self._override[idx] else ""
        self.joint_sliders[idx].description = prefix + self.short_names[idx] + tag

    def _span(self, i):
        return max(self.upper[i] - self.lower[i], 1e-9)

    def _apply_free_axes(self, ix, iy):
        self._ix, self._iy = ix, iy
        for i in range(self.nq):
            self._relabel_slider(i)
        self.fig.layout.xaxis.title = self.names[ix]
        self.fig.layout.yaxis.title = self.names[iy]
        span_x, span_y = self._span(ix), self._span(iy)
        self.fig.layout.xaxis.range = [float(self.lower[ix] - 0.02 * span_x), float(self.upper[ix] + 0.02 * span_x)]
        self.fig.layout.yaxis.range = [float(self.lower[iy] - 0.02 * span_y), float(self.upper[iy] + 0.02 * span_y)]
        self.fig.layout.uirevision = f"{ix}:{iy}"
        self._redraw()

    def _on_axis_dropdown_change(self, change, which):
        if self._suspend:
            return
        ix, iy = self.dd_x.value, self.dd_y.value
        if ix == iy:
            self._suspend = True
            try:
                if which == "x":
                    self.dd_y.value = self._iy
                else:
                    self.dd_x.value = self._ix
            finally:
                self._suspend = False
            return
        self._apply_free_axes(ix, iy)

    def set_free_axes(self, ix: int, iy: int):
        if ix == iy:
            raise ValueError("free axes must be distinct")
        self._suspend = True
        try:
            self.dd_x.value = ix
            self.dd_y.value = iy
        finally:
            self._suspend = False
        self._apply_free_axes(ix, iy)

    def _on_joint_change(self, change, idx):
        if self._suspend:
            return
        self._q[idx] = change["new"]
        if idx not in (self._ix, self._iy):
            self._override[idx] = True
            self._relabel_slider(idx)
        self._redraw()
        self._sync_meshcat()

    def _on_group_visibility(self, change, name):
        self._groups[name].visible = change["new"]
        self._redraw()

    def _on_time_change(self, change):
        if self._suspend or self._traj_Q is None:
            return
        self._apply_time(change["new"])

    def _apply_time(self, t):
        i_nearest = int(np.argmin(np.abs(self._traj_ts - t)))
        q_t = self._traj_Q[i_nearest]
        not_overridden = ~self._override
        self._q[not_overridden] = q_t[not_overridden]
        self._suspend = True
        try:
            for i in np.flatnonzero(not_overridden):
                v = float(np.clip(self._q[i], self.joint_sliders[i].min, self.joint_sliders[i].max))
                self.joint_sliders[i].value = v
        finally:
            self._suspend = False
        self._redraw()
        self._sync_meshcat()

    def clear_overrides(self):
        self._override[:] = False
        for i in range(self.nq):
            self._relabel_slider(i)
        if self._traj_Q is not None:
            self._apply_time(self.time_slider.value)
        else:
            self._redraw()
            self._sync_meshcat()

    def _sync_sliders_from_q(self):
        self._suspend = True
        try:
            for i, slider in enumerate(self.joint_sliders):
                slider.value = float(np.clip(self._q[i], slider.min, slider.max))
        finally:
            self._suspend = False

    def _sync_meshcat(self):
        self.plant.SetPositions(self.plant_context, self._q)
        if self._sync_meshcat_enabled:
            self.diagram.ForcedPublish(self.diagram_context)

    def set_configuration(self, q: np.ndarray):
        self._q = np.array(q, dtype=float).copy()
        self._sync_sliders_from_q()
        self._redraw()
        self._sync_meshcat()

    def get_configuration(self) -> np.ndarray:
        return self._q.copy()

    # -- redraw ------------------------------------------------------------

    def _redraw(self):
        try:
            self._redraw_impl()
        except Exception as exc:
            with self.status_out:
                self.status_out.clear_output(wait=True)
                print(f"redraw failed: {exc!r}")
            raise

    def _redraw_impl(self):
        ix, iy = self._ix, self._iy
        counts = {}
        with self.fig.batch_update():
            for name, group in self._groups.items():
                n_nonempty = 0
                for j, P in enumerate(group.polytopes):
                    trace = self.fig.data[group.trace_start + j]
                    if not group.visible or not group.bounded[j]:
                        trace.visible = False
                        continue
                    verts, kind = slice_polygon_vertices(P, (ix, iy), self._q)
                    if kind == "empty":
                        trace.visible = False
                        continue
                    n_nonempty += 1
                    trace.visible = True
                    if kind == "polygon":
                        ring = np.vstack([verts, verts[0]])
                        trace.mode = "lines"
                        trace.fill = "toself"
                    else:
                        ring = verts
                        trace.mode = "lines+markers"
                        trace.fill = "none"
                    trace.x = ring[:, 0].tolist()
                    trace.y = ring[:, 1].tolist()
                counts[name] = (n_nonempty, len(group.polytopes))
                self.fig.data[group.legend_trace].visible = group.visible

            if self._traj_Q is not None:
                xy = self._traj_Q[:, [ix, iy]]
                self.fig.data[self._traj_line_trace].x = xy[:, 0].tolist()
                self.fig.data[self._traj_line_trace].y = xy[:, 1].tolist()
                self.fig.data[self._traj_line_trace].visible = True
                t = self.time_slider.value
                i_nearest = int(np.argmin(np.abs(self._traj_ts - t)))
                self.fig.data[self._traj_marker_trace].x = [float(xy[i_nearest, 0])]
                self.fig.data[self._traj_marker_trace].y = [float(xy[i_nearest, 1])]
                self.fig.data[self._traj_marker_trace].visible = True
            elif self._traj_line_trace is not None:
                self.fig.data[self._traj_line_trace].visible = False
                self.fig.data[self._traj_marker_trace].visible = False

            self.fig.data[self._config_marker_trace].x = [float(self._q[ix])]
            self.fig.data[self._config_marker_trace].y = [float(self._q[iy])]

        with self.status_out:
            self.status_out.clear_output(wait=True)
            summary = " | ".join(f"{name}: {ok}/{tot}" for name, (ok, tot) in counts.items())
            msg = summary if summary else "(no groups yet)"
            if self.on_config_change is not None:
                try:
                    msg += " | " + self.on_config_change(self._q.copy())
                except Exception as exc:
                    msg += f" | on_config_change failed: {exc!r}"
            print(msg)

    # -- display -------------------------------------------------------------

    @property
    def widget(self):
        return self.widget_box

    def show(self):
        display(self.widget_box)
        return self


def show_cspace_slices(plant_or_visualizer, groups=None, trajectory=None, path=None,
                        q0=None, free_axes=(0, 1), **kwargs) -> CSpaceSliceVisualizer:
    """One-liner: build a CSpaceSliceVisualizer and display it immediately.

    plant_or_visualizer may be a bare MultibodyPlant or a CIrisPlantVisualizer
    instance (duck-typed via .plant/.plant_context/.task_space_diagram*).
    """
    if hasattr(plant_or_visualizer, "plant") and hasattr(plant_or_visualizer, "plant_context"):
        viz = CSpaceSliceVisualizer.from_visualizer(
            plant_or_visualizer, groups=groups, trajectory=trajectory, path=path,
            q0=q0, free_axes=free_axes, **kwargs)
    else:
        viz = CSpaceSliceVisualizer(
            plant_or_visualizer, groups=groups, trajectory=trajectory, path=path,
            q0=q0, free_axes=free_axes, **kwargs)
    return viz.show()


# -----------------------------------------------------------------------------
# Headless smoke test - no Jupyter, no plant, no widgets required.
# -----------------------------------------------------------------------------

def _smoke_test():
    from pathlib import Path
    from pydrake.geometry.optimization import LoadIrisRegionsYamlFile

    repo_root = Path(__file__).resolve().parent
    yaml_path = repo_root / "data" / "cfree" / "cfree_full_98coverage.yaml"
    regions = list(LoadIrisRegionsYamlFile(str(yaml_path)).values())
    ambient_dim = regions[0].ambient_dimension()
    print(f"Loaded {len(regions)} c-free regions, ambient_dimension={ambient_dim}")

    free_axes = (0, 1)
    for i, P in enumerate(regions):
        center = P.ChebyshevCenter()
        verts, kind = slice_polygon_vertices(P, free_axes, center)
        assert kind != "empty", f"region {i} should slice non-empty at its own Chebyshev center"
        assert kind == "polygon" or len(verts) >= 1, f"region {i}: unexpected slice kind {kind!r}"
    print(f"{len(regions)}/{len(regions)} regions sliced non-empty at their own center - OK")

    far = np.full(ambient_dim, 1000.0)
    _, kind = slice_polygon_vertices(regions[0], free_axes, far)
    assert kind == "empty", "expected an empty slice far outside every region"
    print("far-outside slice correctly empty - OK")

    print("cspace_slice_visualizer smoke test passed.")


if __name__ == "__main__":
    _smoke_test()
