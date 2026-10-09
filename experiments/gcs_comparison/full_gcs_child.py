"""One full GCS solve in a fresh process (started by compare_plans.py; not meant to be run by hand).

A fresh process gives a clean memory measurement, unaffected by earlier solves. Unlike a forked child, it
keeps MOSEK multithreaded: a solve that took 44 s in a normal process took ~190 s in a forked one.

Reloads the saved scenario (start, goal, sets), rebuilds the A*'s graph and plan exactly as the parent
did, builds the layered GCS graph and runs SolvePath. Writes a JSON result to --result.
"""
import argparse
import contextlib
import io
import json
import sys
import threading
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--scenario", required=True)
parser.add_argument("--turn", type=float, required=True)
parser.add_argument("--result", required=True)
ARGS = parser.parse_args()

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_notebook import code_cell, run_until_gcs_section   # noqa: E402

with contextlib.redirect_stdout(io.StringIO()):
    run_until_gcs_section(ns=globals())

import numpy as np                                                       # noqa: E402
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, LoadIrisRegionsYamlFile  # noqa: E402

SCENARIO = Path(ARGS.scenario)
sc = np.load(SCENARIO / "scenario.npz")
planner.cs_free = list(LoadIrisRegionsYamlFile(str(SCENARIO / "cfree_fixed.yaml")).values())
q_init_astar, q_goal_astar = sc["q_init"].copy(), sc["q_goal_base"].copy()
with contextlib.redirect_stdout(io.StringIO()):
    for prefix in ("import itertools\nfrom pydrake.solvers import MathematicalProgram, Solve", "def extreme_point(",
                   "L_MIN = 0.05", "n_free = len(free_sets)"):
        exec(code_cell(prefix), globals())
    exec(code_cell("from pydrake.planning import GcsTrajectoryOptimization"), globals())
# Keep the notebook's MOSEK licence held (mosek_licence): SolvePath runs many small programs (Drake's
# per-edge preprocessing, the rounding), and without a held licence each checks one out again. The A*'s
# preprocessing runs with a licence held too.

q_goal = q_goal_astar.copy()
q_goal[idx_cap] = q_init_astar[idx_cap] - ARGS.turn
apath, _ = reachability_astar(q_init_astar, q_goal)
N = int(np.ceil(ARGS.turn / L_MAX)) + 1


def rss_gb():
    for line in open('/proc/self/status'):
        if line.startswith('VmRSS'):
            return int(line.split()[1]) / 1e6


peak = [rss_gb()]


def sample():
    while True:
        peak[0] = max(peak[0], rss_gb())
        time.sleep(0.05)


base = rss_gb()
threading.Thread(target=sample, daemon=True).start()
t0 = time.perf_counter()
built = build_layered_gcs(q_init_astar, q_goal, N, keep={n.set_id for n in apath if n.kind == "F"})
t_build = time.perf_counter() - t0
options = GraphOfConvexSetsOptions()
options.max_rounded_paths = 10
t0 = time.perf_counter()
_, result = built["gcs"].SolvePath(built["source"], built["target"], options)
t_solve = time.perf_counter() - t0

graph = built["gcs"].graph_of_convex_sets()
out = dict(time_parts=dict(build=t_build, solve=t_solve), base=base, peak=peak[0],
           vertices=graph.num_vertices(), edges=graph.num_edges(), layers=N)
if result.is_success():
    seq = solved_vertex_sequence(built, result)
    out.update(cost=result.get_optimal_cost(),
               waypoints=[(q.tolist(), label, list(cert)) for q, label, cert in gcs_waypoints(built, result, seq)])
else:
    out.update(failed=str(result.get_solution_result()))
Path(ARGS.result).write_text(json.dumps(out))
