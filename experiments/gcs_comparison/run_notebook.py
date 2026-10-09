"""Run the code cells of full_arm_planner_gcs.ipynb headlessly, in one namespace.

Runs every code cell before the "## GCS baseline" section: the scene, the free-set fixing, the brute force
and the reachability A* (Steps 1-7). The GCS definitions are left to the caller, which takes them from the
notebook as well (see compare_plans.py), so the experiments always use the notebook's own code.

Widgets are stubbed and, while the cells run, time.sleep is a no-op, so the Meshcat playback cells run
instantly.
"""
import contextlib
import io
import json
import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
NOTEBOOK = REPO_ROOT / "full_arm_planner_gcs.ipynb"
GCS_SECTION = "## GCS baseline"


def notebook_cells(path=NOTEBOOK):
    return json.loads(Path(path).read_text())["cells"]


def code_cell(prefix, path=NOTEBOOK):
    """Source of the one code cell that starts with prefix."""
    matches = [''.join(c["source"]) for c in notebook_cells(path)
               if c["cell_type"] == "code" and ''.join(c["source"]).startswith(prefix)]
    if len(matches) != 1:
        raise ValueError(f"expected one code cell starting with {prefix!r}, found {len(matches)}")
    return matches[0]


def run_until_gcs_section(path=NOTEBOOK, verbose=False, ns=None):
    """Execute the notebook's code cells up to the GCS section, in ns (e.g. a script's globals()), and
    return it. Running in the caller's globals keeps one namespace, so the notebook's functions see any
    object the caller later replaces (as compare_plans.py --scenario does)."""
    os.chdir(REPO_ROOT)                       # the notebook loads my_sdfs/ and data/ by relative path
    sys.path.insert(0, str(REPO_ROOT))
    real_sleep, time.sleep = time.sleep, (lambda *_: None)
    try:
        return _run_cells(path, verbose, {} if ns is None else ns)
    finally:
        time.sleep = real_sleep


def _run_cells(path, verbose, ns):
    ns.setdefault("display", lambda *a, **k: None)
    for i, cell in enumerate(notebook_cells(path)):
        src = ''.join(cell["source"])
        if cell["cell_type"] == "markdown" and src.startswith(GCS_SECTION):
            return ns
        if cell["cell_type"] != "code" or any(l.lstrip().startswith(("%", "!")) for l in src.splitlines()):
            continue
        t0 = time.perf_counter()
        out = io.StringIO()
        with contextlib.redirect_stdout(out if not verbose else sys.stdout):
            exec(compile(src, f"<cell {i}>", "exec"), ns)
        if verbose:
            print(f"----- cell {i}: {time.perf_counter() - t0:.1f}s", flush=True)
    raise ValueError(f"no markdown cell starting with {GCS_SECTION!r} in {path}")
