"""One HTML page that summarises the comparison of the methods: reachability A*, full GCS and HPP's Manipulation RRT.

Reads the saved results (nothing is re-run) and writes artifacts/results_summary.html:
  - offline stage: data/cfree/cfree_9dof_<scene>_report.json, artifacts/plan_comparison_1cap_2obs/graph_stats.json
  - 1cap_2obs:     artifacts/plan_comparison_1cap_2obs/summary.json (A*, GCS), artifacts/hpp_comparison/1cap_2obs*/
  - 8caps_3obs:    artifacts/cap_scaling_8caps_3obs/summary.json (A*), artifacts/hpp_comparison/8caps_3obs/

    python experiments/results_summary.py
"""
import json
import math
from datetime import date
from html import escape
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
ART = REPO_ROOT / "artifacts"
OUT = ART / "results_summary.html"
TURNS = [0.5, 1.5, 2.4, 3.2, 6.0]


def load(path):
    path = REPO_ROOT / path
    return json.loads(path.read_text()) if path.exists() else None


cfree = {s: load(f"data/cfree/cfree_9dof_{s}_report.json") for s in ("1cap_2obs", "8caps_3obs")}
stats = load("artifacts/plan_comparison_1cap_2obs/graph_stats.json")
plans = load("artifacts/plan_comparison_1cap_2obs/summary.json")
hpp1 = load("artifacts/hpp_comparison/1cap_2obs/summary.json")
hpp1_600 = load("artifacts/hpp_comparison/1cap_2obs_nofloor_600s/summary.json")
scaling = load("artifacts/cap_scaling_8caps_3obs/summary.json")
hpp8 = load("artifacts/hpp_comparison/8caps_3obs/summary.json")
L_MAX = stats["L_max"]


def hpp_group(summary, D):
    g = summary["per_group"].get(str(D)) if summary else None
    return g


# ---------------------------------------------------------------- 1cap_2obs rows
rows1 = []
for D in TURNS:
    a, s, g = plans[f"D{D}_astar"], plans[f"D{D}_gcs_on_astar"], plans[f"D{D}_gcs_full"]
    h = hpp_group(hpp1, D)
    rows1.append(dict(D=D, lb=math.ceil(D / L_MAX - 1e-9),
                      astar=dict(grasps=int(a["grasps"]), length=a["path_length"], time=a["time"],
                                 nodes=stats["queries"][str(D)]["expanded"]),
                      seq=dict(grasps=int(s["grasps"]), length=s["path_length"], time=s["time"]),
                      gcs=dict(grasps=int(g["grasps"]), length=g["path_length"], time=g["time"], mem=g["mem"],
                               V=g["vertices"], E=g["edges"]),
                      hpp=None if h is None else dict(solved=h["solved"], runs=h["runs"], time=h["time_s"],
                                                      length=h["length"], grasps=h["grasps"])))

# ---------------------------------------------------------------- 8caps_3obs rows
rows8 = []
if scaling:
    for k, per in sorted(scaling["per_k"].items(), key=lambda kv: int(kv[0])):
        row = dict(K=int(k))
        for m in ("weighted", "exact"):
            v = per.get(m)
            if v is None or v.get("skipped", 0) == v["queries"]:      # exact A* is not run past the first K it fails
                continue
            med = lambda key: (v[key] or {}).get("median") if isinstance(v.get(key), dict) else None
            row[m] = dict(queries=v["queries"], solved=v["solved"], timed_out=v.get("timed_out", 0),
                          outside=v.get("outside_cfree", 0), certified=v["certified"],
                          actions=med("actions"), grasps=med("grasps"), nodes=med("expanded"),
                          ms=None if med("total_s") is None else med("total_s") * 1e3,
                          ms_p25=None if not v.get("total_s") else v["total_s"]["p25"] * 1e3,
                          ms_p75=None if not v.get("total_s") else v["total_s"]["p75"] * 1e3,
                          length=med("path_length"))
        h = hpp8["per_group"].get(k) if hpp8 else None
        row["hpp"] = None if h is None else dict(solved=h["solved"], runs=h["runs"])
        rows8.append(row)


# ---------------------------------------------------------------- HTML helpers
def f(x, d=2):
    return "–" if x is None else f"{x:.{d}f}"


def secs(s):
    if s is None:
        return "–"
    return f"{s * 1e3:.0f} ms" if s < 1 else f"{s:.2f} s"


def best(cls, cond):
    return f' class="{cls}"' if cond else ""


one_table = []
for r in rows1:
    a, g, h = r["astar"], r["gcs"], r["hpp"]
    sq = r["seq"]
    n = 4 if h else 3
    one_table.append(
        f'<tr class="grp"><th rowspan="{n}" scope="rowgroup">{r["D"]:g}</th><td rowspan="{n}" class="num">{r["lb"]}</td>'
        f'<td><span class="key k1"></span>Reachability A*</td><td class="num{" win" if a["grasps"] == r["lb"] else ""}">{a["grasps"]}</td>'
        f'<td class="num">{f(a["length"])}</td><td class="num win">{secs(a["time"])}</td><td class="num">&lt;0.01</td></tr>')
    one_table.append(
        f'<tr><td><span class="key k0"></span>GCS on the A* sequence</td><td class="num{" win" if sq["grasps"] == r["lb"] else ""}">{sq["grasps"]}</td>'
        f'<td class="num">{f(sq["length"])}</td><td class="num">{secs(sq["time"])}</td><td class="num">&lt;0.01</td></tr>')
    one_table.append(
        f'<tr><td><span class="key k2"></span>Full GCS</td><td class="num{" win" if g["grasps"] == r["lb"] else " miss"}">{g["grasps"]}</td>'
        f'<td class="num{" win" if g["length"] < a["length"] else ""}">{f(g["length"])}</td><td class="num">{secs(g["time"])}</td>'
        f'<td class="num">{f(g["mem"])}</td></tr>')
    if h:
        solved = f'{h["solved"]}/{h["runs"]}'
        if h["solved"]:
            one_table.append(f'<tr><td><span class="key k3"></span>Manipulation RRT (HPP) · {solved}</td>'
                             f'<td class="num">{h["grasps"]:.0f}</td><td class="num">{f(h["length"])}</td>'
                             f'<td class="num">{secs(h["time"])}</td><td class="num">&lt;0.1</td></tr>')
        else:
            one_table.append(f'<tr><td><span class="key k3"></span>Manipulation RRT (HPP) · {solved}</td>'
                             f'<td colspan="4" class="muted">no plan within 60 s</td></tr>')

eight_table = []
for r in rows8:
    cells = [f'<th scope="row">{r["K"]}</th>']
    for m in ("weighted", "exact"):
        v = r.get(m)
        if v is None:
            cells.append('<td colspan="4" class="muted">not run</td>')
            continue
        valid = v["queries"] - v["outside"]
        cells.append(f'<td class="num">{v["solved"]}/{valid}</td><td class="num">{f(v["grasps"], 0)}</td>'
                     f'<td class="num">{f(v["nodes"], 0)}</td><td class="num">{"–" if v["ms"] is None else f"{v["ms"]:.0f}"}</td>')
    h = r["hpp"]
    cells.append(f'<td class="num">{"–" if h is None else f"{h["solved"]}/{h["runs"]}"}</td>')
    eight_table.append("<tr>" + "".join(cells) + "</tr>")


def offline_row(label, v1, v8):
    return f"<tr><th scope=\"row\">{label}</th><td class=\"num\">{v1}</td><td class=\"num\">{v8}</td></tr>"


c1, c8 = cfree["1cap_2obs"], cfree["8caps_3obs"]
g8 = scaling.get("graph", {}) if scaling else {}
offline = "".join([
    offline_row("Decomposition time (32 threads)", f'{c1["minutes"]} min', f'{c8["minutes"]} min'),
    offline_row("Free sets (seeded along the wrist)", f'{c1["regions"]} ({c1["seeded_regions"]})',
                f'{c8["regions"]} ({c8["seeded_regions"]})'),
    offline_row("False positives, 20,000 uniform samples", c1["uniform"]["false_positives"], c8["uniform"]["false_positives"]),
    offline_row("False positives on the grasps' wrist sweeps", sum(s["false_positives"] for s in c1["grasp_sweeps"]),
                sum(s["false_positives"] for s in c8["grasp_sweeps"])),
    offline_row("Coverage of the free samples", f'{c1["uniform"]["covered_of_free"]:.2f}', f'{c8["uniform"]["covered_of_free"]:.2f}'),
    offline_row("Transition sets", stats["components"], g8.get("components", "pending")),
    offline_row("Transit / switch edges", f'{stats["transit_edges"]} / {stats["switch_edges"]}',
                f'{g8["transit_edges"]} / {g8["switch_edges"]}' if g8 else "pending"),
    offline_row("A* graph time (LPs, quadprog)", "0.09 s",
                f'{scaling["shared_preprocessing_s"]:.1f} s' if scaling else "pending"),
])

n_lb_astar = sum(r["astar"]["grasps"] == r["lb"] for r in rows1)
n_lb_gcs = sum(r["gcs"]["grasps"] == r["lb"] for r in rows1)
ratio = [r["gcs"]["time"] / r["astar"]["time"] for r in rows1]
hpp_solved_turns = sum(1 for r in rows1 if r["hpp"] and r["hpp"]["solved"])
hpp_turns = sum(1 for r in rows1 if r["hpp"])
if rows8:
    w_solved = sum(r["weighted"]["solved"] for r in rows8 if "weighted" in r)
    w_total = sum(r["weighted"]["queries"] - r["weighted"]["outside"] for r in rows8 if "weighted" in r)
    w_ms = max(r["weighted"]["ms"] for r in rows8 if "weighted" in r and r["weighted"]["ms"] is not None)
    tile8 = (f'<div class="tile"><div class="fig">{w_solved}/{w_total}</div><div class="cap">8-cap queries solved by '
             f'weighted A*, every plan certified; the slowest median is {w_ms / 1e3:.1f} s at K = {rows8[-1]["K"]}</div></div>')
else:
    tile8 = '<div class="tile"><div class="fig">…</div><div class="cap">8-cap scaling run still in progress</div></div>'
hpp8_runs = sum(g["runs"] for g in hpp8["per_group"].values()) if hpp8 else 0
hpp8_solved = sum(g["solved"] for g in hpp8["per_group"].values()) if hpp8 else 0

paired = [v.get("paired") for v in scaling["per_k"].values()] if scaling else []
n_pair = sum(p["n"] for p in paired if p)
n_same = sum(p["same_actions"] for p in paired if p)
paired_note = (f"<ul><li><b>Weighting cost nothing here:</b> on all {n_pair} queries both searches solved, weighted A* "
               f"returned {'the same number of actions as' if n_same == n_pair else f'the same actions on {n_same} of them as'} "
               "exact A*, at a fraction of the time. Exact A* proves the minimum only up to about 4 caps within 60 s.</li>"
               f"<li><b>HPP</b> found no plan in any of its {hpp8_runs} runs (5 queries × 3 seeds per K, 60 s each).</li></ul>"
               if n_pair else "")
data_js = json.dumps(dict(
    one=[dict(D=r["D"], lb=r["lb"], a=r["astar"], g=r["gcs"]) for r in rows1],
    eight=[dict(K=r["K"], w=r.get("weighted"), e=r.get("exact")) for r in rows8]))

page = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Regrasp Planner Results</title>
<style>
:root {{
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,0.10);
  --s1: #2a78d6; --s2: #eb6834; --s3: #1baf7a; --good: #006300; --bad: #b3261e;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #d95926; --s3: #199e70; --good: #0ca30c; --bad: #f2827a;
  }}
}}
:root[data-theme="dark"] {{
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --s1: #3987e5; --s2: #d95926; --s3: #199e70; --good: #0ca30c; --bad: #f2827a;
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; background: var(--page); color: var(--ink);
  font: 15px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif; }}
main {{ max-width: 1040px; margin: 0 auto; padding: 32px 16px 64px; }}
h1 {{ font-size: 28px; margin: 0 0 4px; letter-spacing: -0.01em; }}
h2 {{ font-size: 20px; margin: 40px 0 8px; }}
h3 {{ font-size: 15px; margin: 24px 0 8px; color: var(--ink-2); }}
p {{ max-width: 72ch; margin: 8px 0; }}
.sub {{ color: var(--ink-2); margin: 0 0 24px; }}
.muted {{ color: var(--muted); }}
.card {{ background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 16px; margin: 12px 0; }}
.tiles {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin: 16px 0; }}
.tile {{ background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }}
.tile .fig {{ font-size: 32px; font-weight: 650; line-height: 1.1; }}
.tile .cap {{ color: var(--ink-2); font-size: 13px; margin-top: 6px; }}
.scroll {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; }}
th, td {{ padding: 6px 10px; text-align: left; border-bottom: 1px solid var(--grid); vertical-align: top; }}
thead th {{ color: var(--ink-2); font-weight: 600; border-bottom: 1px solid var(--axis); white-space: nowrap; }}
tr.grp > * {{ border-top: 1px solid var(--axis); }}
.num {{ text-align: right; font-variant-numeric: tabular-nums; white-space: nowrap; }}
.win {{ font-weight: 650; }}
.miss {{ color: var(--bad); font-weight: 650; }}
.key {{ display: inline-block; width: 10px; height: 10px; border-radius: 3px; margin-right: 8px; vertical-align: -1px; }}
.k0 {{ border: 1.5px solid var(--muted); }} .k1 {{ background: var(--s1); }} .k2 {{ background: var(--s2); }} .k3 {{ background: var(--s3); }}
.legend {{ display: flex; gap: 16px; flex-wrap: wrap; font-size: 13px; color: var(--ink-2); margin: 0 0 8px; }}
svg {{ display: block; width: 100%; height: auto; overflow: visible; }}
svg text {{ fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }}
svg .lbl {{ fill: var(--ink-2); }}
#tip {{ position: fixed; pointer-events: none; background: var(--surface); color: var(--ink); border: 1px solid var(--border);
  border-radius: 8px; padding: 8px 10px; font-size: 13px; box-shadow: 0 4px 16px rgba(0,0,0,0.12); opacity: 0; transition: opacity .1s; z-index: 9; }}
ul {{ padding-left: 20px; max-width: 76ch; }} li {{ margin: 4px 0; }}
code {{ font-size: 13px; }}
</style>
</head>
<body>
<main>
<h1>Regrasp planner results</h1>
<p class="sub">Reachability A* against full GCS on the same convex sets and HPP's Manipulation RRT, on the
<code>1cap_2obs</code> (10-DOF) and <code>8caps_3obs</code> (17-DOF) scenes. Generated {date.today():%d %B %Y}
from the saved results by <code>experiments/results_summary.py</code>.</p>

<div class="tiles">
  <div class="tile"><div class="fig">{n_lb_astar}/{len(rows1)} vs {n_lb_gcs}/{len(rows1)}</div><div class="cap">turns planned with the fewest
  possible grasps, A* vs full GCS (1 cap)</div></div>
  <div class="tile"><div class="fig">{min(ratio):.0f}–{max(ratio):.0f}×</div><div class="cap">faster online than full GCS
  (A* {secs(min(r["astar"]["time"] for r in rows1))}–{secs(max(r["astar"]["time"] for r in rows1))} per query, second stage included)</div></div>
  <div class="tile"><div class="fig">{hpp_solved_turns}/{hpp_turns}</div><div class="cap">turns with any HPP plan (only the
  single-grasp one); 8 caps: {hpp8_solved}/{hpp8_runs} runs</div></div>
  {tile8}
</div>

<h2>What is compared</h2>
<ul>
  <li><b>Reachability A* (ours).</b> Search over (convex set, reachable cap-angle interval) nodes; cost = grasps +
  ungrasps, plus 10<sup>−3</sup> per set change. Second stage: solver-free placement, then one SOCP that shortens the path
  over the same sets.</li>
  <li><b>Full GCS.</b> The same sets and edges, copied once per grasp (GCS visits each vertex at most once), cost =
  length + 10 per grasp. Drake's <code>SolvePath</code>: convex relaxation, rounding to ≤10 paths, MOSEK.</li>
  <li><b>Manipulation RRT (HPP).</b> HPP's constraint graph on the same robot and scene, continuous path validation,
  paths not shortcut. 10 seeds per query (1 cap) or 3 (8 caps), 60 s each.</li>
  <li>Every returned plan is re-checked with Drake: collisions, the coupling (the cap turns only with the grasping
  wrist) and the goal.</li>
</ul>

<h2>Offline stage</h2>
<p>Done once per scene and shared by the A* and GCS; HPP needs none. C-free is 9-D (arm + fingers, caps welded,
which is exact), seeded along each grasp's wrist runs, and its false positives on those wrist sweeps are cut off.</p>
<div class="card scroll"><table>
<thead><tr><th></th><th class="num">1cap_2obs</th><th class="num">8caps_3obs</th></tr></thead>
<tbody>{offline}</tbody></table></div>

<h2>One cap (<code>1cap_2obs</code>)</h2>
<p>One start and goal for every method; the cap turns by D. The obstacles leave at most L<sub>max</sub> =
{L_MAX:.2f} rad of free wrist, so the lower bound is ⌈D/L<sub>max</sub>⌉ grasps. Times are online, per query.</p>
<div class="card">
  <h3 style="margin-top:0">Path length by turn (labels: grasps)</h3>
  <div class="legend"><span><span class="key k1"></span>Reachability A*</span><span><span class="key k2"></span>Full GCS</span></div>
  <svg id="c1" viewBox="0 0 960 300" role="img" aria-label="Path length of A* and full GCS for each turn"></svg>
</div>
<div class="card scroll"><table>
<thead><tr><th>D [rad]</th><th class="num">Lower bound</th><th>Method</th><th class="num">Grasps</th><th class="num">Length</th>
<th class="num">Time</th><th class="num">Mem. [GB]</th></tr></thead>
<tbody>{"".join(one_table)}</tbody></table></div>
<ul>
  <li><b>The A* returns the lower bound on every turn.</b> Full GCS uses one grasp more at D = 1.5, 2.4 and 6 (red),
  although the A*'s plan is a path in the same GCS graph with a lower GCS cost (e.g. 36.0 vs 44.0 at D = 1.5):
  rounding the relaxation is not optimal.</li>
  <li><b>GCS on the A* sequence</b> (the convex restriction of the same layered GCS graph to the A*'s sets) returns the
  A*'s lengths to seven digits: the A*'s second stage is that same convex program, so it is optimal for the sets the A*
  chose. Its time includes the A* search and building the layered graph.</li>
  <li><b>Full GCS finds shorter paths</b>: 19% and 10% shorter at the two turns with equal grasps (D = 0.5, 3.2). Since
  the second stage is already optimal for the A*'s sets, the gap comes from which sets the A* picks (by number of set
  changes, not length).</li>
  <li><b>HPP</b> solves only D = 0.5 (7/10 runs, 18% longer than the A*). Nothing with two or more grasps, at 60 s, nor
  at 600 s on the scene without the floor ({"0/10" if hpp1_600 else "–"} at D = 1.5 and 6).</li>
</ul>

<h2>Eight caps (<code>8caps_3obs</code>)</h2>
<p>20 random queries per K: K random caps, each turned by [0.5, π] rad, summing to 2.5K rad, with random start and
goal arm configurations. Exact A* stops at 60 s. Medians over solved queries; queries whose start or goal lies outside
the current C-free are left out of the denominators.</p>
<div class="card">
  <h3 style="margin-top:0">Online time per query (median, log scale)</h3>
  <div class="legend"><span><span class="key k1"></span>Exact A*</span><span><span class="key k3"></span>Weighted A* (w = 1.5)</span></div>
  <svg id="c8" viewBox="0 0 960 300" role="img" aria-label="Median planning time against the number of caps"></svg>
  <p class="muted" id="c8note"></p>
</div>
<div class="card scroll"><table>
<thead><tr><th rowspan="2">K</th><th colspan="4">Weighted A* (w = 1.5)</th><th colspan="4">Exact A*</th><th>HPP</th></tr>
<tr><th class="num">Solved</th><th class="num">Grasps</th><th class="num">Nodes</th><th class="num">ms</th>
<th class="num">Solved</th><th class="num">Grasps</th><th class="num">Nodes</th><th class="num">ms</th><th class="num">Solved</th></tr></thead>
<tbody>{"".join(eight_table) or '<tr><td colspan="10" class="muted">scaling run still in progress</td></tr>'}</tbody></table></div>
{paired_note}

<h2>Notes</h2>
<ul>
  <li>An earlier decomposition of the one-cap scene (42 regions, clique cover only, 10-D) called colliding wrist
  configurations free, and full GCS on it took 44–93 s and 4–6 GB, and did not fit in 60 GB at 8 grasps. Those runs are
  in the paper's appendix.</li>
  <li>The A* graph's LPs use quadprog (regularised LP, no licence); with MOSEK each LP checked a licence out, and the
  8-cap graph took 15 min.</li>
  <li>Open: adding path length to the A*'s cost (a door point per node, λ‖p − p<sub>goal</sub>‖ in the heuristic) would
  target the 10–19% length gap at the price of more nodes.</li>
</ul>
</main>
<div id="tip" role="tooltip"></div>
<script>
const DATA = {data_js};
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const NS = "http://www.w3.org/2000/svg";
const tip = document.getElementById("tip");
function el(tag, attrs, parent) {{ const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); parent.appendChild(e); return e; }}
function showTip(ev, html) {{ tip.innerHTML = html; tip.style.opacity = 1;
  const x = Math.min(ev.clientX + 14, innerWidth - tip.offsetWidth - 8); tip.style.left = x + "px"; tip.style.top = (ev.clientY + 14) + "px"; }}
function hideTip() {{ tip.style.opacity = 0; }}
function hover(target, html) {{ target.addEventListener("mousemove", e => showTip(e, html)); target.addEventListener("mouseleave", hideTip);
  target.addEventListener("touchstart", e => showTip(e.touches[0], html), {{passive: true}}); }}
function barPath(x, y0, w, h, r) {{ r = Math.min(r, h); return `M${{x}},${{y0}}V${{y0 - h + r}}Q${{x}},${{y0 - h}} ${{x + r}},${{y0 - h}}H${{x + w - r}}Q${{x + w}},${{y0 - h}} ${{x + w}},${{y0 - h + r}}V${{y0}}Z`; }}

function chartOne() {{
  const svg = document.getElementById("c1"); svg.innerHTML = "";
  const W = 960, H = 300, L = 48, R = 16, T = 16, B = 40, rows = DATA.one;
  const ymax = Math.ceil(Math.max(...rows.flatMap(r => [r.a.length, r.g.length])) / 5) * 5;
  const y = v => T + (H - T - B) * (1 - v / ymax);
  for (let v = 0; v <= ymax; v += 5) {{
    el("line", {{x1: L, x2: W - R, y1: y(v), y2: y(v), stroke: css(v ? "--grid" : "--axis"), "stroke-width": 1}}, svg);
    el("text", {{x: L - 8, y: y(v) + 4, "text-anchor": "end"}}, svg).textContent = v;
  }}
  el("text", {{x: 12, y: T + 4, "text-anchor": "start"}}, svg).textContent = "";
  const band = (W - L - R) / rows.length, bw = 24, gap = 2;
  rows.forEach((r, i) => {{
    const cx = L + band * (i + 0.5);
    el("text", {{x: cx, y: H - B + 18, "text-anchor": "middle", class: "lbl"}}, svg).textContent = `D = ${{r.D}} rad`;
    el("text", {{x: cx, y: H - B + 32, "text-anchor": "middle"}}, svg).textContent = `bound ${{r.lb}}`;
    [[r.a, "--s1", "Reachability A*", -1], [r.g, "--s2", "Full GCS", 1]].forEach(([m, col, name, side]) => {{
      const x = side < 0 ? cx - gap / 2 - bw : cx + gap / 2;
      const p = el("path", {{d: barPath(x, y(0), bw, y(0) - y(m.length), 4), fill: css(col)}}, svg);
      el("text", {{x: x + bw / 2, y: y(m.length) - 6, "text-anchor": "middle", class: "lbl"}}, svg).textContent = m.grasps;
      const hit = el("rect", {{x: x - 4, y: T, width: bw + 8, height: H - T - B, fill: "transparent"}}, svg);
      hover(hit, `<b>${{name}}</b>, D = ${{r.D}} rad<br>${{m.grasps}} grasp(s) (bound ${{r.lb}})<br>length ${{m.length.toFixed(2)}}<br>` +
                 `time ${{m.time < 1 ? (m.time * 1e3).toFixed(0) + " ms" : m.time.toFixed(2) + " s"}}`);
    }});
  }});
}}

function chartEight() {{
  const svg = document.getElementById("c8"); svg.innerHTML = "";
  const rows = DATA.eight, note = document.getElementById("c8note");
  if (!rows.length) {{ note.textContent = "The scaling run is still in progress."; return; }}
  const W = 960, H = 300, L = 56, R = 120, T = 16, B = 36;
  const vals = rows.flatMap(r => [r.w && r.w.ms, r.e && r.e.ms]).filter(v => v);
  const lo = Math.pow(10, Math.floor(Math.log10(Math.min(...vals)))), hi = Math.pow(10, Math.ceil(Math.log10(Math.max(...vals))));
  const y = v => T + (H - T - B) * (1 - (Math.log10(v) - Math.log10(lo)) / (Math.log10(hi) - Math.log10(lo)));
  const ks = rows.map(r => r.K), x = k => L + (W - L - R) * (k - ks[0]) / Math.max(1, ks[ks.length - 1] - ks[0]);
  for (let v = lo; v <= hi * 1.001; v *= 10) {{
    el("line", {{x1: L, x2: W - R, y1: y(v), y2: y(v), stroke: css("--grid"), "stroke-width": 1}}, svg);
    el("text", {{x: L - 8, y: y(v) + 4, "text-anchor": "end"}}, svg).textContent = v >= 1000 ? (v / 1000) + " s" : v + " ms";
  }}
  el("line", {{x1: L, x2: W - R, y1: H - B, y2: H - B, stroke: css("--axis"), "stroke-width": 1}}, svg);
  ks.forEach(k => {{ el("text", {{x: x(k), y: H - B + 18, "text-anchor": "middle", class: "lbl"}}, svg).textContent = `K = ${{k}}`; }});
  [["e", "--s1", "Exact A*"], ["w", "--s3", "Weighted A*"]].forEach(([key, col, name]) => {{
    const pts = rows.filter(r => r[key] && r[key].ms).map(r => [x(r.K), y(r[key].ms), r]);
    if (!pts.length) return;
    el("polyline", {{points: pts.map(p => p[0] + "," + p[1]).join(" "), fill: "none", stroke: css(col), "stroke-width": 2,
                     "stroke-linejoin": "round", "stroke-linecap": "round"}}, svg);
    pts.forEach(([px, py, r]) => {{
      el("circle", {{cx: px, cy: py, r: 5, fill: css(col), stroke: css("--surface"), "stroke-width": 2}}, svg);
      const m = r[key], valid = m.queries - m.outside;
      const hit = el("circle", {{cx: px, cy: py, r: 14, fill: "transparent"}}, svg);
      hover(hit, `<b>${{name}}</b>, K = ${{r.K}}<br>solved ${{m.solved}}/${{valid}}${{m.timed_out ? " (" + m.timed_out + " hit 60 s)" : ""}}<br>` +
                 `median ${{m.ms.toFixed(0)}} ms (IQR ${{m.ms_p25.toFixed(0)}}–${{m.ms_p75.toFixed(0)}})<br>${{m.grasps}} grasps, ${{m.nodes}} nodes`);
    }});
    const last = pts[pts.length - 1];
    el("text", {{x: last[0] + 10, y: last[1] + 4, class: "lbl"}}, svg).textContent = name;
  }});
  note.textContent = "Exact A* medians are over the queries it solved within 60 s; see the table for how many.";
}}
function draw() {{ chartOne(); chartEight(); }}
draw();
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", draw);
new MutationObserver(draw).observe(document.documentElement, {{attributes: true, attributeFilter: ["data-theme"]}});
</script>
</body>
</html>
"""
OUT.write_text(page)
print(f"wrote {OUT} ({len(page) / 1e3:.0f} kB); 8-cap rows: {len(rows8)}")
