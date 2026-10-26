"""Re-check every HPP run with Drake, and write summary.json and table.tex (hpp_comparison).

For each saved path (raw/*.npz from hpp_plan.py --stop-after batch): Drake's collision check at every sample,
that a cap only turns while grasped and then with the hand, and that the path ends at the goal; plus the
number of grasps and the length in (x, alpha), as in the paper's Table III. Run with the obmp conda env, from
the repo root:
  python experiments/hpp_comparison/check_and_report.py --scene-dir artifacts/hpp_comparison/1cap_2obs
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from export_scene import DrakeChecker, build_scene   # noqa: E402

# contact that the security margins of hpp_plan.py allow on the last approach (step 5): hand and fingers
# against the cap, its rotating part and its base (export_scene.py puts both on cap_link for HPP)
APPROACH_CONTACT = {frozenset((b, c)) for b in ("panda_hand", "panda_leftfinger", "panda_rightfinger")
                    for c in ("cap_link", "base_link")}


def certify(Q, S, OK, q_goal, scene, checker):
    """Drake's check of one HPP path: Q samples (Drake order), S their graph states.
    - collisions: samples in collision (Drake's rule), other than the allowed approach contact;
    - cap_moved_unheld: steps where a cap turns while not grasped;
    - grasp_slip: while cap k is grasped, the largest change of the hand's pose relative to cap k (the cap
      turns with the hand: HPP lets the whole arm move during a grasp, not only the wrist);
    - goal_err: distance of the last sample to the goal;
    - eval_failures: samples HPP could not evaluate (its projection onto the constraints failed)."""
    plant, models, ctx = scene["plant"], scene["models"], checker.plant_context
    E = plant.GetFrameByName("panda_hand", models["hand"])
    collisions = contacts = 0
    for q in Q:
        out_of_limits = bool(np.any(q < checker.lo - 1e-9) or np.any(q > checker.hi + 1e-9))
        pairs = {frozenset(p) for p in checker.colliding_bodies(q)}
        if out_of_limits or pairs - APPROACH_CONTACT:
            collisions += 1
        elif pairs:
            contacts += 1
    dQ = np.diff(Q, axis=0)
    unheld, slip = 0, 0.0
    for k, cap in enumerate(models["caps"]):
        i = plant.GetJointByName("cap_to_base", cap).position_start()
        grasped = np.array([f"grasps cap{k}/handle" in s for s in S])
        unheld += int(np.sum((np.abs(dQ[:, i]) > 1e-9) & ~(grasped[:-1] | grasped[1:])))
        C = plant.GetFrameByName("cap_link", cap)
        X_ref = None
        for q, g in zip(Q, grasped):
            if not g:
                X_ref = None
                continue
            plant.SetPositions(ctx, q)
            X = plant.CalcRelativeTransform(ctx, C, E).GetAsMatrix4()
            X_ref = X if X_ref is None else X_ref
            slip = max(slip, float(np.abs(X - X_ref).max()))
    goal_err = float(np.abs(Q[-1] - q_goal).max())
    return dict(collisions=collisions, approach_contacts=contacts, cap_moved_unheld=unheld, grasp_slip=slip,
                goal_err=goal_err, eval_failures=int(np.sum(~OK)), grasps=int(sum(" grasps " in b and " grasps " not in a for a, b in zip(S[:-1], S[1:]))),
                length=float(np.linalg.norm(dQ, axis=1).sum()),
                certified=bool(collisions == 0 and unheld == 0 and slip < 1e-3 and goal_err < 1e-4 and OK.all()))


def stats(rs):
    ok = [r for r in rs if r["success"]]
    med = lambda key: float(np.median([r[key] for r in ok])) if ok else None
    pct = lambda p: float(np.percentile([r["time_s"] for r in ok], p)) if ok else None
    return dict(runs=len(rs), solved=len(ok), certified=sum(r["certified"] for r in ok), time_s=med("time_s"),
                time_s_q1=pct(25), time_s_q3=pct(75), grasps=med("grasps"), length=med("length"), mem_gb=med("mem_gb"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--scene-dir", default="artifacts/hpp_comparison/1cap_2obs")
    D = Path(ap.parse_args().scene_dir).resolve()
    meta = json.loads((D / "scene.json").read_text())
    scene = build_scene(meta["scene"])
    checker = DrakeChecker(scene)
    queries = {q["name"]: q for q in json.loads((D / "queries.json").read_text())}
    runs = []
    for f in sorted((D / "raw").glob("*.npz")):
        z = np.load(f)
        r = dict(query=str(z["query"]), seed=int(z["seed"]), success=bool(z["success"]), error=str(z["error"]),
                 time_s=float(z["time_s"]), roadmap_nodes=int(z["roadmap_nodes"]), mem_gb=float(z["mem_gb"]))
        if r["success"]:
            r.update(certify(z["path_q"], z["path_states"], z["path_eval_ok"], np.array(queries[r["query"]]["q_goal"]), scene, checker))
        runs.append(r)
        print(r["query"], r["seed"], "solved" if r["success"] else "failed",
              {k: r[k] for k in ("certified", "grasps", "collisions", "approach_contacts", "grasp_slip", "eval_failures") if k in r})
    # 1cap_2obs: one row per turn D; 8caps_3obs: one row per number of active caps K
    group = (lambda q: q["turn"]) if meta["scene"] == "1cap_2obs" else (lambda q: q["k"])
    keys = sorted({group(queries[r["query"]]) for r in runs})
    per = {k: stats([r for r in runs if group(queries[r["query"]]) == k]) for k in keys}
    invalid = [n for n, q in queries.items() if q["q_init_in_collision"] or q["q_goal_in_collision"]]
    if invalid:
        print(f"queries skipped (start or goal in collision, Drake): {invalid}")
    (D / "summary.json").write_text(json.dumps(dict(scene=meta["scene"], invalid_queries=invalid, per_group=per,
                                                    runs=runs), indent=1))
    fmt = lambda v, f: "--" if v is None else format(v, f)
    rows = [f"{k:g} & M-RRT (HPP), {s['solved']}/{s['runs']} & {fmt(s['grasps'], '.0f')} & {fmt(s['length'], '.2f')}"
            f" & {fmt(s['time_s'], '.2f')} & {fmt(s['mem_gb'], '.3f')}\\\\" for k, s in per.items()]
    (D / "table.tex").write_text("\n".join(rows) + "\n")
    print("\n".join(rows))
