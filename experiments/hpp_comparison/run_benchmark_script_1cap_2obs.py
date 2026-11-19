"""Run the HPP 1cap_2obs benchmark: all continuous runs, then all discrete runs, D increasing.

Each (collision_check, D) is a separate call of hpp_1cap_2obs.py, so a crash only loses that call.
If a D has 0 successes, larger D values for the same collision check are skipped (out_of_limit).
"""
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from unicodedata import name
import numpy as np

HERE = Path(__file__).resolve().parent
OUT_ROOT = HERE.parents[1] / "artifacts" / "hpp_comparison" / "1cap_2obs"


DS = [0.5, 1, 2, 3, 4, 5, 6]
COLLISION_CHECKS = ["continuous", "discrete"]
N = 20

out_dir = OUT_ROOT / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
out_dir.mkdir(parents=True, exist_ok=True)
summary = []


def stats(values):
    if not values:
        return None
    return {"mean": float(np.mean(values)), 
            "median": float(np.median(values)),
            "std": float(np.std(values))}


def save_summary():
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)


for check in COLLISION_CHECKS:
    out_of_limit = False
    for D in DS:
        entry = {"collision_check": check, "D": D}
        if out_of_limit:
            entry["status"] = "out_of_limit"
            summary.append(entry)
            save_summary()
            continue
        
        # same file name the HPP script writes
        filename = f"{check}_D{float(D)}"  
        print(f"Running {filename} ...", flush=True)
        
        # Run the hpp script and log its output in <filename>.log
        with open(out_dir / f"{filename}.log", "w") as log:
            proc = subprocess.run(
                [sys.executable, "hpp_1cap_2obs.py", "-N", str(N), "-D", str(D),
                 "--collision_check", check, "--out_dir", str(out_dir)],
                cwd=HERE, stdout=log, stderr=subprocess.STDOUT)

        # Retrieve the results from the JSON file written by the HPP script
        json_file = out_dir / f"{filename}.json"
        result = json.loads(json_file.read_text()) if json_file.exists() else {"runs": []}
        runs = result["runs"]
        ok_runs = [r for r in runs if r["success"]]

        # Report crash, early stop (because of multiple failures), or success.
        if proc.returncode != 0:
            entry["status"] = "crashed"
        elif result.get("stopped_before_N_runs"):
            entry["status"] = "stopped_before_N_runs"
        else:
            entry["status"] = "ok"
          
        # Update the summary with the statistics of the runs.  
        entry.update(
            number_of_runs_N=N,
            runs_attempted=len(runs),
            successes=len(ok_runs),
            success_rate=len(ok_runs) / len(runs) if runs else None,
            search_time=stats([r["search_time"] for r in ok_runs]),
            node_number=stats([r["node_number"] for r in ok_runs]),
            pathlength_raw=stats([r["pathlength_raw"] for r in ok_runs]),
            pathlength_shortcut=stats([r["pathlength_shortcut"] for r in ok_runs]),
        )
        summary.append(entry)
        save_summary()
        print(f"  {entry['status']}: {len(ok_runs)}/{len(runs)} successes", flush=True)

        # Skip larger D for this collision check if this D had no successes.
        if not ok_runs and entry["status"] != "crashed":
            out_of_limit = True

print(f"Done. Results in {out_dir}")
