# Manipulation RRT (HPP) on the cap scenes

The sampling-based baseline of the paper: HPP's Manipulation RRT (`pyhpp.manipulation.ManipulationPlanner`), on
the same scenes and queries as the reachability A*. 1cap_2obs uses Run A of `experiments/gcs_comparison`; the
8-cap scene uses the queries of `experiments/cap_scaling`. [GUIDE.md](GUIDE.md) walks through the code step by
step against the HPP tutorials.

| file | runs in | what it does |
|---|---|---|
| `env/` | — | HPP's tutorial-1 environment, copied: `Dockerfile` (no midori), `config.sh`, `run_docker.sh` (also mounts the repo at `/home/user/obmp`), `src/Makefile` (16 jobs). HPP is built from source into `env/install`; `env/hpp_versions.txt` lists the commits |
| `export_scene.py` | obmp env | builds the scene as the notebooks do, and writes URDF/SRDF, the queries and Drake's reference set |
| `hpp_plan.py` | `env/run_docker.sh` | constraint graph + Manipulation RRT; `--stop-after robot\|env\|cap\|graph\|waypoints\|solve\|batch` |
| `check_and_report.py` | obmp env | re-checks every path with Drake (collisions, coupling, goal); writes `summary.json` and `table.tex` |

## Run

```bash
# once: the image and the HPP build (as in tutorial 1)
cd experiments/hpp_comparison/env
docker build --build-arg DOCKER_USER=`id -u` --build-arg DOCKER_GROUP=`id -g` -t hpp:obmp -f Dockerfile .
./run_docker.sh            # then, inside: cd /home/user/devel/src && make all

# 1cap_2obs (host, obmp env, repo root)
python experiments/hpp_comparison/export_scene.py --scene 1cap_2obs
# 1cap_2obs (container, from /home/user/obmp)
python experiments/hpp_comparison/hpp_plan.py --stop-after batch --seeds 10 --time-limit 60
# 1cap_2obs (host)
python experiments/hpp_comparison/check_and_report.py --scene-dir artifacts/hpp_comparison/1cap_2obs

# 8 caps: the same, with the queries of cap_scaling
python experiments/cap_scaling/cap_scaling.py --out artifacts/cap_scaling_hpp --no-exact --no-render --save-queries
python experiments/hpp_comparison/export_scene.py --scene 8caps_3obs --queries artifacts/cap_scaling_hpp/queries.npz
python experiments/hpp_comparison/hpp_plan.py --scene-dir /home/user/obmp/artifacts/hpp_comparison/8caps_3obs --stop-after batch ...
python experiments/hpp_comparison/check_and_report.py --scene-dir artifacts/hpp_comparison/8caps_3obs
```

## Model

- **Grasp:** the gripper frame (`panda_hand`) on the cap's handle, 0.1075 m above the cap (the middle of
  the A*'s band), pointing down. The rotation about the cap axis is free (handle mask), so any yaw can be
  grasped; HPP then holds it constant while grasping, so the cap turns with the wrist.
- **Transit:** the cap has no contact surface, so the constraint graph factory locks its joint outside a
  grasp, at whatever angle it has.
- **Fingers:** locked at the grasp width (±0.02405) everywhere. Contact between hand and cap is allowed on
  the last approach and the first retreat only, through security margins as in tutorial 3.
- **Collision geometry and filters:** Drake's. `hpp_plan.py --stop-after cap` checks HPP against Drake:
  hand pose and collision verdict on 500 random configurations.
- **Motions are checked continuously:** `Dichotomy` with tolerance 0 as each transition's path-validation
  factory, and a `ProgressiveProjector` (0.01) for motions in constrained states, as in HPP's ur3-spheres
  example. HPP's default (discretized at 0.05, no projector) returned plans that Drake rejects; see
  GUIDE.md, step 6b.

## What is timed

`planner.solve()` per run, with `--time-limit` (60 s) and `--seeds` (10) runs per query. The memory is the
peak extra resident memory of the run. Paths are raw, not shortcut. Every path is re-checked with Drake
(`check_and_report.py`).
