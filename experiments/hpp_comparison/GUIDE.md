# Guide: the cap baseline, step by step against the HPP tutorials

This guide shows how each piece of the cap baseline comes from the tutorials:
<https://humanoid-path-planner.github.io/hpp-doc/reference/hpp-tutorial/>. Tutorial 2 (pick and place)
gives almost everything; tutorial 3 adds waypoints and security margins; HPP's own Manipulation RRT example
(`hpp-python/tests/integration/ur3-spheres.py`) adds how motions are checked.

Each step below gives:

- **Tutorial:** the section it comes from.
- **Ours:** what changed for the cap, and why.
- **Code:** the lines, linked.
- **Run:** the command.
- **Checked:** what the checkpoint showed when it was run.

**Where you run things.** There are two environments. Every `hpp_plan.py` command in this guide runs
inside the HPP container; everything else runs on your machine.

| script | where | how to get there |
|---|---|---|
| `export_scene.py`, `check_and_report.py`, `cap_scaling.py` | host | `conda activate obmp`, from the repo root |
| `hpp_plan.py` | container | `experiments/hpp_comparison/env/run_docker.sh`, then `cd /home/user/obmp` |

```bash
# host terminal, repo root
experiments/hpp_comparison/env/run_docker.sh      # opens a shell inside the container (prompt: user@...)
# now inside the container
cd /home/user/obmp                                # the repo, as seen from the container
python -i experiments/hpp_comparison/hpp_plan.py --stop-after robot
```

- A second terminal into the same container: `docker exec -it hpp-obmp bash`.
- The viewer is at the HTTP address viser prints in its box when the script starts (usually
  <http://localhost:8000>), in your normal browser; the container uses the host network. It shows the robot
  with Drake's Franka meshes, at the first query's start configuration.
- If you see `No module named 'pinocchio'` (or `pyhpp`), you ran `hpp_plan.py` on the host instead of in
  the container.

The planner is HPP's **Manipulation RRT** (`ManipulationPlanner`), the one tutorial 2 uses. It grows an RRT
over the *constraint graph*: states such as "free" and "grasping cap 0", and transitions such as "grasp",
"release", "move while holding". This graph is Siméon's manipulation graph.

---

## Step 0. The environment (tutorial 1)

**Tutorial:** build the `hpp:tuto` image, download `config.sh`, `src/Makefile` and `run_docker.sh`, then
`make all` in the container. That compiles HPP into `install/`.

**Ours:** the same files, copied into [env/](env/), so the baseline is separate from your tutorial folder
(left untouched). Three edits, each against your copy in `experiments/hpp_tutorials`:

- `env/Dockerfile`: the three midori lines removed. With `--net host`, your own browser reaches the viewer,
  so there's no 91 MB `.deb` in the repo.
- `env/run_docker.sh`: also mounts the repo as `/home/user/obmp`, works from any directory, and uses
  container `hpp-obmp` and image `hpp:obmp`, so it can run next to the tutorial's `hpp`.
- `env/src/Makefile`: `BUILD_JOBS=16` instead of 4.

**Run (once):**

```bash
cd experiments/hpp_comparison/env
docker build --build-arg DOCKER_USER=`id -u` --build-arg DOCKER_GROUP=`id -g` -t hpp:obmp -f Dockerfile .
./run_docker.sh
cd /home/user/devel/src && make all            # inside the container
```

**Checked:**

- The image reused `hpp:tuto`'s cached layers (~10 s).
- `make all` finished with exit 0.
- `ManipulationPlanner` and `viser` import.
- [env/hpp_versions.txt](env/hpp_versions.txt) lists the commits built (devel branch, e.g. hpp-manipulation
  `1cc7070`, hpp-core `372535b9`, 2026-10-20). **Cite these in the paper.**

---

## Step 1. The robot (tutorial 2, "Building and displaying a robot")

**Tutorial:**

```python
robot = Device("tuto")
urdf.loadModel(robot, 0, "panda", "anchor", urdf_filename, srdf_filename, SE3.Identity())
```

The URDF is example-robot-data's Panda; the SRDF holds the `<gripper>`.

**Ours:** the same two calls ([hpp_plan.py:85-86](hpp_plan.py#L85-L86)), but the URDF/SRDF are *your*
Panda, written from your Drake plant by [export_scene.py](export_scene.py). `load_model`
([hpp_plan.py:71](hpp_plan.py#L71)) is `urdf.loadModel` on those files. It passes their text
(`loadModelFromString`), so it can first fill in the folder of the visual meshes (`@SCENE_DIR@/meshes`):
Drake's Franka glTF meshes, copied there by the export. The collision geometry stays Drake's spheres; the
meshes are only for the viewer. Your arm differs from
example-robot-data's: the wrist is limited to [−1, 1], the collision geometry is spheres, and the hand is
welded at −45°. Comparing planners on different robots would be meaningless.

- `build_scene` ([export_scene.py:32](export_scene.py#L32)) is the notebooks' scene code.
- `write_model` ([export_scene.py:143](export_scene.py#L143)) writes:
  - one `<link>` per Drake body, with its collision shapes;
  - one `<joint>` per Drake joint, with Drake's limits.
- `filtered_link_pairs` ([export_scene.py:184](export_scene.py#L184)) writes one SRDF `<disable_collisions>`
  per pair of links that Drake doesn't check, so the two planners check the same pairs.
- The gripper ([export_scene.py:226](export_scene.py#L226)) is the `panda_hand` frame, the frame your grasp
  constraint `h(q)` uses.

**Pitfall:** `loadModel` needs absolute paths, so the script resolves `--scene-dir`.

**Run:**

```bash
# 1. host terminal, repo root, obmp env: write the scene files
conda activate obmp
python experiments/hpp_comparison/export_scene.py --scene 1cap_2obs

# 2. still on the host: open the container
experiments/hpp_comparison/env/run_docker.sh

# 3. inside the container: load the robot in HPP, then check it against Drake
cd /home/user/obmp
python -i experiments/hpp_comparison/hpp_plan.py --stop-after robot
>>> check_fk()
```

Every later `hpp_plan.py` command is typed in this same container shell; leave it open between steps.

**Checked:** `check_fk()` ([hpp_plan.py:91](hpp_plan.py#L91)) compares HPP's hand pose with Drake's on 500
random configurations: **max difference 1.3e-8**.

---

## Step 2. The obstacles (tutorial 2, "Adding an obstacle")

**Tutorial:** `urdf.loadModel(robot, 0, "ground", "anchor", ground.urdf, ground.srdf, SE3.Identity())`.

**Ours:** the same call with `env.urdf` ([hpp_plan.py:105](hpp_plan.py#L105)). `export_env`
([export_scene.py:250](export_scene.py#L250)) puts every geometry fixed to the world in one link: on 1cap_2obs the
two posts beside the cap, on the 8-cap scene the walls and the floor.

**Run:** `python -i experiments/hpp_comparison/hpp_plan.py --stop-after env`. The two posts appear next to the
cap's position.

---

## Step 3. The cap (tutorial 2, "Adding an object", and its SRDF section)

**Tutorial:** the box is loaded as a `"freeflyer"`, gets `setJointBounds`, and its SRDF has a `<handle>`
(z pointing down, `approaching_direction="0 0 1"`) and a `<contact>` surface.

**Ours** ([hpp_plan.py:136-142](hpp_plan.py#L136-L142), files from `export_cap`,
[export_scene.py:270](export_scene.py#L270)):

- **`"anchor"`, not `"freeflyer"`.** The cap is bolted at its pose and moves only through its own revolute
  joint `cap_to_base`. No `setJointBounds`: the URDF's ±3.14 apply. One cap is loaded per pose, with
  prefix `cap0`, `cap1`, …
- **The handle** is the tutorial's box handle, on the rotating `cap_link`:

  ```xml
  <handle name="handle" clearance="0.05" approaching_direction="0 0 1">
    <position xyz="0 0 0.1075" wxyz="0 0 1 0"/>   <!-- middle of your band [0.105, 0.110]; z down -->
    <link name="cap_link"/>
  </handle>
  ```

  A grasp in HPP means "gripper frame = handle frame". The pre-grasp is 0.05 m back along the approach
  direction: the hand 5 cm above the cap.
- **No `<contact>`.** A cap is never picked up and put down; step 4 shows what replaces placement.
- **The mask (not in the tutorials).** It is the constraint that makes the cap turn:

  ```python
  HANDLE_MASK = [True, True, True, True, True, False]   # x y z rx ry rz: rotation about the cap axis free
  robot.handles()["cap0/handle"].mask = HANDLE_MASK
  ```

  A grasp is six equations, and the mask drops the one about the cap axis. That has two effects:
  - **Any yaw can be grasped,** as in your `h(q)`.
  - **The dropped component is held during a transfer.** It becomes the grasp's *complement*, which HPP
    keeps constant while the cap is held. So the cap turns with the hand: your coupling.
- **The base's cylinder sits on `cap_link`** (`move_geometry`, [export_scene.py:281](export_scene.py#L281)).
  It is coaxial with the cap joint, so its occupancy is the same at every angle. This lets step 5's
  security margins cover the whole cap.

**Pitfall:** a new `Problem` has **no configuration validation**: `problem.isConfigValid(q)` says `True`
even in collision. `new_problem` ([hpp_plan.py:109](hpp_plan.py#L109)) adds `JointBoundValidation` and
`CollisionValidation`. Step 6b covers the rest of `new_problem`.

**Run:**

```bash
python -i experiments/hpp_comparison/hpp_plan.py --stop-after cap
>>> check_collisions()
```

In the viewer, Display → "Show Frames", then keep only `panda/gripper` and `cap0/handle` (tutorial 2's
viewer tour).

**Checked:** `check_collisions()` ([hpp_plan.py:128](hpp_plan.py#L128)) compares HPP's and Drake's
verdicts on the 500 configurations: **500/500 agree** on 1cap_2obs, and 500/500 on the 8-cap scene.

---

## Step 4. The constraint graph (tutorial 2, "Defining a manipulation problem")

**Tutorial:**

```python
problem = Problem(robot); graph = Graph("robot", robot, problem); factory = ConstraintGraphFactory(graph)
graph.maxIterations(40); graph.errorThreshold(1e-5)
factory.setGrippers(["panda/gripper"])
factory.setObjects(["box"], [["box/handle"]], [["box/surface"]]); factory.environmentContacts(["ground/surface"])
factory.generate()
# fingers held open in every state:
lockedFingers = [LockedJoint(robot, f"panda/panda_finger_joint{i+1}", np.array([0.035]), cts) for i in range(2)]
graph.addNumericalConstraintsToGraph(lockedFingers)
graph.initialize()
```

**Ours:** `build_graph` ([hpp_plan.py:146](hpp_plan.py#L146)), the same calls with three differences:

1. **`setObjects(objects, handles, [[]])`: no contact surfaces.** In that case the factory's placement
   (`buildPlacement` in `pyhpp/manipulation/constraint_graph_factory.py`) locks every joint named `cap0/...`
   with a **parametric** `LockedJoint`. Its value is taken from the configuration where each motion starts.
   Outside a grasp, "the cap stays at whatever angle it has": your transit. Tutorial 3's plate also has no
   contacts.
2. **Fingers:** tutorial 2's `LockedJoint` lines at your grasp width, ±0.02405 instead of 0.035 open
   (`GRASP_FINGERS`, [hpp_plan.py:41](hpp_plan.py#L41)).
3. **Rules** ([hpp_plan.py:157](hpp_plan.py#L157)): only the query's active caps may be grasped, which
   matters for the 8-cap queries. `Rule([...], ["^$"], True)` keeps "holding nothing", the free state:
   combinations no rule matches are rejected.

**Parametric vs. fixed constraints.**

- `ComparisonType.EqualToZero`: the joint equals the given value (the fingers).
- The default `Equality`: the value is read from the configuration where each motion starts (the cap angle).

**Run:** `python -i experiments/hpp_comparison/hpp_plan.py --stop-after graph`. In the viewer, "Show Graph
Viewer", then right-click a state → "Generate random config".

**Checked:** the states are `free`, `panda/gripper > cap0/handle | f_pregrasp` (waypoint) and
`panda/gripper grasps cap0/handle`.

---

## Step 5. Waypoints, security margins, the coupling (tutorial 3)

**Tutorial 3, "Creating waypoint configurations":**

```python
res, qpg, err = graph.generateTargetConfig(transition_f_01, q_init, shooter.shoot())   # pre-grasp
res, qg, err = graph.generateTargetConfig(transition_f_12, qpg, qpg)                 # grasp from it
```

**Ours:** the same loop ([hpp_plan.py:183-212](hpp_plan.py#L183-L212)) on Run A's start configuration.

**Tutorial 3, "Using security margins":** the drill must touch the plate on the last approach motion, so
collision checking between them is switched off on that transition only:

```python
graph.setSecurityMarginForTransition(transition, "staubli/joint_6", "plate/root_joint", float("-inf"))
```

**Ours:** the same ([hpp_plan.py:165-171](hpp_plan.py#L165-L171)). The fingers are at grasp width with
~0.05 mm to spare around the cap, so the last approach (`..._12`) and the first retreat (`..._21`) touch
it. Margins are set between the hand and finger joints and `cap<k>/cap_to_base`, on those transitions only.

**Checked:**

- a pre-grasp and a grasp were found in 4 tries;
- the approach pre-grasp → grasp is valid;
- **the coupling sign:** `getConfigErrorForTransitionLeaf` on the grasp loop
  ([hpp_plan.py:210](hpp_plan.py#L210)) accepts the wrist +0.3 with the cap −0.3 (stays in the grasp:
  `True`), and rejects the opposite sign (`False`). That is Δα = −Δw, your convention.

**API notes:**

- `generateTargetConfig` returns `(ok, q, err)`.
- `getConfigErrorForTransitionLeaf` returns `(err, ok)`.
- `path.timeRange()` returns `.first` and `.second`.

---

## Step 6. Solve one query (tutorial 2, "Solving the manipulation problem")

**Tutorial:**

```python
problem.initConfig(q_init); problem.addGoalConfig(q_goal); problem.constraintGraph(graph)
manipulationPlanner = ManipulationPlanner(problem); manipulationPlanner.maxIterations(500)
p = manipulationPlanner.solve()
```

**Ours:** `solve` ([hpp_plan.py:216](hpp_plan.py#L216)), the same calls plus three things an experiment
needs:

- `timeOut(time_limit)` instead of 500 iterations.
- `ctypes.CDLL("libc.so.6").srand(seed)`: HPP samples with C's `std::rand`, so without a seed every run is
  identical.
- `try/except`: reaching the time limit raises.

The queries ([export_scene.py:294](export_scene.py#L294)) are:

- **1cap_2obs:** Run A's start and goal from `experiments/gcs_comparison`, with the cap turned from 3 to 3 − D,
  D ∈ {0.5, 1.5, 6}.
- **8 caps:** cap_scaling's queries (step 9).

The fingers are set to the grasp width, as the graph requires; Drake checks that the start and goal are
still collision-free.

**Run:**

```bash
python -i experiments/hpp_comparison/hpp_plan.py --stop-after solve --query D0p5 --seed 0
```

Then press Play in the viewer; the path is loaded, as in tutorial 2.

## Step 6b. How motions are checked (HPP's `ur3-spheres.py` example)

Tutorial 2 leaves this at HPP's defaults. For a benchmark the defaults are not enough, and Drake's re-check
(step 8) caught it.

**What the default does.** `new_problem` ([hpp_plan.py:109-125](hpp_plan.py#L109-L125)) has to change two
things:

- **Each transition builds its own path validation** from the problem's *factory*, with that transition's
  collision filters and security margins. The default factory is *discretized*: it checks configurations
  every 0.05 along a motion. A finger passed 4 mm into a post between two checks, and that plan was
  "solved".
- **A motion in a constrained state** (e.g. turning the cap while grasped) is a straight line projected
  onto the constraints when evaluated. Without a path projector, evaluating such a path can fail or jump:
  one "solved" 6 rad plan put the hand 3 cm under the floor between two samples.

**What we set**, as HPP's own Manipulation RRT example does:

```python
problem.pathValidationFactory(Dichotomy(robot, 0.0))   # continuous: certifies the whole motion, not samples
problem.pathProjector(ProgressiveProjector(problem.distance(), problem.steeringMethod(), 0.01))
```

**Pitfall:** `problem.pathValidation(Dichotomy(...))`, without "Factory", replaces the graph's per-transition
validation with one that knows neither the grasp nor the margins. Then no grasp is ever valid and every run
fails. It took reading `hpp-manipulation/src/problem.cc` and `graph/edge.cc` to see this.

---

## Step 7. Batch runs

`--stop-after batch` ([hpp_plan.py:279](hpp_plan.py#L279)) runs `solve` for every query and seed. For each
run it saves `raw/<query>_s<seed>.npz`:

- success, time, roadmap nodes, and peak memory (the sampler of `experiments/gcs_comparison`);
- the path sampled every 0.002, in Drake's joint order, with each sample's graph state and whether HPP could
  evaluate it.

Queries whose start or goal is in collision are skipped and reported (one 8-cap query, step 9).

```bash
python experiments/hpp_comparison/hpp_plan.py --stop-after batch --seeds 10 --time-limit 60    # container
```

---

## Step 8. Drake re-checks every path

[check_and_report.py](check_and_report.py) is in the obmp env. As for every method in the paper, each HPP
path is re-checked with Drake (`certify`, [check_and_report.py:25](check_and_report.py#L25)):

- **collisions:** with the project's rule (`check_collision_q_by_ik`) at every sample. Hand/finger contact
  with the cap counts as allowed contact (step 5's margins), not as a collision.
- **the cap turns only while grasped,** and then **with the hand.** The hand's pose relative to the cap
  must stay fixed (`grasp_slip`). HPP lets the whole arm move during a grasp, not only the wrist, so the
  check is on the hand, not Δα = −Δw.
- **every sample evaluates.**
- **the path ends at the goal.**
- **metrics:** grasps (entries into a grasp state) and length in (x, α), as in Table III.

```bash
python experiments/hpp_comparison/check_and_report.py --scene-dir artifacts/hpp_comparison/1cap_2obs       # host
```

It writes `summary.json` and `table.tex`, with one row per turn D (1cap_2obs) or per number of caps K (8 caps).

**What this check found, all fixed before the final runs:**

1. **Discretized validation and no projector** (step 6b): plans that passed through a post, or jumped.
2. **The fingers grazed the cap's base** on the last 1–4 mm of the approach, by up to 0.13 mm. The margins
   covered only `cap_link`, and the base is fixed to the world, where no joint margin can reach it. Moving
   the base's coaxial cylinder onto `cap_link` (step 3) fixed it.
3. **The coupling check was too strict.** It required Δα = −Δw exactly, but the cap turns with the hand;
   replaced by `grasp_slip`.

---

## Step 9. The 8-cap scene (`experiments/cap_scaling`)

`cap_scaling.py --save-queries` writes its queries to `queries.npz`. The queries depend only on `--seed`,
and a weighted-A*-only run reproduced all 160 of the paper's queries (same node expansions).

```bash
python experiments/cap_scaling/cap_scaling.py --out artifacts/cap_scaling_hpp --no-exact --no-render --save-queries
python experiments/hpp_comparison/export_scene.py --scene 8caps_3obs --queries artifacts/cap_scaling_hpp/queries.npz
python experiments/hpp_comparison/hpp_plan.py --scene-dir /home/user/obmp/artifacts/hpp_comparison/8caps_3obs \
  --stop-after batch --seeds 3 --time-limit 60 \
  --queries $(python -c "print(','.join(f'K{k}_q{n:02d}' for k in range(1, 9) for n in range(5)))")
python experiments/hpp_comparison/check_and_report.py --scene-dir artifacts/hpp_comparison/8caps_3obs
```

**Checked:**

- forward kinematics matches Drake to 1.4e-8, and collision verdicts agree on 500/500;
- **one query, `K8_q08`, has a goal in collision in Drake.** The hand is 3.3 cm inside `left_wall`, already
  in cap_scaling's original query: the A*'s free regions contain this configuration although Drake says it
  collides (the same region-fidelity issue as the paper's validation section). HPP skips it.

---

## Results

*(filled in from `summary.json` once the runs finish)*
