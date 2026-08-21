# Manual Test Plan

Before merging a PR into `main`, manually verify the section(s) below relevant to what the PR actually touched.

## Install / environment
Touches: `environment.yml`, `requirements.txt`, `README.md` (install section)
- Run `python test.py` - confirm it prints `pydrake imported successfully`.

## `algorithms/nlp_sampling/` (the NLP-sampling library)
Touches: `algorithms/nlp_sampling/*.py`
- From `algorithms/nlp_sampling/`, run:
  `python nlp_sampling_standalone_test.py --num-samples 20 --burn-in 10 --num-restarts 2 --quiet --no-plots`
- Confirm it completes without exceptions and reports at least one successful restart.
- If the PR changes sampling *behavior* (not just refactoring), also run without `--no-plots` and check a couple of joint histograms for obviously broken output (all-zero spread, samples outside joint limits, etc).

## `ManipulationPlanner` (copy-pasted inline across notebooks - no shared module yet)
Touches: `manipulation_planner_3dof.ipynb`, `manipulation_planner_4dof.ipynb`, `full_arm_blocked_joints_3dof.ipynb`, `full_arm_blocked_joints_4dof.ipynb`, `full_arm_nhr.ipynb`, `data/generation/full_arm_c_free.ipynb`
- Open the affected notebook(s), run cells through the inline `ManipulationPlanner` class definition and construction.
- Confirm no exceptions and the object constructs successfully. Note: each notebook has its own copy of this class (see `STRUCTURE.md` §11) - changes to one do not propagate to the others, so verify the specific notebook(s) actually touched by the PR, not just one representative.

## Model / scene loading (any notebook building a Drake plant)
Touches: `full_arm_blocked_joints_3dof.ipynb`, `full_arm_blocked_joints_4dof.ipynb`, `full_arm_nhr.ipynb`, `manipulation_planner_3dof.ipynb`, `manipulation_planner_4dof.ipynb`, `full_pipeline.ipynb`, `my_sdfs/*`, `ciris_plant_visualizer.py`, `visualization_utils.py`
- Open the affected notebook(s), run cells through plant construction + the first meshcat publish.
- Confirm: no exceptions, meshcat opens, the robot appears at a sane default pose (not overlapping obstacles, not off-scene).

## IRIS-ZO / clique-cover
Touches: `algorithms/iris_zo_cliquecover/*`
- Currently a placeholder - no check defined yet. Revisit once #42 lands real implementation.

## Data files
Touches: `data/cfree/*.yaml`, `data/cfree/*.npy`, `data/cfree/*.json`
- Confirm whichever notebook reads the changed file still loads it without a shape/key mismatch (see `STRUCTURE.md` §5 for which notebook reads which file).
- If the changed file is one of the frozen legacy inputs (`cfree_drake_1_48.yaml`, `cfree_drake_1_49.yaml`, `path_drake_1_48.npy`), double check the change was intentional - no notebook currently regenerates these from a fresh run.
