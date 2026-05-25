# Optimisation-Based Manipulation Planner in convex Decompositions of C-free

This is the code repository for my MInf 2 project: Optimisation-Based Manipulation Planner in Convex Decompositions of C-free.

### Walkthrough
The main files of interest are `full_pipeline.ipynb`, `manipulation_planner_3dof.ipynb` and `manipulation_planner_4dof.ipynb` from the main directory. The former contains the full pipeline step by step through our manipulation algorithm, while the two latter files include an object-oriented-programming implementation embeeding the planner in a class, and providing example usages. One file contains constraints for a 3 DOF task, and the otherone with a 4 DOF task.

The files within the testing directory have been used for testing and implementing the elements of this planner individually.

The my_sdfs directory contains the sdf files we have used to define our scene.

Finally, helper files are `ciris_plant_visualizer` and `visualization_utils`.

`test.py` and `test_notebook.ipynb` serve for testing the installation of the system and the main libraries. 

Installation steps follow

### Installation
This repository uses Drake's Python bindings (`pydrake`). Drake's pre-built Docker images are being discontinued upstream, so the recommended setup is now via `pip` wheels.

Important Drake pip notes:
- Drake wheels require `pip >= 20.3`.
- Drake's pip packages do **not** support the Gurobi solver (use a source build if you require Gurobi).
- Drake is not tested regularly with Anaconda; using Conda may work but can have compatibility hiccups.
- On macOS, prefer Homebrew Python (not Apple’s system Python) when using `venv`.

Supported configurations (excerpted from Drake release notes):
- macOS Sequoia (15), arm64, Python 3.13, pip (October 2026)
- macOS Sequoia (15), arm64, Python 3.14, pip / binary (October 2026)
- macOS Tahoe (26), arm64, Python 3.13, pip (October 2026)
- macOS Tahoe (26), arm64, Python 3.14, pip / binary (October 2027)

#### Option A: Conda (recommended for this repo)
1. Install Miniforge / Conda (macOS arm64 users: Miniforge is typically the easiest).
2. Create the environment:
   - `conda env create -f environment.yml`
   - `conda activate obmp`
3. Smoke test:
   - `python test.py`
4. Open the notebooks:
   - `jupyter lab`

#### Option B: Python `venv` + pip
1. Create and activate a virtual environment:
   - `python3 -m venv .venv`
   - `source .venv/bin/activate`
2. Install Drake and the remaining dependencies:
   - `python -m pip install --upgrade pip`
   - `python -m pip install drake`
   - `python -m pip install -r requirements.txt`
   - (optional) `python -m pip install -r requirements-optional.txt`
3. Smoke test:
   - `python test.py`

#### Nightly Drake (optional)
Drake nightly wheels are published at `https://drake-packages.csail.mit.edu/whl/nightly/`.

To install a specific nightly build (replace `YYYYMMDD`):
- `python -m pip install --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ 'drake==0.0.YYYYMMDD'`

To install the latest nightly available today:
- `python -m pip install --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/ 'drake<0.1'`

#### Legacy Docker setup
The prior Docker-based setup is preserved under `legacy/docker/` for reference (see `legacy/docker/README.md`).

### MOSEK Solver License
Some notebooks in this implementation require a MOSEK license.

**Academic users can request a free academic license by visiting the MOSEK website**: https://www.mosek.com/products/academic-licenses/. 

Following the instructions, you will instantly receive an email. First, to accept the terms and conditions, and then another one to download your license.

**To use your license in this implementation**, you will need to replace the file `mosek.lic` with your own license file (or paste the content of your license file into the existing `mosek.lic` file).

*MOSEK is a state-of-the-art optimization solver, used for solving large-scale convex optimization problems efficiently. It supports a variety of optimization problem types, including linear programming (LP), quadratic programming (QP), second-order cone programming (SOCP), and semidefinite programming (SDP).* 
