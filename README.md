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
To run the code in this repo, you will need version 1.35.0 of Drake, specifically, the Python bindings. 

We have provided a Dockerfile for easiness. If you are familiar with Docker, you are free to select the method or IDE of your choice. Steps for setup and installation in VSCode follow:
1. Install [Docker](https://www.docker.com/) and [VSCode](https://code.visualstudio.com/).
2. Install VSCode extensions:<br />
  a. [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) <br />
  b. [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
4. In the project folder, right-click on the `Dockerfile` and select: `Build Image...` Give it a name of your choice.<br />
  a. If an error pops up in the terminal, do: `nano ~/.docker/config.json` and delete the line that says `"credsStore:"`.<br />
  b. Close nano using 1) `ctrl+x`, 2) `y`, and 3) `ENTER`.<br />
  c. There is no need to click on 'Build Image...' again.
5. Open the VSCode Command Palette using `F1` or `Ctrl+Shift+P` (`Cmd+Shift+P` on macOS). Select: Dev Containers: Open Folder in Container...<br />. If prompted:
  a. Select the project folder<br />
  b. When asked 'How would you like to create your container configuration?' select: From 'Dockerfile'.<br />
  d. Click OK.
6. Run `test.py` and `test_notebook.py` to make sure the installation works
7. Ready to go!

### MOSEK Solver License
Some notebooks in this implementation require a MOSEK license.

**Academic users can request a free academic license by visiting the MOSEK website**: https://www.mosek.com/products/academic-licenses/. 

Following the instructions, you will instantly receive an email. First, to accept the terms and conditions, and then another one to download your license.

**To use your license in this implementation**, you will need to replace the file `mosek.lic` with your own license file (or paste the content of your license file into the existing `mosek.lic` file).

*MOSEK is a state-of-the-art optimization solver, used for solving large-scale convex optimization problems efficiently. It supports a variety of optimization problem types, including linear programming (LP), quadratic programming (QP), second-order cone programming (SOCP), and semidefinite programming (SDP).* 
