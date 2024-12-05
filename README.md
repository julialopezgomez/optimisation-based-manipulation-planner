# MInf 2 Project - Julia Lopez Gomez
## Tangent Configuration Space Manipulation Planner

This is the code repository for my MInf 2 project: Tangent Configuration Space Manipulation Planner.

### Aditya
Hi Aditya. Here you have the instructions to run the implementation in Vscode, although with the Docker file it there should not be any problem to run it somewhere else.
Files to look at:
- `bottle_cap_TCspace.ipynb` and `bottle_cap_3DTCspace.ipynb` are the main notebooks to run. They start the meshcat, plant, etc and generate the TCspace polytopes, and try to run the implementation. The first one takes a scene with 2DOF: the finger prismatic joint from the gripper and the bottle cap rotation; and the second file takes 3DOF: the finger joint, gripper rotation, and the cap rotation.
- `ciris_plant_visualizer.py` and `visualization_utils.py` are the files I took from the [C-IRIS implementation](https://deepnote.com/workspace/alexandre-amice-c018b305-0386-4703-9474-01b867e6efea/project/C-IRIS-7e82e4f5-f47a-475a-aad3-c88093ed36c6/notebook/2d_example_bilinear_alternation-14f1ee8c795e499ca7f577b6885c10e9) for visualization. Examples are provided in this repo for a 2D TCSpace. `bottle_cap_TCspace` uses them to visualize the TCSpace, with the problem of the all-red rectangle I showed you. In `bottle_cap_3DTCspace` I am trying to reproduce them step by step, because the visualize_collision_constraint_3d method was returning an Error.
- `my_sdfs/wsg_1dof.sdf`, `my_sdfs/wsg_2dof.sdf` and `my_sdfs/bottle_cap.sdf`, are the SDF files used. The latter has 1 rotational DOF, the 1dof wsg has the finger prismatic dof, and the 2dof wsg has the finger prismatic dof and the gripper rotation dof.

I don't think you should be needing to use any other file other than this. Let me know in teams if anything is unclear or you have any question.

Thank you again for all of your help!

### Installation
To run the code in this repo, you will need version 1.35.0 of Drake, specifically, the Python bindings. 

We have provided a Dockerfile for easiness. If you are familiar with Docker, you are free to select the method or IDE of your choice. Steps for setup and installation in VSCode follow:
1. Install [Docker](https://www.docker.com/) and [VSCode](https://code.visualstudio.com/).
2. Install VSCode extensions:<br />
  a. [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) <br />
  b. [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
4. In the project folder, right-click on the `Dockerfile` and select: `Build Image...` Give it a name of your choice.<br />
  a. If an error pops up in the terminal, do: `nano ~/.docker/config.json` and delete the line that says "credsStore:".<br />
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