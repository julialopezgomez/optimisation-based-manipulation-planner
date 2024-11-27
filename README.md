# MInf 2 Project - Julia Lopez Gomez
## Tangent Configuration Space Manipulation Planner

This is the code repository for my MInf 2 project: Tangent Configuration Space Manipulation Planner.

### Installation
To run the code in this repo, you will need version 1.35.0 of Drake, in specific, the python bindings. 

We have provided a Dockerfile for easiness. If you are familiar with Docker, you are free to select the method or IDE of your choice. Steps for setup and installation in VSCode follow:
1. Install Docker, VSCode.
2. Install VSCode extensions:
  a. Docker
  b. Dev Containers
4. Right-click on Dockerfile and select: Build Image... Give it a name of your choice.<br />
  a. If an error pops up in the terminal, do: nano ~/.docker/config.json and delete the line that says "credsStore:".<br />
  b. Close nano using 1) ctril+X, 2) y , and 3) ENTER.<br />
  c. There is no need to click on 'Build Image...' again.
5. Do F1, and select: Dev Containers: Open Folder in Container...<br />
  a. Select the project folder<br />
  b. When prompted 'How would you like to create your container configuration?' select: From 'Dockerfile'.<br />
  c. When prompted to add any additional resources, select Python, from the devcontainers provider.<br />
  d. Click OK.
6. Run test.py and test_notebook.py to make sure the installation works
7. Ready to go!
