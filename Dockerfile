FROM robotlocomotion/drake:1.44.0

# Tools + build chain for compiling Python extensions
RUN apt-get update && apt-get -y install git nano build-essential python3-dev \
    && rm -rf /var/lib/apt/lists/*
    
ENV PYTHONPATH=/opt/drake/lib/python3.12/site-packages:$PYTHONPATH

# Allow system-site installs (PEP 668)
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Jupyter kernel + widgets
RUN python3 -m pip install --no-cache-dir ipykernel ipywidgets==8.0.4 widgetsnbextension==4.0.5 jupyterlab-widgets

# Your deps (Py3.12-compatible)
RUN python3 -m pip install --no-cache-dir "scipy>=1.12" pymcubes==0.1.4 cvxpy "cvxpy[MOSEK]" mosek plotly

# quadprog: try, but don’t fail the build if no wheel
RUN python3 -m pip install --no-cache-dir quadprog || echo "quadprog not available on Py3.12; continuing"

RUN python3 -m pip install --no-cache-dir "gurobipy>=11.0"

# Adjust to your repo path
ENV MOSEKLM_LICENSE_FILE=/workspaces/optimisation-based-manipulation-planner/mosek.lic
