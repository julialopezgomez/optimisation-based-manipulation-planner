FROM robotlocomotion/drake:1.44.0

# Install git
RUN apt-get update && apt-get -y install git && apt-get -y install nano

# # Set PYTHONPATH
ENV PYTHONPATH=/opt/drake/lib/python3.12/site-packages:$PYTHONPATH
ENV MOSEKLM_LICENSE_FILE=/workspaces/minf2-repo/mosek.lic

RUN python3 -m pip install --no-cache-dir ipywidgets==8.0.4 widgetsnbextension==4.0.5 jupyterlab-widgets
RUN python3 -m pip install --no-cache-dir scipy==1.10.1 pymcubes==0.1.4 quadprog cvxpy mosek cvxpy[MOSEK] plotly
RUN python -m pip install gurobipy