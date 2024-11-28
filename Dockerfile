FROM robotlocomotion/drake:1.35.0

# Install git
RUN apt-get update && apt-get -y install git && apt-get -y install nano

# Set PYTHONPATH
ENV PYTHONPATH=/opt/drake/lib/python3.10/site-packages:$PYTHONPATH
