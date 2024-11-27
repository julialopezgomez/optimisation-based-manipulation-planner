FROM robotlocomotion/drake:1.35.0

# # Install nano
# RUN apt-get update && apt-get install -y nano

# Set PYTHONPATH
ENV PYTHONPATH=/opt/drake/lib/python3.10/site-packages:$PYTHONPATH
