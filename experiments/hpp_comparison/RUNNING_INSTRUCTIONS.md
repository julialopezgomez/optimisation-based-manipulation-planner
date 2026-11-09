# Running instructions for the HPP comparison experiments

### Open the container:
```bash
./experiments/hpp_comparison/env/run_docker.sh
```

### Second terminal:
```bash
docker exec -it hpp-obmp bash
```

### For running scripts experiments inside the container:
```bash
cd /home/user/obmp/experiments/hpp_comparison
python -i hpp_1cap_2obs.py # substitude with the script you want to run
```