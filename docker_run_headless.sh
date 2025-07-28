#!/bin/bash

echo "Running DWA 3D Simulation in headless mode..."

# Run headless simulation for testing topics
docker run --rm \
    --name dwa_3d_simulation_headless \
    --network host \
    dwa_3d_simulation:latest \
    bash -c "timeout 10s ros2 launch dwa_3d_simulation dwa_3d_simulation.launch.py headless:=true; echo 'Simulation completed'"