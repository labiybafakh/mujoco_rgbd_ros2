#!/bin/bash

# Default command
DEFAULT_CMD="ros2 launch dwa_3d_simulation dwa_3d_simulation.launch.py"

echo "Running DWA 3D Simulation in Docker..."

# Check if custom command is provided
if [ $# -eq 0 ]; then
    CMD=$DEFAULT_CMD
else
    CMD="$@"
fi

echo "Executing: $CMD"

# Run the Docker container with appropriate settings
docker run -it --rm \
    --name dwa_3d_simulation \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e QT_X11_NO_MITSHM=1 \
    --privileged \
    -v /dev/dri:/dev/dri \
    dwa_3d_simulation:latest \
    bash -c "$CMD"