# Docker Setup for DWA 3D Simulation

This Docker setup provides a clean ROS2 Humble environment to avoid Python/library conflicts.

## Quick Start

1. **Build the Docker image:**
   ```bash
   ./docker_build.sh
   ```

2. **Run the simulation (with GUI):**
   ```bash
   ./docker_run.sh
   ```

3. **Run headless simulation:**
   ```bash
   ./docker_run_headless.sh
   ```

## Custom Commands

You can run custom commands in the container:

```bash
# Check ROS2 topics
./docker_run.sh "ros2 topic list"

# Launch with custom parameters
./docker_run.sh "ros2 launch dwa_3d_simulation dwa_3d_simulation.launch.py headless:=false"

# Interactive bash session
./docker_run.sh "bash"
```

## Monitoring Topics

To monitor camera topics from the host system:
```bash
# In another terminal, check if topics are published
docker exec -it dwa_3d_simulation ros2 topic list | grep camera

# Echo camera data
docker exec -it dwa_3d_simulation ros2 topic echo /camera/image_raw
```

## Troubleshooting

- If GUI doesn't work, ensure X11 forwarding is enabled: `xhost +local:docker`
- For headless mode, all RViz components are automatically disabled
- Camera topics should appear as: `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points`