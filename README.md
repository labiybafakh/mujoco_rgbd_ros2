# MuJoCo RGBD ROS2

## Clone Repository

Clone the repository with submodules:

```bash
git clone --recursive https://github.com/labiybafakh/mujoco_rgbd_ros2.git
```

## Prerequisites

Make sure you have the required dependencies installed:

```bash
sudo apt update
sudo apt install libopencv-dev libpcl-dev libglfw3-dev libgl1-mesa-dev
```

## Building with ROS2

This package provides MuJoCo RGBD camera simulation for ROS2, publishing point clouds and depth images. It uses MuJoCo as a submodule. To build:

1. **Install ROS2 dependencies**:
   ```bash
   cd /path/to/your/workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Initialize submodules** (if not already done):
   ```bash
   git submodule update --init --recursive
   ```

3. **Build with colcon** from your workspace root:
   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select mujoco_rgbd_ros2
   ```

   Or for a release build:
   ```bash
   colcon build --packages-select mujoco_rgbd_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running

### Launch the RGBD node with RViz:
```bash
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py
```

### Launch without RViz:
```bash
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py start_rviz:=false
```

### Launch with custom model file:
```bash
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py model_file:=/path/to/your/model.xml
```

**Note**: The default model file is `config/camera_environment.xml` which includes a test scene with obstacles and an RGBD camera.

## Published Topics

- `/mujoco/pointcloud` - Colored point cloud (sensor_msgs/PointCloud2)
- `/mujoco/color/image_raw` - RGB camera image (sensor_msgs/Image)
- `/mujoco/depth/image_raw` - Depth image (sensor_msgs/Image)
- `/mujoco/camera_info` - Camera calibration info (sensor_msgs/CameraInfo)

## Troubleshooting

- **Submodule not initialized**: Run `git submodule update --init --recursive`
- **Build fails**: Make sure all dependencies are installed
- **MuJoCo library not found**: The build system will compile MuJoCo from source automatically