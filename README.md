# MuJoCo RGBD ROS2

## Clone Repository

Clone the repository:

```bash
git clone https://github.com/labiybafakh/mujoco_rgbd_ros2.git
```

## Prerequisites

Make sure you have the required dependencies installed:

```bash
sudo apt update
sudo apt install libopencv-dev libpcl-dev libglfw3-dev libgl1-mesa-dev libglew-dev
```

## Building with ROS2

This package provides MuJoCo RGBD camera simulation for ROS2, publishing point clouds and depth images. It uses a pre-built MuJoCo library (version 3.3.4) located in the `third_party/` directory. To build:

1. **Install ROS2 dependencies**:
   ```bash
   cd <your_workspace_root>
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Build with colcon** from your workspace root:
   ```bash
   cd <your_workspace_root>
   colcon build --packages-select mujoco_rgbd_ros2
   ```

   Or for a release build:
   ```bash
   colcon build --packages-select mujoco_rgbd_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. **Source the workspace**:
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

### Launch with custom parameters:
```bash
# Custom model file
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py model_file:=/path/to/your/model.xml

# Custom camera settings
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py camera_name:=rgbd_camera frame_id:=camera_optical_frame

# Custom image resolution and rate
ros2 launch mujoco_rgbd_ros2 mujoco_rgbd.launch.py image_width:=1280 image_height:=720 publish_rate:=60.0
```

**Available launch parameters:**
- `model_file`: Path to MuJoCo model file (default: `config/camera_environment.xml`)
- `camera_name`: Name of the camera in MuJoCo model (default: `camera`)
- `frame_id`: Frame ID for published data (default: `camera_link`)
- `publish_rate`: Publishing rate in Hz (default: `30.0`)
- `image_width`: Image width in pixels (default: `640`)
- `image_height`: Image height in pixels (default: `480`)
- `start_rviz`: Whether to start RViz automatically (default: `true`)

**Note**: The default model file is `config/camera_environment.xml` which includes a test scene with obstacles and an RGBD camera.

## Published Topics

- `/mujoco/pointcloud` - Colored point cloud (sensor_msgs/PointCloud2)
- `/mujoco/color/image_raw` - RGB camera image (sensor_msgs/Image)
- `/mujoco/depth/image_raw` - Depth image (sensor_msgs/Image)
- `/mujoco/camera_info` - Camera calibration info (sensor_msgs/CameraInfo)

## Subscribed Topics

- `/mujoco_rgbd_node/base_link/pose_command` - Camera pose commands (geometry_msgs/PoseStamped)
  - Use this topic to dynamically control the camera position and orientation during simulation

## Troubleshooting

- **Build fails**: Make sure all dependencies are installed using the commands in the Prerequisites section
- **MuJoCo library not found**: The pre-built MuJoCo library is included in `third_party/mujoco-3.3.4/lib/`
- **GLFW or OpenGL errors**: Ensure you have the graphics drivers and development libraries installed