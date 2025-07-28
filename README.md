# DWA 3D Simulation

A ROS2 package that simulates a depth camera (mimicking PicoFlexx2 VGA specifications) in Ignition Gazebo with various obstacles for testing depth perception.

## Features

- **PicoFlexx2 VGA Depth Camera Simulation**
  - Resolution: 640x480 (VGA)
  - Frame rate: 45 FPS
  - Range: 0.1 - 4.0 meters
  - Time-of-Flight technology simulation
  - Realistic noise modeling

- **Gazebo World with Obstacles**
  - Various geometric shapes (boxes, cylinders, spheres)
  - Different materials and colors
  - Wall obstacles for occlusion testing
  - Multiple small objects for detail testing

- **ROS2 Integration**
  - Standard sensor_msgs/Image topics for RGB and depth
  - sensor_msgs/PointCloud2 for 3D point cloud data
  - Camera info topics with calibration parameters
  - TF tree for sensor positioning

## Topics Published

- `/camera/image_raw` - RGB image from the depth camera
- `/camera/depth/image_raw` - Depth image (16-bit)
- `/camera/points` - 3D point cloud data
- `/camera/camera_info` - Camera calibration info (RGB)
- `/camera/depth/camera_info` - Depth camera calibration info

## Installation

1. Make sure you have ROS2 Humble and Ignition Gazebo installed
2. Clone this package into your ROS2 workspace
3. Build the package:
   ```bash
   colcon build --packages-select dwa_3d_simulation
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launch the Simulation

```bash
ros2 launch dwa_3d_simulation dwa_3d_simulation.launch.py
```

### Launch Options

- `use_sim_time:=true/false` - Use simulation time (default: true)
- `headless:=true/false` - Run Gazebo without GUI (default: false)
- `world_file:=<path>` - Custom world file path

### Example with Custom Parameters

```bash
ros2 launch dwa_3d_simulation dwa_3d_simulation.launch.py headless:=true use_sim_time:=true
```

## Visualization

The launch file automatically starts RViz2 with a pre-configured setup showing:
- RGB camera view
- Depth image visualization
- 3D point cloud
- TF tree
- Camera pose in 3D space

## Camera Specifications (PicoFlexx2 VGA)

- **Resolution**: 640x480 pixels
- **Field of View**: 60° horizontal
- **Frame Rate**: 45 FPS
- **Measurement Range**: 0.1m - 4.0m
- **Technology**: Time-of-Flight simulation
- **Noise Model**: Gaussian noise with 0.007m standard deviation

## File Structure

```
dwa_3d_simulation/
├── config/
│   └── depth_camera_viz.rviz    # RViz configuration
├── launch/
│   └── dwa_3d_simulation.launch.py  # Main launch file
├── urdf/
│   └── depth_camera_sensor.urdf.xacro     # Camera sensor URDF
├── worlds/
│   └── dwa_3d_world.sdf             # Gazebo world file
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Testing Depth Perception

The simulation includes various obstacles at different distances and angles:

1. **Red Box** (2m distance) - Large rectangular obstacle
2. **Green Cylinder** (1.5m distance) - Cylindrical obstacle
3. **Blue Sphere** (1m distance) - Spherical obstacle
4. **Gray Wall** (3m distance) - Vertical wall for occlusion testing
5. **Small Colored Boxes** - Multiple small objects for detail testing

## Development

To modify the camera parameters, edit the URDF file:
- `dwa_3d_simulation/urdf/depth_camera_sensor.urdf.xacro`

To add more obstacles or modify the environment:
- `dwa_3d_simulation/worlds/dwa_3d_world.sdf`

To customize the RViz visualization:
- `dwa_3d_simulation/config/depth_camera_viz.rviz`