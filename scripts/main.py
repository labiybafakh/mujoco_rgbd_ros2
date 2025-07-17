import genesis as gs
import numpy as np
import cv2
import math
from robot import FlappingRobot

# Initialize Genesis
gs.init(backend=gs.cpu,
        theme='light' # Use 'dark' for dark theme
        ) # Use Metal Performance Shaders for GPU acceleration on macOS


scene = gs.Scene(
    show_viewer = True,
    sim_options = gs.options.SimOptions(
        dt = 0.01, # Simulation time step
        gravity = (0.0, 0.0, -9.81), # Gravity vector
    ),
    viewer_options = gs.options.ViewerOptions(
        res = (1280, 960),
        camera_pos = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov = 40,
        max_FPS = 120,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame = True,
        show_cameras = True,
        plane_reflection = True,
        ambient_light = (0.1, 0.1, 0.1),
    ),
    renderer=gs.renderers.Rasterizer(),
    show_FPS = False,
)

# Add environment
plane = scene.add_entity(
    gs.morphs.Plane(),
)

obstacle_1 = scene.add_entity(
    morph = gs.morphs.Cylinder(
        height = 8.0,
        radius = 0.5,
        pos= (7.0, 0.0, 0.0),
        fixed= True,
    ),
)

obstacle_2 = scene.add_entity(
    morph = gs.morphs.Cylinder(
        height = 5.0,
        radius = 0.6,
        pos= (9.0, -5.0, 0.0),
        fixed= True,
    ),
)

obstacle_3 = scene.add_entity(
    morph = gs.morphs.Cylinder(
        height = 3.0,
        radius = 0.6,
        pos= (4.0, 2.0, 0.0),
        fixed= True,
    ),
)

obstacle_4 = scene.add_entity(
    morph = gs.morphs.Cylinder(
        height = 10.0,
        radius = 1.0,
        pos= (12.0, 4.5, 0.0),
        fixed= True,
    ),
)

obstacle_5 = scene.add_entity(
    morph = gs.morphs.Cylinder(
        height = 10.0,
        radius = 1.0,
        pos= (14.0, 0.0, 0.0),
        fixed= True,
    ),
)

# Add camera with correct initial orientation for forward view
cam = scene.add_camera(
    res = (640, 480),
    pos = (0, 0.0, 1.0),
    lookat = (0, 0, 0),  # Look forward (positive Y direction) and slightly up
    fov = 56,
    GUI = False,
)

scene.build()

# Create flapping robot instance
robot = FlappingRobot(
    flapping_freq=5.0,
    initial_pos=[0, 0, 2.0]
)


# Simulation loop
for i in range(1200):  # 12 seconds at 0.01s timestep
    # Get robot status
    status = robot.get_status()
    current_pos = status['position']
    
    # Get camera transformation and render depth
    camera_transform = robot.get_camera_transform()
    cam.set_pose(transform=camera_transform)
    
    # Render depth image
    depth = cam.render(rgb=False, depth=True, segmentation=False, normal=False)
    depth_image = depth[1]
    
    # Step robot simulation
    robot.step()
    
    # Step simulation
    scene.step()
    
    # Display depth image
    normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    normalized = normalized.astype(np.uint8)
    cv2.imshow("Depth Map", normalized)
    
    # Print status
    print(f"Time: {status['time']:.2f}s, "
            f"Robot Pos: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}), "
            f"Roll: {status['wing_roll_deg']:.1f}°, Pitch: {status['tail_pitch_deg']:.1f}°")
    
    # Exit condition
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()