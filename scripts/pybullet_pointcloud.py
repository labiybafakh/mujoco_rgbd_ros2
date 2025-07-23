import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math

# Import robot and camera utilities
from robot import FlappingRobot
from camera_utils import (
    create_robot_motion_camera,
    create_picoflexx2_processor,
    PointCloudSaver
)

class PyBulletPointCloud:
    def __init__(self, gui=True, use_robot_motion=True):
        # Initialize PyBullet
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        
        # Initialize robot motion model
        self.use_robot_motion = use_robot_motion
        if use_robot_motion:
            self.robot = FlappingRobot(flapping_freq=5.0, initial_pos=[0, 0, 2.0])
            self.camera_controller = create_robot_motion_camera(self.robot)
        
        # Initialize PicoFlexx2 point cloud processor
        self.pc_processor = create_picoflexx2_processor()
        
        # Point cloud storage
        self.point_clouds = []
        self.timestamps = []
        
        self.setup_scene()
    
    def setup_scene(self):
        # Load ground plane
        self.planeId = p.loadURDF("plane.urdf")
        
        # Add multiple obstacles in front of camera (positioned at 0,0,2.0 looking forward)
        self.obstacles = []
        
        # Box obstacles in front (positive Y direction from camera)
        box_positions = [
            [0.5, 1.5, 0.5],    # Front right
            [-0.5, 1.5, 0.5],   # Front left
            [0, 2.5, 0.5],      # Center far
            [1.0, 2.0, 0.5],    # Right far
            [-1.0, 2.0, 0.5]    # Left far
        ]
        
        for i, pos in enumerate(box_positions):
            box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5])
            box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.3, 0.3, 0.5], 
                                           rgbaColor=[0.8, 0.2, 0.2, 1.0])
            box_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_collision,
                                     baseVisualShapeIndex=box_visual, basePosition=pos)
            self.obstacles.append(box_id)
        
        # Cylinder obstacles in front
        cylinder_positions = [
            [0.8, 1.2, 0.5],    # Close right
            [-0.8, 1.2, 0.5],   # Close left
            [0, 3.0, 0.5]       # Far center
        ]
        
        for i, pos in enumerate(cylinder_positions):
            cylinder_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.25, height=1.0)
            cylinder_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.25, length=1.0,
                                                rgbaColor=[0.2, 0.8, 0.2, 1.0])
            cylinder_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=cylinder_collision,
                                          baseVisualShapeIndex=cylinder_visual, basePosition=pos)
            self.obstacles.append(cylinder_id)
        
        # Sphere obstacles in front at different heights
        sphere_positions = [
            [0.3, 1.0, 0.2],    # Close right, low 
            [-0.3, 1.0, 0.2],   # Close left, low
            [0, 1.8, 0.8],      # Center, elevated
            [1.2, 3.5, 0.3]     # Far right
        ]
        
        for i, pos in enumerate(sphere_positions):
            sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)
            sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.2,
                                              rgbaColor=[0.2, 0.2, 0.8, 1.0])
            sphere_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sphere_collision,
                                        baseVisualShapeIndex=sphere_visual, basePosition=pos)
            self.obstacles.append(sphere_id)
    
    def get_camera_matrix(self, camera_pos, camera_target, camera_up):
        # Use the processor's camera matrix method
        return self.pc_processor.get_camera_matrices(camera_pos, camera_target, camera_up)
    
    def capture_pointcloud(self, camera_pos=[3, 3, 2], camera_target=[0, 0, 0]):
        # Use processor for everything - cleaner approach
        camera_up = [0, 0, 1]
        
        # Capture using processor
        rgb_img, depth = self.pc_processor.capture_depth_image(camera_pos, camera_target, camera_up)
        
        # Generate point cloud using processor
        points = self.pc_processor.depth_to_pointcloud(depth, camera_pos, camera_target, camera_up)
        colors = self.pc_processor.get_point_colors(rgb_img, depth)
        
        # Debug information
        print(f"Depth range: {np.min(depth):.3f} to {np.max(depth):.3f}")
        print(f"Generated points: {len(points)}")
        if len(points) > 0:
            print(f"Point cloud bounds: X[{np.min(points[:, 0]):.2f}, {np.max(points[:, 0]):.2f}], "
                  f"Y[{np.min(points[:, 1]):.2f}, {np.max(points[:, 1]):.2f}], "
                  f"Z[{np.min(points[:, 2]):.2f}, {np.max(points[:, 2]):.2f}]")
        
        return points, colors, rgb_img, depth
    
    
    def visualize_pointcloud(self, points, colors=None, title="Point Cloud"):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        if colors is not None:
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                      c=colors, s=1, alpha=0.6)
        else:
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                      c=points[:, 2], cmap='viridis', s=1, alpha=0.6)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)
        plt.show()
    
    
    def capture_pointcloud_robot_motion(self):
        """Capture point cloud using robot motion model"""
        if not self.use_robot_motion:
            print("Robot motion not enabled")
            return None, None, None, None, None
        
        # Get robot attitude-based camera pose
        camera_pos, camera_target, camera_up = self.camera_controller.robot_attitude_to_camera_pose()
        
        # Capture depth image
        rgb_img, depth = self.pc_processor.capture_depth_image(camera_pos, camera_target, camera_up)
        
        # Generate point cloud
        points = self.pc_processor.depth_to_pointcloud(depth, camera_pos, camera_target, camera_up)
        colors = self.pc_processor.get_point_colors(rgb_img, depth)
        
        # Get robot attitude for logging
        roll_deg, pitch_deg, yaw_deg = self.camera_controller.get_robot_attitude_degrees()
        
        # Debug information
        print(f"Robot position: [{camera_pos[0]:.2f}, {camera_pos[1]:.2f}, {camera_pos[2]:.2f}]")
        print(f"Robot attitude: Roll={roll_deg:.1f}°, Pitch={pitch_deg:.1f}°, Yaw={yaw_deg:.1f}°")
        print(f"Robot time: {self.robot.time:.2f}s")
        print(f"Generated points: {len(points)}")
        if len(points) > 0:
            print(f"Point cloud bounds: X[{np.min(points[:, 0]):.2f}, {np.max(points[:, 0]):.2f}], "
                  f"Y[{np.min(points[:, 1]):.2f}, {np.max(points[:, 1]):.2f}], "
                  f"Z[{np.min(points[:, 2]):.2f}, {np.max(points[:, 2]):.2f}]")
        print("-" * 60)
        
        return points, colors, rgb_img, depth, (camera_pos, roll_deg, pitch_deg, yaw_deg)
    
    def run_robot_motion_simulation(self, duration=10.0, capture_interval=1.0):
        """Run simulation with robot motion and capture point clouds"""
        if not self.use_robot_motion:
            print("Robot motion not enabled")
            return
        
        print(f"Starting robot motion simulation for {duration}s...")
        print(f"Camera follows robot roll/pitch motion model")
        print(f"Capturing point clouds every {capture_interval}s")
        print("=" * 60)
        
        # Set matplotlib to non-blocking mode
        plt.ion()
        
        # Wait for simulation to stabilize
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1/240)
        
        last_capture_time = 0
        capture_count = 0
        
        try:
            while self.robot.time < duration:
                # Step robot simulation
                self.robot.step(dt=0.01)
                
                # Step physics simulation
                p.stepSimulation()
                
                # Capture point cloud at intervals
                if self.robot.time - last_capture_time >= capture_interval:
                    print(f"Capture {capture_count + 1} at t={self.robot.time:.2f}s")
                    
                    result = self.capture_pointcloud_robot_motion()
                    if result[0] is not None:
                        points, colors, rgb_img, depth, pose_info = result
                        camera_pos, roll_deg, pitch_deg, yaw_deg = pose_info
                        
                        # Store data
                        self.point_clouds.append((points.copy(), colors.copy(), camera_pos, (roll_deg, pitch_deg, yaw_deg)))
                        self.timestamps.append(self.robot.time)
                        
                        # Save point cloud
                        filename = f"robot_motion_pointcloud_{capture_count+1:03d}.ply"
                        PointCloudSaver.save_ply(points, colors, filename)
                        
                        # Visualize
                        self.visualize_pointcloud(points, colors, f"Robot Motion t={self.robot.time:.2f}s")
                        
                        last_capture_time = self.robot.time
                        capture_count += 1
                
                # Control simulation speed
                time.sleep(1/60)
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        print(f"\nSimulation complete! Captured {len(self.point_clouds)} point clouds")
        return self.point_clouds, self.timestamps

    def disconnect(self):
        p.disconnect()

def main():
    print("PyBullet Point Cloud Generation")
    print("1. Static viewpoints (original)")
    print("2. Robot motion model (with roll/pitch)")
    
    mode = input("Choose mode (1 or 2): ").strip()
    
    if mode == "2":
        # Robot motion mode
        print("\nUsing robot motion model with roll/pitch camera control")
        pc_generator = PyBulletPointCloud(gui=True, use_robot_motion=True)
        
        try:
            # Run robot motion simulation
            pc_generator.run_robot_motion_simulation(duration=15.0, capture_interval=1.0)
            
            # Keep simulation running
            print("\nSimulation complete. Press Ctrl+C to exit or close the PyBullet GUI window.")
            while True:
                p.stepSimulation()
                time.sleep(1/60)
        except KeyboardInterrupt:
            print("\nExiting simulation...")
    
    else:
        # Static viewpoints mode (original)
        print("\nUsing static viewpoints (original mode)")
        pc_generator = PyBulletPointCloud(gui=True, use_robot_motion=False)
        
        # Set matplotlib to non-blocking mode
        plt.ion()
        
        # Wait for simulation to stabilize
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1/240)
        
        # Capture point cloud from static viewpoints
        viewpoints = [
            ([0, 0, 2.0], [0, 1.5, 1.8]),    # Looking forward slightly down
            ([0, 0, 2.0], [0.5, 1.5, 1.8]),  # Looking forward-right
            ([0, 0, 2.0], [-0.5, 1.5, 1.8]), # Looking forward-left
            ([0, 0, 2.0], [0, 2.0, 1.0])     # Looking forward-down more
        ]
        
        for i, (camera_pos, camera_target) in enumerate(viewpoints):
            print(f"Capturing point cloud from viewpoint {i+1}...")
            
            # Capture point cloud using original method
            points, colors, rgb_img, depth = pc_generator.capture_pointcloud(camera_pos, camera_target)
            
            print(f"Generated {len(points)} points")
            
            # Save point cloud
            PointCloudSaver.save_ply(points, colors, f"pointcloud_view_{i+1}.ply")
            
            # Visualize point cloud
            pc_generator.visualize_pointcloud(points, colors, f"Point Cloud - View {i+1}")
        
        # Keep simulation running
        print("Point cloud generation complete!")
        print("Press Ctrl+C to exit or close the PyBullet GUI window.")
        
        try:
            while True:
                p.stepSimulation()
                time.sleep(1/60)
        except KeyboardInterrupt:
            print("\nExiting simulation...")
    
    pc_generator.disconnect()

if __name__ == "__main__":
    main()