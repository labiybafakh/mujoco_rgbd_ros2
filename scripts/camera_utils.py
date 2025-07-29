"""
Camera utilities for point cloud generation with robot motion model
Handles camera pose calculation and robot attitude integration
"""

import numpy as np
import math
import pybullet as p
import open3d as o3d
import threading
import queue
import time

class CameraMotionController:
    """Handles camera movement based on robot motion model"""
    
    def __init__(self, robot):
        """
        Initialize camera controller with robot instance
        
        Args:
            robot: FlappingRobot instance from robot.py
        """
        self.robot = robot
    
    def robot_attitude_to_camera_pose(self):
        """
        Convert robot position and attitude (roll/pitch/yaw) to camera pose
        
        Returns:
            tuple: (camera_pos, camera_target, camera_up) for PyBullet camera
        """
        # Get robot position directly
        camera_pos = self.robot.position.copy().tolist()
        
        # Get robot attitude (roll, pitch, yaw) including flapping oscillations
        roll = self.robot.attitude[0]   # wing_roll + roll_oscillation
        pitch = self.robot.attitude[1]  # tail_pitch + pitch_oscillation  
        yaw = self.robot.attitude[2]    # calculated yaw from motion
        
        # Create rotation matrix from robot attitude (aerospace/NED convention)
        cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
        sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
        
        # Standard aerospace rotation matrix (Roll-Pitch-Yaw)
        R_body_to_world = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])
        
        # Camera looks forward in robot body frame (positive X direction for aircraft)
        forward_body = np.array([1, 0, 0])
        forward_world = R_body_to_world @ forward_body
        
        # Camera target is position + forward direction (scaled for visualization)
        camera_target = (np.array(camera_pos) + forward_world * 2.0).tolist()
        
        # Up vector in robot body frame (negative Z for aircraft body frame)
        up_body = np.array([0, 0, -1]) 
        up_world = R_body_to_world @ up_body
        camera_up = up_world.tolist()
        
        return camera_pos, camera_target, camera_up
    
    def get_robot_attitude_degrees(self):
        """
        Get robot attitude in degrees for logging/visualization
        
        Returns:
            tuple: (roll_deg, pitch_deg, yaw_deg)
        """
        roll_deg = math.degrees(self.robot.attitude[0])
        pitch_deg = math.degrees(self.robot.attitude[1]) 
        yaw_deg = math.degrees(self.robot.attitude[2])
        
        return roll_deg, pitch_deg, yaw_deg

class PointCloudProcessor:
    """Handles point cloud generation and processing"""
    
    def __init__(self, width=224, height=172, fov_h=64, fov_v=50, near=0.1, far=4.0):
        """
        Initialize point cloud processor with PicoFlexx2 VGA specifications
        
        Args:
            width: Image width (224 for PicoFlexx2 VGA)
            height: Image height (172 for PicoFlexx2 VGA)
            fov_h: Horizontal field of view in degrees (64° for PicoFlexx2)
            fov_v: Vertical field of view in degrees (50° for PicoFlexx2)
            near: Near clipping distance in meters (0.1m for PicoFlexx2)
            far: Far clipping distance in meters (4.0m for PicoFlexx2)
        """
        self.width = width
        self.height = height
        self.fov_horizontal = fov_h
        self.fov_vertical = fov_v
        self.fov = fov_v  # Use vertical FOV for projection matrix
        self.near = near
        self.far = far
    
    def get_camera_matrices(self, camera_pos, camera_target, camera_up):
        """
        Compute PyBullet view and projection matrices
        
        Args:
            camera_pos: Camera position [x, y, z]
            camera_target: Camera target point [x, y, z]
            camera_up: Camera up vector [x, y, z]
            
        Returns:
            tuple: (view_matrix, projection_matrix)
        """
        view_matrix = p.computeViewMatrix(camera_pos, camera_target, camera_up)
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.width / self.height,
            nearVal=self.near,
            farVal=self.far
        )
        return view_matrix, projection_matrix
    
    def capture_depth_image(self, camera_pos, camera_target, camera_up):
        """
        Capture RGB and depth images from PyBullet
        
        Args:
            camera_pos: Camera position [x, y, z]
            camera_target: Camera target point [x, y, z]
            camera_up: Camera up vector [x, y, z]
            
        Returns:
            tuple: (rgb_img, depth_array)
        """
        view_matrix, projection_matrix = self.get_camera_matrices(camera_pos, camera_target, camera_up)
        
        # Capture image
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Convert depth buffer to actual distances
        depth_buffer = np.array(depth_img, dtype=np.float32).reshape(self.height, self.width)
        depth = self.far * self.near / (self.far - (self.far - self.near) * depth_buffer)
        
        return rgb_img, depth
    
    def depth_to_pointcloud(self, depth, camera_pos, camera_target, camera_up):
        """
        Convert depth image to 3D point cloud
        
        Args:
            depth: Depth array (height x width)
            camera_pos: Camera position [x, y, z]
            camera_target: Camera target point [x, y, z]
            camera_up: Camera up vector [x, y, z]
            
        Returns:
            np.array: Point cloud array (N x 3)
        """
        # PicoFlexx2 VGA intrinsic parameters
        fx = self.width / (2 * np.tan(np.radians(self.fov_horizontal) / 2))
        fy = self.height / (2 * np.tan(np.radians(self.fov_vertical) / 2))
        cx = self.width / 2
        cy = self.height / 2
        
        # Create coordinate arrays
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        
        # Convert to camera coordinates using proper intrinsics
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        # Stack coordinates
        points_cam = np.stack([x, y, z], axis=-1)
        
        # Transform to world coordinates
        camera_pos = np.array(camera_pos)
        camera_target = np.array(camera_target)
        camera_up = np.array(camera_up)
        
        # Calculate camera rotation matrix
        forward = camera_target - camera_pos
        forward = forward / np.linalg.norm(forward)
        right = np.cross(forward, camera_up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)
        
        # Correct rotation matrix for camera-to-world transformation
        # Camera coordinates: X=right, Y=down, Z=forward (depth)
        # World coordinates: X=right, Y=forward, Z=up
        rotation_matrix = np.array([
            [right[0], -up[0], forward[0]],
            [right[1], -up[1], forward[1]], 
            [right[2], -up[2], forward[2]]
        ])
        
        # Apply transformation
        points_world = np.dot(points_cam, rotation_matrix.T) + camera_pos
        
        # Filter out invalid points
        valid_mask = (depth > self.near) & (depth < (self.far * 0.95)) & np.isfinite(depth)
        points_filtered = points_world[valid_mask].reshape(-1, 3)
        
        # Additional filtering for realistic points
        if len(points_filtered) > 0:
            distance_from_camera = np.linalg.norm(points_filtered - np.array(camera_pos), axis=1)
            realistic_mask = (distance_from_camera >= self.near) & (distance_from_camera <= self.far)
            points_filtered = points_filtered[realistic_mask]
        
        return points_filtered
    
    def get_point_colors(self, rgb_img, depth, camera_pos):
        """
        Extract colors for valid points from RGB image
        
        Args:
            rgb_img: RGB image from PyBullet
            depth: Depth array (height x width)
            camera_pos: Camera position for distance filtering
            
        Returns:
            np.array: Color array (N x 3) with values 0-1
        """
        # Convert RGB image to point colors
        rgb_array = np.array(rgb_img).reshape(self.height, self.width, 4)[:, :, :3]  # Remove alpha channel
        
        # Apply same filtering as in depth_to_pointcloud
        valid_mask = (depth > self.near) & (depth < (self.far * 0.95)) & np.isfinite(depth)
        colors_initial = rgb_array[valid_mask].reshape(-1, 3) / 255.0
        
        # Apply additional realistic distance filtering to match points
        if len(colors_initial) > 0:
            # Get corresponding points for distance check
            fx = self.width / (2 * np.tan(np.radians(self.fov_horizontal) / 2))
            fy = self.height / (2 * np.tan(np.radians(self.fov_vertical) / 2))
            cx = self.width / 2
            cy = self.height / 2
            
            u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
            x = (u - cx) * depth / fx
            y = (v - cy) * depth / fy
            z = depth
            points_cam = np.stack([x, y, z], axis=-1)
            points_filtered_cam = points_cam[valid_mask].reshape(-1, 3)
            
            # Calculate distances same as in depth_to_pointcloud
            distance_from_camera = np.linalg.norm(points_filtered_cam, axis=1)
            realistic_mask = (distance_from_camera >= self.near) & (distance_from_camera <= self.far)
            colors_filtered = colors_initial[realistic_mask]
        else:
            colors_filtered = colors_initial
            
        return colors_filtered

class PointCloudViewer:
    """Handles point cloud visualization using Open3D"""
    
    def __init__(self):
        self.vis = None
        self.pcd = None
        self.viewer_thread = None
        self.data_queue = queue.Queue()
        self.running = False
        
    def create_viewer(self, window_name="Point Cloud Viewer", width=800, height=600):
        """Create Open3D visualizer window"""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name, width, height)
        return self.vis
    
    def update_pointcloud(self, points, colors=None):
        """Update point cloud in the viewer"""
        if len(points) == 0:
            return
            
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if colors is not None and len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            # Default to height-based coloring
            colors_default = np.zeros_like(points)
            colors_default[:, 2] = (points[:, 2] - np.min(points[:, 2])) / (np.max(points[:, 2]) - np.min(points[:, 2]))
            pcd.colors = o3d.utility.Vector3dVector(colors_default)
        
        # Update or add point cloud to visualizer
        if self.pcd is None:
            self.vis.add_geometry(pcd)
            self.pcd = pcd
        else:
            self.pcd.points = pcd.points
            self.pcd.colors = pcd.colors
            self.vis.update_geometry(self.pcd)
        
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def show_pointcloud_interactive(self, points, colors=None, window_name="Point Cloud Viewer"):
        """Show point cloud in an interactive window with keyboard controls"""
        if len(points) == 0:
            print("No points to display")
            return
        
        # Check if we're on Wayland or if Open3D is likely to fail
        import os
        wayland_session = os.environ.get('WAYLAND_DISPLAY') or os.environ.get('XDG_SESSION_TYPE') == 'wayland'
        
        if wayland_session:
            print("Wayland detected - using matplotlib viewer for compatibility")
            return self._show_matplotlib_streaming(points, colors, window_name)
        
        # Try Open3D first, fallback to matplotlib if it fails
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            
            if colors is not None and len(colors) == len(points):
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Add coordinate frame for reference
            coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
            
            # Create visualizer with key callbacks
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window(window_name, width=800, height=600)
            
            # Store data for saving
            self.current_points = points
            self.current_colors = colors
            self.save_counter = 1
            
            def save_callback(vis):
                """Save point cloud when 's' is pressed"""
                filename_ply = f"saved_pointcloud_{self.save_counter:03d}.ply"
                filename_pcd = f"saved_pointcloud_{self.save_counter:03d}.pcd"
                
                PointCloudSaver.save_ply(self.current_points, self.current_colors, filename_ply)
                PointCloudSaver.save_pcd(self.current_points, self.current_colors, filename_pcd)
                
                print(f"Point cloud saved as {filename_ply} and {filename_pcd}")
                print("Press 's' to save again, 'q' to quit")
                self.save_counter += 1
                return False
            
            def quit_callback(vis):
                """Quit viewer when 'q' is pressed"""
                print("Exiting viewer...")
                return True
            
            def help_callback(vis):
                """Show help when 'h' is pressed"""
                print("\n=== Point Cloud Viewer Controls ===")
                print("s - Save point cloud (PLY and PCD formats)")
                print("q - Quit viewer")
                print("h - Show this help")
                print("Mouse: Left click + drag to rotate")
                print("Mouse: Right click + drag to zoom")
                print("Mouse: Middle click + drag to pan")
                print("====================================\n")
                return False
            
            # Register key callbacks
            vis.register_key_callback(ord('S'), save_callback)
            vis.register_key_callback(ord('Q'), quit_callback)
            vis.register_key_callback(ord('H'), help_callback)
            
            # Add geometries
            vis.add_geometry(pcd)
            vis.add_geometry(coord_frame)
            
            # Show initial help
            print("\n=== Point Cloud Viewer Controls ===")
            print("s - Save point cloud (PLY and PCD formats)")
            print("q - Quit viewer")
            print("h - Show help")
            print("====================================\n")
            
            # Run visualizer
            vis.run()
            vis.destroy_window()
            
        except Exception as e:
            print(f"Open3D viewer failed: {e}")
            print("Falling back to matplotlib viewer...")
            return self._show_matplotlib_streaming(points, colors, window_name)
    
    def start_threaded_viewer(self, window_name="Point Cloud Viewer"):
        """Start the threaded matplotlib viewer"""
        if self.viewer_thread is not None and self.viewer_thread.is_alive():
            return  # Already running
            
        self.running = True
        self.viewer_thread = threading.Thread(
            target=self._threaded_matplotlib_viewer, 
            args=(window_name,),
            daemon=True
        )
        self.viewer_thread.start()
        
        # Wait a moment for viewer to initialize
        time.sleep(0.5)
    
    def update_viewer(self, points, colors=None):
        """Update the viewer with new point cloud data"""
        if not self.running:
            return
        
        # Put new data in queue (non-blocking)
        try:
            self.data_queue.put_nowait({'points': points, 'colors': colors})
        except queue.Full:
            # Skip this update if queue is full
            pass
    
    def _threaded_matplotlib_viewer(self, window_name):
        """Threaded matplotlib viewer that updates from queue"""
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend for thread safety
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        plt.ion()
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Initialize with empty plot
        scatter = ax.scatter([], [], [], s=1, alpha=0.6)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"{window_name}\nPress 's' to save point cloud")
        
        # Store for saving
        self.current_points = None
        self.current_colors = None
        self.save_counter = 1
        
        # Set up key press handling
        def on_key_press(event):
            if event.key == 's' and self.current_points is not None:
                filename_ply = f"saved_pointcloud_{self.save_counter:03d}.ply"
                filename_pcd = f"saved_pointcloud_{self.save_counter:03d}.pcd"
                
                PointCloudSaver.save_ply(self.current_points, self.current_colors, filename_ply)
                PointCloudSaver.save_pcd(self.current_points, self.current_colors, filename_pcd)
                
                print(f"Point cloud saved as {filename_ply} and {filename_pcd}")
                self.save_counter += 1
        
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        
        print(f"\n=== {window_name} Started ===")
        print("Press 's' in the plot window to save point cloud")
        print("Close window to stop viewer")
        
        # Main update loop
        while self.running and plt.fignum_exists(fig.number):
            try:
                # Check for new data (non-blocking)
                data = self.data_queue.get_nowait()
                points = data['points']
                colors = data['colors']
                
                # Update stored data
                self.current_points = points
                self.current_colors = colors
                
                # Clear and redraw
                ax.clear()
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title(f"{window_name}\nPress 's' to save point cloud")
                
                if len(points) > 0:
                    if colors is not None and len(colors) == len(points):
                        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                  c=colors, s=1, alpha=0.6)
                    else:
                        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                  c=points[:, 2], cmap='viridis', s=1, alpha=0.6)
                
                plt.draw()
                
            except queue.Empty:
                # No new data, just refresh
                plt.pause(0.1)
            except Exception as e:
                print(f"Viewer error: {e}")
                break
        
        self.running = False
        plt.ioff()
        plt.close('all')  # Close all matplotlib figures
        
    def stop_viewer(self):
        """Stop the threaded viewer"""
        self.running = False
        if self.viewer_thread is not None:
            self.viewer_thread.join(timeout=1.0)
            
        # Clean up matplotlib
        import matplotlib.pyplot as plt
        plt.close('all')
    
    def show_pointcloud(self, points, colors=None, window_name="Point Cloud", block=True):
        """Show point cloud in a new window"""
        if len(points) == 0:
            print("No points to display")
            return
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if colors is not None and len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Add coordinate frame for reference
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        
        if block:
            o3d.visualization.draw_geometries([pcd, coord_frame], window_name=window_name)
        else:
            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name)
            vis.add_geometry(pcd)
            vis.add_geometry(coord_frame)
            vis.run()
            vis.destroy_window()
    
    def close(self):
        """Close the visualizer"""
        if self.vis is not None:
            self.vis.destroy_window()

class PointCloudSaver:
    """Handles saving point clouds to various formats"""
    
    @staticmethod
    def save_ply(points, colors, filename):
        """
        Save point cloud in PLY format
        
        Args:
            points: Point cloud array (N x 3)
            colors: Color array (N x 3) with values 0-1
            filename: Output filename
        """
        if len(points) == 0:
            print(f"Warning: No points to save for {filename}")
            return
            
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            
            for i in range(len(points)):
                if i < len(colors):
                    r, g, b = (colors[i] * 255).astype(int)
                else:
                    r, g, b = 128, 128, 128  # Default gray color
                f.write(f"{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f} {r} {g} {b}\n")
        
        print(f"Point cloud saved to {filename}")
    
    @staticmethod
    def save_pcd(points, colors, filename):
        """
        Save point cloud in Open3D PCD format
        
        Args:
            points: Point cloud array (N x 3)
            colors: Color array (N x 3) with values 0-1
            filename: Output filename
        """
        if len(points) == 0:
            print(f"Warning: No points to save for {filename}")
            return
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        if colors is not None and len(colors) == len(points):
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        o3d.io.write_point_cloud(filename, pcd)
        print(f"Point cloud saved to {filename}")

def create_robot_motion_camera(robot):
    """
    Factory function to create a camera controller for robot motion
    
    Args:
        robot: FlappingRobot instance
        
    Returns:
        CameraMotionController: Configured camera controller
    """
    return CameraMotionController(robot)

def create_picoflexx2_processor():
    """
    Factory function to create a PicoFlexx2 VGA point cloud processor
    
    Returns:
        PointCloudProcessor: Configured for PicoFlexx2 VGA specifications
    """
    return PointCloudProcessor(
        width=224,      # PicoFlexx2 VGA width
        height=172,     # PicoFlexx2 VGA height
        fov_h=64,       # Horizontal FOV in degrees
        fov_v=50,       # Vertical FOV in degrees
        near=0.1,       # 10cm minimum range
        far=4.0         # 4m maximum range
    )