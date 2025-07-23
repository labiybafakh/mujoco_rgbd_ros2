"""
Camera utilities for point cloud generation with robot motion model
Handles camera pose calculation and robot attitude integration
"""

import numpy as np
import math
import pybullet as p

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
        
        rotation_matrix = np.array([right, -up, forward]).T
        
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
    
    def get_point_colors(self, rgb_img, depth):
        """
        Extract colors for valid points from RGB image
        
        Args:
            rgb_img: RGB image from PyBullet
            depth: Depth array (height x width)
            
        Returns:
            np.array: Color array (N x 3) with values 0-1
        """
        # Convert RGB image to point colors
        rgb_array = np.array(rgb_img).reshape(self.height, self.width, 4)[:, :, :3]  # Remove alpha channel
        valid_mask = (depth > self.near) & (depth < (self.far * 0.95)) & np.isfinite(depth)
        colors = rgb_array[valid_mask].reshape(-1, 3) / 255.0
        return colors

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