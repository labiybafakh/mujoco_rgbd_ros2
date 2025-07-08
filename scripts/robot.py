import numpy as np
import math

class FlappingRobot:
    """
    Flapping robot model with control inputs and motion dynamics
    """
    def __init__(self, flapping_freq=5.0, initial_pos=[0, 0, 1], trajectory_data=None):
        # Robot parameters
        self.flapping_freq = flapping_freq
        self.dt = 0.01
        self.g = 9.81
        
        # Flight parameters
        self.v_forward_base = 0.4  # base forward velocity (m/s)
        self.k_tail = 0.1         # tail effectiveness
        
        # SIMULATION CAMERA ADJUSTMENT (not for real robot)
        self.use_simulation_camera_adjustment = True
        
        # State variables
        self.position = np.array(initial_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        self.attitude = np.array([0.0, 0.0, 0.0], dtype=float)  # roll, pitch, yaw
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # w, x, y, z
        
        # Time tracking
        self.time = 0.0
        
        # Control inputs
        self.wing_roll = 0.0
        self.tail_pitch = 0.0
        
    def get_wing_roll_angle(self, t):
        """
        Wing roll angle based on trajectory data
        - Cruise phase (0-6s): minimal roll (~0°)
        - Avoidance phase (6-9s): peak roll of ±40°
        - Recovery phase (9s+): return to 0°
        """
        if t < 6.0:
            return math.radians(-2.0)  # slight baseline roll
        elif t <= 9.0:
            # Avoidance maneuver - exponential profile
            t_center = 7.5
            sigma = 0.6
            roll_max = 40.0  # degrees
            roll_factor = math.exp(-((t - t_center) / sigma) ** 2)
            return math.radians(-roll_max * roll_factor)  # negative for right turn
        else:
            # Recovery phase - exponential decay back to cruise
            decay_rate = 2.0
            remaining_roll = -2.0 * math.exp(-decay_rate * (t - 9.0))
            return math.radians(remaining_roll)
    
    def get_tail_pitch_angle(self, t):
        """
        Tail pitch angle based on flight phase
        Uses 30° pitch angle as specified in your data
        """
        base_pitch = math.radians(30.0)  # 30 degrees base pitch
        
        # Additional compensation during avoidance maneuver
        if 6.0 <= t <= 9.0:
            # Slight adjustment during aggressive maneuvers
            maneuver_compensation = math.radians(5.0) * abs(math.sin((t - 6.0) * math.pi / 3.0))
            return base_pitch + maneuver_compensation
        else:
            return base_pitch
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion (w, x, y, z)"""
        cr, cp, cy = math.cos(roll * 0.5), math.cos(pitch * 0.5), math.cos(yaw * 0.5)
        sr, sp, sy = math.sin(roll * 0.5), math.sin(pitch * 0.5), math.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix"""
        w, x, y, z = q
        
        # Normalize quaternion
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # Convert to rotation matrix
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    def add_flapping_oscillations(self):
        """Add small oscillations due to flapping motion"""
        flap_phase = 2 * math.pi * self.flapping_freq * self.time
        
        # Forward velocity oscillation
        v_flap_oscillation = 0.05 * math.sin(flap_phase)
        
        # Small attitude oscillations from flapping
        roll_oscillation = 0.02 * math.sin(flap_phase)
        pitch_oscillation = 0.01 * math.cos(flap_phase)
        
        return v_flap_oscillation, roll_oscillation, pitch_oscillation
    
    def update_control_inputs(self):
        """Update control inputs based on current time"""
        self.wing_roll = self.get_wing_roll_angle(self.time)
        self.tail_pitch = self.get_tail_pitch_angle(self.time)
    
    def update_motion(self):
        """Update robot motion based on control inputs"""
        # Get flapping oscillations
        v_flap_osc, roll_osc, pitch_osc = self.add_flapping_oscillations()
        
        # Forward motion ONLY (disable lateral motion for debugging)
        v_forward = self.v_forward_base + v_flap_osc
        self.position[0] += v_forward * self.dt
        
        # DISABLED: Lateral motion from wing roll
        # a_lateral = self.g * math.tan(self.wing_roll)
        # self.velocity[1] += a_lateral * self.dt
        # self.position[1] += self.velocity[1] * self.dt
        
        # DISABLED: Altitude control with tail pitch
        # self.position[2] += v_forward * math.sin(self.tail_pitch) * self.dt
        
        # DISABLED: Update attitude
        # self.attitude[0] = self.wing_roll + roll_osc  # roll
        # self.attitude[1] = self.tail_pitch + pitch_osc  # pitch
        
        # DISABLED: Yaw from coordinated turn
        # if v_forward > 0.1:  # avoid division by zero
        #     yaw_rate = self.g * math.tan(self.wing_roll) / v_forward
        #     self.attitude[2] += yaw_rate * self.dt  # yaw
        
        # DISABLED: Convert to quaternion
        # self.quaternion = self.euler_to_quaternion(
        #     self.attitude[0], self.attitude[1], self.attitude[2]
        # )
    
    def get_camera_transform(self):
        """Get 4x4 transformation matrix for camera"""
        # Translation only - no rotation to isolate position issues
        # Camera maintains fixed orientation, only follows robot position
        
        # COORDINATE FRAME DOCUMENTATION:
        # ===============================
        # Robot Internal Coordinate System:
        #   - X-axis: Forward direction (positive X = robot moves forward)
        #   - Y-axis: Left direction (positive Y = robot moves left)
        #   - Z-axis: Up direction (positive Z = robot moves upward)
        #
        # Genesis Coordinate System:
        #   - X-axis: Forward direction (positive X = camera moves forward in scene)
        #   - Y-axis: Left direction (positive Y = camera moves left in scene)
        #   - Z-axis: Up direction (positive Z = camera moves upward in scene)
        #
        # Current Mapping: DIRECT (no transformation needed)
        #   - Robot X → Genesis X (forward)
        #   - Robot Y → Genesis Y (left)
        #   - Robot Z → Genesis Z (up)
        #
        # Variables:
        #   - self.position[0]: Robot X position (forward/backward)
        #   - self.position[1]: Robot Y position (left/right)
        #   - self.position[2]: Robot Z position (up/down)
        
        # Initialize 4x4 transformation matrix (homogeneous coordinates)
        # np.eye(4) creates a 4x4 identity matrix:
        # [[1, 0, 0, 0],     # X axis (right)
        #  [0, 1, 0, 0],     # Y axis (up) 
        #  [0, 0, 1, 0],     # Z axis (forward)
        #  [0, 0, 0, 1]]     # homogeneous coordinate (always 1)
        #
        # Structure of 4x4 transform matrix:
        # [R R R T]  where R = 3x3 rotation matrix, T = 3x1 translation vector
        # [R R R T]        transform[:3, :3] = rotation
        # [R R R T]        transform[:3, 3] = translation
        # [0 0 0 1]        bottom row always [0,0,0,1]
        transform = np.eye(4)
        
        # Apply simulation camera adjustment if enabled
        if self.use_simulation_camera_adjustment:
            # SIMULATION POSITION ADJUSTMENT
            # Direct position mapping - coordinate systems match
            sim_position = self.position
            
            # SIMULATION ORIENTATION ADJUSTMENT
            # Genesis camera default orientation looks down (-Z direction)
            # We need camera to look forward (+X direction) to see obstacles ahead
            # Required rotation: X+90° (roll), Z-90° (yaw) to correct orientation
            
            # Camera rotation quaternion: X+90°, Z-90°
            # Convert to radians and create quaternion
            roll_x = math.radians(90)   # X-axis rotation +90° (roll camera)
            pitch_y = math.radians(0)   # Y-axis rotation 0° (no pitch)
            yaw_z = math.radians(-90)   # Z-axis rotation -90° (yaw camera)
            
            # Create quaternion for combined rotation
            sim_quaternion = self.euler_to_quaternion(roll_x, pitch_y, yaw_z)
            
            # Convert quaternion to rotation matrix
            sim_rotation = self.quaternion_to_rotation_matrix(sim_quaternion)
            
            # Set rotation part of transform matrix (top-left 3x3)
            transform[:3, :3] = sim_rotation
            # Set translation part of transform matrix (top-right 3x1)
            transform[:3, 3] = sim_position
        else:
            # REAL ROBOT CAMERA (no adjustments)
            # Use robot's actual orientation and position directly
            rotation_matrix = self.quaternion_to_rotation_matrix(self.quaternion)
            transform[:3, :3] = rotation_matrix
            transform[:3, 3] = self.position
        
        return transform
    
    def step(self, dt=None):
        """Advance simulation by one time step"""
        if dt is not None:
            self.dt = dt
            
        self.update_control_inputs()
        self.update_motion()
        self.time += self.dt
    
    def get_status(self):
        """Get current robot status for debugging"""
        return {
            'time': self.time,
            'position': self.position.copy(),
            'attitude_deg': np.degrees(self.attitude),
            'wing_roll_deg': math.degrees(self.wing_roll),
            'tail_pitch_deg': math.degrees(self.tail_pitch)
        }