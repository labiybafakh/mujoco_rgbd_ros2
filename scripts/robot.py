import numpy as np
import math

class FlappingRobot:
    def __init__(self, flapping_freq=5.0, initial_pos=[0, 0, 1]):
        # Robot parameters
        self.flapping_freq = flapping_freq
        self.dt = 0.01
        self.g = 9.81
        
        # Flight parameters
        self.v_forward_base = 3.5  # base forward velocity (m/s)
        self.k_tail = 0.1         # tail effectiveness
        
        # Rate-based movement parameters
        self.K_x = 0.1  # X-axis movement scaling factor
        self.K_y = 0.1  # Y-axis movement scaling factor  
        self.K_z = 0.1  # Z-axis movement scaling factor
        
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
        self.tail_pitch = math.radians(30.0)
        
        # Altitude control mode
        self.pitch_controls_altitude = False  # Set to True to enable pitch-based altitude control
        
    def get_wing_roll_angle(self, t):
        return 0.0
    
    def get_tail_pitch_angle(self, t):
        return math.radians(30.0)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cr, cp, cy = math.cos(roll * 0.5), math.cos(pitch * 0.5), math.cos(yaw * 0.5)
        sr, sp, sy = math.sin(roll * 0.5), math.sin(pitch * 0.5), math.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def quaternion_to_rotation_matrix(self, q):
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
        flap_phase = 2 * math.pi * self.flapping_freq * self.time
        
        v_flap_oscillation = 0.05 * (1 if math.sin(flap_phase) >= 0 else -1)
        roll_oscillation = 0.02 * math.sin(flap_phase)
        pitch_oscillation = 0.01 * math.cos(flap_phase)
        
        return v_flap_oscillation, roll_oscillation, pitch_oscillation
    
    def calculate_rate_based_movement(self):
        roll_degrees = math.degrees(self.wing_roll)
        pitch_degrees = math.degrees(self.tail_pitch)
        
        rate_position_x = self.flapping_freq * self.K_x * pitch_degrees
        rate_position_y = self.flapping_freq * self.K_y * roll_degrees  
        rate_position_z = self.flapping_freq * self.K_z * roll_degrees
        
        return rate_position_x, rate_position_y, rate_position_z
    
        
    def update_control_inputs(self):
        self.wing_roll = self.get_wing_roll_angle(self.time)
        self.tail_pitch = self.get_tail_pitch_angle(self.time)
    
    def update_motion(self):
        v_flap_osc, roll_osc, pitch_osc = self.add_flapping_oscillations()
        rate_x, rate_y, rate_z = self.calculate_rate_based_movement()
        
        v_forward = self.v_forward_base + v_flap_osc
        self.position[0] += (v_forward + rate_x) * self.dt
        
        a_lateral = self.g * math.tan(self.wing_roll)
        self.velocity[1] += a_lateral * self.dt
        self.position[1] += (self.velocity[1] + rate_y) * self.dt
        
        pitch_deviation = self.tail_pitch - math.radians(30.0)
        self.position[2] += (v_forward * math.sin(pitch_deviation) + rate_z) * self.dt
        
        self.attitude[0] = self.wing_roll + roll_osc
        self.attitude[1] = self.tail_pitch + pitch_osc
        
        if v_forward > 0.1:
            yaw_rate = self.g * math.tan(self.wing_roll) / v_forward
            self.attitude[2] += yaw_rate * self.dt
        
        self.quaternion = self.euler_to_quaternion(
            self.attitude[0], self.attitude[1], self.attitude[2]
        )

    def euler_to_rotation_matrix(roll, pitch, yaw):
        cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
        sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)

        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]
        ])
        
        return R
    
    def get_camera_transform(self):
        transform = np.eye(4)
        
        if self.use_simulation_camera_adjustment:
            sim_position = self.position
            
            roll_x = math.radians(90 + self.wing_roll)
            pitch_y = math.radians(0)
            yaw_z = math.radians(-90 + (self.tail_pitch - 30.0))
            
            sim_quaternion = self.euler_to_quaternion(roll_x, pitch_y, yaw_z)
            sim_rotation = self.quaternion_to_rotation_matrix(sim_quaternion)

            transform[:3, :3] = sim_rotation
            transform[:3, 3] = sim_position
        else:
            rotation_matrix = self.quaternion_to_rotation_matrix(self.quaternion)
            transform[:3, :3] = rotation_matrix
            transform[:3, 3] = self.position

        return transform
    
    def step(self, dt=None):
        if dt is not None:
            self.dt = dt
            
        self.update_control_inputs()
        self.update_motion()
        self.time += self.dt
    
    def get_status(self):
        return {
            'time': self.time,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'attitude_deg': np.degrees(self.attitude),
            'wing_roll_deg': math.degrees(self.wing_roll),
            'tail_pitch_deg': math.degrees(self.tail_pitch),
            'flapping_freq': self.flapping_freq,
            'forward_velocity': self.v_forward_base
        }