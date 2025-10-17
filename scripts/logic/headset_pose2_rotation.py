# NAO head angles: HeadYaw (-119.5 to 119.5 degrees), HeadPitch (-38.5 to 29.5 degrees)
# Pepper head angles: HeadYaw (-119.5 to 119.5 degrees), HeadPitch (-40.5 to 25.5 degrees)

import math
from geometry_msgs.msg import Quaternion

class HeadsetToRobotRotation:
    def __init__(self, robot_type="NAO"):
        """
        Initialize the converter with robot-specific angle limits
        
        Args:
            robot_type (str): "NAO" or "Pepper"
        """
        self.robot_type = robot_type
        
        if robot_type == "NAO":
            self.yaw_limits = (-119.5, 119.5)  # degrees
            self.pitch_limits = (-38.5, 29.5)  # degrees
        elif robot_type == "Pepper":
            self.yaw_limits = (-119.5, 119.5)  # degrees
            self.pitch_limits = (-40.5, 25.5)  # degrees
        else:
            raise ValueError("Robot type must be 'NAO' or 'Pepper'")
    
    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        
        Args:
            quaternion: geometry_msgs/Quaternion
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def radians_to_degrees(self, radians):
        """Convert radians to degrees"""
        return radians * 180.0 / math.pi
    
    def clamp_angle(self, angle, limits):
        """
        Clamp angle to within specified limits
        
        Args:
            angle (float): Angle in degrees
            limits (tuple): (min_angle, max_angle) in degrees
            
        Returns:
            float: Clamped angle in degrees
        """
        return max(limits[0], min(limits[1], angle))
    
    def convert_headset_to_robot_angles(self, headset_quaternion):
        """
        Convert headset quaternion orientation to robot head angles
        
        Args:
            headset_quaternion: geometry_msgs/Quaternion from headset
            
        Returns:
            tuple: (head_yaw, head_pitch) in degrees, clamped to robot limits
        """
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(headset_quaternion)
        
        # Convert to degrees
        roll_deg = self.radians_to_degrees(roll)
        pitch_deg = self.radians_to_degrees(pitch)
        yaw_deg = self.radians_to_degrees(yaw)
        
        # Map headset orientation to robot head angles
        # Assuming headset yaw maps to robot HeadYaw and headset pitch maps to robot HeadPitch
        # You may need to adjust the signs based on your coordinate system conventions
        head_yaw = yaw_deg
        head_pitch = -pitch_deg  # Often pitch is inverted
        
        # Clamp to robot limits
        head_yaw = self.clamp_angle(head_yaw, self.yaw_limits)
        head_pitch = self.clamp_angle(head_pitch, self.pitch_limits)
        
        return head_yaw, head_pitch
    
    def get_robot_limits(self):
        """
        Get the current robot's angle limits
        
        Returns:
            dict: Dictionary with yaw and pitch limits
        """
        return {
            'yaw_limits': self.yaw_limits,
            'pitch_limits': self.pitch_limits
        }