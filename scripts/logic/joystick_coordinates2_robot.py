#!/usr/bin/env python3

import numpy as np
from tf.transformations import euler_from_quaternion

def calculate_user_orientation(left_pos, right_pos, headset_rot, left_rot=None, right_rot=None):
    """
    Calculates the user's orientation in 2D.
    The orientation is determined by the vector from the right to the left hand,
    rotated by -90 degrees around the Z-axis to point forward.
    
    Args:
        left_pos: Position of left controller
        right_pos: Position of right controller
        headset_rot: Rotation quaternion of headset
        left_rot: Rotation quaternion of left controller (optional)
        right_rot: Rotation quaternion of right controller (optional)
    """
    # Vector from right to left hand
    hand_vector = np.array(left_pos) - np.array(right_pos)

    # We only care about the 2D direction
    hand_vector[2] = 0
    
    # Normalize the vector
    norm = np.linalg.norm(hand_vector)
    if norm == 0:
        # If hands are at the same position, fall back to headset orientation
        _, _, yaw = euler_from_quaternion(headset_rot)
    else:
        hand_vector /= norm
        # Calculate yaw from the hand vector
        # The vector from right to left hand points to the user's left.
        # We want the forward direction, so we calculate the angle of the vector
        # and subtract pi/2 (90 degrees).
        yaw = np.arctan2(hand_vector[1], hand_vector[0]) - np.pi / 2
        
        # Check if controllers are facing the same direction as the calculated vector
        # If they're facing the opposite direction (arms crossed), invert the orientation
        if left_rot is not None and right_rot is not None:
            # Get the forward direction of each controller (assuming forward is +Z in local frame)
            # Extract yaw angles from controller rotations
            _, _, left_yaw = euler_from_quaternion(left_rot)
            _, _, right_yaw = euler_from_quaternion(right_rot)
            
            # Calculate average controller yaw
            avg_controller_yaw = np.arctan2(
                (np.sin(left_yaw) + np.sin(right_yaw)) / 2,
                (np.cos(left_yaw) + np.cos(right_yaw)) / 2
            )
            
            # Calculate the difference between calculated orientation and controller direction
            angle_diff = yaw - avg_controller_yaw
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi
            
            # If controllers are facing more than 90 degrees away from calculated direction,
            # the arms are likely crossed - invert the orientation
            if abs(angle_diff) > np.pi / 2:
                yaw += np.pi
                # Normalize yaw to [-pi, pi]
                while yaw > np.pi:
                    yaw -= 2 * np.pi
                while yaw < -np.pi:
                    yaw += 2 * np.pi
    
    return yaw
