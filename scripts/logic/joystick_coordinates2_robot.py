#!/usr/bin/env python3

import numpy as np
from tf.transformations import euler_from_quaternion

def calculate_user_orientation(left_pos, right_pos, headset_rot):
    """
    Calculates the user's orientation in 2D.
    The orientation is determined by the vector from the right to the left hand,
    rotated by -90 degrees around the Z-axis to point forward.
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
    
    return yaw
