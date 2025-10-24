#!/usr/bin/env python3

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Twist
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, inverse_matrix, euler_from_quaternion
import numpy as np
from logic.joystick_coordinates2_robot import calculate_user_orientation


class ImitationNode:
    """
    ROS node to make a robot imitate human movements from Meta Quest 3 controllers.
    The node listens to the /tf topic to get the position of the controllers.
    """

    def __init__(self):
        """
        Initializes the imitation node.
        """
        # Initialize the node
        rospy.init_node('imitation_node', anonymous=True)

        # Get robot type from ROS parameter server, default to "NAO"
        self.robot_type = rospy.get_param('~robot_type', 'NAO').upper()

        # Create a transform listener to get data from /tf
        self.tf_listener = tf.TransformListener()

        # Publisher for user orientation
        self.orientation_publisher = rospy.Publisher('/user_orientation', PoseStamped, queue_size=1)

        # Publisher for robot velocity commands
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Parameters for rotation control
        self.max_angular_vel = 0.5  # Maximum angular velocity (rad/s)
        self.angle_tolerance = 0.1  # Tolerance in radians (~5.7 degrees)
        self.kp_angular = 1  # Proportional gain for angular velocity control

        # Store user orientation
        self.user_yaw = 0.0
        
        # Coordinate system alignment
        self.orientation_offset = 0.0  # Offset between robot and VR coordinate systems
        self.alignment_complete = False
        self.alignment_duration = 5.0  # seconds to wait for user alignment
        self.start_time = None

        # Initialize calibration flag and other attributes
        self.calibrated = False

        rospy.loginfo("Imitation Node initialized.")
        rospy.loginfo("Listening for transforms on /tf topic...")

    def publish_user_orientation(self, left_pos, right_pos, headset_rot, left_rot=None, right_rot=None):
        """
        Calculates and publishes the user's orientation in 2D.
        The orientation is determined by the vector from the right to the left hand,
        rotated by -90 degrees around the Z-axis to point forward.
        """
        yaw = calculate_user_orientation(left_pos, right_pos, headset_rot, left_rot, right_rot)

        # Store the user yaw for robot rotation control
        self.user_yaw = yaw

        # Create a quaternion from the 2D yaw
        orientation_q = quaternion_from_euler(0, 0, yaw)

        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "vr_origin"

        # The position of the orientation arrow will be the average of the hands' position on the ground
        pose_stamped.pose.position.x = (left_pos[0] + right_pos[0]) / 2
        pose_stamped.pose.position.y = (left_pos[1] + right_pos[1]) / 2
        pose_stamped.pose.position.z = 0 # On the ground plane

        # The orientation will be the calculated 2D orientation
        pose_stamped.pose.orientation.x = orientation_q[0]
        pose_stamped.pose.orientation.y = orientation_q[1]
        pose_stamped.pose.orientation.z = orientation_q[2]
        pose_stamped.pose.orientation.w = orientation_q[3]

        # Publish the message
        self.orientation_publisher.publish(pose_stamped)

    def get_robot_orientation(self):
        """
        Gets the current orientation of the robot from /tf.
        Returns the yaw angle in radians, or None if not available.
        """
        try:
            # Try to get the robot's base_link transform
            # Common robot base frames: base_link, base_footprint, odom
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            
            # Convert quaternion to euler angles
            euler = euler_from_quaternion(rot)
            robot_yaw = euler[2]  # Yaw is the rotation around Z-axis
            
            return robot_yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get robot orientation: {e}")
            return None

    def rotate_robot_to_user(self):
        """
        Rotates the robot to face the user's orientation using cmd_vel.
        Uses proportional control to smoothly align with the user.
        """
        # Get current robot orientation
        robot_yaw = self.get_robot_orientation()
        
        if robot_yaw is None:
            rospy.logwarn_throttle(5.0, "Cannot rotate robot: robot orientation not available")
            return
        
        # Apply the orientation offset to compensate for different coordinate systems
        adjusted_user_yaw = self.user_yaw - self.orientation_offset
        
        # Calculate angle difference (shortest path)
        angle_diff = adjusted_user_yaw - robot_yaw
        
        # Normalize angle to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Create velocity command
        cmd_vel = Twist()
        
        # Only rotate if the angle difference is above tolerance
        if abs(angle_diff) > self.angle_tolerance:
            # Proportional control for smooth rotation
            angular_vel = self.kp_angular * angle_diff
            
            # Clamp to max angular velocity
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
            
            cmd_vel.angular.z = angular_vel
            
            rospy.loginfo_throttle(1.0, f"Rotating robot: angle_diff={np.degrees(angle_diff):.2f}°, vel={angular_vel:.2f} rad/s")
        else:
            # Stop rotation when aligned
            cmd_vel.angular.z = 0.0
        
        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd_vel)

    def get_controller_transforms(self):
        """
        Gets the transforms for left controller, right controller, and headset.
        Returns (left_transform, right_transform, headset_transform) as tuples of (position, rotation).
        Returns (None, None, None) if transforms are not available.
        """
        try:
            # Get left controller transform
            (left_trans, left_rot) = self.tf_listener.lookupTransform('/vr_origin', '/hand_left', rospy.Time(0))
            
            # Get right controller transform
            (right_trans, right_rot) = self.tf_listener.lookupTransform('/vr_origin', '/hand_right', rospy.Time(0))
            
            # Get headset transform
            (headset_trans, headset_rot) = self.tf_listener.lookupTransform('/vr_origin', '/headset', rospy.Time(0))
            
            return (left_trans, left_rot), (right_trans, right_rot), (headset_trans, headset_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get controller transforms: {e}")
            return None, None, None

    def run(self):
        """
        Main loop of the node.
        """
        rate = rospy.Rate(10)

        self.last_movement_time = rospy.Time.now()
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("ALIGNMENT PHASE: Please face the same direction as the robot!")
        rospy.loginfo(f"You have {self.alignment_duration} seconds to align...")
        rospy.loginfo("=" * 60)

        while not rospy.is_shutdown():
            # In each loop, get the latest transforms
            left_transform, right_transform, headset_transform = self.get_controller_transforms()

            if left_transform and right_transform and headset_transform:
                # Publish user orientation
                self.publish_user_orientation(
                    left_transform[0], right_transform[0], headset_transform[1],
                    left_transform[1], right_transform[1]
                )
                
                # Check if we're still in the alignment phase
                elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
                
                if not self.alignment_complete:
                    if elapsed_time < self.alignment_duration:
                        # Still in alignment phase - show countdown
                        remaining = self.alignment_duration - elapsed_time
                        rospy.loginfo_throttle(1.0, f"Alignment phase: {remaining:.1f} seconds remaining...")
                    else:
                        # Alignment phase complete - calculate offset
                        robot_yaw = self.get_robot_orientation()
                        if robot_yaw is not None:
                            self.orientation_offset = self.user_yaw - robot_yaw
                            rospy.loginfo("=" * 60)
                            rospy.loginfo("ALIGNMENT COMPLETE!")
                            rospy.loginfo(f"Orientation offset calculated: {np.degrees(self.orientation_offset):.2f}°")
                            rospy.loginfo("Robot will now track your orientation.")
                            rospy.loginfo("=" * 60)
                            self.alignment_complete = True
                        else:
                            rospy.logwarn("Could not get robot orientation for alignment. Retrying...")
                            self.start_time = rospy.Time.now()  # Reset timer
                else:
                    # Alignment complete - start rotating robot to track user
                    self.rotate_robot_to_user()

            # Wait for the next cycle
            rate.sleep()

    def shutdown(self):
        """
        Actions to perform on node shutdown.
        """
        rospy.loginfo("Shutting down Imitation Node...")
        
        # Send stop command to robot
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        
        # Stop any ongoing movements
        try:
            if hasattr(self, 'left_arm_group') and self.left_arm_group:
                self.left_arm_group.stop()
                self.left_arm_group.clear_pose_targets()
            if hasattr(self, 'right_arm_group') and self.right_arm_group:
                self.right_arm_group.stop()
                self.right_arm_group.clear_pose_targets()
            moveit_commander.roscpp_shutdown()
        except (AttributeError, rospy.ROSException) as e:
            rospy.logwarn(f"Error during shutdown: {e}")
            pass
        rospy.loginfo("Imitation Node shut down.")


def main():
    """
    Main function to create and run the ROS node.
    """
    node = None
    try:
        node = ImitationNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"Unhandled exception in main: {e}")
    finally:
        if node:
            node.shutdown()
        else:
            moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
