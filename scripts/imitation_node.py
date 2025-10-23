#!/usr/bin/env python3

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, inverse_matrix
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

        # Create a transform listener to get data from /tf
        self.tf_listener = tf.TransformListener()

        # Publisher for user orientation
        self.orientation_publisher = rospy.Publisher('/user_orientation', PoseStamped, queue_size=1)

        rospy.loginfo("Imitation Node initialized.")
        rospy.loginfo("Listening for transforms on /tf topic...")

    def publish_user_orientation(self, left_pos, right_pos, headset_rot):
        """
        Calculates and publishes the user's orientation in 2D.
        The orientation is determined by the vector from the right to the left hand,
        rotated by -90 degrees around the Z-axis to point forward.
        """
        yaw = calculate_user_orientation(left_pos, right_pos, headset_rot)

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

    def run(self):
        """
        Main loop of the node.
        """
        rate = rospy.Rate(10)

        while not self.calibrated and not rospy.is_shutdown():
            self.calibrate()
            if not self.calibrated:
                rospy.logwarn("Calibration failed, retrying in 5 seconds...")
                rospy.sleep(5.0)

        self.last_movement_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # In each loop, get the latest transforms
            left_transform, right_transform, headset_transform = self.get_controller_transforms()

            if left_transform and right_transform and headset_transform:
                # Publish user orientation
                self.publish_user_orientation(left_transform[0], right_transform[0], headset_transform[1])

            # Wait for the next cycle
            rate.sleep()

    def shutdown(self):
        """
        Actions to perform on node shutdown.
        """
        rospy.loginfo("Shutting down Imitation Node...")
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
