#!/usr/bin/env python3

import sys
import rospy
import tf
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, inverse_matrix, euler_from_quaternion
import numpy as np


class UserArm2RobotNode:
    """
    ROS node to map Meta Quest 3 controller positions to robot arm movements.
    The node:
    - Listens to /tf for VR controller positions
    - Gets robot end effector positions via MoveIt
    - Transforms controller positions to robot coordinate system
    - Publishes transformed poses for visualization and control
    """

    def __init__(self):
        """
        Initializes the user arm to robot node.
        """
        # Initialize moveit_commander and the ROS node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('user_arm2_robot_node', anonymous=True)

        # Get robot type from ROS parameter server, default to "PEPPER"
        self.robot_type = rospy.get_param('~robot_type', 'PEPPER').upper()

        # Create a transform listener to get data from /tf
        self.tf_listener = tf.TransformListener()

        # Initialize MoveIt commanders for robot arms
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Setup move groups for left and right arms
        # Adjust group names based on your robot's MoveIt configuration
        try:
            self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
            self.left_arm_group.set_goal_position_tolerance(0.01)
            self.left_arm_group.set_goal_orientation_tolerance(0.01)
            rospy.loginfo("Left arm MoveGroup initialized successfully")
        except Exception as e:
            rospy.logwarn(f"Could not initialize left arm group: {e}")
            self.left_arm_group = None

        try:
            self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
            self.right_arm_group.set_goal_position_tolerance(0.01)
            self.right_arm_group.set_goal_orientation_tolerance(0.01)
            rospy.loginfo("Right arm MoveGroup initialized successfully")
        except Exception as e:
            rospy.logwarn(f"Could not initialize right arm group: {e}")
            self.right_arm_group = None

        # Publishers for transformed controller positions
        self.left_hand_transformed_pub = rospy.Publisher(
            '/vr_left_hand_transformed', PoseStamped, queue_size=1)
        self.right_hand_transformed_pub = rospy.Publisher(
            '/vr_right_hand_transformed', PoseStamped, queue_size=1)
        
        # Publishers for robot end effector positions (for visualization)
        self.left_ee_pub = rospy.Publisher(
            '/robot_left_ee_pose', PoseStamped, queue_size=1)
        self.right_ee_pub = rospy.Publisher(
            '/robot_right_ee_pose', PoseStamped, queue_size=1)

        # Coordinate system transformation parameters
        self.calibration_complete = False
        self.calibration_duration = 5.0  # seconds to collect calibration data
        self.start_time = None
        
        # Transformation matrices (to be calculated during calibration)
        self.vr_to_robot_transform = None
        self.robot_to_vr_transform = None
        
        # Storage for calibration data
        self.calibration_vr_points = []
        self.calibration_robot_points = []

        rospy.loginfo("User Arm2Robot Node initialized.")
        rospy.loginfo("Listening for VR controller and robot transforms...")

    def get_controller_transforms(self):
        """
        Gets the transforms for left and right controllers from VR system.
        Returns (left_transform, right_transform) as tuples of (position, rotation).
        Returns (None, None) if transforms are not available.
        """
        try:
            # Get left controller transform
            (left_trans, left_rot) = self.tf_listener.lookupTransform(
                '/vr_origin', '/hand_left', rospy.Time(0))
            
            # Get right controller transform
            (right_trans, right_rot) = self.tf_listener.lookupTransform(
                '/vr_origin', '/hand_right', rospy.Time(0))
            
            return (left_trans, left_rot), (right_trans, right_rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get controller transforms: {e}")
            return None, None

    def get_robot_end_effector_poses(self):
        """
        Gets the current poses of the robot's end effectors using MoveIt.
        Returns (left_pose, right_pose) as Pose objects.
        Returns (None, None) if poses are not available.
        """
        left_pose = None
        right_pose = None

        try:
            if self.left_arm_group:
                left_pose = self.left_arm_group.get_current_pose().pose
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Could not get left arm pose: {e}")

        try:
            if self.right_arm_group:
                right_pose = self.right_arm_group.get_current_pose().pose
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Could not get right arm pose: {e}")

        return left_pose, right_pose

    def publish_robot_end_effector_poses(self, left_pose, right_pose):
        """
        Publishes the current robot end effector poses for visualization.
        """
        timestamp = rospy.Time.now()

        if left_pose:
            left_pose_stamped = PoseStamped()
            left_pose_stamped.header.stamp = timestamp
            left_pose_stamped.header.frame_id = "base_link"  # Adjust frame as needed
            left_pose_stamped.pose = left_pose
            self.left_ee_pub.publish(left_pose_stamped)

        if right_pose:
            right_pose_stamped = PoseStamped()
            right_pose_stamped.header.stamp = timestamp
            right_pose_stamped.header.frame_id = "base_link"  # Adjust frame as needed
            right_pose_stamped.pose = right_pose
            self.right_ee_pub.publish(right_pose_stamped)

    def transform_vr_to_robot_coordinates(self, vr_position, vr_orientation):
        """
        Transforms a VR controller position and orientation to robot coordinate system.
        
        Args:
            vr_position: tuple of (x, y, z) in VR coordinates
            vr_orientation: tuple of (x, y, z, w) quaternion in VR coordinates
        
        Returns:
            Pose object in robot coordinate system, or None if transformation not ready
        """
        if not self.calibration_complete or self.vr_to_robot_transform is None:
            # Return identity transform until calibration is complete
            pose = Pose()
            pose.position.x = vr_position[0]
            pose.position.y = vr_position[1]
            pose.position.z = vr_position[2]
            pose.orientation.x = vr_orientation[0]
            pose.orientation.y = vr_orientation[1]
            pose.orientation.z = vr_orientation[2]
            pose.orientation.w = vr_orientation[3]
            return pose

        # TODO: Apply transformation matrix to convert VR coordinates to robot coordinates
        # This will be implemented after calibration logic is complete
        
        pose = Pose()
        pose.position.x = vr_position[0]
        pose.position.y = vr_position[1]
        pose.position.z = vr_position[2]
        pose.orientation.x = vr_orientation[0]
        pose.orientation.y = vr_orientation[1]
        pose.orientation.z = vr_orientation[2]
        pose.orientation.w = vr_orientation[3]
        
        return pose

    def publish_transformed_controller_poses(self, left_transform, right_transform):
        """
        Transforms and publishes VR controller positions in robot coordinate system.
        """
        timestamp = rospy.Time.now()

        if left_transform:
            left_pos, left_rot = left_transform
            transformed_pose = self.transform_vr_to_robot_coordinates(left_pos, left_rot)
            
            left_pose_stamped = PoseStamped()
            left_pose_stamped.header.stamp = timestamp
            left_pose_stamped.header.frame_id = "base_link"  # Will match robot frame
            left_pose_stamped.pose = transformed_pose
            self.left_hand_transformed_pub.publish(left_pose_stamped)

        if right_transform:
            right_pos, right_rot = right_transform
            transformed_pose = self.transform_vr_to_robot_coordinates(right_pos, right_rot)
            
            right_pose_stamped = PoseStamped()
            right_pose_stamped.header.stamp = timestamp
            right_pose_stamped.header.frame_id = "base_link"  # Will match robot frame
            right_pose_stamped.pose = transformed_pose
            self.right_hand_transformed_pub.publish(right_pose_stamped)

    def perform_calibration(self):
        """
        Performs calibration to align VR and robot coordinate systems.
        Collects corresponding points from both systems and calculates transformation.
        
        TODO: Implement calibration logic using collected data points
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo("CALIBRATION PHASE")
        rospy.loginfo("TODO: Implement coordinate system calibration")
        rospy.loginfo("=" * 60)
        
        # For now, mark calibration as complete
        # In the future, this will calculate the transformation matrix
        self.calibration_complete = True

    def run(self):
        """
        Main loop of the node.
        """
        rate = rospy.Rate(30)  # 30 Hz update rate

        self.start_time = rospy.Time.now()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("User Arm2Robot Node started")
        rospy.loginfo("Collecting VR controller and robot arm data...")
        rospy.loginfo("=" * 60)

        while not rospy.is_shutdown():
            # Get VR controller transforms
            left_vr_transform, right_vr_transform = self.get_controller_transforms()

            # Get robot end effector poses
            left_robot_pose, right_robot_pose = self.get_robot_end_effector_poses()

            # Publish robot end effector poses
            self.publish_robot_end_effector_poses(left_robot_pose, right_robot_pose)

            # Check if we need to perform calibration
            elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
            
            if not self.calibration_complete:
                if elapsed_time < self.calibration_duration:
                    # Still in calibration phase
                    remaining = self.calibration_duration - elapsed_time
                    rospy.loginfo_throttle(1.0, f"Calibration phase: {remaining:.1f} seconds remaining...")
                    
                    # TODO: Collect calibration data points here
                    # self.calibration_vr_points.append(...)
                    # self.calibration_robot_points.append(...)
                else:
                    # Calibration phase complete - calculate transformation
                    self.perform_calibration()
                    rospy.loginfo("Calibration complete! Starting pose transformation...")
            else:
                # Calibration complete - transform and publish controller poses
                if left_vr_transform or right_vr_transform:
                    self.publish_transformed_controller_poses(
                        left_vr_transform, right_vr_transform)

            # Wait for the next cycle
            rate.sleep()

    def shutdown(self):
        """
        Actions to perform on node shutdown.
        """
        rospy.loginfo("Shutting down User Arm2Robot Node...")
        
        # Stop any ongoing movements
        try:
            if self.left_arm_group:
                self.left_arm_group.stop()
                self.left_arm_group.clear_pose_targets()
            if self.right_arm_group:
                self.right_arm_group.stop()
                self.right_arm_group.clear_pose_targets()
            moveit_commander.roscpp_shutdown()
        except (AttributeError, rospy.ROSException) as e:
            rospy.logwarn(f"Error during shutdown: {e}")
            pass
        
        rospy.loginfo("User Arm2Robot Node shut down.")


def main():
    """
    Main function to create and run the ROS node.
    """
    node = None
    try:
        node = UserArm2RobotNode()
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
