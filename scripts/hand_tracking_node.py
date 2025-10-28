#!/usr/bin/env python3
"""
Hand Tracking Node for NAO Robot VR Teleoperation
Tracks VR hand controllers and maps them to robot arm movements using IK.
No MoveIt - uses ikpy for direct inverse kinematics control.
"""

import rospy
import tf
import numpy as np
import rospkg
import os
from geometry_msgs.msg import PoseStamped
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from ikpy.chain import Chain
from tf.transformations import quaternion_from_matrix
from robot_toolkit_msgs.srv import go_to_posture_srv


class HandTrackingNode:
    """
    ROS node to track VR hand controllers and control NAO robot arms via IK.
    """

    # NAO calibration pose joint angles (arms stretched horizontally)
    CALIBRATION_POSE_LEFT = {
        'joint_names': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
        'joint_angles': [0.0, 1.5708, 0.0, 0.0]  # 90 degrees shoulder roll
    }
    
    CALIBRATION_POSE_RIGHT = {
        'joint_names': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
        'joint_angles': [0.0, -1.5708, 0.0, 0.0]  # -90 degrees shoulder roll
    }

    def __init__(self):
        """Initialize the hand tracking node."""
        # Initialize ROS node
        rospy.init_node('hand_tracking_node', anonymous=True)
        
        # Get an instance of RosPack
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('softbank_robots_vr_teleop')

        # Construct file paths
        default_left_chain = os.path.join(package_path, 'ik_data', 'nao_left_arm.json')
        default_right_chain = os.path.join(package_path, 'ik_data', 'nao_right_arm.json')

        # Get parameters
        self.left_chain_file = rospy.get_param('~left_chain_file', default_left_chain)
        self.right_chain_file = rospy.get_param('~right_chain_file', default_right_chain)
        self.control_rate = rospy.get_param('~control_rate', 30)  # Hz - high rate for low latency
        self.calibration_duration = rospy.get_param('~calibration_duration', 5.0)  # seconds
        self.joint_speed = rospy.get_param('~joint_speed', 0.1)  # 30% of max speed
        
        # State machine
        self.state = 'INIT'  # INIT -> STANDING -> CALIBRATION_POSE -> CALIBRATING -> TRACKING
        self.calibration_start_time = None
        
        # Calibration data
        self.calibrated = False
        self.calibration_samples = []
        self.left_hand_offset = np.zeros(3)
        self.right_hand_offset = np.zeros(3)
        self.headset_position_offset = np.zeros(3)
        self.left_robot_calibration_pos = None  # Will be computed from FK
        self.right_robot_calibration_pos = None  # Will be computed from FK
        
        # Transform listener
        self.tf_listener = tf.TransformListener()
        
        # Publishers
        self.joint_angles_pub = rospy.Publisher(
            '/joint_angles',
            JointAnglesWithSpeed,
            queue_size=1
        )
        
        self.left_hand_viz_pub = rospy.Publisher(
            '/left_hand_target_pose',
            PoseStamped,
            queue_size=1
        )
        
        self.right_hand_viz_pub = rospy.Publisher(
            '/right_hand_target_pose',
            PoseStamped,
            queue_size=1
        )
        
        self.left_end_effector_pub = rospy.Publisher(
            '/left_arm_end_effector_pose',
            PoseStamped,
            queue_size=1
        )
        
        self.right_end_effector_pub = rospy.Publisher(
            '/right_arm_end_effector_pose',
            PoseStamped,
            queue_size=1
        )
        
        # Load IKPy chains
        self.left_arm_chain = None
        self.right_arm_chain = None
        self.load_ik_chains()

        self.go_to_posture_srv = rospy.ServiceProxy("/pytoolkit/ALRobotPosture/go_to_posture_srv", go_to_posture_srv)
        self.go_to_posture_srv.wait_for_service()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Hand Tracking Node initialized")
        rospy.loginfo(f"Control rate: {self.control_rate} Hz")
        rospy.loginfo("=" * 60)

    def load_ik_chains(self):
        """Load IKPy kinematic chains for left and right arms."""
        try:
            self.left_arm_chain = Chain.from_json_file(self.left_chain_file)
            rospy.loginfo(f"Loaded left arm chain: {self.left_chain_file}")
            rospy.loginfo(f"  Links: {len(self.left_arm_chain.links)}")
        except Exception as e:
            rospy.logerr(f"Failed to load left arm chain: {e}")
            raise
        
        try:
            self.right_arm_chain = Chain.from_json_file(self.right_chain_file)
            rospy.loginfo(f"Loaded right arm chain: {self.right_chain_file}")
            rospy.loginfo(f"  Links: {len(self.right_arm_chain.links)}")
        except Exception as e:
            rospy.logerr(f"Failed to load right arm chain: {e}")
            raise

    def move_to_calibration_pose(self):
        """Move arms to calibration pose (stretched horizontally)."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Moving to CALIBRATION POSE")
        rospy.loginfo("Please stretch your arms horizontally (T-pose)")
        rospy.loginfo("=" * 60)
        
        # Move left arm
        left_msg = JointAnglesWithSpeed()
        left_msg.header.stamp = rospy.Time.now()
        left_msg.joint_names = self.CALIBRATION_POSE_LEFT['joint_names']
        left_msg.joint_angles = self.CALIBRATION_POSE_LEFT['joint_angles']
        left_msg.speed = 0.1  # Slow speed for safety
        left_msg.relative = 0
        self.joint_angles_pub.publish(left_msg)
        
        rospy.sleep(0.1)  # Small delay between commands
        
        # Move right arm
        right_msg = JointAnglesWithSpeed()
        right_msg.header.stamp = rospy.Time.now()
        right_msg.joint_names = self.CALIBRATION_POSE_RIGHT['joint_names']
        right_msg.joint_angles = self.CALIBRATION_POSE_RIGHT['joint_angles']
        right_msg.speed = 0.1
        right_msg.relative = 0
        self.joint_angles_pub.publish(right_msg)
        
        rospy.loginfo("Calibration pose commands sent")
        rospy.sleep(7.0)  # Wait for arms to reach position

    def get_controller_transforms(self):
        """
        Get transforms for VR controllers and headset.
        Returns: (left_trans, left_rot), (right_trans, right_rot), (headset_trans, headset_rot)
        """
        try:
            # Get left hand
            (left_trans, left_rot) = self.tf_listener.lookupTransform(
                '/vr_origin', '/hand_left', rospy.Time(0)
            )
            
            # Get right hand
            (right_trans, right_rot) = self.tf_listener.lookupTransform(
                '/vr_origin', '/hand_right', rospy.Time(0)
            )
            
            # Get headset
            (headset_trans, headset_rot) = self.tf_listener.lookupTransform(
                '/vr_origin', '/headset', rospy.Time(0)
            )
            
            return (left_trans, left_rot), (right_trans, right_rot), (headset_trans, headset_rot)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Could not get controller transforms: {e}")
            return None, None, None

    def calibrate_hand_tracking(self):
        """
        Calibrate hand tracking by collecting samples while user holds T-pose.
        Transforms VR hand coordinates relative to headset, then to robot frame.
        """
        # Get current transforms
        left_tf, right_tf, headset_tf = self.get_controller_transforms()
        
        if left_tf is None or right_tf is None or headset_tf is None:
            return False
        
        # Store samples
        sample = {
            'left_hand': np.array(left_tf[0]),
            'right_hand': np.array(right_tf[0]),
            'headset': np.array(headset_tf[0])
        }
        self.calibration_samples.append(sample)
        
        # Check if calibration period is complete
        elapsed = (rospy.Time.now() - self.calibration_start_time).to_sec()
        remaining = self.calibration_duration - elapsed
        
        if remaining > 0:
            rospy.loginfo_throttle(1.0, f"Calibrating... {remaining:.1f}s remaining")
            return False
        
        # Calibration complete - compute offsets
        rospy.loginfo("=" * 60)
        rospy.loginfo("Computing calibration offsets...")
        
        # Average all samples
        n_samples = len(self.calibration_samples)
        avg_left = np.zeros(3)
        avg_right = np.zeros(3)
        avg_headset = np.zeros(3)
        
        for sample in self.calibration_samples:
            avg_left += sample['left_hand']
            avg_right += sample['right_hand']
            avg_headset += sample['headset']
        
        avg_left /= n_samples
        avg_right /= n_samples
        avg_headset /= n_samples
        
        # Calculate offsets: VR coords relative to headset
        self.left_hand_offset = avg_left - avg_headset
        self.right_hand_offset = avg_right - avg_headset
        self.headset_position_offset = avg_headset
        
        # Calculate actual NAO hand positions in T-pose using forward kinematics
        # Left arm calibration pose
        left_calibration_joints = np.zeros(len(self.left_arm_chain.links))
        left_joint_idx = 0
        for i, link in enumerate(self.left_arm_chain.links):
            if link.has_rotation and link.name in self.CALIBRATION_POSE_LEFT['joint_names']:
                joint_name_idx = self.CALIBRATION_POSE_LEFT['joint_names'].index(link.name)
                left_calibration_joints[i] = self.CALIBRATION_POSE_LEFT['joint_angles'][joint_name_idx]
        
        left_end_effector_frame = self.left_arm_chain.forward_kinematics(left_calibration_joints)
        self.left_robot_calibration_pos = left_end_effector_frame[:3, 3]
        
        # Right arm calibration pose
        right_calibration_joints = np.zeros(len(self.right_arm_chain.links))
        right_joint_idx = 0
        for i, link in enumerate(self.right_arm_chain.links):
            if link.has_rotation and link.name in self.CALIBRATION_POSE_RIGHT['joint_names']:
                joint_name_idx = self.CALIBRATION_POSE_RIGHT['joint_names'].index(link.name)
                right_calibration_joints[i] = self.CALIBRATION_POSE_RIGHT['joint_angles'][joint_name_idx]
        
        right_end_effector_frame = self.right_arm_chain.forward_kinematics(right_calibration_joints)
        self.right_robot_calibration_pos = right_end_effector_frame[:3, 3]
        
        rospy.loginfo(f"Calibration complete! Collected {n_samples} samples")
        rospy.loginfo(f"Left hand offset (VR): {self.left_hand_offset}")
        rospy.loginfo(f"Right hand offset (VR): {self.right_hand_offset}")
        rospy.loginfo(f"Left robot T-pose position: {self.left_robot_calibration_pos}")
        rospy.loginfo(f"Right robot T-pose position: {self.right_robot_calibration_pos}")
        rospy.loginfo("=" * 60)
        
        self.calibrated = True
        return True

    def transform_hand_to_robot_frame(self, hand_pos, is_left=True):
        """
        Transform hand position from VR frame to robot base_link frame.
        
        Args:
            hand_pos: Hand position in VR frame [x, y, z]
            is_left: True for left hand, False for right hand
        
        Returns:
            Position in robot frame [x, y, z]
        """
        # Get current headset position
        _, _, headset_tf = self.get_controller_transforms()
        if headset_tf is None:
            return None
        
        headset_pos = np.array(headset_tf[0])
        
        # Get hand position relative to headset
        hand_relative = np.array(hand_pos) - headset_pos
        
        # Get calibration offset for this hand
        offset = self.left_hand_offset if is_left else self.right_hand_offset
        
        # Compute delta from calibration
        delta = hand_relative - offset
        
        # Get robot calibration position (calculated from forward kinematics)
        robot_calibration_pos = self.left_robot_calibration_pos if is_left else self.right_robot_calibration_pos
        
        # Apply delta (with potential scaling/remapping)
        # VR Y-axis typically maps to robot Y-axis (left-right)
        # VR Z-axis typically maps to robot Z-axis (up-down)
        # VR X-axis typically maps to robot X-axis (forward-back)
        
        # Scale factor to make movements more reasonable for NAO's workspace
        scale_factor = 0.5
        
        robot_pos = robot_calibration_pos + delta * scale_factor
        
        # Clamp to NAO's workspace
        robot_pos[0] = np.clip(robot_pos[0], -0.1, 0.25)  # X: forward/back
        robot_pos[1] = np.clip(robot_pos[1], -0.25 if not is_left else 0.0, 
                                             0.25 if is_left else 0.0)  # Y: left/right
        robot_pos[2] = np.clip(robot_pos[2], -0.05, 0.3)  # Z: up/down
        
        return robot_pos

    def compute_and_publish_ik(self, target_pos, chain, joint_names, is_left=True):
        """
        Compute IK for target position and publish joint commands.
        
        Args:
            target_pos: Target position [x, y, z] in robot frame
            chain: IKPy chain object
            joint_names: List of joint names for this arm
            is_left: True for left arm, False for right arm
        """
        # Create target frame
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_pos
        
        # Compute inverse kinematics
        try:
            ik_solution = chain.inverse_kinematics_frame(
                target_frame,
                optimizer="scalar"
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0, f"IK computation failed: {e}")
            return
        
        # Extract joint angles from IK solution (skip fixed joints)
        joint_msg = JointAnglesWithSpeed()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.joint_names = joint_names
        joint_msg.joint_angles = []
        joint_msg.speed = self.joint_speed
        joint_msg.relative = 0
        
        for link, angle in zip(chain.links, ik_solution):
            if link.has_rotation and link.name in joint_names:
                joint_msg.joint_angles.append(float(angle))
        
        # Publish joint command
        self.joint_angles_pub.publish(joint_msg)
        
        # Compute and publish end effector pose for visualization
        end_effector_frame = chain.forward_kinematics(ik_solution)
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = end_effector_frame[0, 3]
        pose_msg.pose.position.y = end_effector_frame[1, 3]
        pose_msg.pose.position.z = end_effector_frame[2, 3]
        
        quaternion = quaternion_from_matrix(end_effector_frame)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        if is_left:
            self.left_end_effector_pub.publish(pose_msg)
        else:
            self.right_end_effector_pub.publish(pose_msg)

    def publish_hand_visualization(self, target_pos, is_left=True):
        """Publish target hand position for RViz visualization."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = target_pos[0]
        pose_msg.pose.position.y = target_pos[1]
        pose_msg.pose.position.z = target_pos[2]
        pose_msg.pose.orientation.w = 1.0
        
        if is_left:
            self.left_hand_viz_pub.publish(pose_msg)
        else:
            self.right_hand_viz_pub.publish(pose_msg)

    def run(self):
        """Main control loop."""
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            if self.state == 'INIT':
                # Go to stand posture
                # self.go_to_posture_srv("stand")
                self.state = 'STANDING'
            
            elif self.state == 'STANDING':
                # Move to calibration pose
                self.move_to_calibration_pose()
                self.state = 'CALIBRATION_POSE'
            
            elif self.state == 'CALIBRATION_POSE':
                # Start calibration
                rospy.loginfo("=" * 60)
                rospy.loginfo("CALIBRATION STARTED")
                rospy.loginfo(f"Hold T-pose for {self.calibration_duration} seconds")
                rospy.loginfo("=" * 60)
                self.calibration_start_time = rospy.Time.now()
                self.calibration_samples = []
                self.state = 'CALIBRATING'
            
            elif self.state == 'CALIBRATING':
                # Collect calibration samples
                if self.calibrate_hand_tracking():
                    self.state = 'TRACKING'
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("TRACKING MODE STARTED")
                    rospy.loginfo("Move your hands - robot will follow!")
                    rospy.loginfo("=" * 60)
            
            elif self.state == 'TRACKING':
                # Track and control robot arms
                left_tf, right_tf, headset_tf = self.get_controller_transforms()
                
                if left_tf and right_tf and headset_tf:
                    # Transform left hand
                    left_target = self.transform_hand_to_robot_frame(left_tf[0], is_left=True)
                    if left_target is not None:
                        self.publish_hand_visualization(left_target, is_left=True)
                        self.compute_and_publish_ik(
                            left_target,
                            self.left_arm_chain,
                            self.CALIBRATION_POSE_LEFT['joint_names'],
                            is_left=True
                        )
                    
                    # Transform right hand
                    right_target = self.transform_hand_to_robot_frame(right_tf[0], is_left=False)
                    if right_target is not None:
                        self.publish_hand_visualization(right_target, is_left=False)
                        self.compute_and_publish_ik(
                            right_target,
                            self.right_arm_chain,
                            self.CALIBRATION_POSE_RIGHT['joint_names'],
                            is_left=False
                        )
            
            rate.sleep()

    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Shutting down Hand Tracking Node...")
        rospy.loginfo("=" * 60)
        
        # Stop arms by sending current position commands with zero speed
        # Or simply let them stay where they are
        rospy.loginfo("Shutdown complete")


def main():
    """Main entry point."""
    node = None
    try:
        node = HandTrackingNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()


if __name__ == '__main__':
    main()
