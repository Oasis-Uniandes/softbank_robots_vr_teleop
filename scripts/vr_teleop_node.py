#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from robot_toolkit_msgs.srv import go_to_posture_srv
from robot_toolkit_msgs.msg import set_angles_msg, motion_tools_msg, navigation_tools_msg, vision_tools_msg
from robot_toolkit_msgs.srv import motion_tools_srv, navigation_tools_srv, vision_tools_srv
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from logic.headset_pose2_rotation import HeadsetToRobotRotation
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import tf
from logic.joystick_coordinates2_robot import calculate_user_orientation

class VRTeleopNode:
    """
    Nodo de ROS para teleoperar un robot usando datos de joystick/VR
    """
    
    def __init__(self):
        """
        Inicializa el nodo de teleoperación VR
        """
        # Inicializar el nodo
        rospy.init_node('vr_teleop_node', anonymous=True)
        
        # Get robot type from ROS parameter server, default to "NAO"
        self.robot_type = rospy.get_param('~robot_type', 'NAO').upper()
        
        # Parámetros del nodo
        self.max_linear_vel = 0.5
        self.max_angular_vel = 0.5
        self.head_movement_speed = 0.3  # Slow speed for head movement (30% of max speed)

        # Head tracking sensitivity (0.5 = half the movement, 2.0 = double the movement)
        self.head_sensitivity = 1.5  # Increase this to make the robot head move more with less headset movement

        # Initialize headset to robot rotation converter
        self.rotation_converter = HeadsetToRobotRotation(self.robot_type)

        # Store user body orientation (from imitation node)
        self.user_body_yaw = None  # User's body orientation in radians
        self.headset_yaw = None  # Absolute headset orientation in radians

        # Publicador para movimiento de cabeza - topic and message type depend on robot
        if self.robot_type == "NAO":
            # NAO uses /joint_angles topic with JointAnglesWithSpeed message
            self.joint_publisher = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size=1)
            self.head_sensitivity = 1.5  # Increase this to make the robot head move more with less headset movement
        else:  # PEPPER
            rospy.loginfo("Waiting for /robot_toolkit/navigation_tools_srv...")
            rospy.wait_for_service('/robot_toolkit/navigation_tools_srv', timeout=5.0)
            navigation_tools_service = rospy.ServiceProxy('/robot_toolkit/navigation_tools_srv', navigation_tools_srv)
            
            # Enable the robots cmd_vel topic
            enable_nav_msg = navigation_tools_msg()
            enable_nav_msg.command = "custom"
            enable_nav_msg.tf_enable = True
            enable_nav_msg.tf_frequency = 50.0
            enable_nav_msg.cmd_vel_enable = True
            enable_nav_msg.security_timer = 1
             
            navigation_tools_service(enable_nav_msg)
            rospy.loginfo("Robot command velocity topic enabled successfully")
            
            
            rospy.loginfo("Waiting for /robot_toolkit/vision_tools_srv...")
            rospy.wait_for_service('/robot_toolkit/vision_tools_srv', timeout=5.0)
            vision_tools_service = rospy.ServiceProxy('/robot_toolkit/vision_tools_srv', vision_tools_srv)
            
            activate_front_camera_msg = vision_tools_msg()
            activate_front_camera_msg.camera_name = "front_camera"
            activate_front_camera_msg.command = "enable"
            
            vision_tools_service(activate_front_camera_msg)
            
            activate_bottom_camera_msg = vision_tools_msg()
            activate_bottom_camera_msg.camera_name = "bottom_camera"
            activate_bottom_camera_msg.command = "enable"
            
            vision_tools_service(activate_bottom_camera_msg)
            
            
            rospy.loginfo("Waiting for /robot_toolkit/motion_tools_srv...")
            rospy.wait_for_service('/robot_toolkit/motion_tools_srv', timeout=5.0)
            motion_tools_service = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            
            enable_tf_msg = motion_tools_msg()
            enable_tf_msg.command = "enable_all"
            
            motion_tools_service(enable_tf_msg)
            rospy.loginfo("Robot TF tree enabled successfully")
            
            # Pepper uses /set_angles topic with set_angles_msg message
            self.joint_publisher = rospy.Publisher("/set_angles", set_angles_msg, queue_size=1)
            self.head_movement_speed = 0.1  # Slow speed for head movement (30% of max speed)
            self.head_sensitivity = 1  # Increase this to make the robot head move more with less headset movement

        # Suscriptor al tópico del joystick
        self.joy_subscriber = rospy.Subscriber("/quest/joystick",Joy, self.joy_callback, queue_size=1)

        # Suscriptor al tópico de posicion del headset
        self.headset_subscriber = None

        # Create a transform listener to get data from /tf
        self.tf_listener = tf.TransformListener()

        # Publisher for user orientation
        self.orientation_publisher = rospy.Publisher('/user_orientation', PoseStamped, queue_size=1)

        # Parameters for rotation control
        self.max_angular_vel = 0.5  # Maximum angular velocity (rad/s)
        self.angle_tolerance = 0.1  # Tolerance in radians (~5.7 degrees)
        self.kp_angular = 1  # Proportional gain for angular velocity control

        # Store user orientation
        self.user_yaw = 0.0
        self.user_pose = None
        
        # Coordinate system alignment
        self.orientation_offset = 0.0  # Offset between robot and VR coordinate systems
        self.alignment_complete = False
        self.alignment_duration = 5.0  # seconds to wait for user alignment
        self.start_time = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.is_moving = False

        # Initialize calibration flag and other attributes
        self.calibrated = False

        rospy.loginfo("Listening for transforms on /tf topic...")

        # Publicador para comandos de velocidad
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # Go to posture proxy
        self.go_to_posture_srv = rospy.ServiceProxy("/pytoolkit/ALRobotPosture/go_to_posture_srv", go_to_posture_srv)
        self.go_to_posture_srv.wait_for_service()
        self.go_to_posture_srv("stand")

        rospy.loginfo(f"VR Teleop Node iniciado")
        rospy.loginfo(f"Escuchando joystick en: /quest/joystick")
        rospy.loginfo(f"Escuchando orientación del usuario en: /user_orientation")
        rospy.loginfo(f"Publicando comandos en: /cmd_vel")
        if self.robot_type == "NAO":
            rospy.loginfo(f"Publicando movimientos de cabeza en: /joint_angles (JointAnglesWithSpeed)")
        else:
            rospy.loginfo(f"Publicando movimientos de cabeza en: /set_angles (set_angles_msg)")
                           

    def move_head(self, yaw_degrees, pitch_degrees):
        """
        Mueve la cabeza del robot a los ángulos especificados
        
        Args:
            yaw_degrees (float): Ángulo de yaw en grados
            pitch_degrees (float): Ángulo de pitch en grados
        """
        # Convertir de grados a radianes
        yaw_radians = yaw_degrees * 3.14159265359 / 180.0
        pitch_radians = pitch_degrees * 3.14159265359 / 180.0
        
        if self.robot_type == "NAO":
            # NAO uses JointAnglesWithSpeed message
            head_msg = JointAnglesWithSpeed()
            head_msg.header.stamp = rospy.Time.now()
            head_msg.joint_names = ['HeadYaw', 'HeadPitch']
            head_msg.joint_angles = [yaw_radians, pitch_radians]
            head_msg.speed = self.head_movement_speed
            head_msg.relative = 0  # Absolute positioning
        else:  # PEPPER
            # Pepper uses set_angles_msg message
            head_msg = set_angles_msg()
            head_msg.names = ['HeadYaw', 'HeadPitch']
            head_msg.angles = [yaw_radians, pitch_radians]
            head_msg.fraction_max_speed = [self.head_movement_speed, self.head_movement_speed]
        
        # Publicar
        self.joint_publisher.publish(head_msg)

    def move_hands(self, left_hand_value, right_hand_value):
        """
        Controls the opening/closing of NAO's hands based on trigger values.
        
        Args:
            left_hand_value (float): Left hand position from 0 (closed) to 1 (open)
            right_hand_value (float): Right hand position from 0 (closed) to 1 (open)
        """
        # Create message for hand control
        hand_msg = JointAnglesWithSpeed()
        hand_msg.header.stamp = rospy.Time.now()
        hand_msg.joint_names = ['LHand', 'RHand']
        hand_msg.joint_angles = [1-left_hand_value, 1-right_hand_value]
        hand_msg.speed = 0.3
        hand_msg.relative = 0  # Absolute positioning
        
        # Publish hand command
        self.joint_publisher.publish(hand_msg)
        
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
        
        self.user_pose = pose_stamped

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

        # If using nao, rotating to user doesnt work properly
        if self.robot_type == 'NAO':
            cmd_vel = Twist()
        
            cmd_vel.linear = self.linear_velocity
            cmd_vel.angular = self.angular_velocity
            self.cmd_vel_publisher.publish(cmd_vel)
            return


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
        cmd_vel.linear = self.linear_velocity
        if self.is_moving:
            # Don't rotate while moving linearly
            cmd_vel.angular.z = 0.0
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
        
    def update_user_body_orientation(self):
        """
        Updates the user's body orientation from self.user_pose
        This is needed for relative head tracking
        """
        try:
            if self.user_pose is None:
                return
                
            # Extract yaw from user body orientation quaternion
            orientation = self.user_pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler = euler_from_quaternion(quaternion)
            self.user_body_yaw = euler[2]  # Yaw is rotation around Z-axis
            
            rospy.loginfo_throttle(2.0, f"User body orientation updated: {np.degrees(self.user_body_yaw):.2f}°")
            
        except Exception as e:
            rospy.logerr(f"Error procesando orientación del usuario: {e}")

    def headset_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan datos de la posición del headset
        
        Args:
            msg (PoseStamped): Mensaje con la posición y orientación del headset
        """
        # Procesar los datos del headset si es necesario
        try:
            pose_msg = msg.pose
            orientation = pose_msg.orientation
            
            # Extract absolute headset yaw
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            euler = euler_from_quaternion(quaternion)
            self.headset_yaw = euler[2]  # Yaw is rotation around Z-axis
            
            # Convert headset orientation to robot head angles
            head_yaw, head_pitch = self.rotation_converter.convert_headset_to_robot_angles(orientation)
            
            # If we have user body orientation, calculate relative head yaw
            if self.user_body_yaw is not None:
                # Calculate relative yaw (where the user is looking relative to their body)
                relative_yaw = self.headset_yaw - self.user_body_yaw
                
                # Normalize angle to [-pi, pi]
                while relative_yaw > np.pi:
                    relative_yaw -= 2 * np.pi
                while relative_yaw < -np.pi:
                    relative_yaw += 2 * np.pi
                
                # Convert to degrees and use as head yaw
                head_yaw = np.degrees(relative_yaw)
                
                rospy.loginfo_throttle(1.0, f"Relative head yaw: {head_yaw:.2f}° (User body: {np.degrees(self.user_body_yaw):.2f}°, Headset: {np.degrees(self.headset_yaw):.2f}°)")
            
            # Apply sensitivity scaling
            head_yaw *= self.head_sensitivity
            head_pitch *= self.head_sensitivity
            
            # Clamp to robot limits after scaling (use robot-specific limits)
            limits = self.rotation_converter.get_robot_limits()
            head_yaw = max(limits['yaw_limits'][0], min(limits['yaw_limits'][1], head_yaw))
            head_pitch = max(limits['pitch_limits'][0], min(limits['pitch_limits'][1], head_pitch))
            
            rospy.loginfo_throttle(1.0, f"Headset -> Robot Head: Yaw={head_yaw:.2f}°, Pitch={-head_pitch:.2f}°")
            
            # Move robot head
            self.move_head(head_yaw, -head_pitch)
            
        except Exception as e:
            rospy.logerr(f"Error procesando datos del headset: {e}")

    def joy_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan datos del joystick
        
        Args:
            msg (Joy): Mensaje del joystick con axes y buttons
        """
        # Procesar los datos del joystick
        try:
            # Crear mensaje de comando de velocidad
            cmd_vel = Twist()
            
            print(msg.axes)

            # Mapear axes del joystick a velocidades

            left_stick_horizontal = msg.axes[2]   # Eje horizontal del stick izquierdo
            left_stick_vertical = msg.axes[3]     # Eje vertical del stick izquierdo

            cmd_vel.linear.x = left_stick_vertical * self.max_linear_vel
            cmd_vel.linear.y = -left_stick_horizontal * self.max_linear_vel
            
            self.is_moving = True

            if abs(cmd_vel.linear.x) < 0.1 and abs(cmd_vel.linear.y) < 0.1:
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                self.is_moving = False

            self.linear_velocity = cmd_vel.linear

            if self.robot_type == 'NAO':
                # NAO can rotate with the right stick
                right_stick_horizontal = msg.axes[0]  # Eje horizontal del stick derecho
                #right_stick_vertical = msg.axes[1]    # Eje vertical del stick derecho

                cmd_vel.angular.z = -right_stick_horizontal * self.max_angular_vel
                self.angular_velocity = cmd_vel.angular
                
            # The second-to-last axis is the left trigger
            # The last axis is the right trigger
            if len(msg.axes) >= 2:
                left_trigger = msg.axes[-2]   # Left lower trigger
                right_trigger = msg.axes[-1]  # Right lower trigger
                
                # Triggers range from 0 to 1, which matches the hand joint range
                self.move_hands(left_trigger, right_trigger)
                
                rospy.loginfo_throttle(2.0, f"Hand control: L={left_trigger:.2f}, R={right_trigger:.2f}")
            
        except Exception as e:
            rospy.logerr(f"Error procesando datos del joystick: {e}")
    
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
                
                # Update user body orientation for head tracking
                self.update_user_body_orientation()
                
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

                            # Suscriptor al tópico de posicion del headset
                            self.headset_subscriber = rospy.Subscriber("/quest/pose/headset", PoseStamped, self.headset_callback, queue_size=1)
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
        Limpieza al cerrar el nodo
        """
        rospy.loginfo("Cerrando VR Teleop Node...")
        
        # Enviar comando de parada
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

        self.go_to_posture_srv("stand")
        
        rospy.loginfo("VR Teleop Node cerrado correctamente")


def main():
    """
    Función principal
    """
    try:
        # Crear y ejecutar el nodo
        vr_teleop = VRTeleopNode()
        
        # Registrar función de limpieza
        rospy.on_shutdown(vr_teleop.shutdown)
        
        # Ejecutar el bucle principal
        vr_teleop.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"Error en el nodo VR Teleop: {e}")


if __name__ == '__main__':
    main()