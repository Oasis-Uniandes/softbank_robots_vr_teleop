#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from robot_toolkit_msgs.srv import go_to_posture_srv
from robot_toolkit_msgs.msg import set_angles_msg, motion_tools_msg, navigation_tools_msg, vision_tools_msg
from robot_toolkit_msgs.srv import motion_tools_srv, navigation_tools_srv, vision_tools_srv
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from logic.headset_pose2_rotation import HeadsetToRobotRotation
from tf.transformations import euler_from_quaternion
import numpy as np

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
            self.head_publisher = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size=1)
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
            self.head_publisher = rospy.Publisher("/set_angles", set_angles_msg, queue_size=1)
            self.head_movement_speed = 0.1  # Slow speed for head movement (30% of max speed)
            self.head_sensitivity = 1  # Increase this to make the robot head move more with less headset movement

        # Suscriptor al tópico del joystick
        self.joy_subscriber = rospy.Subscriber("/quest/joystick",Joy, self.joy_callback, queue_size=1)

        # Suscriptor al tópico de posicion del headset
        self.headset_subscriber = rospy.Subscriber("/quest/pose/headset", PoseStamped, self.headset_callback, queue_size=1)

        # Suscriptor al tópico de orientación del usuario (body orientation from imitation node)
        self.user_orientation_subscriber = rospy.Subscriber("/user_orientation", PoseStamped, self.user_orientation_callback, queue_size=1)

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
        self.head_publisher.publish(head_msg)

    def user_orientation_callback(self, msg):
        """
        Callback que recibe la orientación del cuerpo del usuario desde el nodo de imitación
        
        Args:
            msg (PoseStamped): Mensaje con la orientación del cuerpo del usuario
        """
        try:
            # Extract yaw from user body orientation quaternion
            orientation = msg.pose.orientation
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
            
            right_stick_horizontal = msg.axes[0]  # Eje horizontal del stick derecho
            right_stick_vertical = msg.axes[1]    # Eje vertical del stick derecho

            left_stick_horizontal = msg.axes[2]   # Eje horizontal del stick izquierdo
            left_stick_vertical = msg.axes[3]     # Eje vertical del stick izquierdo

            cmd_vel.linear.x = left_stick_vertical * self.max_linear_vel
            cmd_vel.linear.y = -left_stick_horizontal * self.max_linear_vel
            cmd_vel.angular.z = -right_stick_horizontal * self.max_angular_vel

            self.cmd_vel_publisher.publish(cmd_vel)
            print(f"Comando enviado - Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}")
            
        except Exception as e:
            rospy.logerr(f"Error procesando datos del joystick: {e}")
    
    def run(self):
        """
        Bucle principal del nodo
        """
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Aquí puedes añadir lógica adicional que se ejecute periódicamente
            rate.sleep()
    
    def shutdown(self):
        """
        Limpieza al cerrar el nodo
        """
        rospy.loginfo("Cerrando VR Teleop Node...")
        
        # Enviar comando de parada
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)
        
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