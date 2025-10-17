#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from robot_toolkit_msgs.srv import go_to_posture_srv
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from logic.headset_pose2_rotation import HeadsetToRobotRotation

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
        
        # Parámetros del nodo
        self.max_linear_vel = 0.3
        self.max_angular_vel = 0.5
        self.head_movement_speed = 0.3  # Slow speed for head movement (30% of max speed)

        # Head tracking sensitivity (0.5 = half the movement, 2.0 = double the movement)
        self.head_sensitivity = 1.5  # Increase this to make the robot head move more with less headset movement

        # Initialize headset to robot rotation converter
        # Change "NAO" to "Pepper" if using Pepper robot
        self.rotation_converter = HeadsetToRobotRotation("NAO")
        
        # Suscriptor al tópico del joystick
        self.joy_subscriber = rospy.Subscriber("/quest/joystick",Joy, self.joy_callback, queue_size=1)

        # Suscriptor al tópico de posicion del headset
        self.headset_subscriber = rospy.Subscriber("/quest/pose/headset", PoseStamped, self.headset_callback, queue_size=1)

        # Publicador para comandos de velocidad
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Publicador para movimiento de cabeza
        self.head_publisher = rospy.Publisher("/joint_angles", JointAnglesWithSpeed, queue_size=1)

        # Go to posture proxy
        self.go_to_posture_srv = rospy.ServiceProxy("/pytoolkit/ALRobotPosture/go_to_posture_srv", go_to_posture_srv)
        self.go_to_posture_srv.wait_for_service()
        self.go_to_posture_srv("stand")

        rospy.loginfo(f"VR Teleop Node iniciado")
        rospy.loginfo(f"Escuchando joystick en: /quest/joystick")
        rospy.loginfo(f"Publicando comandos en: /cmd_vel")
        rospy.loginfo(f"Publicando movimientos de cabeza en: /joint_angles")

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
        
        # Crear mensaje
        head_msg = JointAnglesWithSpeed()
        head_msg.header.stamp = rospy.Time.now()
        head_msg.joint_names = ['HeadYaw', 'HeadPitch']
        head_msg.joint_angles = [yaw_radians, pitch_radians]
        head_msg.speed = self.head_movement_speed
        head_msg.relative = 0  # Absolute positioning
        
        # Publicar
        self.head_publisher.publish(head_msg)

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
            
            # Convert headset orientation to robot head angles
            head_yaw, head_pitch = self.rotation_converter.convert_headset_to_robot_angles(orientation)
            
            # Apply sensitivity scaling
            head_yaw *= self.head_sensitivity
            head_pitch *= self.head_sensitivity
            
            # Clamp to robot limits after scaling
            head_yaw = max(-119.5, min(119.5, head_yaw))
            head_pitch = max(-38.5, min(29.5, head_pitch))
            
            rospy.loginfo(f"Headset -> Robot Head: Yaw={head_yaw:.2f}°, Pitch={-head_pitch:.2f}°")
            
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