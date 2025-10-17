#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


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
        
        # Suscriptor al tópico del joystick
        self.joy_subscriber = rospy.Subscriber("/quest/joystick",Joy, self.joy_callback, queue_size=1)
        
        # Publicador para comandos de velocidad
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.loginfo(f"VR Teleop Node iniciado")
        rospy.loginfo(f"Escuchando joystick en: /quest/joystick")
        rospy.loginfo(f"Publicando comandos en: /cmd_vel")

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
            cmd_vel.angular.z = right_stick_horizontal * self.max_angular_vel

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