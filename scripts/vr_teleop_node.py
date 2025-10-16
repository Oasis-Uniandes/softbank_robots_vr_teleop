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
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 1.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.0)
        
        # Variables de estado
        self.current_joy_msg = None
        
        # Suscriptor al tópico del joystick
        self.joy_subscriber = rospy.Subscriber(
            self.joy_topic, 
            Joy, 
            self.joy_callback, 
            queue_size=1
        )
        
        # Publicador para comandos de velocidad
        self.cmd_vel_publisher = rospy.Publisher(
            self.cmd_vel_topic, 
            Twist, 
            queue_size=1
        )
        
        rospy.loginfo(f"VR Teleop Node iniciado")
        rospy.loginfo(f"Escuchando joystick en: {self.joy_topic}")
        rospy.loginfo(f"Publicando comandos en: {self.cmd_vel_topic}")
    
    def joy_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan datos del joystick
        
        Args:
            msg (Joy): Mensaje del joystick con axes y buttons
        """
        self.current_joy_msg = msg
        
        # Procesar los datos del joystick
        self.process_joystick_data(msg)
    
    def process_joystick_data(self, joy_msg):
        """
        Procesa los datos del joystick y genera comandos de movimiento
        
        Args:
            joy_msg (Joy): Mensaje del joystick
        """
        try:
            # Crear mensaje de comando de velocidad
            cmd_vel = Twist()
            
            # Verificar que hay suficientes axes
            if len(joy_msg.axes) >= 2:
                rospy.logdebug(f"Axes recibidos: {joy_msg.axes}")
                rospy.logdebug(f"Buttons recibidos: {joy_msg.buttons}")
                # Mapear axes del joystick a velocidades
                # Típicamente: axes[0] = lateral, axes[1] = adelante/atrás
                #linear_x = joy_msg.axes[1] * self.max_linear_vel
                #angular_z = joy_msg.axes[0] * self.max_angular_vel
                
                #cmd_vel.linear.x = linear_x
                #cmd_vel.angular.z = angular_z
                
                #self.cmd_vel_publisher.publish(cmd_vel)
                #rospy.logdebug(f"Comando enviado - Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}")
            
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