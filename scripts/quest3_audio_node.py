#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Joy
from logic.transcribir_audio import transcribe_spanish
from robot_toolkit_msgs.srv import audio_tools_srv
from robot_toolkit_msgs.msg import audio_tools_msg, speech_msg

class Quest3AudioNode:
    """
    Nodo de ROS para recibir y procesar audio del Meta Quest 3
    """
    
    def __init__(self):
        """
        Inicializa el nodo de audio del Quest 3
        """
        # Inicializar el nodo
        rospy.init_node('quest3_audio_node', anonymous=True)
        
        # Parámetros de audio
        self.sample_rate = 16000  # Hz
        self.audio_format = "wave"
        self.processing_method = 'frombuffer' # 'frombuffer' or 'array'
        
        # Enable robot TTS (text-to-speech)
        self.enable_robot_tts()
        
        # Publisher to /speech topic for robot to speak
        self.speech_publisher = rospy.Publisher(
            "/speech",
            speech_msg,
            queue_size=10
        )
        
        # Suscriptor al tópico de audio
        self.audio_subscriber = rospy.Subscriber(
            "/audio/unity_audio", 
            AudioData, 
            self.audio_callback, 
            queue_size=10
        )

        # Suscriptor al tópico del joystick para detectar presión del botón X
        self.joy_subscriber = rospy.Subscriber(
            "/quest/joystick",
            Joy,
            self.joy_callback,
            queue_size=1
        )

        self.audio_buffer = []
        self.listening = False
        self.prev_x_button_state = 0  # Para detectar flanco de subida del botón X
        
        rospy.loginfo(f"Quest3 Audio Node iniciado")
        rospy.loginfo(f"Escuchando audio en: /audio/unity_audio")
        rospy.loginfo(f"Escuchando joystick en: /quest/joystick")
        rospy.loginfo(f"Configuración: Sample Rate={self.sample_rate}Hz, Formato={self.audio_format}")
        rospy.loginfo(f"Presiona el botón X para iniciar/detener la grabación")
    
    def enable_robot_tts(self):
        """
        Enable the robot's text-to-speech (TTS) capability
        """
        try:
            rospy.loginfo("Waiting for /robot_toolkit/audio_tools_srv...")
            rospy.wait_for_service('/robot_toolkit/audio_tools_srv', timeout=5.0)
            audio_tools_service = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
            
            # Enable text-to-speech only
            enable_tts = audio_tools_msg()
            enable_tts.command = "enable_tts"
            audio_tools_service(enable_tts)
            rospy.loginfo("Robot TTS enabled successfully")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            raise
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for robot toolkit audio service: {e}")
            raise
    
    def audio_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan datos de audio
        
        Args:
            msg (AudioData): Mensaje con los datos de audio en formato wave
        """
        try:
            # Obtener los datos de audio
            if self.listening:
                audio_data = msg.data
                self.audio_buffer.extend(audio_data)

                rospy.logdebug(f"Audio recibido: {len(audio_data)} bytes")

        except Exception as e:
            rospy.logerr(f"Error procesando datos de audio: {e}")
    
    def joy_callback(self, msg):
        """
        Callback que se ejecuta cuando llegan datos del joystick
        
        Args:
            msg (Joy): Mensaje del joystick con axes y buttons
        """
        try:
            # Según quest_joystick_buttons.txt, el botón X está en buttons[0]
            x_button_state = msg.buttons[0] if len(msg.buttons) > 0 else 0
            
            # Detectar flanco de subida (botón presionado)
            if x_button_state == 1 and self.prev_x_button_state == 0:
                self.toggle_recording()
            
            self.prev_x_button_state = x_button_state
            
        except Exception as e:
            rospy.logerr(f"Error procesando datos del joystick: {e}")
    
    def toggle_recording(self):
        """
        Alterna entre iniciar y detener la grabación de audio
        """
        if not self.listening:
            # Iniciar grabación
            self.listening = True
            self.audio_buffer = []  # Limpiar el buffer
            rospy.loginfo("Grabación de audio iniciada")
        else:
            # Detener grabación y transcribir
            self.listening = False
            rospy.loginfo("Grabación de audio detenida")
            
            if len(self.audio_buffer) > 0:
                rospy.loginfo(f"Transcribiendo {len(self.audio_buffer)} bytes de audio...")
                self.transcribe_audio()
            else:
                rospy.logwarn("No se grabó audio para transcribir")
    
    def transcribe_audio(self):
        """
        Transcribe el audio almacenado en el buffer
        """
        try:
            # Llamar a la función de transcripción
            transcription = transcribe_spanish(self.audio_buffer, self.sample_rate, self.processing_method)
            
            if transcription and transcription != "None":
                rospy.loginfo(f"Transcripción: {transcription}")
                
                # Publish to /speech topic to make robot speak in Spanish
                speech_message = speech_msg()
                speech_message.text = transcription
                speech_message.language = "Spanish"
                speech_message.animated = False  # No animation
                
                self.speech_publisher.publish(speech_message)
                rospy.loginfo(f"Robot hablando: {transcription}")
            else:
                rospy.logwarn("No se pudo transcribir el audio")
            
            # Limpiar el buffer después de transcribir
            self.audio_buffer = []
            
        except Exception as e:
            rospy.logerr(f"Error transcribiendo audio: {e}")
    
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
        rospy.loginfo("Cerrando Quest3 Audio Node...")
        
        # Si hay grabación en curso, transcribir antes de cerrar
        if self.listening and len(self.audio_buffer) > 0:
            rospy.loginfo("Transcribiendo audio pendiente antes de cerrar...")
            self.transcribe_audio()
        
        rospy.loginfo("Quest3 Audio Node cerrado correctamente")


def main():
    """
    Función principal
    """
    try:
        # Crear y ejecutar el nodo
        quest3_audio = Quest3AudioNode()
        
        # Registrar función de limpieza
        rospy.on_shutdown(quest3_audio.shutdown)
        
        # Ejecutar el bucle principal
        quest3_audio.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido por el usuario")
    except Exception as e:
        rospy.logerr(f"Error en el nodo Quest3 Audio: {e}")


if __name__ == '__main__':
    main()