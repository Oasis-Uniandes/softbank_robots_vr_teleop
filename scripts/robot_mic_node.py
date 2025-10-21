#!/usr/bin/env python3

import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer
from audio_common_msgs.msg import AudioData
from robot_toolkit_msgs.srv import audio_tools_srv
from robot_toolkit_msgs.msg import audio_tools_msg


class RobotMicNode:
    """
    Node that translates robot microphone audio from AudioBuffer to AudioData format
    """
    
    def __init__(self):
        """
        Initialize the Robot Mic Node
        """
        # Initialize the node
        rospy.init_node('robot_mic_node', anonymous=True)
        
        # Audio parameters
        self.sample_rate = 16000  # Hz - default for robot mic
        self.channels = 1
          # Mono - we'll use single channel
        
        # Enable robot audio toolkit
        self.enable_robot_audio()
        
        # Publisher to /audio/audio topic (AudioData format for Unity)
        self.audio_publisher = rospy.Publisher(
            "/audio/robot_audio",
            AudioData,
            queue_size=10
        )
        
        # Subscriber to /mic topic (AudioBuffer format from robot)
        self.mic_subscriber = rospy.Subscriber(
            "/mic",
            AudioBuffer,
            self.mic_callback,
            queue_size=10
        )
        
        rospy.loginfo("Robot Mic Node initialized")
        rospy.loginfo("Subscribing to: /mic (AudioBuffer)")
        rospy.loginfo("Publishing to: /audio/robot_audio (AudioData)")
        rospy.loginfo(f"Configuration: Sample Rate={self.sample_rate}Hz, Channels={self.channels}")
    
    def enable_robot_audio(self):
        """
        Enable the robot's audio toolkit and microphone
        Similar to what's done in speech_utilities.py
        """
        try:
            rospy.loginfo("Waiting for /robot_toolkit/audio_tools_srv...")
            rospy.wait_for_service('/robot_toolkit/audio_tools_srv', timeout=5.0)
            self.audioToolsService = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
            
            # Configure custom audio parameters for microphone
            customSpeech = audio_tools_msg()
            customSpeech.command = "custom"
            customSpeech.frequency = 16000
            customSpeech.channels = 1
            self.audioToolsService(customSpeech)
            rospy.loginfo("Robot audio configured: 16000Hz, 3 channels")
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            raise
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for robot toolkit services: {e}")
            raise
    
    def mic_callback(self, msg):
        """
        Callback that receives audio from the robot's microphone
        Converts AudioBuffer format to AudioData format
        
        Args:
            msg (AudioBuffer): Audio message from robot microphone
                - msg.header: timestamp and frame info
                - msg.frequency: sampling frequency
                - msg.channelMap: channel configuration
                - msg.data: int16[] audio data (interleaved if multi-channel)
        """
        try:
            # Extract audio data from AudioBuffer
            # msg.data is already a list of int16 values
            audio_data = np.array(msg.data, dtype=np.int16)
            
            # Get the number of channels from the message
            num_channels = 1 
            
            # If multi-channel, extract first channel (front left) or mix to mono
            if num_channels > 1:
                # Check if data length is divisible by number of channels
                data_len = len(audio_data)
                samples_per_channel = data_len // num_channels
                
                # Truncate data to fit evenly into channels if needed
                if data_len % num_channels != 0:
                    truncate_len = samples_per_channel * num_channels
                    audio_data = audio_data[:truncate_len]
                    rospy.logwarn_once(f"Audio data length ({data_len}) not divisible by channels ({num_channels}), truncating to {truncate_len}")
                
                # Reshape to separate channels (data is interleaved)
                audio_data = audio_data.reshape(-1, num_channels)
                # Take first channel (front left) or average all channels for mono
                audio_data = audio_data[:, 0]  # Use first channel
                # Alternatively, to mix all channels: audio_data = audio_data.mean(axis=1).astype(np.int16)
            
            # Convert int16 numpy array to bytes
            audio_bytes = audio_data.tobytes()
            
            # Create AudioData message
            audio_msg = AudioData()
            audio_msg.data = list(audio_bytes)  # Convert bytes to list of uint8
            
            # Publish the audio data
            self.audio_publisher.publish(audio_msg)
            
            rospy.logdebug(f"Published audio: {len(audio_msg.data)} bytes, "
                          f"frequency={msg.frequency}Hz, channels={num_channels}")
            
        except Exception as e:
            rospy.logerr(f"Error processing microphone data: {e}")
    
    def run(self):
        """
        Main loop of the node
        """
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Robot Mic Node running...")
        
        while not rospy.is_shutdown():
            # Main loop - subscriber callbacks handle the work
            rate.sleep()
    
    def shutdown(self):
        """
        Cleanup when shutting down the node
        """
        rospy.loginfo("Shutting down Robot Mic Node...")
        rospy.loginfo("Robot Mic Node closed successfully")


def main():
    """
    Main function
    """
    try:
        # Create and run the node
        robot_mic = RobotMicNode()
        
        # Register shutdown callback
        rospy.on_shutdown(robot_mic.shutdown)
        
        # Run the main loop
        robot_mic.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in Robot Mic Node: {e}")


if __name__ == '__main__':
    main()
