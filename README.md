# SoftBank Robots VR Teleoperation

This project is a collaboration between [SinfonIA Uniandes](https://github.com/SinfonIAUniandes) and [Oasis Uniandes](https://github.com/Oasis-Uniandes) to teleoperate SoftBank Robotics' robots, such as NAO and Pepper, using a Meta Quest 3 headset.

This ROS package receives data from a Unity application running on the Meta Quest 3 and translates it into movement commands for the robot.

## Prerequisites

### Hardware
*   A SoftBank Robotics NAO or Pepper robot.
*   A Meta Quest 3 headset.

### Software
1.  **Unity Application**: The [unity_ros_teleoperation](https://github.com/leggedrobotics/unity_ros_teleoperation) application must be built and installed on the Meta Quest 3. It needs to be configured to connect to the ROS master.
2.  **Robot Drivers**: The drivers for the robot, specifically the `robot_toolkit_node` developed by SinfonIA Uniandes, must be running.
3.  **MoveIt! Configuration**: The MoveIt! configurations for your specific robot (Pepper/NAO) must be running. A guide on how to install and run these can be found at [SinfonIAUniandes/manipulation_utilities](https://github.com/SinfonIAUniandes/manipulation_utilities).

## Workspace Setup

To run this project, you need to create a Catkin workspace and clone the necessary dependencies.

1.  Create and initialize a new Catkin workspace:
    ```bash
    mkdir -p ~/vr_teleop_ws/src
    cd ~/vr_teleop_ws/
    catkin_make
    ```

2.  Navigate to the `src` directory:
    ```bash
    cd src
    ```

3.  Clone the required ROS packages:
    ```bash
    git clone https://github.com/leggedrobotics/ROS-TCP-Endpoint.git
    git clone https://github.com/ros-drivers/audio_common.git
    git clone https://github.com/leggedrobotics/vr_haptic_msgs.git
    git clone https://github.com/ethz-asl/nvblox_ros1.git
    git clone https://github.com/Oasis-Uniandes/softbank_robots_vr_teleop.git
    ```

4.  Build the workspace:
    ```bash
    cd ~/vr_teleop_ws/
    catkin_make
    ```

5.  Source the workspace's setup file:
    ```bash
    source devel/setup.bash
    ```

## Running the Teleoperation Node

1.  Ensure all prerequisites are running (Robot drivers, MoveIt!, ROS Master).
2.  Start the Unity application on the Meta Quest 3 and ensure it connects to the ROS Master.
3.  In a new terminal, source your workspace and run the teleoperation node:
    ```bash
    source ~/vr_teleop_ws/devel/setup.bash
    rosrun softbank_robots_vr_teleop vr_teleop_node.py
    ```

## Configuration

To switch between **NAO** and **Pepper**, you need to edit the [`vr_teleop_node.py`](src/softbank_robots_vr_teleop/scripts/vr_teleop_node.py) file.

In the `__init__` method of the `VRTeleopNode` class, change the robot type passed to the `HeadsetToRobotRotation` constructor:

```python
        # Initialize headset to robot rotation converter
        # Change "NAO" to "Pepper" if using Pepper robot
        self.rotation_converter = HeadsetToRobotRotation("NAO")
```