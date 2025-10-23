#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import random

class NaoMoveIt(object):
    """NaoMoveIt"""
    def __init__(self):
        super(NaoMoveIt, self).__init__()

        # Initialize moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('nao_moveit_random_move', anonymous=True)

        # Instantiate a RobotCommander object.
        robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object.
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object for the "left_arm" group.
        group_name = "right_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        # Get basic information
        planning_frame = group.get_planning_frame()

        eef_link = group.get_end_effector_link()

        group_names = robot.get_group_names()

        # Store the objects for later use
        self.robot = robot
        self.scene = scene
        self.group = group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def move_to_random_and_back(self):
        group = self.group

        joint_names = group.get_active_joints()
        print("Active joints:",joint_names)

        for joint_name in joint_names:
            joint = self.robot.get_joint(joint_name)

            limits = joint.bounds()
            min_lim = joint.min_bound()
            max_lim = joint.max_bound()
            print(f"Joint {joint_name}: min={min_lim}, max={max_lim},bounds={limits}")

        starting_joint_values = group.get_current_joint_values()
        print("Starting joint values: ", starting_joint_values)
        # 1. Save the current joint values as the starting point
        #group.go([1.454, 0.204, -1.184, -0.405, 0.117], wait=True)
        #group.go([0.0, 0.0, 0.0, 0.7897633000000001, 0.0], wait=True)
        # [LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll,LWristYaw]
        # [- arriba, levantar,-rotar_izquierda, -cerrar, rotar_izquierda]
        # [(-119.5,119.5),(18,-76),(119.5,-119.5),(-88.5,-1.5),(104.5,-104.5)]
        #group.go([1.0, 0.0, -0.0, -0.2, 0.0], wait=True)
        #Left_arm T-Pose
        #group.go([0, 1.32, 0, -0.2, 0], wait=True)
        #RIght_arm T-Pose
        #group.go([0, -1.32, 0, 0.2, 0], wait=True)
        group.stop()
        rospy.sleep(3)
        starting_joint_values = group.get_current_joint_values()
        print("Starting joint values: ", starting_joint_values)
        
        # get_random_joint_values() will return a random set of joint values
        # that are within the joint limits
        #random_joint_goal = group.get_random_joint_values()
        #print("Random joint goal: ", random_joint_goal)
        
        # Plan and execute the motion
        #group.go(random_joint_goal, wait=True)

        # Calling stop() ensures that there is no residual movement
        #group.stop()

        #rospy.sleep(3) # Pause for a moment
        #current_joint_values = group.get_current_joint_values()
        #print("Current joint values after random move: ", current_joint_values)

        #print("going to back to starting joint values...",starting_joint_values)

        #for starting_joint_index in range(len(starting_joint_values)):
        #    starting_joint_values[starting_joint_index] = round(starting_joint_values[starting_joint_index], 2)
        #    print("rounded value:",starting_joint_values[starting_joint_index])

        # 3. Go back to the starting joint state
        #group.go(starting_joint_values, wait=True)
        #group.stop()

        #current_joint_values = group.get_current_joint_values()
        #print("Current joint values after moving back: ", current_joint_values)


def main():
    try:
        tutorial = NaoMoveIt()
        tutorial.move_to_random_and_back()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()