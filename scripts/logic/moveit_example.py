#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import random

def move_left_arm_random():
    # Inicializar moveit_commander y el nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_left_arm_random', anonymous=True)

    # Crear un objeto MoveGroupCommander para el brazo izquierdo
    group_name = "left_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Establecer tolerancias para permitir soluciones aproximadas
    move_group.set_goal_position_tolerance(0.05)  # 5 cm de tolerancia en posición
    move_group.set_goal_orientation_tolerance(0.1)  # ~5.7 grados de tolerancia en orientación

    # Generar una configuración articular aleatoria válida
    move_group.set_random_target()

    # Planificar y ejecutar el movimiento
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo("Movimiento completado a una posición aleatoria válida.")

    # Cerrar la comunicación con MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_left_arm_random()
    except rospy.ROSInterruptException:
        pass
