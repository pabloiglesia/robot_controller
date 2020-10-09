#!/usr/bin/env python
# coding: utf-8

"""
- We need to establish a connection to the robot with the following comand:
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml

- Then, we ned to activate moovit server:
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

- Finally, we can run the program
rosrun ur_icam_description pick_and_place.py

"""

import copy
import random
from math import pi
import rospy
from geometry_msgs.msg import Pose

from ur_icam_description.robotUR import RobotUR

BOX_X = 0.30  # Box size in meters
BOX_Y = 0.44  # Box size in meters
BOX_CENTER_ANGULAR = [2.7776150703430176, -1.5684941450702112, 1.299912452697754, -1.3755658308612269,
                      -1.5422008673297327, -0.3250663916217249]
BOX_CENTER_CARTESIAN = [-0.31899288568, -0.00357907370787, 0.226626573286]

PICK_MOVEMENT_DISTANCE = 0.215

def generate_coordinates(BOX_X, BOX_Y):
    coordinate_x = random.uniform(-BOX_X / 2, BOX_X / 2)
    coordinate_y = random.uniform(-BOX_Y / 2, BOX_Y / 2)
    return [coordinate_x, coordinate_y]


def relative_move(myRobot, x, y, z):
    waypoints = []
    wpose = myRobot.get_current_pose().pose
    if x:
        wpose.position.x -= x  # First move up (x)
        waypoints.append(copy.deepcopy(wpose))
    if y:
        wpose.position.y -= y  # Second move forward/backwards in (y)
        waypoints.append(copy.deepcopy(wpose))
    if z:
        wpose.position.z += z  # Third move sideways (z)
        waypoints.append(copy.deepcopy(wpose))

    myRobot.exec_cartesian_path(waypoints)


def calculate_relative_movement(myRobot, relative_coordinates):
    absolute_coordinates_x = BOX_CENTER_CARTESIAN[0] - relative_coordinates[0]
    absolute_coordinates_y = BOX_CENTER_CARTESIAN[1] - relative_coordinates[1]

    current_pose = myRobot.get_current_pose()

    x_movement = current_pose.pose.position.x - absolute_coordinates_x
    y_movement = current_pose.pose.position.y - absolute_coordinates_y

    return x_movement, y_movement

def pick_and_place(z_distance):
    # myRobot.open_gripper()
    # rospy.sleep(3)
    relative_move(myRobot, 0, 0, -z_distance)
    # myRobot.close_gripper()
    # rospy.sleep(3)
    relative_move(myRobot, 0, 0, z_distance)


if __name__ == '__main__':
    myRobot = RobotUR()
    rospy.init_node('robotUR')

    # Test of positioning with angular coordinates
    targetReached = myRobot.go_to_joint_state(BOX_CENTER_ANGULAR)

    if targetReached:
        print("Target reached")
    else:
        print("Target not reached")

    try:
        while True:
            # Move robot to random positions using relative moves. Get coordinates
            relative_coordinates = generate_coordinates(BOX_X, BOX_Y)
            print("La coordenada relativa X es:", relative_coordinates[0])
            print("La coordenada relativa Y es:", relative_coordinates[1])

            x_movement, y_movement = calculate_relative_movement(myRobot, relative_coordinates)
            print("Las coordenadas X son:", x_movement)
            print("Las coordenadas Y son:", y_movement)

            relative_move(myRobot, x_movement, y_movement, 0)
            # myRobot.open_gripper()
            # rospy.sleep(3)

            print(myRobot.get_current_pose().pose.position.z - PICK_MOVEMENT_DISTANCE)

            pick_and_place(myRobot.get_current_pose().pose.position.z - PICK_MOVEMENT_DISTANCE)
    except KeyboardInterrupt:
        print('interrupted!')




# print("Press ENTER to continue")
# raw_input()
# pose_goal = Pose()
# pose_goal.position.x = 0.4
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.4
# myRobot.go_to_pose_goal(pose_goal)
# print("The end")