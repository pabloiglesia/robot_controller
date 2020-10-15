#!/usr/bin/env python
# coding: utf-8

"""
- We need to connect the camera and the nodes
roslaunch ur_icam_description webcam.launch

- We need to establish a connection to the robot with the following comand:
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=10.31.56.102 kinematics_config:=${HOME}/Calibration/ur3_calibration.yaml

- Then, we ned to activate moovit server:
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch

- Activate the talker
rosrun ai_manager talker.py

- Activate the node
rosrun robot_controller distance_sensor_simulator.py
rosrun robot_controller object_gripped.py

- Finally, we can run the program
rosrun robot_controller main.py

"""

import rospy

from ai_manager.srv import GetActions
from Robot import Robot


def get_action(robot):
    relative_coordinates = robot.calculate_current_coordinates()
    rospy.wait_for_service('get_actions')
    try:
        get_actions = rospy.ServiceProxy('get_actions', GetActions)
        return get_actions(relative_coordinates[0], relative_coordinates[1]).action
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


# This function defines the movements that robot should make depending on the action listened
def take_action(action, robot):
    distance = 0.02 # Movement in metres
    print(action)
    if action == 'north':
        robot.take_north(distance)
    elif action == 'south':
        robot.take_south(distance)
    elif action == 'east':
        robot.take_east(distance)
    elif action == 'west':
        robot.take_west(distance)
    elif action == 'pick':
        robot.take_pick()
    elif action == 'random_state':
        robot.take_random_state()


if __name__ == '__main__':

    rospy.init_node('robotUR')

    robot = Robot()

    # Test of positioning with angular coordinates
    robot.go_to_initial_pose()

    # Let's put the robot in a random position to start, creation of new state
    take_action('random_state')

    while True:
        action = get_action(robot)
        take_action(action, robot)