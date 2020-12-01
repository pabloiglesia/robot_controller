#!/usr/bin/env python2

import rospy
import time
import copy
from std_msgs.msg import Bool
import datetime

from Robot import Robot
from ai_manager.Environment import Environment

if __name__ == '__main__':
    rospy.init_node('robotUR')

    robot = Robot()
    robot.go_to_initial_pose()

    while True:
        waypoints = []
        wpose = robot.robot.get_current_pose().pose
        wpose.position.z -= (wpose.position.z - 0.24)  # Third move sideways (z)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = robot.robot.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        robot.robot.move_group.execute(plan, wait=False)

        distance_ok = rospy.wait_for_message('distance', Bool).data  # We retrieve sensor distance
        timestamp = datetime.datetime.now()

        while not distance_ok:
            previous_timestamp = timestamp
            timestamp = datetime.datetime.now()
            print("Difference: {}".format( timestamp - previous_timestamp ))
            distance_ok = rospy.wait_for_message('distance', Bool, 0.2).data  # We retrieve sensor distance

        robot.robot.move_group.stop()
        robot.relative_move(0, 0, 0.001)
        time.sleep(1)
        distance = Environment.CARTESIAN_CENTER[2] - robot.robot.get_current_pose().pose.position.z
        robot.relative_move(0, 0, distance)
