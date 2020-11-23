#!/usr/bin/env python2

import rospy
import copy
from std_msgs.msg import Bool

from Robot import Robot
from ai_manager.Environment import Environment

if __name__ == '__main__':

    robot = Robot()

    while True:
        waypoints = []
        wpose = robot.robot.get_current_pose().pose
        wpose.position.z -= (wpose.position.z - 0.24)  # Third move sideways (z)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = robot.robot.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        robot.robot.move_group.execute(plan, wait=True)

        distance_ok = rospy.wait_for_message('distance', Bool).data  # We retrieve sensor distance
        while distance_ok:
            distance_ok = rospy.wait_for_message('distance', Bool).data  # We retrieve sensor distance

        robot.robot.move_group.stop()

        distance = Environment.CARTESIAN_CENTER[2] - robot.robot.get_current_pose().pose.position.z
        robot.relative_move(0, 0, distance)
