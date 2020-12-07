#!/usr/bin/env python2

import rospy
import time
import copy
from std_msgs.msg import Bool
from moveit_msgs import RobotTrajectory
import datetime

from Robot import Robot
from ai_manager.Environment import Environment

if __name__ == '__main__':
    rospy.init_node('robotUR')

    robot = Robot()
    robot.go_to_initial_pose()

    def change_plan_speed(plan, new_speed):
        new_plan = RobotTrajectory()
        new_plan.joint_trajectory = plan.joint_trajectory
        n_joints = len(plan.joint_trajectory.joint_names)
        n_points = len(plan.joint_trajectory.points)

        for i in range(n_points):
            plan.joint_trajectory.points[i].time_from_start = plan.joint_trajectory.points[i].time_from_start / new_speed
            for j in range(n_joints):
                new_plan.joint_trajectory.points[i].velocities[j] = plan.joint_trajectory.points[i].velocities[j] * new_speed
                new_plan.joint_trajectory.points[i].accelerations[j] = plan.joint_trajectory.points[i].accelerations[
                                                                           j] * new_speed
                new_plan.joint_trajectory.points[i].positions[j] = plan.joint_trajectory.points[i].positions[j]

        return new_plan

    while True:
        waypoints = []
        wpose = robot.robot.get_current_pose().pose
        wpose.position.z -= (wpose.position.z - 0.24)  # Third move sideways (z)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = robot.robot.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        plan = change_plan_speed(plan, 0.1)

        robot.robot.move_group.execute(plan, wait=False)

        distance_ok = rospy.wait_for_message('distance', Bool).data  # We retrieve sensor distance
        timestamp = datetime.datetime.now()

        while not distance_ok:
            previous_timestamp = timestamp
            timestamp = datetime.datetime.now()
            print("Difference: {}".format( timestamp - previous_timestamp ))
            try:
                distance_ok = rospy.wait_for_message('distance', Bool, 0.2).data  # We retrieve sensor distance
            except:
                print("Fallo en la comunicaci'on")
                break

        robot.robot.move_group.stop()
        robot.relative_move(0, 0, 0.001)
        time.sleep(1)
        distance = Environment.CARTESIAN_CENTER[2] - robot.robot.get_current_pose().pose.position.z
        robot.relative_move(0, 0, distance)
