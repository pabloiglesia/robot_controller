import copy
import rospy
import time

from std_msgs.msg import Bool
from std_msgs.msg import Float32

from ai_manager.Environment import Environment
from ur_icam_description.robotUR import RobotUR

"""
Class used to establish connection with the robot and perform different actions such as move in all cardinal directions 
or pick and place an object. 
"""


class Robot:
    def __init__(self, robot=RobotUR(), gripper_topic='switch_on_off'):
        self.robot = robot  # Robot we want to control
        self.gripper_topic = gripper_topic  # Gripper topic
        self.gripper_publisher = rospy.Publisher(self.gripper_topic, Bool)  # Publisher for the gripper topic

    def relative_move(self, x, y, z):
        """
        Perform a relative move in all x, y or z coordinates.

        :param x:
        :param y:
        :param z:
        :return:
        """
        waypoints = []
        wpose = self.robot.get_current_pose().pose
        if x:
            wpose.position.x -= x  # First move up (x)
            waypoints.append(copy.deepcopy(wpose))
        if y:
            wpose.position.y -= y  # Second move forward/backwards in (y)
            waypoints.append(copy.deepcopy(wpose))
        if z:
            wpose.position.z += z  # Third move sideways (z)
            waypoints.append(copy.deepcopy(wpose))

        self.robot.exec_cartesian_path(waypoints)

    def calculate_relative_movement(self, relative_coordinates):
        absolute_coordinates_x = Environment.CARTESIAN_CENTER[0] - relative_coordinates[0]
        absolute_coordinates_y = Environment.CARTESIAN_CENTER[1] - relative_coordinates[1]

        current_pose = self.robot.get_current_pose()

        x_movement = current_pose.pose.position.x - absolute_coordinates_x
        y_movement = current_pose.pose.position.y - absolute_coordinates_y

        return x_movement, y_movement

    def calculate_current_coordinates(self):
        absolut_coordinate_x = self.robot.get_current_pose().pose.position.x
        absolut_coordinate_y = self.robot.get_current_pose().pose.position.y

        relative_coordinate_x = Environment.CARTESIAN_CENTER[0] - absolut_coordinate_x
        relative_coordinate_y = Environment.CARTESIAN_CENTER[1] - absolut_coordinate_y

        return [relative_coordinate_x, relative_coordinate_y]

    # Action north: positive x
    def take_north(self, distance=Environment.ACTION_DISTANCE):
        self.relative_move(distance, 0, 0)

    # Action south: negative x
    def take_south(self, distance=Environment.ACTION_DISTANCE):
        self.relative_move(-distance, 0, 0)

    # Action east: negative y
    def take_east(self, distance=Environment.ACTION_DISTANCE):
        self.relative_move(0, -distance, 0)

    # Action west: positive y
    def take_west(self, distance=Environment.ACTION_DISTANCE):
        self.relative_move(0, distance, 0)

    def take_random_state(self):
        # Move robot to random positions using relative moves. Get coordinates
        relative_coordinates = Environment.generate_random_state()
        # Calculate the new coordinates
        x_movement, y_movement = self.calculate_relative_movement(relative_coordinates)
        # Move the robot to the random state
        self.relative_move(x_movement, y_movement, 0)

    def send_gripper_message(self, msg, timer=2, n_msg=10):
        """
        Function that sends a burst of n messages of the gripper_topic during an indicated time
        :param msg: True or False
        :param time: time in seconds
        :param n_msg: number of messages
        :return:
        """
        time_step = (timer/2)/n_msg
        i=0
        while(i <= n_msg):
            self.gripper_publisher.publish(msg)
            time.sleep(time_step)
            i += 1

        time.sleep(timer/2)

    # Action pick: Pick and place
    def take_pick(self):
        # In this function we should read the distance to the object
        # up_distance = 0  # Variable were we store the distance that we have move the robot so that we can go back to the
        # original pose

        def back_to_original_pose(robot):
            """
            Function used to go back to the original height once a vertical movement has been performed.
            :param robot: robot_controller.Robot.py object
            :return:
            """
            distance = Environment.CARTESIAN_CENTER[2] - robot.robot.get_current_pose().pose.position.z
            robot.relative_move(0, 0, distance)

        def down_movement(robot):
            """
            This function performs the down movement of the pick action.

            It creates an asynchronous move group trajectory planning. This way the function is able to receive distance
            messages while the robot is moving and stop it once the robot is in contact with an object.

            Finally, when there is any problems with the communications the movement is stopped and
            communication_problem boolean flag is set to True. It is considered that there is a problem with
            communications when the robot is not receiving any distance messages during 200 milli-seconds (timeout=0.2)

            :param robot: robot_controller.Robot.py object
            :return: communication_problem flag
            """
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

            while not distance_ok:
                try:
                    communication_problem = False
                    distance_ok = rospy.wait_for_message('distance', Bool, 0.2).data  # We retrieve sensor distance
                except:
                    communication_problem = True
                    rospy.loginfo("Error in communications, trying again")
                    break

            # Both stop and 10 mm up movement to stop the robot
            self.robot.move_group.stop()
            self.relative_move(0, 0, 0.001)

            return communication_problem

        def pick_action():
            """
            Function that actually performs the pick action using down_movement() and back_to_original_pose()
            :return:
            """
            while True:
                communication_problem = down_movement(self)
                if communication_problem:
                    back_to_original_pose(self)
                else:
                    break

        distance_ok = rospy.wait_for_message('distance', Bool).data  # We retrieve sensor distance
        while True:
            time.sleep(0.2)
            if not rospy.wait_for_message('distance', Bool).data:  # We retrieve sensor distance
                self.relative_move(0, 0, -0.01)
            else:
                self.relative_move(0, 0, -0.005)
                break

        self.send_gripper_message(True, timer=4)  # We turn on the gripper

        distance = Environment.CARTESIAN_CENTER[2] - self.robot.get_current_pose().pose.position.z
        self.relative_move(0,0,distance)

        object_gripped = rospy.wait_for_message('object_gripped', Bool).data
        if object_gripped:  # If we have gripped an object we place it into the desired point
            self.take_place()
        else:
            self.send_gripper_message(False)  # We turn off the gripper

        return object_gripped

    # Function to define the place for placing the grasped objects
    def take_place(self):
        # First, we get the cartesian coordinates of one of the corner
        x_box, y_box = Environment.get_relative_corner('se')
        x_move, y_move = self.calculate_relative_movement([x_box, y_box])
        # We move the robot to the corner of the box
        self.relative_move(x_move, y_move, 0)
        # We calculate the trajectory for our robot to reach the box
        trajectory_x = self.robot.get_current_pose().pose.position.x - Environment.PLACE_CARTESIAN_CENTER[0]
        trajectory_y = self.robot.get_current_pose().pose.position.y - Environment.PLACE_CARTESIAN_CENTER[1]
        trajectory_z = - Environment.CARTESIAN_CENTER[2] + Environment.PLACE_CARTESIAN_CENTER[2]
        # We move the robot to the coordinates desired to place the object
        self.relative_move(0, 0, trajectory_z)
        self.relative_move(0, trajectory_y, 0)
        self.relative_move(trajectory_x, 0, 0)
        # Then, we left the object
        self.relative_move(0, 0, -0.05)
        # Then, we switch off the vacuum gripper so the object can be placed
        # TODO : Check the correct information to send the topic
        self.send_gripper_message(False)
        # Wait some seconds, in order to the msg to arrive to the gripper
        time.sleep(2)
        # Then the robot goes up
        self.relative_move(0, 0, 0.05)
        # Final we put the robot in the center of the box, the episode should finish now
        self.robot.go_to_joint_state(Environment.ANGULAR_CENTER)
        # TODO : publicar si se ha acabado el episodio

    def go_to_initial_pose(self):
        target_reached = self.robot.go_to_joint_state(Environment.ANGULAR_CENTER)

        if target_reached:
            print("Target reachead")
        else:
            print("Target not reached")