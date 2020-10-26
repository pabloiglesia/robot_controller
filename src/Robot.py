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
        time_step = timer/n_msg
        i=0
        while(i != n_msg):
            self.gripper_publisher.publish(msg)
            time.sleep(time_step)
            i += 1

    # Action pick: Pick and place
    def take_pick(self):
        # In this function we should read the distance to the object
        up_distance = 0  # Variable were we store the distance that we have move the robot so that we can go back to the
        # original pose
        distance_ok = False  # False until target is reached
        while not distance_ok:
            # Check if the distance is the correct one
            # TODO : check if the distance is in the correct measures
            distance = rospy.wait_for_message('distance', Float32).data  # We retrieve sensor distance

            if distance <= Environment.PICK_DISTANCE:  # If distance is equal or smaller than the target distance
                # TODO : Check what kind of msg the subscriber is waiting
                self.send_gripper_message(True)  # We try to pick the object enabling the vacuum gripper
                time.sleep(2)  # We wait some time to let the vacuum pick the object
                distance_ok = True  # End the loop by setting this variable to True
            else:  # If the distance to the objects is higher than the target distance
                difference = distance - Environment.PICK_DISTANCE  # We calculate the difference between the two distances
                up_distance += difference  # We increment the up_distance variable
                self.relative_move(0, 0, -difference)  # We try to move to the desired distance

        self.relative_move(0, 0, up_distance)  # We went back to the original pose

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