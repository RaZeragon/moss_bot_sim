import os
import sys
import math
import rclpy
import argparse
import numpy

from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from functools import partial 

class Movement(Node):
    def __init__(self, robot_name = '', robot_namespace = '', swarm_amount = ''):
        super().__init__('Movement')

        # Passing arguments from launch command into the class
        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
        self.robot_id = int(self.robot_name.split("moss_bot")[1])
        self.swarm_amount = int(swarm_amount)

        # Initializing some variables
        self.square_coords = [0.0] * (self.swarm_amount * 2)
        self.first = True

        self.corner_selection_subscriber = self.create_subscription(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/corner_selection', 
                        self.corner_selection_callback,
                        10)

        self.movement_publisher = self.create_publisher(
                        Twist, 
                        '/' + self.robot_name + '/cmd_vel', 
                        10)
    
    # Chunks a list into sublists of size n
    def chunks(self, lst, n):
        for i in range(0, len(lst), n):
            yield lst[i:i+n]

    def corner_selection_callback(self, msg):
        current_data = msg.data

        # Parses through the /moss_bot#/square_coords data
        # [0] Corner 0 x | [1] Corner 0 y
        # [2] Corner 1 x | [3] Corner 1 y
        # [4] Corner 2 x | [5] Corner 2 y
        # [6] Corner 3 x | [7] Corner 3 y
        # [8] Current Robot x position | [9] Current Robot y position
        # [10] Current Robot yaw angle (radians) from -pi to pi
        # [11] Corner_ID selection

        # Add failsafe to prevent square_coords containing 0.0, 0.0
        if self.first == True:
            self.square_coords[0:8] = current_data[0:8]
            self.corner_selection = current_data[11]
            self.first = False

        self.current_x = current_data[8]
        self.current_y = current_data[9]
        self.current_yaw = current_data[10]
        self.current_position = [self.current_x, self.current_y]

        self.move_to_target()

    def move_to_target(self):
        # Chunks the corners into sublists of 2 and then selects its
        # target coordinates based on its chosen Corner_ID
        self.corner_paired_list = list(self.chunks(self.square_coords, 2))
        self.target_position = self.corner_paired_list[int(self.corner_selection)]

        # Due to some bugs where robots would head to 0.0, 0.0
        if self.target_position == [0.0, 0.0]:
            self.first = True

        self.distance_error = 0.25
        self.angle_error = 0.25

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0 #yaw

        # Calculating angle between the current robot's yaw and its target position
        # Converts the -pi to pi angles to 0 to 2 * pi
        self.angle_rad = math.atan2(self.target_position[1] - self.current_y, self.target_position[0] - self.current_x)
        self.angle_rad_360 = (6.28318 - abs(self.angle_rad))
        self.current_yaw_360 = (6.28318 - abs(self.current_yaw))

        # Turn robot until it is angles correctly and then move forward until its 
        # distance to the target position is within the defined threshold
        if (math.dist(self.current_position, self.target_position)) > self.distance_error:
            if abs((self.current_yaw - self.angle_rad)) > self.angle_error:
                msg.linear.x = 0.0
                msg.angular.z = 0.25

                ## Has bug where bots get stuck vibrating between rotating each direction
                # if (abs(self.current_yaw_360 - self.angle_rad_360) > 180.0):
                #     msg.angular.z = 0.25 * numpy.sign(self.current_yaw_360 - self.angle_rad_360)
                # else:
                #     msg.angular.z = 0.25 * numpy.sign(self.angle_rad_360 - self.current_yaw_360)
            else:
                msg.linear.x = 0.25
                msg.angular.z = 0.0

        self.movement_publisher.publish(msg)

def main(args=None):
    """
    Entry point for the program.
    """
    # Initialize rclpy library
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Spawn robot movement nodes')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the estimators robot')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-sa', '--swarm_amount', type=str, default='4', 
                        help='Amount of robots in swarm')

    args, unknown = parser.parse_known_args()
 
    # Create the node
    movement = Movement(args.robot_name, args.robot_namespace, args.swarm_amount)
    rclpy.spin(movement)
    movement.destroy_node()
     
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()