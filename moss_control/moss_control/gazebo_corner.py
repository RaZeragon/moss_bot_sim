import os
import sys
import math
import rclpy
import argparse

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from functools import partial 

class Corner(Node):
    def __init__(self, robot_name = '', robot_namespace = '', swarm_amount = ''):
        super().__init__('Corner')

        # Passing arguments from launch command into the class
        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
        self.robot_id = int(self.robot_name.split("moss_bot")[1])
        self.swarm_amount = int(swarm_amount)

        # Initializing some arrays
        empty_square_coords = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.all_square_coords = []
        self.pair_coords = [0.0, 0.0, 0.0, 0.0]
        for i in range(self.swarm_amount):
            self.all_square_coords.append(empty_square_coords)

        # Subscribes to every swarm robot's topic
        for i in range(self.swarm_amount):
            self.square_coords_subscriber = self.create_subscription(
                                Float32MultiArray,
                                '/moss_bot'+str(i)+'/square_coords',
                                partial(self.square_coords_callback, index=i),
                                10)
            
            self.square_coords_subscriber  # prevent unused variable warning

        self.corner_vote_publisher = self.create_publisher(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/corner_ranked', 
                        10)

    def square_coords_callback(self, msg, index):
        current_square = msg.data

        # Parses through the /moss_bot#/square_coords data
        # [0] Corner 0 x | [1] Corner 0 y
        # [2] Corner 1 x | [3] Corner 1 y
        # [4] Corner 2 x | [5] Corner 2 y
        # [6] Corner 3 x | [7] Corner 3 y
        # [8] Current Robot x position | [9] Current Robot y position
        # [10] Current Robot yaw angle (radians) from -pi to pi
        # Seems to copy the square coords into each sublist?

        # Takes the corners calculated by each robot and lists them
        # with the index being the robot's ID
        # [[/moss_bot1/square_coords], [/moss_bot2/square_coords], ...]
        for i in range(8):
            self.all_square_coords[index][i] = current_square[i]

        if index == self.robot_id:
            self.current_x = current_square[8]
            self.current_y = current_square[9]
            self.current_yaw = current_square[10]
            self.current_position = [self.current_x, self.current_y]

            self.select_corner()

    def select_corner(self):
        self.square_coords = self.all_square_coords[0]

        # Takes list of corner coordinates and chunks them into pairs
        # [[Corner_0_x, Corner_0_y], [Corner_1_x, Corner_1_y], ...]
        chunk_size = 2
        for i in range(0, len(self.square_coords), chunk_size):
            self.pair_coords[int(i/2)] = self.square_coords[i:i+chunk_size]

        self.corner_index = 0
        self.target_position = []
        self.target_position_dict = {}

        # Calculates the distance of the robot to each corner coordinate pair
        # and adds it to a dictionary with the key being the corner's ID
        # {Corner_ID_0: Distance_0, Corner_ID_1: Distance_1, ...}
        for i in range(self.swarm_amount):
            self.target_position_dict[i] = abs(math.dist(self.current_position, self.pair_coords[i]))

        # Sorts the dictionary by distances from smallest to largest
        # Turns it into a list of lists
        # [[Corner_ID_#, Distance_#], [Corner_ID_#, Distance_#], ...]
        self.sorted_target_position = sorted(self.target_position_dict.items(), key = lambda x:x[1])

        # Flattens the list into a 1D list
        self.sorted_target_position_flat = [item for sublist in self.sorted_target_position for item in sublist]

        # Turns all the values into floats (the keys were originally ints)
        # and publishes to /moss_bot#/corner_ranked
        for i in range(len(self.sorted_target_position_flat)):
            self.sorted_target_position_flat[i] = float(self.sorted_target_position_flat[i])

        msg = Float32MultiArray()
        msg.data = self.sorted_target_position_flat
        self.corner_vote_publisher.publish(msg)

def main(args=None):
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
    corner = Corner(args.robot_name, args.robot_namespace, args.swarm_amount)
    rclpy.spin(corner)
    corner.destroy_node()
     
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()