import os
import sys
import math
import rclpy
import argparse

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from functools import partial 
from operator import itemgetter

class Voter(Node):
    def __init__(self, robot_name = '', robot_namespace = '', swarm_amount = ''):
        super().__init__('Voter')

        vote_group = MutuallyExclusiveCallbackGroup()

        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
        self.robot_id = int(self.robot_name.split("moss_bot")[1])
        self.swarm_amount = int(swarm_amount)

        self.goal_positions = [0.0, 0.0, 0.0, 0.0]
        self.all_positions = [0.0] * (self.swarm_amount * self.swarm_amount)
        self.position_data = [0.0] * ((self.swarm_amount * 2) + 4)

        for i in range(self.swarm_amount):
            self.corner_vote_subscriber = self.create_subscription(
                                Float32MultiArray,
                                '/moss_bot'+str(i)+'/corner_ranked',
                                partial(self.corner_ranked_callback, index=i),
                                10,
                                callback_group = vote_group)
            
            self.corner_vote_subscriber  # prevent unused variable warning

        self.square_coords_subscriber = self.create_subscription(
                                Float32MultiArray,
                                '/' + self.robot_name + '/square_coords',
                                self.square_coords_callback,
                                10)

        self.corner_selection_publisher = self.create_publisher(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/corner_selection', 
                        10)
        
        self.test_publisher = self.create_publisher(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/test', 
                        10)
    
    def chunks(self, lst, n):
        for i in range(0, len(lst), n):
            yield lst[i:i+n]

    def square_coords_callback(self, msg):
        current_coords = msg.data
        self.position_data[0:11] = current_coords[:]

    def corner_ranked_callback(self, msg, index):
        corner_list = msg.data
        self.corner_paired_list = list(self.chunks(corner_list, 2))

        for item in self.corner_paired_list:
            item.append(index)

        self.goal_positions[index] = self.corner_paired_list
        
        counter = 0
        for item in self.goal_positions:
            if item == 0.0:
                counter += 1
        
        if counter == 0:
            self.vote()
    
    def vote(self):
        # Flattens the list of lists of lists into list of lists
        # [[Corner_ID, Distance, Robot_ID], ...]
        for item_id in range(len(self.goal_positions)):
            for subitem_id in range(len(self.goal_positions[item_id])):
                self.all_positions[(item_id * 4) + subitem_id] = self.goal_positions[item_id][subitem_id]

        self.all_positions_sorted=sorted(self.all_positions, key=itemgetter(1))

        # [Corner_Selection, ...] Index = Robot_ID
        self.corner_selections = [4.0, 4.0, 4.0, 4.0]

        self.corner_selections[int(self.all_positions_sorted[0][2])] = self.all_positions_sorted[0][0]
        for item in self.all_positions_sorted:
            # If the robot has already chosen a corner, continue to the next iteration
            if self.corner_selections[int(item[2])] != 4.0:
                continue
            
            # Assuming the robot has not chosen a corner, check if the current corner exists in the corner selection list
            # If the current corner exists in the corner selection list, continue to the next iteration
            if item[0] in self.corner_selections:
                continue

            self.corner_selections[int(item[2])] = item[0]

        msg = Float32MultiArray()
        msg.data = self.corner_selections
        self.test_publisher.publish(msg)

        self.position_data[11] = self.corner_selections[self.robot_id]

        msg = Float32MultiArray()
        msg.data = self.position_data
        self.corner_selection_publisher.publish(msg)

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
    voter = Voter(args.robot_name, args.robot_namespace, args.swarm_amount)
    rclpy.spin(voter)
    voter.destroy_node()
     
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()