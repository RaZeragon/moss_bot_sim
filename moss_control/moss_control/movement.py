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
        self.first = True

        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
        self.robot_id = int(self.robot_name.split("moss_bot")[1])
        self.swarm_amount = int(swarm_amount)

        self.square_coords = [0.0] * (self.swarm_amount * 2)

        self.corner_selection_subscriber = self.create_subscription(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/corner_selection', 
                        self.corner_selection_callback,
                        10)

        self.movement_publisher = self.create_publisher(
                        Twist, 
                        '/' + self.robot_name + '/cmd_vel', 
                        10)
    
    def chunks(self, lst, n):
        for i in range(0, len(lst), n):
            yield lst[i:i+n]

    def corner_selection_callback(self, msg):
        current_data = msg.data

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
        self.corner_paired_list = list(self.chunks(self.square_coords, 2))
        self.target_position = self.corner_paired_list[int(self.corner_selection)]

        # print('ROBOT NUMBER ' + str(self.robot_id) + ' POSITION IS ' + str(self.target_position))

        self.distance_error = 0.25
        self.angle_error = 0.25

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0 #yaw

        self.angle_rad = math.atan2(self.target_position[1] - self.current_y, self.target_position[0] - self.current_x)
        self.angle_rad_360 = (6.28318 - abs(self.angle_rad))
        self.current_yaw_360 = (6.28318 - abs(self.current_yaw))

        if (math.dist(self.current_position, self.target_position)) > self.distance_error:
            if abs((self.current_yaw - self.angle_rad)) > self.angle_error:
                msg.linear.x = 0.0
                msg.angular.z = 0.25

                # Has bug where bots get stuck vibrating between rotating each direction
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