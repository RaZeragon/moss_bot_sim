import os
import sys
import math
import rclpy
import argparse

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from functools import partial

class Square(Node):
    def __init__(self, robot_name = '', robot_namespace = '', swarm_amount = '', square_size = ''):
        super().__init__('Square')

        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
        self.robot_id = int(self.robot_name.split("moss_bot")[1])
        self.swarm_amount = int(swarm_amount)
        self.square_size = float(square_size)

        self.positions_message = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.positions_pair = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        self.square_coords = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(self.swarm_amount):
            self.state_est_subscriber = self.create_subscription(
                                Float32MultiArray,
                                '/moss_bot'+str(i)+'/state_est',
                                partial(self.state_est_callback, index=i),
                                10)
            self.state_est_subscriber  # prevent unused variable warning
        
        self.square_coords_publisher = self.create_publisher(
                        Float32MultiArray, 
                        '/' + self.robot_name + '/square_coords', 
                        10)
    
    def state_est_callback(self, msg, index):
        current_state = msg.data
        self.positions_pair[index][0] = current_state[0]
        self.positions_pair[index][1] = current_state[1]

        if index == self.robot_id:
            self.square_coords[8] = current_state[0]
            self.square_coords[9] = current_state[1]
            self.square_coords[10] = current_state[2]

        self.centroid()

    def centroid(self):
        self.centroid_x = (self.positions_pair[0][0] + self.positions_pair[1][0] + self.positions_pair[2][0] + self.positions_pair[3][0]) / 4
        self.centroid_y = (self.positions_pair[0][1] + self.positions_pair[1][1] + self.positions_pair[2][1] + self.positions_pair[3][1]) / 4
        self.centroid_position = [self.centroid_x, self.centroid_y]

        self.minimum_distance = 100
        self.starting_point = []
        for pair in self.positions_pair:
            if (abs(math.dist(pair, self.centroid_position) - (self.square_size * 0.7071))) < self.minimum_distance:
                self.minimum_distance = abs(math.dist(pair, self.centroid_position) - (self.square_size / 2))
                self.starting_point = pair

        self.create_square_coords()

    def create_square_coords(self):
        self.angle_rad = math.atan2(self.starting_point[1] - self.centroid_position[1], self.starting_point[0] - self.centroid_position[0])
        self.angle_deg = math.degrees(self.angle_rad)

        self.square_initial_x = (math.cos(self.angle_rad) * (self.square_size / 2)) + self.centroid_position[0]
        self.square_initial_y = (math.sin(self.angle_rad) * (self.square_size / 2)) + self.centroid_position[1]

        self.square_initial_coords = [self.square_initial_x, self.square_initial_y]

        self.square_coords[0] = self.square_initial_x
        self.square_coords[1] = self.square_initial_y
        self.square_coords[2] = self.square_initial_x
        self.square_coords[3] = self.square_initial_y
        self.square_coords[4] = self.square_initial_x
        self.square_coords[5] = self.square_initial_y
        self.square_coords[6] = self.square_initial_x
        self.square_coords[7] = self.square_initial_y

        if (self.angle_rad >= 0) and (self.angle_rad < (math.pi / 2)):
            self.square_coords[2] = self.square_coords[2] - self.square_size
            self.square_coords[4] = self.square_coords[4] - self.square_size
            self.square_coords[5] = self.square_coords[5] - self.square_size
            self.square_coords[7] = self.square_coords[7] - self.square_size
        elif (self.angle_rad >= (math.pi / 2)) and (self.angle_rad < math.pi):
            self.square_coords[2] = self.square_coords[2] + self.square_size
            self.square_coords[4] = self.square_coords[4] + self.square_size
            self.square_coords[5] = self.square_coords[5] - self.square_size
            self.square_coords[7] = self.square_coords[7] - self.square_size
        elif (self.angle_rad < 0) and (self.angle_rad > ((-1) * math.pi / 2)):
            self.square_coords[2] = self.square_coords[2] - self.square_size
            self.square_coords[4] = self.square_coords[4] - self.square_size
            self.square_coords[5] = self.square_coords[5] + self.square_size
            self.square_coords[7] = self.square_coords[7] + self.square_size
        elif (self.angle_rad <= ((-1) * math.pi / 2)) and (self.angle_rad > ((-1) * math.pi)):
            self.square_coords[2] = self.square_coords[2] + self.square_size
            self.square_coords[4] = self.square_coords[4] + self.square_size
            self.square_coords[5] = self.square_coords[5] + self.square_size
            self.square_coords[7] = self.square_coords[7] + self.square_size

        msg = Float32MultiArray()
        msg.data = self.square_coords
        self.square_coords_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Spawn robot amount')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the estimators robot')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-sa', '--swarm_amount', type=str, default='4', 
                        help='Amount of robots in swarm')
    parser.add_argument('-ss', '--square_size', type=str, default='5', 
                        help='Side length of the square')

    args, unknown = parser.parse_known_args()
 
    # Create the node
    square = Square(args.robot_name, args.robot_namespace, args.swarm_amount, args.square_size)
    rclpy.spin(square)
    square.destroy_node()
     
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()