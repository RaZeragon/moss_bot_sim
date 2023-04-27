# Author: Addison Sears-Collins
# Date: March 19, 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
import math
import rclpy
import argparse
import time
 
# Used to create nodes
from rclpy.node import Node
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist 
 
# Position, orientation, linear velocity, angular velocity
from nav_msgs.msg import Odometry
 
# Handles laser distance scan to detect obstacles
from sensor_msgs.msg import LaserScan
 
# Used for laser scan
#from rclpy.qos import qos_profile_sensor_data
 
# Enable use of std_msgs/Float64MultiArray message
from std_msgs.msg import Float64MultiArray, Float32MultiArray 
 
# Scientific computing library for Python
import numpy as np
 
class Estimator(Node):
    def __init__(self, robot_name = '', robot_namespace = ''):
  
        super().__init__('Estimator')

        self.robot_name = robot_name
        self.robot_namespace = robot_namespace 
  
        self.odom_subscriber = self.create_subscription(
                            Odometry,
                            '/' + self.robot_name + '/odom',
                            self.odom_callback,
                            10)

        self.velocity_subscriber = self.create_subscription(
                                Twist,
                                '/' + self.robot_name + '/cmd_vel',
                                self.velocity_callback,
                                10)
  
        self.publisher_state_est = self.create_publisher(
                                Float32MultiArray, 
                                '/' + self.robot_name + '/state_est', 
                                10)
  
    def odom_callback(self, msg):
        """
        Receive the odometry information containing the position and orientation
        of the robot in the global reference frame. 
        The position is x, y, z.
        The orientation is a x,y,z,w quaternion. 
        """                    
        roll, pitch, yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
  
        obs_state_vector_x_y_yaw = [msg.pose.pose.position.x,msg.pose.pose.position.y,yaw]
  
      # Publish the estimated state (x position, y position, yaw angle)
        self.publish_estimated_state(obs_state_vector_x_y_yaw)
  
    def publish_estimated_state(self, state_vector_x_y_yaw):
        """
        Publish the estimated pose (position and orientation) of the 
        robot to the '/demo/state_est' topic. 
        :param: state_vector_x_y_yaw [x, y, yaw] 
        x is in meters, y is in meters, yaw is in radians
        """
        msg = Float32MultiArray()
        msg.data = state_vector_x_y_yaw
        self.publisher_state_est.publish(msg)
  
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
  
    def velocity_callback(self, msg):
        """
        Listen to the velocity commands (linear forward velocity 
        in the x direction in the robot's reference frame and 
        angular velocity (yaw rate) around the robot's z-axis.
        [v,yaw_rate]
        [meters/second, radians/second]
        """
        # Forward velocity in the robot's reference frame
        v = msg.linear.x
    
        # Angular velocity around the robot's z axis
        yaw_rate = msg.angular.z
 
def main(args=None):
    """
    Entry point for the program.
    """
    # Initialize rclpy library
    rclpy.init(args=args)  

    parser = argparse.ArgumentParser(description='Spawn robot estimators')
    parser.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the estimators robot')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')

    args, unknown = parser.parse_known_args()
 
    # Create the node
    estimator = Estimator(args.robot_name, args.robot_namespace)
 
    rclpy.spin(estimator)
 
    estimator.destroy_node()
     
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()