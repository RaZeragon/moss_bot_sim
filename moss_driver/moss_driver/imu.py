#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import board
import busio
import serial
from adafruit_bno08x_rvc import BNO08x_RVC  # pylint:disable=wrong-import-position


class IMU_Feedback(Node):

    def __init__(self):
        super().__init__('IMU_Feedback')
        uart = serial.Serial("/dev/serial0", 115200)
        self.rvc = BNO08x_RVC(uart)
        self.publisher_ = self.create_publisher(Float32, 'imu_angle', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.imu_callback)
        self.i = 0

    def imu_callback(self):
        imu_data = imu_data()
        msg = Float32MultiArray(data=list)
        self.publisher_.publish(msg)
    
    def imu_data(self):
        heading = self.rvc.heading
        rvc._uart.reset_input_buffer()
        return heading


def main(args=None):
    rclpy.init(args=args)

    imu = IMU_Feedback()

    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()