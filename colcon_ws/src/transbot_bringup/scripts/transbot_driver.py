#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import pi
from time import sleep

from transbot_msgs.msg import *
from transbot_msgs.srv import *

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

sys.path.append("")

class TransbotDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        ## Limits
        # velocity
        self.linear_max = 0.4
        self.linear_min = 0.4
        self.angular_max = 2.0
        self.angular_min = 0.0
        # servo angle for depth camera
        self.min_servo_angle = 60
        self.max_servo_angle = 120

        # Subscribers
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            5
        )
        self.cmd_vel_sub_

        self.target_angle_sub_ = self.create_subscription(
            Arm,
            'target_angle',
            self.arm_callback,
            10
        )
        self.target_angle_sub_

        self.pwm_servo_sub_ = self.create_subscription(
            PWMServo,
            'pwm_servo',
            self.pwm_servo_callback,
            10
        )
        self.pwm_servo_sub_

        self.adjust_sub_ = self.create_subscription(
            Adjust,
            'adjust',
            self.adjust_callback,
            10
        )
        self.adjust_sub_

        # Publishers
        self.voltage_pub_ = self.create_publisher(
            Voltage,
            'voltage',
            self.voltage_callback,
            10
        )
        self.voltage_pub_

        self.velocity_pub_ = create_publisher(
            Twist,
            '/transbot/get_vel',
            10
        )
        self.velocity_pub_

        self.imu_pub_ = create_publisher(
            Imu,
            '/transbot/get_imu',
            10
        )

        
        self.get_logger().info('Starting driver node')


    
    def arm_callback(self, msg):
        #TODO
    
    def cmd_vel_callback(self, msg):
        # TODO

    def pwm_servo_callback(self, msg):
        """Controls depth camera servo pan."""

    def adjust_callback(self, msg):
        """Dictates whether turning is imu assisted."""

    def voltage_callback(self):
        """Get batter voltage."""
    

def main(args=None):
    rclpy.init(args=args)
    node = TransbotDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()