#!/usr/bin/env python3
from math import pi

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from transbot_msgs.msg import *
from transbot_msgs.srv import *

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

from transbot_bringup.Transbot_Lib import Transbot

class TransbotDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        ## Limits
        # velocity
        self.linear_max = 0.4
        self.linear_min = 0.0
        self.angular_max = 2.0
        self.angular_min = 0.0
        # servo angle for depth camera
        self.min_servo_angle = 60
        self.max_servo_angle = 120

        # Start connection with extension board
        self.bot = Transbot(com="/dev/ttyAMA0")

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

        self.velocity_pub_ = self.create_publisher(
            Twist,
            '/transbot/get_vel',
            10
        )

        self.imu_pub_ = self.create_publisher(
            Imu,
            '/transbot/get_imu',
            10
        )

        self.bot.create_receive_threading()
        self.bot.set_uart_servo_angle(9,90)

        self.get_logger().info('Starting driver node')
    

    def arm_callback(self, msg):
        """Sets joint angle of the arms"""
        for joint in msg.joint:
            if joint.run_time != 0:
                self.bot.set_uart_servo_angle(joint.id, joint.angle, joint.run_time)


    def cmd_vel_callback(self, msg):
        """controls motion of tank."""

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        if linear_velocity > self.linear_max:
            linear_velocity = self.linear_max
        elif linear_velocity < -self.linear_max:
            linear_velocity = -self.linear_max
        elif -self.linear_min < linear_velocity < 0:
            linear_velocity = -self.linear_min
        elif 0 < linear_velocity < self.linear_min:
            linear_velocity = self.linear_min

        if angular_velocity > self.angular_max:
            angular_velocity = self.angular_max
        elif angular_velocity < -self.angular_max:
            angular_velocity = -self.angular_max
        elif -self.angular_min < angular_velocity < 0:
            angular_velocity = -self.angular_min
        elif 0 < angular_velocity < self.angular_min:
            angular_velocity = self.angular_min

        self.get_logger().info(f"cmd_vel: {linear_velocity}, cmd_ang: {angular_velocity}")
        self.bot.set_car_motion(0.8*linear_velocity, 0.5*angular_velocity)


    def pwm_servo_callback(self, msg):
        """Controls depth camera servo pan."""



    def adjust_callback(self, msg):
        """Dictates whether turning is imu assisted."""
        print("TODO adjust")


    def voltage_callback(self):
        """Get batter voltage."""
        print("TODO get battery ")
    

    def pub_data(self):
        """Get data from Transbot API and publish to topic."""
        print('TODO pubdata')


def main(args=None):
    rclpy.init(args=args)
    node = TransbotDriver()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()