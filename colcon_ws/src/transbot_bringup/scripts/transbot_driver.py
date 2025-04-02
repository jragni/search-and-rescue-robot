#!/usr/bin/env python3
from math import pi
import signal

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from transbot_msgs.msg import *
from transbot_msgs.srv import *

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Int32

from transbot_bringup.Transbot_Lib import Transbot
from transbot_bringup.helpers import is_arm_within_limits

class TransbotDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        ## Limits
        # velocity
        self.linear_max = 0.4  # [m]
        self.linear_min = 0.0  # [m]
        self.angular_max = 2.0  # [rad]
        self.angular_min = 0.0  # [rad]
        # servo angle for depth camera
        self.min_servo_angle = 60  # [deg]
        self.max_servo_angle = 120  # [deg]
        # Arm limits to prevent self collision
        self.arm_x_min = -0.1  # [m]
        self.arm_y_min = -0.04  # [m]

        # Start connection with extension board
        self.bot = Transbot(com="/dev/ttyAMA0")

        self.reentrant_group_1_ = ReentrantCallbackGroup()

        # Subscribers
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            5,
            callback_group=self.reentrant_group_1_,
        )
        self.cmd_vel_sub_

        self.target_angle_sub_ = self.create_subscription(
            Arm,
            'target_angle',
            self.arm_callback,
            10,
            callback_group=self.reentrant_group_1_,
        )
        self.target_angle_sub_

        self.pwm_servo_sub_ = self.create_subscription(
            PWMServo,
            'pwm_servo',
            self.pwm_servo_callback,
            10,
            callback_group=self.reentrant_group_1_,
        )
        self.pwm_servo_sub_

        self.adjust_sub_ = self.create_subscription(
            Adjust,
            'adjust',
            self.adjust_callback,
            10,
            callback_group=self.reentrant_group_1_,
        )
        self.adjust_sub_

        # Publishers
        self.battery_pub_ = self.create_publisher(
            Battery,
            'voltage',
            10
        )

        self.velocity_pub_ = self.create_publisher(
            Twist,
            '/transbot/get_vel',
            10
        )

        self.imu_pub_ = self.create_publisher(
            Imu,
            '/transbot/imu',
            10
        )

        self.create_timer(0.1, self.imu_callback, self.reentrant_group_1_)
        self.create_timer(0.1, self.velocity_callback, self.reentrant_group_1_)
        self.create_timer(0.1, self.battery_callback, self.reentrant_group_1_)

        self.bot.create_receive_threading()
        self.bot.set_uart_servo_angle(9,90)

        self.get_logger().info('Starting driver node')


    def arm_callback(self, msg):
        """Sets joint angle of the arms"""
        angles = [joint.angle for joint in msg.joint]
        if is_arm_within_limits(
            angles,
            self.arm_x_min, self.arm_y_min
        ):
            for joint in msg.joint:
                if joint.run_time != 0:
                    self.bot.set_uart_servo_angle(joint.id, joint.angle, joint.run_time)
        else:
            self.get_logger.warn(f"angles received will cause collision, ignoring...")


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
        self.min_servo_angle = 60
        self.max_servo_angle = 120

        angle = msg.angle

        if msg.id == 1:
            if angle < self.min_servo_angle:
                angle = self.min_servo_angle
            if angle > self.max_servo_angle:
                angle = self.max_servo_angle

            self.bot.set_pwm_servo(msg.id, angle)


    def adjust_callback(self, msg):
        """Dictates whether turning is imu assisted."""
        self.bot.set_imu_adjust(msg.adjust)


    def imu_callback(self):
        """Handle getting IMU data from API."""
        try:
            ax, ay, az = self.bot.get_accelerometer_data()
            gx, gy, gz = self.bot.get_gyroscope_data()
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.header = Header()
            imu_msg.header.frame_id = "imu_link"
            self.imu_pub_.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error getting IMU data: {e}")


    def velocity_callback(self):
        """Handle getting velocity data from API."""
        try:
            vel_msg = Twist()
            linear_velocity, angular_velocity = self.bot.get_motion_data()
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.velocity_pub_.publish(vel_msg)
        except Exception as e:
            self.get_logger().error(f"Error getting motion data: {e}")


    def battery_callback(self):
        """Handle getting battery data from API."""
        try:
            voltage = self.bot.get_battery_voltage()
            battery_msg = Battery()
            battery_msg.voltage = voltage
            self.battery_pub_.publish(battery_msg)
        except Exception as e:
            self.get_logger().error(f"Error getting battery data: {e}")
    
    def stop_robot(self):
        stop_msg = Twist()
        self.velocity_pub_.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TransbotDriver()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    def signal_handler(signal, frame):
        node.get_logger().info("Stopping robot...")
        node.stop_robot()
        node.get_logger().info("Shutting down!")
        node.destroy_node()
        rclpy.shutdown()

    executor.spin()
    signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    main()