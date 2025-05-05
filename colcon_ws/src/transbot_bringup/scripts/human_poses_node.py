#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from transbot_msgs.msg import HumanLocations
from geometry_msgs.msg import PoseStamped

class HumanPosesNode(Node):
    def __init__(self):
        super().__init__("human_pose_client_node")

        self.LOCATION_TOLERANCE = 0.15

        self.human_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/human_detection/pose_stamped',
            self.human_pose_sub_callback,
            10
        )
        self.human_locations = []

        self.human_locations_pub_ = self.create_publisher(
            HumanLocations,
            '/human_detection/locations',
            1,
        )

        self.create_timer(0.01, self.human_locations_pub_callback)


    def human_locations_pub_callback(self):
        human_locations_msg = HumanLocations()
        human_locations_msg.locations = self.human_locations
        self.human_locations_pub_.publish(human_locations_msg)


    def human_pose_sub_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # transform to odom or map frame
        
        # check if poses in within distance
        for location in self.human_locations:
            location_x = location.pose.position.x
            location_y = location.pose.position.y
            location_z = location.pose.position.z

            delta_x = location_x - x
            delta_y = location_y - y
            delta_z = location_z - z

            point_to_location_distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

            if (point_to_location_distance < self.LOCATION_TOLERANCE):
                self.human_locations.append(msg)
            
    
def main(args=None):
    rclpy.init(args=args)

    node = HumanPosesNode()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()