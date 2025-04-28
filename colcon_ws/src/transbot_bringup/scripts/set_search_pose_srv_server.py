#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from transbot_msgs.msg import SearchPoseArray
from transbot_msgs.srv import SearchPose

class SetSearchPose(Node):
    """Updates search points."""

    def __init__(self):
        super().__init__('set_search_point_srv_server')

        self.search_poses_ = []

        self.search_pose_srv_ = self.create_service(
            SearchPose,
            'search_poses_service',
            self.set_search_point_callback
        )

        qos_profile = QosProfile(
            depth=10,
            durability=DurabilityPolicy(DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.search_pose_pub_ = self.create_publisher(
           SearchPoseArray,
           'search_poses',
           qos_profile
        )

        self.get_logger().info('Starting set search service...')


    def set_search_point_callback(self, request, response):
        operation_ = request.operation;
        pose_ = request.pose

        # check for valid operation
        if (operation_ not in ['add', 'remove', 'pop']):
            response.success = False
            response.message = "Invalid operation type"
            self.get_logger().warn(response.message)
            return response

        # TODO add a check to return false if pos
        # .    is already added (no stamp comparison)
        if (operation_ == 'add'):
            self.search_poses_.append(pose_)
            response.message = f"Adding [{pose_.pose.position.x}, {pose_.pose.position.y}]"

        elif (operation_ == 'pop'):
            if (not self.search_poses_):
                response.success = False
                response.message = "Cannot perform 'pop'. Array is empty."
                return response
            
            self.search_poses_.pop()
            response.message = f"Removed [{pose_.pose.position.x}, {pose_.pose.position.y}]"

        else:
            ## remove
            # filter out pose
            self.search_poses_ = [
                p
                for p in self.search_poses_
                if  p.pose != pose_.pose
            ]
            response.message = f"Removed [{pose_.pose.position.x}, {pose_.pose.position.y}]"

        msg = SearchPoseArray()
        msg.poses = self.search_poses_

        self.search_pose_pub_.publish(msg)
        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)
    service = SetSearchPose()

    rclpy.spin(service)
    service.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()