#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from transbot_msgs.msg import SearchPoseArray
from transbot_msgs.srv import GetSearchPoses, SetSearchPose

class SearchPoseService(Node):
    """Updates search points.

    Allows users to set search poses and get search poses.
    
    """

    def __init__(self):
        super().__init__('search_point_srv_server')

        self.search_poses_ = []

        self.get_search_pose_srv_ = self.create_service(
            GetSearchPoses,
            'get_search_poses_service',
            self.get_search_poses_callback
        )

        self.set_search_pose_srv_ = self.create_service(
            SetSearchPose,
            'set_search_poses_service',
            self.set_search_pose_callback
        )

        self.search_pose_pub_ = self.create_publisher(
           SearchPoseArray,
           'search_poses',
           10,
        )

        self.get_logger().info('Starting search service...')


    def get_search_poses_callback(self, request, response):
        response.search_poses = self.search_poses_
        return response


    def set_search_pose_callback(self, request, response):
        operation_ = request.operation
        pose_ = request.pose
        allowed_operations = ['add', 'remove', 'pop', 'priority']
        # check for valid operation
        if (operation_ not in allowed_operations):
            response.success = False
            response.message = "Invalid operation type"
            self.get_logger().warn(response.message)
            return response

        # TODO add a check to return false if pos
        # .    is already added (no stamp comparison)
        if operation_ == 'add':
            self.search_poses_.append(pose_)
            response.message = f"Adding [{pose_.pose.position.x}, {pose_.pose.position.y}]"

        elif operation_ == 'pop':
            if (not self.search_poses_):
                response.success = False
                response.message = "Cannot perform 'pop'. Array is empty."
                return response
            
            self.search_poses_.pop()
            response.message = f"Removed [{pose_.pose.position.x}, {pose_.pose.position.y}]"
        elif operation_ == 'priority':
            self.search_poses_ = [pose_, *self.search_poses_]
            response.message = f"Adding to top of the list: [{pose_.pose.position.x}, {pose_.pose.position.y}]"

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
    service = SearchPoseService()

    rclpy.spin(service)
    service.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()