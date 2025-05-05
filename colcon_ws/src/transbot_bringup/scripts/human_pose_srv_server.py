#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from transbot_msgs.msg import HumanPoseArray

class HumanPoseServer(Node):
    def __init__(self):
        super().__init__("human_pose_service_server")
        self.human_poses = []

        self.get_human_poses_srv_ = self.create_service(
            GetSearchPoses,
            'get_human_poses_service',
            self.get_human_poses_callback
        )

        self.set_search_pose_srv_ = self.create_service(
            SetSearchPose,
            'set_human_poses_service',
            self.set_human_pose_callback
        )
        self.get_logger().info('Starting human pose service...')

    def get_human_poses(self, request, response):
        response.search_poses = self.search_poses_
        return response


    def set_search_pose_callback(self, request, response):
        operation_ = request.operation;
        pose_ = request.pose
        allowed_operations = ['add', 'remove', 'pop', 'priority']
        # check for valid operation
        if (operation_ not in allowed_operations):
            response.success = False
            response.message = "Invalid operation type"
            self.get_logger().warn(response.message)
            return response

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

        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)

     
    rclpy.shutdown()

if __name__ == '__main__':
    main()