#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from transbot_bringup.helpers import get_approach_pose, is_within_tolerance, pose_to_tuple
from transbot_bringup.tasks import MissionTask
from transbot_msgs.action import SearchAndRescue


class SearchAndRescueActionServer(Node):
    """Search and Rescue Action Server.
    
    This node will receive search poses
    """

    def __init__(self):
        super().__init__('search_and_rescue_action_server')

        self.human_location_distance_tolerance = 0.15
        self.current_task = MissionTask.NONE
        self.human_poses = {} # Set of human poses in tuple form
        self.visited_human_poses = {}
        self.search_poses = []  # list of PoseStamped
        self.visited_poses = {}

        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.action_server_ = ActionServer(
            self,
            SearchAndRescue,
            'search_and_rescue_server',
            self.execute_callback,
            callback_group=self.callback_group
        )

        self.human_poses_sub_ = self.create_subscription(
            PoseStamped,
            "/human_detection/pose",
            self.human_pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.nav_ = BasicNavigator()
        self.get_logger().info('Awaiting for Nav2 to become active...')
        self.nav_.waitUntilNav2Active()

        self.get_logger().info('Search and Rescue Action server ready!')


    def human_pose_callback(self, msg):
        """Handles human detection."""
        try:
            # Transform the pose from camera_link to map frame
            transformed_pose = self.tf_buffer.transform(msg, "map")
            human_pose_tuple = pose_to_tuple(transformed_pose)

            is_not_searching = self.current_task != [MissionTask.SEARCHING]
            is_pose_included = human_pose_tuple in self.human_poses
            is_pose_visited = human_pose_tuple in self.visited_human_poses
            is_pose_within_tolerance = is_within_tolerance(
                human_pose_tuple,
                self.human_poses,
                self.human_location_distance_tolerance
            )
            is_visited_within_tolerance = is_within_tolerance(
                human_pose_tuple,
                self.visited_human_poses,
                self.human_location_distance_tolerance
            )

            if (
                is_not_searching
                or is_pose_included
                or is_pose_visited
                or is_pose_within_tolerance
                or is_visited_within_tolerance
            ):
                return
            
            self.human_poses.add(human_pose_tuple)

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform pose: {ex}')


    def execute_callback(self, goal_handle):
        mission_result = SearchAndRescue.Result()
        mission_feedback_msg = SearchAndRescue.Feedback()
        self.search_poses = [*goal_handle.request.search_poses]

        if not self.search_poses:
            mission_result.success = False
            self.get_logger().info("Mission Failed! No search poses given")
            return mission_result
        
        self.current_task = MissionTask.SEARCHING
        mission_feedback_msg.current_task = self.current_task
        goal_handle.publish_feedback(mission_feedback_msg)

        while self.search_poses:
            current_search_pose = self.search_poses.pop()
            self.nav_.goToPose(current_search_pose)

            while not self.nav_.isTaskComplete() and self.current_task == MissionTask.SEARCHING:
                if len(self.human_poses):
                    self.nav_.cancelTask()
                    self.current_task = MissionTask.APPROACHING_VICTIM
                    mission_feedback_msg.current_task = self.current_task
                    goal_handle.publish_feedback(mission_feedback_msg)
                    # add the current_search pose back since it was interreupted for a rescue
                    self.search_poses.append(current_search_pose)
            
            while self.current_task == MissionTask.APPROACHING_VICTIM:
                victim_pose_tuple = self.human_poses.pop()
                robot_transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_footprint',
                    self.get_clock().now()
                )

                approach_pose = get_approach_pose(victim_pose_tuple, robot_transform)
                self.nav_.goToPose(approach_pose)
                while not self.nav_.isTaskComplete():
                    self.get_logger.info("approaching...", once=True)

                self.current_task = MissionTask.RESCUING
                mission_feedback_msg.current_task = self.current_task
                goal_handle.publish_feedback(mission_feedback_msg)
            
            while self.current_task == MissionTask.RESCUING:
                # TODO implement this
                # will need to center/align with human detected in front of it
                # grasp victim may create a nested action?
                self. current_task = MissionTask.RETURNING_TO_BASE
                mission_feedback_msg.current_task = self.current_task
                goal_handle.publish_feedback(mission_feedback_msg)

            
            while self.current_task == MissionTask.RETURNING_TO_BASE:
                home_pose = PoseStamped()
                self.nav_.goToPose(home_pose)

                while not self.nav_.isTaskComplete():
                    self.get_logger.info("Returning to base...", once=True)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = SearchAndRescueActionServer()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

