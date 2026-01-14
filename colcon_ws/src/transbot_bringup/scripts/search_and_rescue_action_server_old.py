#!/usr/bin/env python3

# Standard library
import math

# ROS2 core
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# ROS2 navigation
from nav2_simple_commander.robot_navigator import BasicNavigator

# TF2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Messages
from geometry_msgs.msg import PoseStamped

# Local
from transbot_bringup.helpers import get_approach_pose
from transbot_bringup.helpers import is_within_tolerance
from transbot_bringup.helpers import pose_to_tuple
from transbot_bringup.tasks import MissionTask
from transbot_msgs.action import SearchAndRescue


class SearchAndRescueActionServer(Node):
    '''ROS2 action server for autonomous search and rescue missions.

    Implements a state machine that:
    1. Navigates to search waypoints looking for humans
    2. Approaches detected humans outside safe zone
    3. Rescues victims (grasp with arm - TODO)
    4. Returns victims to base

    Subscribes:
        /human_detection/pose (PoseStamped): Detected human locations

    Actions:
        search_and_rescue_server (SearchAndRescue): Mission control
    '''

    def __init__(self) -> None:
        super().__init__('search_and_rescue_action_server')

        self.human_location_distance_tolerance = 0.15
        self.safezone_radius = 1.0
        self.current_task = MissionTask.NONE

        self.human_poses = set()  # Set of human poses in tuple form
        self.visited_human_poses = set()
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
            '/human_detection/pose',
            self.human_pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.nav_ = BasicNavigator()
        self.get_logger().info('Awaiting for Nav2 to become active...')
        self.nav_.waitUntilNav2Active()

        self.get_logger().info('Search and Rescue Action server ready!')


    def human_pose_callback(self, msg: PoseStamped) -> None:
        '''Process detected human pose and add to tracking set if valid.

        Filters out poses that:
        - Arrive when not in SEARCHING state
        - Are within the safe zone (home area)
        - Are already tracked or visited
        - Are within tolerance of existing poses (duplicate filtering)

        Args:
            msg: PoseStamped from human detection in camera frame
        '''
        try:
            # Transform the pose from camera_link to map frame
            transformed_pose = self.tf_buffer.transform(msg, 'map')
            human_pose_tuple = pose_to_tuple(transformed_pose)
            
            hx, hy, *_ = human_pose_tuple
            is_within_safe_zone = abs(hx) <= self.safezone_radius and abs(hy) <= self.safezone_radius
            is_not_searching = self.current_task != MissionTask.SEARCHING
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
                or is_within_safe_zone
                or is_pose_included
                or is_pose_visited
                or is_pose_within_tolerance
                or is_visited_within_tolerance
            ):
                return
            
            self.human_poses.add(human_pose_tuple)

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform pose: {ex}')


    def execute_callback(self, goal_handle) -> SearchAndRescue.Result:
        '''Execute search and rescue mission via state machine.

        State flow: SEARCHING -> APPROACHING_VICTIM -> RESCUING -> RETURNING_TO_BASE
        Repeats until all search poses visited and no humans remain.

        Args:
            goal_handle: Action goal with search_poses list

        Returns:
            SearchAndRescue.Result with success status
        '''
        result = SearchAndRescue.Result()
        feedback = SearchAndRescue.Feedback()
        self.search_poses = list(goal_handle.request.search_poses)

        if not self._validate_goal(goal_handle):
            result.success = False
            return result

        self.current_task = MissionTask.SEARCHING
        feedback.current_task = self.current_task
        goal_handle.publish_feedback(feedback)

        while self.search_poses:
            current_search_pose = self.search_poses.pop()
            self.nav_.goToPose(current_search_pose)

            self._handle_searching_state(goal_handle, feedback, current_search_pose)
            self._handle_approaching_state(goal_handle, feedback)
            self._handle_rescuing_state(goal_handle, feedback)
            self._handle_returning_state(goal_handle, feedback)

    def _validate_goal(self, goal_handle) -> bool:
        '''Validate that the goal contains search poses.

        Args:
            goal_handle: Action goal to validate

        Returns:
            True if valid (has search poses), False otherwise
        '''
        if not self.search_poses:
            self.get_logger().info('Mission Failed! No search poses given')
            return False
        return True

    def _handle_searching_state(self, goal_handle, feedback, current_search_pose: PoseStamped) -> None:
        '''Navigate to search pose while monitoring for human detections.

        If human detected, cancels navigation, switches to APPROACHING_VICTIM,
        and re-queues the interrupted search pose for later.

        Args:
            goal_handle: Action goal for publishing feedback
            feedback: Feedback message to update and publish
            current_search_pose: The waypoint being navigated to
        '''
        while not self.nav_.isTaskComplete() and self.current_task == MissionTask.SEARCHING:
            if len(self.human_poses):
                self.nav_.cancelTask()
                self.current_task = MissionTask.APPROACHING_VICTIM
                feedback.current_task = self.current_task
                goal_handle.publish_feedback(feedback)
                # add the current_search pose back since it was interrupted for a rescue
                self.search_poses.append(current_search_pose)

    def _handle_approaching_state(self, goal_handle, feedback) -> None:
        '''Navigate to approach pose near detected victim.

        Calculates approach pose 0.5m from victim facing them,
        navigates there, then transitions to RESCUING state.

        Args:
            goal_handle: Action goal for publishing feedback
            feedback: Feedback message to update and publish
        '''
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
                self.get_logger().info('approaching...', once=True)

            self.current_task = MissionTask.RESCUING
            feedback.current_task = self.current_task
            goal_handle.publish_feedback(feedback)

    def _handle_rescuing_state(self, goal_handle, feedback) -> None:
        '''Execute rescue of victim (placeholder).

        TODO: Implement centering/alignment with victim and arm grasp.
        Currently just transitions to RETURNING_TO_BASE.

        Args:
            goal_handle: Action goal for publishing feedback
            feedback: Feedback message to update and publish
        '''
        while self.current_task == MissionTask.RESCUING:
            # TODO implement centering/alignment with human detected in front
            # grasp victim may create a nested action?
            self.current_task = MissionTask.RETURNING_TO_BASE
            feedback.current_task = self.current_task
            goal_handle.publish_feedback(feedback)

    def _handle_returning_state(self, goal_handle, feedback) -> None:
        '''Navigate back to base and release victim.

        TODO: Implement victim release via arm control.
        Currently just navigates to origin and logs release.

        Args:
            goal_handle: Action goal for publishing feedback
            feedback: Feedback message to update and publish
        '''
        while self.current_task == MissionTask.RETURNING_TO_BASE:
            home_pose = PoseStamped()
            self.nav_.goToPose(home_pose)

            while not self.nav_.isTaskComplete():
                self.get_logger().info('Returning to base...', once=True)

            # TODO add release here
            self.get_logger().info('Releasing victim')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = SearchAndRescueActionServer()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

