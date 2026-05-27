#!usr/bin/env python3

from math import atan2, cos, sin, pi
import numpy as np
import math

from geometry_msgs.msg import PoseStamped


def is_arm_within_limits(
    angles: list[float | int],
    x_min: float | int,
    y_min: float | int,
) -> bool:
    """Checks if the arm is within limits

    x = L1 * cos(t1) + L2 * cos(t1 + t2) >= x_min
    y = L1 * sin(t1) + L2 * sin(t1 + t2) >= y_min
    """
    # Horizontal is at 90
    t1 = (angles[0] - 90.0) * pi / 180.0
    # Horizontal is at 180
    t2 = (angles[1] - 180.0) * pi / 180.0

    # Measured links
    L1 = 0.08  # [cm]
    L2 = 0.14  # [cm]

    # Forward Kinematics
    x = L1 * cos(t1) + (L2 * cos(t1 + t2))
    y = L1 * sin(t1) + (L2 * sin(t1 + t2))

    print(f'x: {x}, y: {y}')
    return x >= x_min and y >= y_min


def correct_gyro(ax, ay, az):
    """correct the gyro readings.

    Since the IMU is mounted with the x facing 90 degrees clockwise
    about the z-axis, we need to adjust it
    """
    p = np.transpose(np.array([ax, ay, az]))
    rotation_matrix = [
        [cos(-pi / 2), sin(-pi / 2), 0.0],
        [-sin(-pi / 2), cos(-pi / 2), 0.0],
        [0.0, 0.0, 1.0]
    ]
    corrected = np.dot(rotation_matrix, p)
    return corrected[0], corrected[1], corrected[2]


def pose_to_tuple(pose) -> tuple:
    x = pose.pose.position.x
    y = pose.pose.position.y
    z = pose.pose.position.z
    qx = pose.pose.orientation.x
    qy = pose.pose.orientation.y
    qz = pose.pose.orientation.z
    qw = pose.pose.orientation.w

    return (x, y, z, qx, qy, qz, qw)


def tuple_to_pose(tuple_pose):
    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = tuple_pose[0]
    pose_stamped.pose.position.y = tuple_pose[1]
    pose_stamped.pose.position.z = tuple_pose[2]
    pose_stamped.pose.orientation.x = tuple_pose[3]
    pose_stamped.pose.orientation.y = tuple_pose[4]
    pose_stamped.pose.orientation.z = tuple_pose[5]
    pose_stamped.pose.orientation.w = tuple_pose[6]
    return pose_stamped

def get_approach_pose(victim_pose_tuple, robot_transform):
    # Extract robot position from transform
    robot_x = robot_transform.transform.translation.x
    robot_y = robot_transform.transform.translation.y
    
    # Extract victim position from tuple
    victim_x = victim_pose_tuple[0]
    victim_y = victim_pose_tuple[1]
    
    # Calculate vector from robot to victim
    direction_x = victim_x - robot_x
    direction_y = victim_y - robot_y
    
    # Normalize the direction vector
    length = math.sqrt(direction_x**2 + direction_y**2)
    direction_x = direction_x / length
    direction_y = direction_y / length
    
    # Calculate approach point (50cm away from victim)
    approach_x = victim_x - (direction_x * 0.5)
    approach_y = victim_y - (direction_y * 0.5)
    
    # Calculate yaw angle to face victim
    yaw = atan2(direction_y, direction_x)
    
    # Convert yaw to quaternion
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw/2)
    qw = math.cos(yaw/2)

    # Create pose tuple and convert to PoseStamped
    approach_pose_tuple = (approach_x, approach_y, 0.0, qx, qy, qz, qw)
    return tuple_to_pose(approach_pose_tuple)

def is_within_tolerance(human_pose_tuple, pose_set, tolerance):
    x, y, z, *_ = human_pose_tuple

    for human_pose in pose_set:
        location_x, location_y, location_z, *_ = human_pose

        delta_x = location_x - x
        delta_y = location_y - y
        delta_z = location_z - z
        point_to_location_distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        if point_to_location_distance <= tolerance:
            return True
    
    return False
