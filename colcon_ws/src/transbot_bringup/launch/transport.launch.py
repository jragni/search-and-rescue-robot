import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rgb_image_transport_remappings=[
        ('in', '/camera/color/image_raw'),
        ('out/compressed', '/camera/color/image_raw')
    ]

    rgb_image_transport_node = Node(
        package="image_transport",
        executable="republish",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=rgb_image_transport_remappings
    )

    return LaunchDescription([
        rgb_image_transport_node,
    ])