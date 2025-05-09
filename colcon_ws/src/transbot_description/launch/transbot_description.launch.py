import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory('transbot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'transbot_astra.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use sim time if true'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])
