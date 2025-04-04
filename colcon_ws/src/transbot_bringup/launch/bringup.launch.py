import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    # bringup node
    bringup_node = Node(
        package="transbot_bringup",
        executable="transbot_driver.py",
        output="screen"
    )

    # Transbot urdf and robot state publisher 

    description_pkg_path = os.path.join(get_package_share_directory('transbot_description'))
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg_path,
                'launch/transbot_description.launch.py'
            )
        )
    )

    # imu filter node
    imu_filter_params = {
        'gain': 0.05,
        'constant_dt': 0.0,
        'fixed_frame': 'base_link',
        'use_mag': False,
        'publish_tf': False,
        'use_magnetic_field_msg': False,
        'world_frame': 'enu',
        'orientation_stddev': 0.05,
    }

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_params]
    )

    # static transform
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.2',
            '--qx', '0',
            '--qy', '0.7071',
            '--qw', '0.7071',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ]
    )

    # base node
    base_node = Node(
        package='transbot_bringup',
        executable='base_node',
        name='base_node',
        parameters=[{'linear_scale': 1.2}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        description_launch,
        bringup_node,
        base_node,
        imu_filter_node,
    ])
