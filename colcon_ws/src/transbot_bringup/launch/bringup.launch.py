import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    xacro_file = os.path.join(description_pkg_path, 'urdf', 'transbot_astra.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    robot_state_params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_state_params]
    )

    # joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # imu calib node
    bringup_pkg_path = get_package_share_directory('transbot_bringup')
    apply_calib_params = {
        "calib_file": os.path.join(bringup_pkg_path, 'config', 'imu_calib.yaml'),
        "calibrate_gyros": True,
    }

    apply_calib_node = Node(
        package="imu_calib",
        executable="apply_calib",
        output="screen",
        parameters=[apply_calib_params]
    )

    # imu filter node
    imu_filter_params = {
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
            '--qy', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ]
        #arguments=['0.0', '0', '0.02', '0', '0', '0', '1', '/base_link', '/imu_link']
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
        bringup_node,
        base_node,
        apply_calib_node,
        imu_filter_node,
        static_transform_publisher_node,
        joint_state_publisher_node,
    ])
