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
    imu_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.02',
            '--qx', '0',
            '--qy', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ]
    )

    # astra camera static transform
    rgbd_camera_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='astra_link_to_camera_link',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.02',
            '--qx', '0',
            '--qy', '0',
            '--qw', '1',
            '--frame-id', 'astra_link',
            '--child-frame-id', 'camera_link',
      ]
    )

    # lidar launch
    lidar_launch_path = os.path.join(get_package_share_directory('sllidar_ros2'))
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lidar_launch_path,
                'launch/sllidar_a1_launch.py'
            )
        )
    )

    # astra camera launch
    astra_launch_path = os.path.join(get_package_share_directory('orbbec_camera'))
    astra_launch_args = {
        "color_height": "480",
        "color_width": "640",
        "color_fps": "30",
        "depth_width": "320",
        "depth_height": "240",
        "depth_fps": "30",
        "depth_format": "Y11"
    }

    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(astra_launch_path, 'launch/astra.launch.py')),
        launch_arguments=[(key, str(value)) for key, value in astra_launch_args.items()],
    )

    # base node for odometry
    base_node = Node(
        package='transbot_bringup',
        executable='base_node',
        name='base_node',
        parameters=[{'linear_scale': 1.2}]
    )

    # robot state estimation
    bringup_package_path = get_package_share_directory("transbot_bringup")
    ekf_params_path = os.path.join(
        bringup_package_path,
        "config",
        "ekf_localization.yaml"
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_path],
        remappings=[("odometry/filtered", "odom")],
    )

    # rtab map test
    parameters={
        'frame_id':'base_footprint',
        'use_sim_time':False,
        'subscribe_rgbd':True,
        'subscribe_scan':True,
        'use_action_for_goal':True,
        'qos_scan': 2,
        'qos_image': 2,
        'qos_imu': 2,
        'Reg/Strategy':'1',
        'Reg/Force3DoF':'true',
        'RGBD/NeighborLinkRefining':'True',
        'Grid/RangeMin':'0.2',
        'Optimizer/GravitySigma':'0'
    }

    remappings=[
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('odom', '/odom')
    ]

    rtabmap_sync_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time':use_sim_time, 'qos':2 }],
        remappings=remappings
    )

    rtab_slam_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d'],
    )

    return LaunchDescription([
        declare_use_sim_time,
        bringup_node,
        base_node,
        description_launch,
        imu_static_transform_publisher_node,
        imu_filter_node,
        ekf_node,
        lidar_launch,
        rgbd_camera_static_transform_publisher_node,
        astra_launch,
        rtabmap_sync_node, # TODO temp test
        rtab_slam_node
    ])
