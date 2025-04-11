
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rtab_parameters={
        'frame_id':'base_footprint',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'use_sim_time':False,
        'subscribe_depth': True,
        'subscribe_rgbd':True,
        'subscribe_scan':True,
        'use_action_for_goal':True,
        #'qos_scan': 2,
        #'qos_image': 2,
        #'qos_imu': 2,
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
        parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time': False, 'qos':2 }],
        remappings=remappings
    )

    rtab_slam_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[rtab_parameters],
        remappings=remappings,
    )

    return LaunchDescription([
        rtab_slam_node,
        rtabmap_sync_node,
    ])