import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch for nav2 and slam_toolbox"""

    bringup_path = os.path.join(get_package_share_directory('transbot_bringup'))

    # Launch SLAM toolbox
    online_async_config_path = os.path.join(bringup_path, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox_path = os.path.join(get_package_share_directory('slam_toolbox'))
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_path,'launch','online_async_launch.py')),
        launch_arguments=[{ 'slam_params_file': online_async_config_path, }.items()]
    )

    # Launch Nav2
    nav2_config_path = os.path.join(bringup_path, 'config', 'nav2_params.yaml')

    nav2_path = os.path.join(get_package_share_directory('nav2_bringup'))
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join( nav2_path, 'launch', 'navigation_launch.py')),
        launch_arguments=[{ 'params_file': nav2_config_path }.items()]
    )