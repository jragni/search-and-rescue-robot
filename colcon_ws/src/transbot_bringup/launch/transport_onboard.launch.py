from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """On board data transports.
    
        Publishes compressed image and point cloud data
        for lower bandwidth.
    """

    rgb_image_transport_node = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='republish_onboard_color',
        remappings=[
            ('in', '/camera/color/image_raw'),
            ('out/compressed', '/camera/color/image_raw/compressed')
        ],
        arguments=['raw', 'compressed']
    )

    depth_image_transport_node = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='republish_onboard_depth',
        remappings=[
            ('in', '/camera/depth/image_raw'),
            ('out/compressed', '/camera/depth/image_raw/compressed')
        ],
        arguments=['raw', 'compressed']
    )

    return LaunchDescription([
        rgb_image_transport_node,
        depth_image_transport_node,
    ])