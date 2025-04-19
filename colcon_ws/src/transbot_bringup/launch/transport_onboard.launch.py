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

    point_cloud_transport_launch = Node(
        package="point_cloud_transport",
        executable="republish",
        output="screen",
        parameters=[{
            "in_transport": "raw",
            "out_transport": "draco",
        }],
        remappings=[
            ('in', '/camera/depth/points'),
            ('out/draco', '/camera/depth/points/draco')
        ]
    )

    return LaunchDescription([
        rgb_image_transport_node,
        point_cloud_transport_launch,
    ])