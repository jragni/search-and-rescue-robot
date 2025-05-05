
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Off board data transports.
    
        Takes compressed images from on board republish and
        decompresses them to original message type.
    """

    rgb_image_transport_node = Node(
        package="image_transport",
        executable="republish",
        output="screen",
        name="republish_off_board_color",
        remappings=[
            ('in/compressed', '/camera/color/image_raw/compressed'),
            ('out', '/camera/color/image_raw/transport'),
        ],
        arguments=['compressed', 'raw']
    )

    point_cloud_transport_launch = Node(
        package="point_cloud_transport",
        executable="republish",
        output="screen",
        parameters=[{
            "in_transport": "draco",
            "out_transport": "raw",
        }],
        remappings=[
            ('in/draco', '/camera/depth/points/draco'),
            ('out', '/camera/depth/points/transport')
        ]
    )

    human_detection_node = Node(
        package="transbot_bringup",
        executable="human_detection_node.py",
        name="human_detection_node",
        output="screen"
    )

    human_poses_node = Node(
        package="transbot_bringup",
        executable="human_poses_node.py",
        name="human_poses_node",
        output="screen"
    )

    return LaunchDescription([
        rgb_image_transport_node,
        point_cloud_transport_launch,
        human_detection_node,
        human_poses_node,
    ])