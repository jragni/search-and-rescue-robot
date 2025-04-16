
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Off board data transports.
    
        Takes compressed images from on board republish and
        decompresses them to original message type.
    """

    rgb_image_transport_node = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='republish_off_board_color',
        remappings=[
            ('in/compressed', '/camera/color/image_raw/compressed'),
            ('out', '/camera/color/image_raw/transport'),
        ],
        arguments=['compressed', 'raw']
    )

    depth_image_transport_node = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='republish_off_board_depth',
        remappings=[
            ('in/compressed', '/camera/depth/image_raw/compressed'),
            ('out', '/camera/depth/image_raw/transport'),
        ],
        arguments=['compressed', 'raw']
    )

    ir_image_transport_node = Node(
        package='image_transport',
        executable='republish',
        output='screen',
        name='republish_off_board_ir',
        remappings=[
            ('in/compressed', '/camera/depth/compressed'),
            ('out', '/camera/ir/image_raw/transport'),
        ],
        arguments=['compressed', 'raw']
    )

    return LaunchDescription([
        rgb_image_transport_node,
        depth_image_transport_node,
        ir_image_transport_node,
    ])