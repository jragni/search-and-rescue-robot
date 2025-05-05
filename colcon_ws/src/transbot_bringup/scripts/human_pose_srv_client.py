import rclpy
from rclpy.node import Node

class HumanPoseClient(Node):
    def __init__(self):
        super().__init__("human_pose_client_node")
