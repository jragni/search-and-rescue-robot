import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.human_detection_sub_ = self.create_subscription(
           Detection2DArray,
           '/human_detections',
           10
        )

        # sub to rgb-d


    def human_detections_callback(self, human_detections_msg):
        # extract data from message

        # loop through each message and check if current position is within the space

        # check to ensure human detected isn't double counted

        # check to see if human 
