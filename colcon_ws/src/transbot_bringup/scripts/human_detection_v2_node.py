#!/usr/bin/env python3

import os
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from message_filters import Subscriber, TimeSynchronizer
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2


class HumanDetectionNode(Node):
    """Detects humans in image.

    This node is meant to be run offboard, so it will listen
    to the /camera/color/image_raw/transport message which is a
    decompressed message using image_transport.
    """

    def __init__(self):
        super().__init__("human_detection_node")
        self.MIN_THRESHOLD_SCORE = 0.74  # lowest score to consider detection

        transbot_bringup_path = get_package_share_directory('transbot_bringup')
        model_path = os.path.join(transbot_bringup_path, 'config', 'yolo11n.pt')
        self.model = YOLO(model_path)

        self.cv_bridge = CvBridge()

        self.rgb_sub_ = Subscriber(self, Image, '/camera/color/image_raw/transport')
        self.depth_sub_ = Subscriber(self, Image, '/camera/depth/image_raw')

        queue_size = 10
        self.sync_ = TimeSynchronizer([ self.rgb_sub_, self.depth_sub_] , queue_size)
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info("Staring Human Detection Node...")

    def synced_callback(self, img_msg, depth_img_msg):
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        depth_image = self.cv2_to_imgmsg(img_msg, 'passthrough')

        results = self.model(img)[0]

        # get results with only humans in it
        results_list = [
            r
            for r in results.boxes.data.tolist()
            if (r[5] == 0 and r[4] >= self.MIN_THRESHOLD_SCORE)
        ]

        if not results_list:
            self.img_pub_.publish(img_msg)
            return

        for result in results_list:
            x1, y1, x2, y2, score, class_id = result
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            distance = depth_image[y_center][x_center]

            cv2.rectangle(
                img,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2
            )

            cv2.putText(
                img,
                results.names[int(class_id)].upper() + f": {score:.2f} dist: {distance}",
                (int(x1), int(y1-10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.3,
                (0, 255, 0),
                1,
                cv2.LINE_AA
            )
        annotated_message = self.cv_bridge.cv2_to_imgmsg(img, "bgr8", Header())
        annotated_message.header.stamp = self.get_clock().now().to_msg()
        annotated_message.header.frame_id = "camera_link"

        self.img_pub_.publish(annotated_message)

if __name__ == "__main__":
    rclpy.init(args=None)
    human_detection_node = HumanDetectionNode()
    rclpy.spin(human_detection_node)

    human_detection_node.destroy_node()
    rclpy.shutdown()