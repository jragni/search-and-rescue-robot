#!/usr/bin/env python3

import os

from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class HumanDetectionNode(Node):
    """Detect human using yolov11
    
    If the human is detected, it publishes its location to another.
    
    Should be offboard for performance.
    """
    def __init__(self):
        super().__init__('human_detection_node')

        #self.min_threshold = 0.74  # lowest score to consider detection
        self.declare_parameter('min_threshold_score', 0.74)
        self.min_threshold_score = self.get_parameter('min_threshold_score').value

        # ML model
        transbot_bringup_path = get_package_share_directory('transbot_bringup')
        model_path = os.path.join(transbot_bringup_path, 'config', 'yolo11n.pt')
        self.model = YOLO(model_path)

        # initialize cv_bridge
        self.cv_bridge = CvBridge()

        self.image_sub_ = self.create_subscription(
            Image,
            '/camera/color/image_raw/transport',
            self.image_sub_callback,
            qos=SensorDataQoS(),
        )

        self.human_detection_pub_ = self.create_publisher(
            Detection2DArray,
            '/human_detections',
            1
        )


    def image_sub_callback(self, msg):
        cv2_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        results_object = self.model(cv2_img)[0]

        human_results = [
            result
            for result in results_object.boxes.data.tolist()
            if result[5] == 0 and result[4] >= self.min_threshold_score
        ]

        # build DetectionArray
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = msg.header.frame_id

        if human_results:
            for x1, y1, x2, y2, score, class_id in human_results:
                detection = Detection2D()
                detection.header = detection_array.header

                # get bounding box
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                # classification
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.score = float(score)
                hypothesis.hypothesis.class_id = str(int(class_id))

                detection.results.append(hypothesis)

                detection_array.detections.append(detection)


        self.human_detection_pub_.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    human_detection_node = HumanDetectionNode()
    try:
        rclpy.spin(human_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        human_detection_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
