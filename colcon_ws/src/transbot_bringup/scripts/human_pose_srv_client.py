import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

class HumanPoseClient(Node):
    def __init__(self):
        super().__init__("human_pose_client_node")

        self.human_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/human_detection/pose_stamped',
            self.human_pose_sub_callback
        )

    def human_pose_sub_callback(self, msg):
        
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        msg = self.get_logger().info(f'{x:.2f}, {y:.2f}, {z:.2f}')
    
def main(args=None):
    rclpy.init(args=args)

    node = HumanPoseClient()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()