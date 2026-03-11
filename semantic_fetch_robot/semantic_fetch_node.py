import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SemanticFetchNode(Node):

    def __init__(self):
        super().__init__("semantic_fetch_node")

        self.publisher_ = self.create_publisher(String, "fetch_status", 10)

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info("Semantic Fetch Robot Node Initialized")

    def timer_callback(self):
        msg = String()
        msg.data = "Robot ready to fetch objects"
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing status message")


def main(args=None):
    rclpy.init(args=args)

    node = SemanticFetchNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
