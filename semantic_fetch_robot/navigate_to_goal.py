import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__("navigate_to_goal")
        self._client = ActionClient(
            self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self._client.wait_for_server()
        self._client.send_goal_async(goal)
        self.get_logger().info(f"Navigating to x={x}, y={y}")

def main():
    rclpy.init()
    node = NavigateToGoal()
    node.send_goal(2.0, 1.0)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
