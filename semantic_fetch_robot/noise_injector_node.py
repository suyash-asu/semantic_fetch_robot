import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

class NoiseInjectorNode(Node):
    def __init__(self):
        super().__init__("noise_injector_node")
        self.declare_parameter("lidar_noise_std", 0.02)
        self.declare_parameter("odom_noise_std", 0.005)
        self.lidar_std = self.get_parameter("lidar_noise_std").value
        self.odom_std = self.get_parameter("odom_noise_std").value

        self.scan_sub = self.create_subscription(
            LaserScan, "/bcr_bot/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/bcr_bot/odom", self.odom_callback, 10)

        self.scan_pub = self.create_publisher(LaserScan, "/scan_noisy", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom_noisy", 10)
        self.get_logger().info("Noise injector node started")

    def scan_callback(self, msg):
        noisy = LaserScan()
        noisy.header = msg.header
        noisy.angle_min = msg.angle_min
        noisy.angle_max = msg.angle_max
        noisy.angle_increment = msg.angle_increment
        noisy.time_increment = msg.time_increment
        noisy.scan_time = msg.scan_time
        noisy.range_min = msg.range_min
        noisy.range_max = msg.range_max
        ranges = np.array(msg.ranges)
        noise = np.random.normal(0, self.lidar_std, ranges.shape)
        noisy.ranges = (ranges + noise).tolist()
        self.scan_pub.publish(noisy)

    def odom_callback(self, msg):
        noisy = msg
        noisy.pose.pose.position.x += np.random.normal(0, self.odom_std)
        noisy.pose.pose.position.y += np.random.normal(0, self.odom_std)
        self.odom_pub.publish(noisy)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(NoiseInjectorNode())

if __name__ == "__main__":
    main()
