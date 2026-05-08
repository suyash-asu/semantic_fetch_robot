#!/usr/bin/env python3
"""
arm_controller.py
Publishes joint position commands to the 3-DOF arm + gripper
using Gazebo native joint position controllers.

Poses:
  home     — arm folded up, safe for navigation
  pregrasp — arm reaching forward-down, gripper open, above object
  grasp    — arm lowered to object level, gripper closes
  carry    — arm raised with object, safe for driving back

Usage:
  Import ArmController into fetch_coordinator.py, or
  run standalone for testing:
    ros2 run semantic_fetch_robot arm_controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


# ── Hardcoded joint poses (radians) ──────────────────────────────────────────
POSES = {
    "home": {
        # Arm fully upright — safe for navigation
        "joint1": 0.0,
        "joint2": 0.0,
        "joint3": 0.0,
        "left_finger": 0.0,
        "right_finger": 0.0,
    },
    "pregrasp": {
        # Gripper hovering ~15cm above ground, approaching object
        # Measured from Gazebo with Y-axis joints
        "joint1": 1.043,
        "joint2": 0.934,
        "joint3": 0.495,
        "left_finger": 0.03,
        "right_finger": 0.03,
    },
    "grasp": {
        # Gripper at ground level, touching object
        "joint1": 1.156,
        "joint2": 1.163,
        "joint3": 0.913,
        "left_finger": 0.03,
        "right_finger": 0.03,
    },
    "grasp_close": {
        # Same as grasp but gripper closed on object
        "joint1": 1.156,
        "joint2": 1.163,
        "joint3": 0.913,
        "left_finger": 0.0,
        "right_finger": 0.0,
    },
    "carry": {
        # Arm raised with object, safe for driving back
        "joint1": 0.827,
        "joint2": 0.527,
        "joint3": 0.299,
        "left_finger": 0.0,
        "right_finger": 0.0,
    },
}

# Settle times — how long to wait after sending command before next action
SETTLE_TIME = {
    'home': 3.0
    'pregrasp': 3.5
    'grasp': 3.0
    'grasp_close': 2.0
    'carry': 3.0
}


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller")

        self._pub_j1 = self.create_publisher(Float64, "/arm_joint1_cmd", 10)
        self._pub_j2 = self.create_publisher(Float64, "/arm_joint2_cmd", 10)
        self._pub_j3 = self.create_publisher(Float64, "/arm_joint3_cmd", 10)
        self._pub_lf = self.create_publisher(Float64, "/left_finger_cmd", 10)
        self._pub_rf = self.create_publisher(Float64, "/right_finger_cmd", 10)

        self.get_logger().info("ArmController ready")

    def _send(self, joint1, joint2, joint3, left_finger, right_finger):
        """Publish one set of joint commands."""
        self._pub_j1.publish(Float64(data=float(joint1)))
        self._pub_j2.publish(Float64(data=float(joint2)))
        self._pub_j3.publish(Float64(data=float(joint3)))
        self._pub_lf.publish(Float64(data=float(left_finger)))
        self._pub_rf.publish(Float64(data=float(right_finger)))

    def move_to_pose(self, pose_name: str, resend_count: int = 5,#10,
                     resend_interval: float = 0.2):#0.4):
        """
        Move arm to a named pose.
        Resends the command multiple times to overcome PID inertia.
        Blocks until settle time has elapsed.
        """
        if pose_name not in POSES:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            return False

        p = POSES[pose_name]
        self.get_logger().info(f"Moving arm to pose: {pose_name}")

        for _ in range(resend_count):
            self._send(p["joint1"], p["joint2"], p["joint3"],
                       p["left_finger"], p["right_finger"])
            time.sleep(resend_interval)

        settle = SETTLE_TIME.get(pose_name, 2.0)
        self.get_logger().info(
            f"Waiting {settle}s for arm to settle...")
        time.sleep(settle)
        self.get_logger().info(f"Pose {pose_name} reached")
        time.sleep(0.5)
        return True

    def open_gripper(self):
        """Open gripper fully."""
        self.get_logger().info("Opening gripper")
        for _ in range(5):
            self._pub_lf.publish(Float64(data=0.03))
            self._pub_rf.publish(Float64(data=0.03))
            time.sleep(0.2)
        time.sleep(0.5)

    def close_gripper(self):
        """Close gripper to grasp."""
        self.get_logger().info("Closing gripper")
        for _ in range(5):
            self._pub_lf.publish(Float64(data=0.0))
            self._pub_rf.publish(Float64(data=0.0))
            time.sleep(0.2)
        time.sleep(0.5)

    def full_grasp_sequence(self):
        """
        Execute the complete grasp sequence:
        pregrasp → grasp (open) → close gripper → carry
        """
        self.get_logger().info("Starting grasp sequence")
        self.move_to_pose("pregrasp")
        self.move_to_pose("grasp")
        self.close_gripper()
        time.sleep(0.5)
        self.move_to_pose("carry")
        self.get_logger().info("Grasp sequence complete")

    def full_deliver_sequence(self):
        """
        Execute the delivery sequence:
        carry → grasp position → open gripper → home
        """
        self.get_logger().info("Starting deliver sequence")
        self.move_to_pose("grasp")
        self.open_gripper()
        time.sleep(0.5)
        self.move_to_pose("home")
        self.get_logger().info("Deliver sequence complete")


def main(args=None):
    """Standalone test — cycles through all poses."""
    rclpy.init(args=args)
    node = ArmController()

    import sys
    if len(sys.argv) > 1:
        pose = sys.argv[1]
        node.move_to_pose(pose)
    else:
        # Demo: full fetch and return cycle
        node.get_logger().info("Running demo grasp sequence...")
        node.move_to_pose("home")
        node.full_grasp_sequence()
        node.get_logger().info("Demo complete — arm in carry pose")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
