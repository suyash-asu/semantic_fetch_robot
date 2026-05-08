#!/usr/bin/env python3
"""
navigate_to_goal.py (FIXED VERSION)
A complete ROS 2 action client for Nav2 navigation with proper result handling.

This node:
  - Accepts navigation goals (x, y coordinates)
  - Sends them to Nav2 via the NavigateToPose action
  - Waits for and monitors the result
  - Logs success/failure
  - Supports async/await patterns

Previous Issues Fixed:
  - ✗ OLD: send_goal_async() called but result never awaited (fire-and-forget bug)
  - ✓ NEW: Proper result future handling with callbacks or blocking wait
  - ✗ OLD: No timeout protection
  - ✓ NEW: Added configurable timeout with proper error handling
  - ✗ OLD: Silent failures (no feedback on navigation completion)
  - ✓ NEW: Logs all state transitions and final status

Usage:
  # Run the node
  ros2 run semantic_fetch_robot navigate_to_goal

  # Or use as a library in another node
  from semantic_fetch_robot.navigate_to_goal import NavigateToGoal
  nav = NavigateToGoal(node=my_node)
  success = nav.send_goal_and_wait(2.0, 1.0, timeout=30.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time


class NavigateToGoal(Node):
    """ROS 2 action client for Nav2 NavigateToPose."""
    
    def __init__(self, node_name: str = "navigate_to_goal"):
        """Initialize the NavigateToGoal client."""
        super().__init__(node_name)
        
        self._client = ActionClient(
            self, NavigateToPose, "navigate_to_pose")
        
        self.get_logger().info(
            "NavigateToGoal initialized — waiting for Nav2 action server...")

    def send_goal_and_wait(self, x: float, y: float, 
                          timeout: float = 60.0) -> bool:
        """
        Send a navigation goal and wait for the result (blocking).
        
        Args:
            x: Target X coordinate (map frame)
            y: Target Y coordinate (map frame)
            timeout: Maximum time to wait for navigation (seconds)
        
        Returns:
            True if navigation succeeded, False otherwise
        """
        self.get_logger().info(f"→ Sending goal: x={x:.2f}, y={y:.2f}")
        
        # Wait for Nav2 action server to be available
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "✗ Nav2 action server not available (timeout 10s)")
            return False
        
        # Create the goal
        goal = self._make_goal(x, y)
        
        # Send goal asynchronously
        self.get_logger().info("  Sending goal to Nav2...")
        send_goal_future = self._client.send_goal_async(goal)
        
        # Wait for goal to be accepted (blocking)
        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=10.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                "✗ Goal rejected by Nav2")
            return False
        
        self.get_logger().info("  ✓ Goal accepted by Nav2 — robot navigating...")
        
        # Wait for result (blocking)
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        
        while not result_future.done():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().error(
                    f"✗ Navigation timeout after {timeout:.1f}s")
                goal_handle.cancel_goal_async()
                return False
            
            # Spin briefly to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check result
        result = result_future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"✓ Navigation succeeded after {time.time() - start_time:.1f}s")
            return True
        else:
            self.get_logger().error(
                f"✗ Navigation failed with status: {status}")
            return False

    def send_goal_with_callback(self, x: float, y: float) -> bool:
        """
        Send a navigation goal with callback-based result handling (non-blocking).
        
        Args:
            x: Target X coordinate (map frame)
            y: Target Y coordinate (map frame)
        
        Returns:
            True if goal was accepted, False otherwise.
            Result status will be logged in callbacks (asynchronous).
        """
        self.get_logger().info(f"→ Sending goal (async): x={x:.2f}, y={y:.2f}")
        
        # Wait for Nav2 action server
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("✗ Nav2 action server not available")
            return False
        
        goal = self._make_goal(x, y)
        
        # Send goal with callbacks (non-blocking)
        send_goal_future = self._client.send_goal_async(
            goal,
            done_callback=self._goal_response_callback
        )
        
        self.get_logger().info(
            "  Goal sent asynchronously — callbacks will log result")
        return True

    def _goal_response_callback(self, future):
        """Callback when Nav2 responds to goal (accepted/rejected)."""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("✗ Goal rejected by Nav2")
            return
        
        self.get_logger().info("✓ Goal accepted — awaiting result...")
        
        # Set up result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Callback when navigation completes."""
        result = future.result()
        status = result.status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✓ Navigation succeeded!")
        else:
            self.get_logger().error(
                f"✗ Navigation failed with status: {status}")

    def _make_goal(self, x: float, y: float) -> NavigateToPose.Goal:
        """Create a NavigateToPose goal message."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        return goal


def main():
    """Example: Single navigation goal with blocking wait."""
    rclpy.init()
    node = NavigateToGoal()
    
    # Example 1: Blocking wait for result
    print("\n" + "="*60)
    print("Example 1: Blocking Navigation")
    print("="*60)
    success = node.send_goal_and_wait(x=2.0, y=1.0, timeout=60.0)
    print(f"Result: {'SUCCESS' if success else 'FAILED'}")
    
    # Example 2: Non-blocking with callback (requires spin)
    print("\n" + "="*60)
    print("Example 2: Non-Blocking Navigation (spinning for callbacks)")
    print("="*60)
    node.send_goal_with_callback(x=3.0, y=2.0)
    
    # Spin to process callbacks
    print("Spinning to process callbacks... (Ctrl+C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
