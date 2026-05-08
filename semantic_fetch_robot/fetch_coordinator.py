#!/usr/bin/env python3
"""
fetch_coordinator.py
Full fetch pipeline:
  1. Move arm to home (safe for navigation)
  2. Navigate to object location
  3. Execute grasp sequence (pregrasp → grasp → close → carry)
  4. Navigate back to start position
  5. Execute deliver sequence (lower → open gripper → home)

Object location and start position are hardcoded.
Adjust OBJECT_POSE and HOME_POSE to match your world setup.

Run:
  ros2 run semantic_fetch_robot fetch_coordinator
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import time
from semantic_fetch_robot.arm_controller import ArmController

OBJECT_APPROACH_POSE = {
    'x': 0.3,
    'y': -2.5,
    'yaw_w': 0.707,
    'yaw_z': -0.707,
}

HOME_POSE = {
    'x': -2.4,
    'y': -2.5,
    'yaw_w': 1.0,
    'yaw_z': 0.0,
}

# How long to wait for navigation goals (seconds)
NAV_TIMEOUT = 180.0



class FetchCoordinator(Node):
    def __init__(self):
        super().__init__('fetch_coordinator')

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._arm = ArmController()

        self.get_logger().info('FetchCoordinator initialized')
        self.get_logger().info(
            f'Object approach: x={OBJECT_APPROACH_POSE["x"]}, '
            f'y={OBJECT_APPROACH_POSE["y"]}')
        self.get_logger().info(
            f'Home delivery: x={HOME_POSE["x"]}, y={HOME_POSE["y"]}')

    def _make_pose(self, x, y, yaw_z=0.0, yaw_w=1.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = float(yaw_z)
        pose.pose.orientation.w = float(yaw_w)
        return pose

    def navigate_to(self, pose_dict: dict, label: str) -> bool:
        """
        Send a Nav2 NavigateToPose goal and wait for completion.
        Returns True on success, False on failure/timeout.
        """
        self.get_logger().info(f'Navigating to: {label}')

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(
            pose_dict['x'], pose_dict['y'],
            pose_dict.get('yaw_z', 0.0),
            pose_dict.get('yaw_w', 1.0)
        )

        self.get_logger().info(
            f'Sending goal: x={goal.pose.pose.position.x:.2f}, '
            f'y={goal.pose.pose.position.y:.2f}')

        send_goal_future = self._nav_client.send_goal_async(goal)

        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(
            self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return False

        self.get_logger().info('Goal accepted — robot navigating...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - start
            if elapsed > NAV_TIMEOUT:
                self.get_logger().error(
                    f'Navigation timeout after {NAV_TIMEOUT}s')
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached: {label}')
            return True
        else:
            self.get_logger().error(
                f'Navigation failed with status: {status}')
            return False

    def run(self):
        """Execute the full fetch mission."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('FETCH MISSION STARTED')
        self.get_logger().info('=' * 50)

        # Step 1 — Safe arm position for navigation
        self.get_logger().info('Step 1: Moving arm to home position')
        self._arm.move_to_pose('home')

        # Step 2 — Navigate to object
        self.get_logger().info('Step 2: Navigating to object')
        success = self.navigate_to(OBJECT_APPROACH_POSE, 'object location')
        if not success:
            self.get_logger().error('Failed to reach object — aborting mission')
            return False

        # Step 3 — Stop, let robot settle
        self.get_logger().info('Step 3: Robot arrived — settling for 2s')
        time.sleep(2.0)

        # Step 4 — Grasp sequence
        self.get_logger().info('Step 4: Executing grasp sequence')
        self._arm.full_grasp_sequence()

        # Step 5 — Navigate back to operator
        self.get_logger().info('Step 5: Navigating back to operator')
        success = self.navigate_to(HOME_POSE, 'home/operator position')
        if not success:
            self.get_logger().error(
                'Failed to return home — holding object in carry pose')
            return False

        # Step 6 — Deliver
        self.get_logger().info('Step 6: Delivering object')
        time.sleep(1.0)
        self._arm.full_deliver_sequence()

        self.get_logger().info('=' * 50)
        self.get_logger().info('FETCH MISSION COMPLETE')
        self.get_logger().info('=' * 50)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FetchCoordinator()

    try:
        success = node.run()
        if success:
            node.get_logger().info('Mission succeeded!')
        else:
            node.get_logger().error('Mission failed!')
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
