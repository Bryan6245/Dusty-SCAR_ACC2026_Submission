#!/usr/bin/env python3
import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__("goto_pose_once")
        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._feedback = None

    def feedback_cb(self, msg):
        self._feedback = msg.feedback

    def send_goal(self, x: float, y: float, yaw: float, frame_id: str = "map"):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info("Waiting for navigate_to_pose server...")
        self._client.wait_for_server()

        self.get_logger().info(
            f"Sending goal: frame={frame_id} x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        last_log = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._feedback and (time.time() - last_log) > 1.0:
                try:
                    dist = self._feedback.distance_remaining
                    nav_time = self._feedback.navigation_time.sec
                    self.get_logger().info(
                        f"distance_remaining={dist:.3f} nav_time={nav_time}s"
                    )
                except Exception:
                    pass
                last_log = time.time()

            if result_future.done():
                result = result_future.result()
                status = result.status

                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("Goal succeeded")
                    return True

                self.get_logger().error(f"Goal failed with status={status}")
                return False

        self.get_logger().error("ROS shutdown before result")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--frame", default="map")
    args = ap.parse_args()

    rclpy.init()
    node = GoToPoseClient()
    ok = node.send_goal(args.x, args.y, args.yaw, args.frame)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
