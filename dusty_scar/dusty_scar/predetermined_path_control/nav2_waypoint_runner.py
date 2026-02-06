#!/usr/bin/env python3
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def yaw_deg_to_quat(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class Nav2WaypointRunner(Node):
    def __init__(self):
        super().__init__("nav2_waypoint_runner")

        self.declare_parameter("route_file", "")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")

        route_file = str(self.get_parameter("route_file").value).strip()
        self.action_name = str(self.get_parameter("action_name").value).strip()
        self.default_frame_id = str(self.get_parameter("frame_id").value).strip()

        if not route_file:
            route_file = os.path.join(os.path.dirname(__file__), "routes", "hub_to_pickup.json")

        self.route = self._load_route(route_file)
        self.frame_id = self.route.get("frame_id", self.default_frame_id)
        self.waypoints = self.route.get("waypoints", [])

        if not self.waypoints:
            raise RuntimeError(f"No waypoints in route file: {route_file}")

        self.client = ActionClient(self, NavigateToPose, self.action_name)
        self.idx = 0

        self.get_logger().info(f"Route: {route_file}")
        self.get_logger().info(f"Action: {self.action_name}")
        self.get_logger().info(f"Frame: {self.frame_id}")
        self.get_logger().info(f"Waypoints: {len(self.waypoints)}")

        self.timer = self.create_timer(0.2, self._maybe_start)

    def _load_route(self, path: str):
        with open(path, "r") as f:
            return json.load(f)

    def _maybe_start(self):
        if self.client.wait_for_server(timeout_sec=0.0):
            self.timer.cancel()
            self.get_logger().info("Nav2 action server ready. Starting route.")
            self._send_next()
        else:
            self.get_logger().info("Waiting for /navigate_to_pose...")

    def _make_goal(self, x: float, y: float, yaw_deg: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_deg_to_quat(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose
        return goal

    def _send_next(self):
        if self.idx >= len(self.waypoints):
            self.get_logger().info("✅ Route complete.")
            rclpy.shutdown()
            return

        wp = self.waypoints[self.idx]
        x = wp["x"]
        y = wp["y"]
        yaw = wp.get("yaw_deg", 0.0)

        self.get_logger().info(
            f"➡️  Waypoint {self.idx+1}/{len(self.waypoints)}: x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}"
        )
        goal_msg = self._make_goal(x, y, yaw)

        self.client.send_goal_async(goal_msg).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("✅ Accepted. Waiting for result...")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.get_logger().info(f"✅ Waypoint {self.idx+1} done (status={status}).")
        self.idx += 1
        self._send_next()


def main():
    rclpy.init()
    node = Nav2WaypointRunner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

