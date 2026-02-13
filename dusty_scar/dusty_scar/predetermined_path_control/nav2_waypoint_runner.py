#!/usr/bin/env python3
"""
nav2_waypoint_runner.py

Two modes:
  1) record  - drive with teleop, record map-frame waypoints from TF (no RViz clicks)
  2) run     - send NavigateToPose goals to Nav2 from a JSON route

RECORD MODE (recommended for you):
  ros2 run dusty_scar nav2_waypoint_runner --ros-args \
    -p mode:=record \
    -p target_frame:=map \
    -p base_frame:=base_scan \
    -p output_route:=/ABS/PATH/routes/hub_to_pickup.json

  Commands (type + Enter):
    p = print current pose
    a = add waypoint (only if moved enough)
    u = undo last
    s = save + exit
    q = quit (no save)

RUN MODE:
  ros2 run dusty_scar nav2_waypoint_runner --ros-args \
    -p route_file:=/ABS/PATH/routes/hub_to_pickup.json
"""

import json
import math
import os
import threading
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


def yaw_deg_to_quat(yaw_deg: float) -> Tuple[float, float, float, float]:
    """Yaw (deg) -> Quaternion (x,y,z,w) assuming roll=pitch=0."""
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


def quat_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion -> yaw (deg)"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


def wrap_angle_deg(a: float) -> float:
    """Wrap to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


class Nav2WaypointRunner(Node):
    def __init__(self):
        super().__init__("nav2_waypoint_runner")

        # ---------- Common ----------
        self.declare_parameter("mode", "run")  # run | record

        # ---------- Run-mode ----------
        self.declare_parameter("route_file", "")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("frame_id", "map")

        # ---------- Record-mode ----------
        self.declare_parameter("output_route", "")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        # Record filters (reduce noise / duplicate points)
        self.declare_parameter("min_dist_m", 0.75)     # meters
        self.declare_parameter("min_yaw_deg", 15.0)    # degrees

        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.action_name = str(self.get_parameter("action_name").value).strip()
        self.default_frame_id = str(self.get_parameter("frame_id").value).strip()

        # ===========================
        # RECORD MODE
        # ===========================
        if self.mode == "record":
            self.target_frame = str(self.get_parameter("target_frame").value).strip()
            self.base_frame = str(self.get_parameter("base_frame").value).strip()
            self.min_dist_m = float(self.get_parameter("min_dist_m").value)
            self.min_yaw_deg = float(self.get_parameter("min_yaw_deg").value)

            out = str(self.get_parameter("output_route").value).strip()
            if not out:
                out = os.path.join(os.path.dirname(__file__), "routes", "demo_route.json")
            self.output_route = out

            # TF listener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self._last_pose: Optional[Tuple[float, float, float]] = None
            self._waypoints: List[Dict[str, Any]] = []
            self._shutdown_requested = False

            self.get_logger().info("=== RECORD MODE ===")
            self.get_logger().info(f"Recording TF: {self.target_frame} -> {self.base_frame}")
            self.get_logger().info(f"Output: {self.output_route}")
            self.get_logger().info(f"Filters: min_dist_m={self.min_dist_m:.2f}, min_yaw_deg={self.min_yaw_deg:.1f}")
            self.get_logger().info("Commands (type + Enter):")
            self.get_logger().info("  p = print current pose")
            self.get_logger().info("  a = add waypoint at current pose")
            self.get_logger().info("  u = undo last waypoint")
            self.get_logger().info("  s = save + exit")
            self.get_logger().info("  q = quit (no save)")

            self.pose_timer = self.create_timer(0.1, self._update_pose)
            self.shutdown_timer = self.create_timer(0.2, self._check_shutdown)

            t = threading.Thread(target=self._input_loop, daemon=True)
            t.start()
            return

        # ===========================
        # RUN MODE
        # ===========================
        route_file = str(self.get_parameter("route_file").value).strip()
        if not route_file:
            route_file = os.path.join(os.path.dirname(__file__), "routes", "hub_to_pickup.json")

        self.route = self._load_route(route_file)
        self.frame_id = str(self.route.get("frame_id", self.default_frame_id)).strip()
        self.waypoints = self.route.get("waypoints", [])

        if not self.waypoints:
            raise RuntimeError(f"No waypoints in route file: {route_file}")

        self.client = ActionClient(self, NavigateToPose, self.action_name)
        self.idx = 0
        self._wait_log_counter = 0

        self.get_logger().info(f"Route: {route_file}")
        self.get_logger().info(f"Action: {self.action_name}")
        self.get_logger().info(f"Frame: {self.frame_id}")
        self.get_logger().info(f"Waypoints: {len(self.waypoints)}")

        self.timer = self.create_timer(0.2, self._maybe_start)

    # ---------------- RUN MODE ----------------
    def _load_route(self, path: str) -> Dict[str, Any]:
        with open(path, "r") as f:
            return json.load(f)

    def _maybe_start(self):
        if self.client.wait_for_server(timeout_sec=0.0):
            self.timer.cancel()
            self.get_logger().info("Nav2 action server ready. Starting route.")
            self._send_next()
        else:
            self._wait_log_counter += 1
            if self._wait_log_counter % 10 == 0:
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

    def _infer_yaw_deg(self, idx: int) -> float:
        """If yaw isn't in JSON, infer yaw from direction to next (or prev if last)."""
        wp = self.waypoints[idx]
        x = float(wp["x"])
        y = float(wp["y"])

        if idx < len(self.waypoints) - 1:
            nxt = self.waypoints[idx + 1]
            dx = float(nxt["x"]) - x
            dy = float(nxt["y"]) - y
        else:
            prv = self.waypoints[idx - 1]
            dx = x - float(prv["x"])
            dy = y - float(prv["y"])

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        return math.degrees(math.atan2(dy, dx))

    def _send_next(self):
        if self.idx >= len(self.waypoints):
            self.get_logger().info("✅ Route complete.")
            rclpy.shutdown()
            return

        wp = self.waypoints[self.idx]
        x = float(wp["x"])
        y = float(wp["y"])

        if "yaw_deg" in wp:
            yaw = float(wp["yaw_deg"])
        else:
            yaw = self._infer_yaw_deg(self.idx)

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

    # ---------------- RECORD MODE ----------------
    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw_deg = quat_to_yaw_deg(r.x, r.y, r.z, r.w)
            self._last_pose = (float(t.x), float(t.y), float(yaw_deg))
        except Exception:
            self._last_pose = None

    def _input_loop(self):
        while not self._shutdown_requested:
            try:
                cmd = input().strip().lower()
            except EOFError:
                cmd = "q"

            if cmd == "p":
                self._print_pose()
            elif cmd == "a":
                self._add_waypoint()
            elif cmd == "u":
                self._undo()
            elif cmd == "s":
                self._save_and_exit()
            elif cmd == "q":
                self.get_logger().info("Quit (no save).")
                self._shutdown_requested = True
            elif cmd == "":
                continue
            else:
                self.get_logger().info("Commands: a(add), u(undo), p(print), s(save), q(quit)")

    def _print_pose(self):
        if not self._last_pose:
            self.get_logger().warn("No pose yet (TF not ready).")
            return
        x, y, yaw = self._last_pose
        self.get_logger().info(f"POSE: x={x:.3f}, y={y:.3f}, yaw={yaw:.1f} deg")

    def _add_waypoint(self):
        if not self._last_pose:
            self.get_logger().warn("No pose yet (TF not ready).")
            return

        x, y, yaw = self._last_pose

        if self._waypoints:
            last = self._waypoints[-1]
            lx = float(last["x"])
            ly = float(last["y"])
            lyaw = float(last.get("yaw_deg", yaw))

            dist = math.hypot(x - lx, y - ly)
            yaw_diff = abs(wrap_angle_deg(yaw - lyaw))

            if dist < self.min_dist_m and yaw_diff < self.min_yaw_deg:
                self.get_logger().info(
                    f"Skipped (too close): dist={dist:.2f}m yaw_diff={yaw_diff:.1f}deg"
                )
                return

        self._waypoints.append({"x": x, "y": y, "yaw_deg": yaw})
        self.get_logger().info(
            f"Added waypoint #{len(self._waypoints)}: x={x:.3f}, y={y:.3f}, yaw={yaw:.1f}"
        )

    def _undo(self):
        if not self._waypoints:
            self.get_logger().info("No waypoints to undo.")
            return
        wp = self._waypoints.pop()
        self.get_logger().info(f"Undid waypoint: {wp}")

    def _save_and_exit(self):
        os.makedirs(os.path.dirname(self.output_route), exist_ok=True)
        payload = {"frame_id": self.target_frame, "waypoints": self._waypoints}
        with open(self.output_route, "w") as f:
            json.dump(payload, f, indent=2)
        self.get_logger().info(f"✅ Saved {len(self._waypoints)} waypoints to {self.output_route}")
        self._shutdown_requested = True

    def _check_shutdown(self):
        if self._shutdown_requested:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Nav2WaypointRunner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
