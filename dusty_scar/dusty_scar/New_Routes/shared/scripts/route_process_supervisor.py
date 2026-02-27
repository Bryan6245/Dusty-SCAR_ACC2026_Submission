#!/usr/bin/env python3
import os
import signal
import subprocess
import time
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


@dataclass
class RouteCfg:
    name: str
    controller_py: str           # path relative to repo root
    linear_x: float = 0.12
    max_ang: float = 1.5
    rate_hz: float = 12.0

    # Auto-switch conditions (pick one or use both)
    switch_after_m: float = 0.0  # 0 disables distance switching
    switch_after_s: float = 0.0  # 0 disables time switching


def yaw_from_quat(q) -> float:
    # Not strictly needed, but handy if you extend later
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RouteProcessSupervisor(Node):
    def __init__(self):
        super().__init__("route_process_supervisor")

        # --- Params (can be overridden by ros-args -p ...)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("publish_zero_on_switch", False)  # keep False to preserve "one publisher" principle
        self.declare_parameter("min_seconds_before_switch", 2.0)  # avoid instant switch at startup
        self.declare_parameter("status_print_period_s", 1.0)

        self.start_index = int(self.get_parameter("start_index").value)
        self.publish_zero_on_switch = bool(self.get_parameter("publish_zero_on_switch").value)
        self.min_seconds_before_switch = float(self.get_parameter("min_seconds_before_switch").value)
        self.status_print_period_s = float(self.get_parameter("status_print_period_s").value)

        # Repo root auto-detect (…/Dusty-SCAR_ACC2026_Submission)
        this_file = Path(__file__).resolve()
        self.repo_root = this_file.parents[5]

        # ---- EDIT THESE to match your real controller filenames + desired chaining ----
        self.routes: List[RouteCfg] = [
            RouteCfg("Route1", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route1_v8.py",
                     switch_after_m=18.0),
            RouteCfg("Route2", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route2_v2.py",
                     switch_after_m=18.0),
            RouteCfg("Route3", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route3_v?.py",
                     switch_after_m=18.0),
            RouteCfg("Route4", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route4_v5.py",
                     switch_after_m=18.0),
            RouteCfg("Route5", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route5_v?.py",
                     switch_after_m=18.0),
            RouteCfg("Route6", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route6_v4.py",
                     switch_after_m=18.0),
            RouteCfg("Route7", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route7_v8.py",
                     switch_after_m=18.0),
            RouteCfg("Route8", "dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route8_v5.py",
                     switch_after_m=18.0),
        ]

        # Manual override topic: publish an Int32 index (0..N-1) to /route_cmd
        self.sub_cmd = self.create_subscription(Int32, "/route_cmd", self.on_route_cmd, 10)

        # Odom for distance integration
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 20)

        # Optional: publish zeros on switch (if enabled)
        if self.publish_zero_on_switch:
            from geometry_msgs.msg import Twist
            self._twist_type = Twist
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        else:
            self.pub_cmd = None

        self.current_idx = max(0, min(self.start_index, len(self.routes) - 1))
        self.proc: Optional[subprocess.Popen] = None

        self.last_xy = None
        self.route_dist_m = 0.0
        self.route_start_wall = time.time()

        self._last_status_print = 0.0
        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info(f"Repo root: {self.repo_root}")
        self.start_route(self.current_idx)

    def on_route_cmd(self, msg: Int32):
        idx = int(msg.data)
        if idx < 0 or idx >= len(self.routes):
            self.get_logger().warn(f"Invalid route index {idx}. Must be 0..{len(self.routes)-1}")
            return
        if idx == self.current_idx:
            self.get_logger().info(f"Already on {self.routes[idx].name}")
            return
        self.get_logger().warn(f"Manual switch requested: {self.routes[self.current_idx].name} -> {self.routes[idx].name}")
        self.switch_to(idx)

    def on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_xy is None:
            self.last_xy = (x, y)
            return
        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        self.route_dist_m += math.hypot(dx, dy)
        self.last_xy = (x, y)

    def tick(self):
        now = time.time()
        elapsed = now - self.route_start_wall
        cfg = self.routes[self.current_idx]

        # Periodic status
        if now - self._last_status_print >= self.status_print_period_s:
            self._last_status_print = now
            self.get_logger().info(
                f"[ACTIVE] idx={self.current_idx} {cfg.name} | dist={self.route_dist_m:.2f} m | t={elapsed:.1f} s"
            )

        # Don’t allow instant switching
        if elapsed < self.min_seconds_before_switch:
            return

        # Auto switching logic (distance/time)
        dist_hit = (cfg.switch_after_m > 0.0 and self.route_dist_m >= cfg.switch_after_m)
        time_hit = (cfg.switch_after_s > 0.0 and elapsed >= cfg.switch_after_s)

        if dist_hit or time_hit:
            next_idx = (self.current_idx + 1) % len(self.routes)
            self.get_logger().warn(
                f"AUTO switch: {cfg.name} -> {self.routes[next_idx].name} "
                f"(dist_hit={dist_hit}, time_hit={time_hit})"
            )
            self.switch_to(next_idx)

    def start_route(self, idx: int):
        cfg = self.routes[idx]
        controller_path = (self.repo_root / cfg.controller_py).resolve()
        if not controller_path.exists():
            self.get_logger().error(f"Controller not found: {controller_path}")
            self.get_logger().error("Fix the filename in route_process_supervisor.py and re-run.")
            return

        cmd = [
            "python3", str(controller_path),
            "--ros-args",
            "-p", f"linear_x:={cfg.linear_x}",
            "-p", f"max_ang:={cfg.max_ang}",
            "-p", f"rate_hz:={cfg.rate_hz}",
        ]

        self.get_logger().info(f"Starting {cfg.name}: {' '.join(cmd)}")
        # New process group so we can SIGINT/kill cleanly
        self.proc = subprocess.Popen(
            cmd,
            cwd=str(self.repo_root),
            start_new_session=True,
        )

        self.current_idx = idx
        self.route_start_wall = time.time()
        self.route_dist_m = 0.0
        self.last_xy = None

    def stop_route(self):
        if self.proc is None:
            return
        if self.proc.poll() is not None:
            self.proc = None
            return

        self.get_logger().info("Stopping active controller (SIGINT)...")
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
        except ProcessLookupError:
            self.proc = None
            return

        # Wait briefly, then force kill if needed
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if self.proc.poll() is not None:
                self.proc = None
                return
            time.sleep(0.05)

        self.get_logger().warn("Controller didn’t exit on SIGINT; killing...")
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc = None

    def switch_to(self, idx: int):
        # Optional safety: publish a couple zeros (only if enabled)
        if self.pub_cmd is not None:
            z = self._twist_type()
            z.linear.x = 0.0
            z.angular.z = 0.0
            for _ in range(3):
                self.pub_cmd.publish(z)
                time.sleep(0.05)

        self.stop_route()
        time.sleep(0.2)  # small gap to avoid overlap
        self.start_route(idx)

    def destroy_node(self):
        self.get_logger().info("Shutting down supervisor; stopping controller...")
        self.stop_route()
        super().destroy_node()


def main():
    rclpy.init()
    node = RouteProcessSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

