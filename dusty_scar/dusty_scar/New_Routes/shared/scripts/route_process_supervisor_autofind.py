#!/usr/bin/env python3
import os, signal, subprocess, time, math, glob
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
    controller_py: str  # resolved absolute path at runtime
    linear_x: float = 0.12
    max_ang: float = 1.5
    rate_hz: float = 12.0


def newest_controller(shared_scripts_dir: Path, route_num: int) -> Path:
    pats = [
        f"bc_controller_route{route_num}_v*.py",
        f"bc_controller_route{route_num}_*.py",
    ]
    candidates = []
    for p in pats:
        candidates += glob.glob(str(shared_scripts_dir / p))
    candidates = [Path(c) for c in candidates if Path(c).is_file()]

    if not candidates:
        raise FileNotFoundError(f"No controller found for Route{route_num} in {shared_scripts_dir}")

    # newest by mtime
    candidates.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return candidates[0]


class RouteProcessSupervisor(Node):
    def __init__(self):
        super().__init__("route_process_supervisor")

        self.declare_parameter("start_index", 0)

        start_index = int(self.get_parameter("start_index").value)

        this_file = Path(__file__).resolve()
        # .../Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/shared/scripts/thisfile.py
        self.repo_root = this_file.parents[5]
        self.shared_scripts = self.repo_root / "dusty_scar/dusty_scar/New_Routes/shared/scripts"

        self.get_logger().info(f"Repo root: {self.repo_root}")
        self.get_logger().info(f"Shared scripts: {self.shared_scripts}")

        routes: List[RouteCfg] = []
        for n in range(1, 9):
            ctrl = newest_controller(self.shared_scripts, n)
            routes.append(RouteCfg(f"Route{n}", str(ctrl)))

        # Print chosen controllers (super helpful)
        for i, r in enumerate(routes):
            self.get_logger().info(f"[ROUTE {i}] {r.name} -> {r.controller_py}")

        self.routes = routes
        self.current_idx = max(0, min(start_index, len(self.routes) - 1))

        self.sub_cmd = self.create_subscription(Int32, "/route_cmd", self.on_route_cmd, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.on_odom, 20)

        self.proc: Optional[subprocess.Popen] = None
        self.last_xy = None
        self.route_dist_m = 0.0
        self.route_start_wall = time.time()

        self.start_route(self.current_idx)

    def on_route_cmd(self, msg: Int32):
        idx = int(msg.data)
        if idx < 0 or idx >= len(self.routes):
            self.get_logger().warn(f"Invalid route index {idx}. Must be 0..{len(self.routes)-1}")
            return
        if idx == self.current_idx:
            self.get_logger().info(f"Already on {self.routes[idx].name}")
            return
        self.get_logger().warn(f"Switch requested: {self.routes[self.current_idx].name} -> {self.routes[idx].name}")
        self.switch_to(idx)

    def on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_xy is None:
            self.last_xy = (x, y)
            return
        self.route_dist_m += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

    def start_route(self, idx: int):
        cfg = self.routes[idx]
        controller_path = Path(cfg.controller_py).resolve()
        if not controller_path.exists():
            self.get_logger().error(f"Controller not found: {controller_path}")
            return

        cmd = [
            "python3", str(controller_path),
            "--ros-args",
            "-p", f"linear_x:={cfg.linear_x}",
            "-p", f"max_ang:={cfg.max_ang}",
            "-p", f"rate_hz:={cfg.rate_hz}",
        ]

        self.get_logger().info(f"Starting {cfg.name}: {' '.join(cmd)}")
        self.proc = subprocess.Popen(cmd, cwd=str(self.repo_root), start_new_session=True)

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
        self.stop_route()
        time.sleep(0.2)
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

