#!/usr/bin/env python3
import csv
import os
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener

# Headless plotting (no GUI window)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class SpeedLogger(Node):
    def __init__(self):
        super().__init__("speed_logger")

        # DO NOT declare use_sim_time here (ROS may already declare it)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("odom_topic", "/odom")  # optional (may not publish)
        self.declare_parameter("out_dir", "")
        self.declare_parameter("sample_hz", 20.0)

        # TF frames to compute speed from motion
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("base_frame", "base_scan")  # change to base_link if needed

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.sample_hz = float(self.get_parameter("sample_hz").value)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        out_dir = str(self.get_parameter("out_dir").value).strip()
        if not out_dir:
            out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "logs"))
        os.makedirs(out_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(out_dir, f"speed_log_{ts}.csv")

        # Save ONLY one png on exit
        self.png_path = os.path.join(out_dir, "speed_plot_final.png")

        # Data
        self.t = []
        self.cmd_vx = []
        self.meas_speed = []   # from TF (or odom if you want later)
        self._last_cmd = (0.0, 0.0)  # vx, wz

        # Timing / pose for TF speed
        self._t0 = None
        self._prev_xy = None
        self._prev_time = None

        # CSV
        self._csv = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._csv)
        self._writer.writerow(["t_sec", "cmd_vx", "cmd_wz", "meas_speed_tf"])

        # Subscriptions
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd, 10)

        # Optional: keep this subscription if /odom ever starts publishing
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Sample timer (TF speed)
        period = 1.0 / max(self.sample_hz, 1.0)
        self.create_timer(period, self._sample_tf)

        self.get_logger().info(f"Logging CSV: {self.csv_path}")
        self.get_logger().info(f"Will save ONE PNG on exit: {self.png_path}")
        self.get_logger().info(f"cmd_vel: {self.cmd_vel_topic}")
        self.get_logger().info(f"TF speed: {self.target_frame} -> {self.base_frame}")
        self.get_logger().info(f"(Note: /odom may not publish in your current stack; TF is used.)")

    def _now_sec(self) -> float:
        # Prefer ROS time if sim time is active; else fallback monotonic
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if ros_now > 1.0:
            return ros_now
        return time.monotonic()

    def _on_cmd(self, msg: Twist):
        self._last_cmd = (float(msg.linear.x), float(msg.angular.z))

    def _on_odom(self, msg: Odometry):
        # Not required, but useful for debugging:
        # If /odom starts publishing you can compare.
        pass

    def _sample_tf(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, self.base_frame, Time())
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
        except Exception:
            # TF not ready yet; just wait silently
            return

        now_abs = self._now_sec()
        if self._t0 is None:
            self._t0 = now_abs
            self._prev_xy = (x, y)
            self._prev_time = now_abs
            return

        dt = now_abs - self._prev_time
        if dt <= 0.0:
            return

        dx = x - self._prev_xy[0]
        dy = y - self._prev_xy[1]
        speed = math.hypot(dx, dy) / dt

        t_rel = now_abs - self._t0
        cmd_vx, cmd_wz = self._last_cmd

        self.t.append(t_rel)
        self.cmd_vx.append(cmd_vx)
        self.meas_speed.append(speed)

        self._writer.writerow([f"{t_rel:.3f}", f"{cmd_vx:.3f}", f"{cmd_wz:.3f}", f"{speed:.3f}"])
        self._csv.flush()

        self._prev_xy = (x, y)
        self._prev_time = now_abs

    def save_final_png(self):
        # Save once, only if we have data
        if len(self.t) < 2:
            return False

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("Velocity Tracking")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("speed (m/s)")
        ax.plot(self.t, self.cmd_vx, label="cmd_vx (/cmd_vel_nav)")
        ax.plot(self.t, self.meas_speed, label="measured speed (TF)")
        ax.legend()
        fig.savefig(self.png_path, dpi=150)
        plt.close(fig)
        return True


def main():
    rclpy.init()
    node = SpeedLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save PNG BEFORE shutting down ROS context
        saved = False
        try:
            saved = node.save_final_png()
        except Exception:
            saved = False

        try:
            node._csv.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

        # No ROS logging after shutdown; use plain print if you want:
        if saved:
            print(f"[speed_logger] Saved PNG: {node.png_path}")
        else:
            print("[speed_logger] No samples collected; PNG not saved.")


if __name__ == "__main__":
    main()
