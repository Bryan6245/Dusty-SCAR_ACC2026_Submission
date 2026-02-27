#!/usr/bin/env python3
import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class StopYieldSupervisor(Node):
    def __init__(self):
        super().__init__("stop_yield_supervisor")

        self.declare_parameter("cmd_in", "/cmd_vel_bc")
        self.declare_parameter("cmd_out", "/cmd_vel_nav")
        self.declare_parameter("dets_json", "/yolo/detections_json")

        # STOP (your tuned behavior)
        self.declare_parameter("stop_hold_s", 3.0)
        self.declare_parameter("stop_cooldown_s", 10.0)
        self.declare_parameter("stop_h", 0.15)
        self.declare_parameter("stop_min_conf", 0.45)
        self.declare_parameter("stop_frames_required", 2)

        # YIELD (slow down 25% while near and visible)
        self.declare_parameter("yield_scale", 0.75)
        self.declare_parameter("yield_h", 0.10)        # “early-ish” distance like your first stop test
        self.declare_parameter("yield_min_conf", 0.40)
        self.declare_parameter("yield_latch_s", 0.6)   # brief persistence to avoid flicker

        self.cmd_in = self.get_parameter("cmd_in").value
        self.cmd_out = self.get_parameter("cmd_out").value
        self.dets_json = self.get_parameter("dets_json").value

        self.stop_hold_s = float(self.get_parameter("stop_hold_s").value)
        self.stop_cooldown_s = float(self.get_parameter("stop_cooldown_s").value)
        self.stop_h = float(self.get_parameter("stop_h").value)
        self.stop_min_conf = float(self.get_parameter("stop_min_conf").value)
        self.stop_frames_required = int(self.get_parameter("stop_frames_required").value)

        self.yield_scale = float(self.get_parameter("yield_scale").value)
        self.yield_h = float(self.get_parameter("yield_h").value)
        self.yield_min_conf = float(self.get_parameter("yield_min_conf").value)
        self.yield_latch_s = float(self.get_parameter("yield_latch_s").value)

        self.last_cmd = Twist()
        self.last_dets = None
        self.last_dets_t = 0.0

        self.stop_until = 0.0
        self.cooldown_until = 0.0
        self.close_count = 0

        self.last_yield_seen = 0.0
        self.last_dbg = 0.0

        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)
        self.sub_det = self.create_subscription(String, self.dets_json, self.on_det, 10)
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info(f"StopYieldSupervisor cmd_in={self.cmd_in} cmd_out={self.cmd_out}")
        self.get_logger().info(f"STOP: h={self.stop_h} hold={self.stop_hold_s}s cooldown={self.stop_cooldown_s}s")
        self.get_logger().info(f"YIELD: h={self.yield_h} scale={self.yield_scale} latch={self.yield_latch_s}s")

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def on_det(self, msg: String):
        try:
            self.last_dets = json.loads(msg.data)
            self.last_dets_t = time.time()
        except Exception:
            pass

    def tick(self):
        now = time.time()

        out = Twist()
        out.linear.x = float(self.last_cmd.linear.x)
        out.angular.z = float(self.last_cmd.angular.z)

        # stop hold
        if now < self.stop_until:
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.pub.publish(out)
            return

        # stale dets -> passthrough
        if self.last_dets is None or (now - self.last_dets_t) > 1.0:
            self.pub.publish(out)
            return

        h_img = float(self.last_dets.get("h", 1.0))
        dets = self.last_dets.get("dets", [])

        def hr(xyxy):
            x1, y1, x2, y2 = xyxy
            return max(0.0, (float(y2) - float(y1)) / max(h_img, 1.0))

        # best stop sign
        best_stop = 0.0
        for d in dets:
            if d.get("cls") != "Stop Sign":
                continue
            if float(d.get("conf", 0.0)) < self.stop_min_conf:
                continue
            best_stop = max(best_stop, hr(d["xyxy"]))

        # best yield sign
        best_yield = 0.0
        for d in dets:
            if d.get("cls") != "Yield Sign":
                continue
            if float(d.get("conf", 0.0)) < self.yield_min_conf:
                continue
            best_yield = max(best_yield, hr(d["xyxy"]))

        # debug every 1s
        if now - self.last_dbg > 1.0:
            self.last_dbg = now
            self.get_logger().info(f"STOP_hr={best_stop:.3f} count={self.close_count} | YIELD_hr={best_yield:.3f}")

        # STOP trigger (same logic as before)
        if best_stop >= self.stop_h and now >= self.cooldown_until:
            self.close_count += 1
        else:
            self.close_count = 0

        if self.close_count >= self.stop_frames_required:
            self.stop_until = now + self.stop_hold_s
            self.cooldown_until = now + self.stop_cooldown_s
            self.close_count = 0
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.get_logger().warn("STOP SIGN TRIGGERED -> hold 3s")
            self.pub.publish(out)
            return

        # YIELD slow while near OR recently seen (latch)
        if best_yield >= self.yield_h:
            self.last_yield_seen = now

        if (now - self.last_yield_seen) < self.yield_latch_s:
            out.linear.x *= self.yield_scale

        self.pub.publish(out)

def main():
    rclpy.init()
    node = StopYieldSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()
