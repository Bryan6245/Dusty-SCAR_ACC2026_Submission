#!/usr/bin/env python3
import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class StopSignSupervisor(Node):
    def __init__(self):
        super().__init__("stop_sign_supervisor")

        self.declare_parameter("cmd_in", "/cmd_vel_bc")
        self.declare_parameter("cmd_out", "/cmd_vel_nav")
        self.declare_parameter("dets_json", "/yolo/detections_json")

        self.declare_parameter("stop_hold_s", 3.0)
        self.declare_parameter("stop_cooldown_s", 10.0)

        # bbox height ratio trigger (close distance proxy)
        self.declare_parameter("stop_h", 0.15)   # start here; tune if needed
        self.declare_parameter("min_conf", 0.45)
        self.declare_parameter("frames_required", 2)

        self.cmd_in = self.get_parameter("cmd_in").value
        self.cmd_out = self.get_parameter("cmd_out").value
        self.dets_json = self.get_parameter("dets_json").value

        self.stop_hold_s = float(self.get_parameter("stop_hold_s").value)
        self.stop_cooldown_s = float(self.get_parameter("stop_cooldown_s").value)
        self.stop_h = float(self.get_parameter("stop_h").value)
        self.min_conf = float(self.get_parameter("min_conf").value)
        self.frames_required = int(self.get_parameter("frames_required").value)

        self.last_cmd = Twist()
        self.last_dets = None
        self.last_dets_t = 0.0

        self.stop_until = 0.0
        self.cooldown_until = 0.0
        self.close_count = 0

        self.sub_cmd = self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)
        self.sub_det = self.create_subscription(String, self.dets_json, self.on_det, 10)
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)

        self.last_dbg = 0.0
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(f"StopSignSupervisor cmd_in={self.cmd_in} cmd_out={self.cmd_out} dets={self.dets_json}")
        self.get_logger().info(f"Params: stop_h={self.stop_h} hold={self.stop_hold_s}s cooldown={self.stop_cooldown_s}s min_conf={self.min_conf}")

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

        # hold stop
        if now < self.stop_until:
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.pub.publish(out)
            return

        # stale detections -> passthrough
        if self.last_dets is None or (now - self.last_dets_t) > 1.0:
            self.pub.publish(out)
            return

        h_img = float(self.last_dets.get("h", 1.0))
        dets = self.last_dets.get("dets", [])

        best_hr = 0.0
        best_conf = 0.0
        for d in dets:
            if d.get("cls") != "Stop Sign":
                continue
            conf = float(d.get("conf", 0.0))
            if conf < self.min_conf:
                continue
            x1, y1, x2, y2 = d["xyxy"]
            hr = max(0.0, (float(y2) - float(y1)) / max(h_img, 1.0))
            if hr > best_hr:
                best_hr = hr
                best_conf = conf

        # debug every 1s
        if now - self.last_dbg > 1.0:
            self.last_dbg = now
            self.get_logger().info(
                f"stop_best_hr={best_hr:.3f} conf={best_conf:.2f} close_count={self.close_count} cooldown={max(0.0, self.cooldown_until-now):.1f}s"
            )

        # count consecutive "close" frames
        if best_hr >= self.stop_h and now >= self.cooldown_until:
            self.close_count += 1
        else:
            self.close_count = 0

        if self.close_count >= self.frames_required:
            self.stop_until = now + self.stop_hold_s
            self.cooldown_until = now + self.stop_cooldown_s
            self.close_count = 0
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.get_logger().warn(f"STOP SIGN TRIGGERED (hr={best_hr:.3f}, conf={best_conf:.2f}) -> hold {self.stop_hold_s}s")
            self.pub.publish(out)
            return

        self.pub.publish(out)

def main():
    rclpy.init()
    node = StopSignSupervisor()
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
