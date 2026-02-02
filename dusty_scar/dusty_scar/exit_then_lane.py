#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from qcar2_interfaces.msg import MotorCommands

import cv2

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except Exception:
    HAS_CV_BRIDGE = False


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ExitThenLane(Node):
    def __init__(self):
        super().__init__("exit_then_lane")

        # --- Tunables ---
        self.exit_speed = 0.14          # speed while exiting hub
        self.lane_speed = 0.18          # speed once on road
        self.max_steer = 0.30

        self.stop_dist = 0.65
        self.hard_stop = 0.40

        # lane detection thresholds
        self.roi_start = 0.60           # bottom 40% of image
        self.min_lane_pixels = 450      # how many mask pixels = "lane is visible"
        self.lane_confirm_frames = 6    # consecutive frames before switching to lane mode

        # lane steering gain + smoothing
        self.kp = 0.55
        self.alpha = 0.85
        self.prev_steer = 0.0

        # state
        self.mode = "EXIT"
        self.lane_seen_count = 0

        # ROS I/O
        self.scan = None
        self.last_frame = None
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self._setup_camera_subscription()

        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        self.create_timer(0.05, self.step)  # 20 Hz

        self.get_logger().info(f"exit_then_lane started. mode={self.mode}, cv_bridge={HAS_CV_BRIDGE}")

    # ---------- Camera setup ----------
    def _setup_camera_subscription(self):
        topics = dict(self.get_topic_names_and_types())
        candidates = [t for t in topics.keys() if "image" in t.lower() and "camera" in t.lower()]

        chosen = None
        chosen_type = None
        for t in candidates:
            for typ in topics[t]:
                if typ.endswith("sensor_msgs/msg/Image") or typ.endswith("sensor_msgs/msg/CompressedImage"):
                    chosen = t
                    chosen_type = typ
                    break
            if chosen:
                break

        if not chosen:
            self.get_logger().warn("No camera image topic found yet. Waiting...")
            self.img_sub = None
            return

        self.get_logger().info(f"Using image topic: {chosen} [{chosen_type}]")
        if chosen_type.endswith("sensor_msgs/msg/Image"):
            self.img_sub = self.create_subscription(Image, chosen, self.on_image, 10)
        else:
            self.img_sub = self.create_subscription(CompressedImage, chosen, self.on_compressed, 10)

    def on_image(self, msg: Image):
        if not HAS_CV_BRIDGE:
            return
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def on_compressed(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            self.last_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            pass

    # ---------- LiDAR helpers ----------
    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def sector_min(self, s: LaserScan, deg0: float, deg1: float) -> float:
        a0, inc = s.angle_min, s.angle_increment
        r0, r1 = math.radians(deg0), math.radians(deg1)
        if r0 > r1:
            r0, r1 = r1, r0

        i0 = int((r0 - a0) / inc)
        i1 = int((r1 - a0) / inc)
        i0 = max(0, min(i0, len(s.ranges) - 1))
        i1 = max(0, min(i1, len(s.ranges) - 1))
        if i0 > i1:
            i0, i1 = i1, i0

        best = float("inf")
        for r in s.ranges[i0:i1 + 1]:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                best = min(best, r)
        return best if math.isfinite(best) else s.range_max

    def publish_cmd(self, steer: float, throttle: float):
        m = MotorCommands()
        m.motor_names = ["steering_angle", "motor_throttle"]
        m.values = [float(steer), float(throttle)]
        self.pub.publish(m)

    # ---------- Lane detection + steering ----------
    def lane_pixels_and_steer(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.roi_start)
        roi = frame[y0:h, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, (0, 0, 200), (180, 50, 255))
        yellow = cv2.inRange(hsv, (15, 60, 120), (40, 255, 255))
        mask = cv2.bitwise_or(white, yellow)
        mask = cv2.medianBlur(mask, 5)

        ys, xs = np.where(mask > 0)
        count = int(xs.size)
        if count < self.min_lane_pixels:
            return count, None

        cx = float(np.mean(xs))
        err = (cx - (w / 2.0)) / (w / 2.0)  # -1..+1
        steer = -self.kp * err              # if steering is backwards, flip sign
        steer = clamp(steer, -self.max_steer, self.max_steer)
        return count, steer

    # ---------- Main loop ----------
    def step(self):
        # if camera wasn't ready at startup, retry
        if getattr(self, "img_sub", None) is None:
            self._setup_camera_subscription()

        # LiDAR safety always on
        if self.scan is None:
            self.publish_cmd(0.0, 0.0)
            return

        front = self.sector_min(self.scan, -12, 12)
        if front < self.hard_stop:
            self.publish_cmd(0.0, 0.0)
            return

        # See if lane is visible yet
        steer_lane = None
        if self.last_frame is not None:
            pix, steer_lane = self.lane_pixels_and_steer(self.last_frame)
            if steer_lane is not None:
                self.lane_seen_count += 1
            else:
                self.lane_seen_count = 0

            if self.mode == "EXIT" and self.lane_seen_count >= self.lane_confirm_frames:
                self.mode = "LANE"
                self.get_logger().info("Switched to LANE mode.")

        if self.mode == "LANE" and steer_lane is not None:
            # Lane-follow with smoothing + LiDAR stop_dist
            if front < self.stop_dist:
                self.publish_cmd(0.0, 0.0)
                return

            steer = self.alpha * self.prev_steer + (1 - self.alpha) * steer_lane
            self.prev_steer = steer
            self.publish_cmd(steer, self.lane_speed)
            return

                # EXIT mode (or lane not visible): steer toward the most open direction to escape hub
        s = self.scan

        best_i = None
        best_r = -1.0

        # Search angles in front arc [-75°, +75°]
        deg_min, deg_max = -75.0, 75.0
        a0, inc = s.angle_min, s.angle_increment

        i0 = int((math.radians(deg_min) - a0) / inc)
        i1 = int((math.radians(deg_max) - a0) / inc)
        i0 = max(0, min(i0, len(s.ranges) - 1))
        i1 = max(0, min(i1, len(s.ranges) - 1))
        if i0 > i1:
            i0, i1 = i1, i0

        for i in range(i0, i1 + 1):
            r = s.ranges[i]
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                if r > best_r:
                    best_r = r
                    best_i = i

        if best_i is None:
            self.publish_cmd(0.0, 0.0)
            return

        best_angle = a0 + best_i * inc  # radians, negative=right, positive=left (usually)

        # Steering toward open space
        k_angle = 0.9  # tune 0.6..1.2
        steer = clamp(-k_angle * best_angle, -self.max_steer, self.max_steer)

        # Safety: if something is close ahead, stop instead of pushing forward
        if front < self.stop_dist:
            self.publish_cmd(0.0, 0.0)
            return

        steer = self.alpha * self.prev_steer + (1 - self.alpha) * steer
        self.prev_steer = steer

        self.publish_cmd(steer, self.exit_speed)
        return


def main():
    rclpy.init()
    node = ExitThenLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
