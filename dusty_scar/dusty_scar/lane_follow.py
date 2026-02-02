#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from qcar2_interfaces.msg import MotorCommands

try:
    from cv_bridge import CvBridge
    _HAS_CV_BRIDGE = True
except Exception:
    _HAS_CV_BRIDGE = False

import cv2

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class LaneFollow(Node):
    def __init__(self):
        super().__init__("lane_follow")

        # --- Parameters (tune these) ---
        self.declare_parameter("image_topic", "")      # leave empty to auto-pick
        self.declare_parameter("speed", 0.18)
        self.declare_parameter("max_steer", 0.30)
        self.declare_parameter("kp", 0.55)            # steering gain
        self.declare_parameter("roi_start", 0.55)     # use bottom 45% of image
        self.declare_parameter("stop_dist", 0.65)
        self.declare_parameter("hard_stop", 0.40)

        self.speed = float(self.get_parameter("speed").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.kp = float(self.get_parameter("kp").value)
        self.roi_start = float(self.get_parameter("roi_start").value)
        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.hard_stop = float(self.get_parameter("hard_stop").value)

        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)

        # LiDAR
        self.scan = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        # Camera (auto-detect topic + type)
        self.bridge = CvBridge() if _HAS_CV_BRIDGE else None
        self.img_sub = None
        self.last_frame = None
        self._setup_camera_subscription()

        self.prev_steer = 0.0
        self.alpha = 0.80  # smoothing

        self.create_timer(0.05, self.step)  # 20 Hz

        self.get_logger().info(
            f"LaneFollow running. cv_bridge={_HAS_CV_BRIDGE}. "
            f"speed={self.speed}, kp={self.kp}, roi_start={self.roi_start}"
        )

    def _setup_camera_subscription(self):
        user_topic = str(self.get_parameter("image_topic").value).strip()
        topics = dict(self.get_topic_names_and_types())

        # Candidate topics (prefer explicit user topic)
        candidates = []
        if user_topic:
            candidates.append(user_topic)
        # common names
        for t in topics.keys():
            tl = t.lower()
            if ("image" in tl) and ("camera" in tl):
                candidates.append(t)

        # pick first candidate that exists
        chosen = None
        chosen_type = None
        for t in candidates:
            if t in topics:
                for typ in topics[t]:
                    if typ.endswith("sensor_msgs/msg/Image") or typ.endswith("sensor_msgs/msg/CompressedImage"):
                        chosen = t
                        chosen_type = typ
                        break
            if chosen:
                break

        if not chosen:
            self.get_logger().warn("No camera image topic found yet. Waiting...")
            return

        self.get_logger().info(f"Using image topic: {chosen} [{chosen_type}]")

        if chosen_type.endswith("sensor_msgs/msg/Image"):
            self.img_sub = self.create_subscription(Image, chosen, self.on_image, 10)
        else:
            self.img_sub = self.create_subscription(CompressedImage, chosen, self.on_compressed, 10)

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def on_image(self, msg: Image):
        if not _HAS_CV_BRIDGE:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_frame = frame
        except Exception:
            pass

    def on_compressed(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            self.last_frame = frame
        except Exception:
            pass

    def sector_min(self, s: LaserScan, deg0: float, deg1: float) -> float:
        a0, inc = s.angle_min, s.angle_increment
        r0, r1 = math.radians(deg0), math.radians(deg1)
        if r0 > r1:
            r0, r1 = r1, r0
        i0 = int((r0 - a0) / inc)
        i1 = int((r1 - a0) / inc)
        i0 = max(0, min(i0, len(s.ranges)-1))
        i1 = max(0, min(i1, len(s.ranges)-1))
        if i0 > i1:
            i0, i1 = i1, i0

        best = float("inf")
        for r in s.ranges[i0:i1+1]:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                best = min(best, r)
        return best if math.isfinite(best) else s.range_max

    def publish_cmd(self, steer: float, throttle: float):
        m = MotorCommands()
        m.motor_names = ["steering_angle", "motor_throttle"]
        m.values = [float(steer), float(throttle)]
        self.pub.publish(m)

    def compute_lane_steer(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * self.roi_start)
        roi = frame[y0:h, :]

        # Convert + threshold (white + yellow-ish)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # white mask
        white = cv2.inRange(hsv, (0, 0, 200), (180, 50, 255))
        # yellow mask (tweak if needed)
        yellow = cv2.inRange(hsv, (15, 60, 120), (40, 255, 255))

        mask = cv2.bitwise_or(white, yellow)

        # Clean up noise
        mask = cv2.medianBlur(mask, 5)

        # Compute centroid of lane pixels
        ys, xs = np.where(mask > 0)
        if xs.size < 300:  # not enough lane pixels seen
            return None

        cx = float(np.mean(xs))  # 0..w-1
        err = (cx - (w / 2.0)) / (w / 2.0)  # -1..+1
        steer = -self.kp * err               # sign may need flip
        steer = clamp(steer, -self.max_steer, self.max_steer)
        return steer

    def step(self):
        # If camera subscription wasn't ready at startup, try again
        if self.img_sub is None:
            self._setup_camera_subscription()

        # LiDAR stop logic
        if self.scan is not None:
            front = self.sector_min(self.scan, -12, 12)
            if front < self.hard_stop:
                self.publish_cmd(0.0, 0.0)
                return
            if front < self.stop_dist:
                # stop but keep wheels straight
                self.publish_cmd(0.0, 0.0)
                return

        if self.last_frame is None:
            # no camera yet -> don't drive blindly
            self.publish_cmd(0.0, 0.0)
            return

        steer = self.compute_lane_steer(self.last_frame)
        if steer is None:
            # can't see lane -> slow down / go straight cautiously
            self.publish_cmd(0.0, 0.10)
            return

        # Smooth steering
        steer = self.alpha * self.prev_steer + (1.0 - self.alpha) * steer
        self.prev_steer = steer

        self.publish_cmd(steer, self.speed)

def main():
    rclpy.init()
    node = LaneFollow()
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
