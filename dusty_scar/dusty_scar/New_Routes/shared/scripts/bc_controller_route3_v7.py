#!/usr/bin/env python3
import time
import numpy as np
import cv2
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

MODEL_PATH = "/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route3/models/route3_steer_v7.pt"
IMG_TOPIC = "/camera/color_image"
CMD_TOPIC = "/cmd_vel_nav"

class SmallCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.feat = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 3, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3, stride=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64, 64), nn.ReLU(),
            nn.Linear(64, 1)
        )
    def forward(self, x):
        return self.head(self.feat(x))

class BCController(Node):
    def __init__(self):
        super().__init__("bc_controller")

        self.declare_parameter("linear_x", 0.18)
        self.declare_parameter("max_ang", 1.2)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("timeout_sec", 0.25)
        self.declare_parameter("log_bad_frames", False)

        self.linear_x = float(self.get_parameter("linear_x").value)
        self.max_ang = float(self.get_parameter("max_ang").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.log_bad = bool(self.get_parameter("log_bad_frames").value)

        ckpt = torch.load(MODEL_PATH, map_location="cpu")
        self.img_w = int(ckpt["img_w"])
        self.img_h = int(ckpt["img_h"])
        self.label_max_ang = float(ckpt.get("max_ang", 1.5))

        self.model = SmallCNN()
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        self.last_img_time = 0.0
        self.last_cmd = (0.0, 0.0)
        self.bad_count = 0

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.sub = self.create_subscription(Image, IMG_TOPIC, self.on_img, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"Loaded model: {MODEL_PATH}")
        self.get_logger().info(f"Device: {self.device}, resize: {self.img_w}x{self.img_h}")
        self.get_logger().info(f"Publishing to {CMD_TOPIC} at {self.rate_hz} Hz, linear_x={self.linear_x}")

    def rosimg_to_bgr(self, msg: Image):
        h, w = int(msg.height), int(msg.width)
        if h <= 0 or w <= 0:
            return None

        enc = (msg.encoding or "").lower()
        step = int(getattr(msg, "step", 0)) or (w * 3)
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        if len(buf) < h * step:
            return None

        rows = buf[:h * step].reshape((h, step))

        if enc in ("rgb8", "bgr8"):
            row_bytes = w * 3
            if step < row_bytes:
                return None
            img = rows[:, :row_bytes].reshape((h, w, 3))
            if img.size == 0:
                return None
            if enc == "rgb8":
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img

        if enc in ("rgba8", "bgra8"):
            row_bytes = w * 4
            if step < row_bytes:
                return None
            img = rows[:, :row_bytes].reshape((h, w, 4))
            if img.size == 0:
                return None
            if enc == "rgba8":
                return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        if enc in ("mono8",):
            row_bytes = w
            if step < row_bytes:
                return None
            img = rows[:, :row_bytes].reshape((h, w))
            if img.size == 0:
                return None
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        return None

    def on_img(self, msg: Image):
        img_bgr = self.rosimg_to_bgr(msg)
        if img_bgr is None or img_bgr.size == 0:
            self.bad_count += 1
            if self.log_bad and (self.bad_count % 20 == 0):
                self.get_logger().warn(f"Skipped bad frame #{self.bad_count} (enc={msg.encoding}, h={msg.height}, w={msg.width}, step={getattr(msg,'step',None)})")
            return

        try:
            img_bgr = cv2.resize(img_bgr, (self.img_w, self.img_h), interpolation=cv2.INTER_AREA)
        except Exception:
            self.bad_count += 1
            return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        x = (img_rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)
        x = torch.from_numpy(x).unsqueeze(0).to(self.device)

        with torch.no_grad():
            ang = float(self.model(x).detach().cpu().numpy().reshape(-1)[0])

        ang = max(-self.label_max_ang, min(self.label_max_ang, ang))
        ang = max(-self.max_ang, min(self.max_ang, ang))

        self.last_cmd = (self.linear_x, ang)
        self.last_img_time = time.time()

    def on_timer(self):
        # safety: stop if camera stale
        if time.time() - self.last_img_time > self.timeout_sec:
            cmd = Twist()
            self.pub.publish(cmd)
            return

        lin, ang = self.last_cmd
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = BCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # stop car on exit
    cmd = Twist()
    node.pub.publish(cmd)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
