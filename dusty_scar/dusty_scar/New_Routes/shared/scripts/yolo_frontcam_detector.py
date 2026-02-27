#!/usr/bin/env python3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloFrontCam(Node):
    def __init__(self):
        super().__init__('yolo_frontcam_detector')

        self.declare_parameter('weights', 'dusty_scar/dusty_scar/models/yolo/yolo_best.pt')
        self.declare_parameter('image_topic', '/camera/color_image')
        self.declare_parameter('conf', 0.25)

        # -1 CPU, 0 = cuda:0
        self.declare_parameter('device_id', -1)

        self.declare_parameter('annotated_topic', '/yolo/annotated')
        self.declare_parameter('detections_topic', '/yolo/detections_text')

        # Demo-friendly throttle
        self.declare_parameter('rate_hz', 3.0)

        # Debug outputs (no GUI needed)
        self.declare_parameter('save_debug_dir', '/tmp/yolo_debug')
        self.declare_parameter('save_every_n', 10)

        # Temp file for inference
        self.declare_parameter('tmp_image_path', '/tmp/yolo_in.jpg')

        self.weights = self.get_parameter('weights').value
        self.image_topic = self.get_parameter('image_topic').value
        self.conf = float(self.get_parameter('conf').value)
        self.device_id = int(self.get_parameter('device_id').value)
        self.device = f'cuda:{self.device_id}' if self.device_id >= 0 else 'cpu'
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.save_debug_dir = Path(str(self.get_parameter('save_debug_dir').value))
        self.save_every_n = int(self.get_parameter('save_every_n').value)
        self.tmp_image_path = str(self.get_parameter('tmp_image_path').value)

        self.save_debug_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"Loading YOLO weights: {self.weights}")
        self.get_logger().info(f"Device: {self.device} | conf={self.conf} | rate_hz={self.rate_hz}")
        self.model = YOLO(self.weights)
        self.bridge = CvBridge()

        self.latest_msg = None
        self.frame_count = 0
        self.save_count = 0

        self.sub = self.create_subscription(Image, self.image_topic, self.on_img, qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, self.annotated_topic, 10)
        self.pub_txt = self.create_publisher(String, self.detections_topic, 10)

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.5), self.on_timer)

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"Publishing annotated: {self.annotated_topic}")
        self.get_logger().info(f"Publishing detections text: {self.detections_topic}")

    def on_img(self, msg: Image):
        self.latest_msg = msg

    def on_timer(self):
        if self.latest_msg is None:
            return

        msg = self.latest_msg
        self.latest_msg = None

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None:
                return
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        # Write a temp image and run YOLO on the FILE PATH (stable)
        try:
            ok = cv2.imwrite(self.tmp_image_path, frame)
            if not ok:
                self.get_logger().warn("cv2.imwrite failed")
                return
        except Exception as e:
            self.get_logger().warn(f"imwrite exception: {e}")
            return

        try:
            results = self.model.predict(
                source=self.tmp_image_path,
                conf=self.conf,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().warn(f"YOLO predict failed: {e}")
            return

        # Build detections text
        dets = []
        try:
            r0 = results[0]
            names = r0.names
            if r0.boxes is not None and len(r0.boxes) > 0:
                for b in r0.boxes:
                    cls = int(b.cls.item())
                    cf = float(b.conf.item())
                    dets.append(f"{names.get(cls, str(cls))}:{cf:.2f}")
        except Exception:
            pass

        msg_txt = String()
        msg_txt.data = ", ".join(dets) if dets else "none"
        try:
            self.pub_txt.publish(msg_txt)
        except Exception as e:
            self.get_logger().warn(f"publish text failed: {e}")

        # Annotate + publish + optional save
        try:
            annotated = results[0].plot()
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.pub_img.publish(out)

            self.frame_count += 1
            if self.save_every_n > 0 and (self.frame_count % self.save_every_n == 0):
                self.save_count += 1
                fn = self.save_debug_dir / f"annotated_{self.save_count:04d}.jpg"
                cv2.imwrite(str(fn), annotated)
        except Exception as e:
            self.get_logger().warn(f"publish annotated failed: {e}")


def main():
    rclpy.init()
    node = YoloFrontCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
