#!/usr/bin/env python3
import json
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
        self.declare_parameter('device_id', -1)   # -1 cpu, 0 cuda:0
        self.declare_parameter('rate_hz', 3.0)

        self.declare_parameter('annotated_topic', '/yolo/annotated')
        self.declare_parameter('detections_text_topic', '/yolo/detections_text')
        self.declare_parameter('detections_json_topic', '/yolo/detections_json')

        # Stable inference input for ultralytics
        self.declare_parameter('tmp_image_path', '/tmp/yolo_in.jpg')

        self.weights = self.get_parameter('weights').value
        self.image_topic = self.get_parameter('image_topic').value
        self.conf = float(self.get_parameter('conf').value)
        self.device_id = int(self.get_parameter('device_id').value)
        self.device = f'cuda:{self.device_id}' if self.device_id >= 0 else 'cpu'
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.det_txt_topic = self.get_parameter('detections_text_topic').value
        self.det_json_topic = self.get_parameter('detections_json_topic').value

        self.tmp_image_path = str(self.get_parameter('tmp_image_path').value)
        Path(self.tmp_image_path).parent.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f"Loading YOLO weights: {self.weights}")
        self.get_logger().info(f"Device: {self.device} | conf={self.conf} | rate_hz={self.rate_hz}")
        self.model = YOLO(self.weights)
        self.bridge = CvBridge()

        self.latest_msg = None

        self.sub = self.create_subscription(Image, self.image_topic, self.on_img, qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, self.annotated_topic, 10)
        self.pub_txt = self.create_publisher(String, self.det_txt_topic, 10)
        self.pub_json = self.create_publisher(String, self.det_json_topic, 10)

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.5), self.on_timer)

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"Publishing annotated: {self.annotated_topic}")
        self.get_logger().info(f"Publishing text: {self.det_txt_topic}")
        self.get_logger().info(f"Publishing json: {self.det_json_topic}")

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

        h, w = frame.shape[:2]

        # Stable inference: write temp image then predict(path)
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

        r0 = results[0]
        names = r0.names

        dets = []
        det_strs = []

        try:
            if r0.boxes is not None and len(r0.boxes) > 0:
                for b in r0.boxes:
                    cls = int(b.cls.item())
                    cf = float(b.conf.item())
                    x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                    name = names.get(cls, str(cls))
                    dets.append({"cls": name, "conf": cf, "xyxy": [x1, y1, x2, y2]})
                    det_strs.append(f"{name}:{cf:.2f}")
        except Exception:
            pass

        # publish text
        txt = String()
        txt.data = ", ".join(det_strs) if det_strs else "none"
        self.pub_txt.publish(txt)

        # publish json with bbox info
        js = String()
        js.data = json.dumps({"w": w, "h": h, "dets": dets})
        self.pub_json.publish(js)

        # publish annotated
        try:
            annotated = r0.plot()
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.pub_img.publish(out)
        except Exception as e:
            self.get_logger().warn(f"annotated publish failed: {e}")


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
