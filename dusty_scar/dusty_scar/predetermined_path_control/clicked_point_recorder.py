#!/usr/bin/env python3
import json
import os
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class ClickedPointRecorder(Node):
    def __init__(self):
        super().__init__("clicked_point_recorder")
        self.declare_parameter("output_route", "")
        self.declare_parameter("frame_id", "map")

        out = str(self.get_parameter("output_route").value).strip()
        self.frame_id = str(self.get_parameter("frame_id").value).strip()

        if not out:
            out = os.path.join(os.path.dirname(__file__), "routes", "clicked_route.json")
        self.output_route = out

        self.waypoints = []
        self.min_dist_m = 0.50   # only save a click if it's at least 0.5m away
        self.last_xy = None

        os.makedirs(os.path.dirname(self.output_route), exist_ok=True)

        self.sub = self.create_subscription(PointStamped, "/clicked_point", self.cb, 10)

        self.get_logger().info("Click points in RViz using 'Publish Point'.")
        self.get_logger().info(f"Saving to: {self.output_route}")
        self._save()  # create file immediately

    def cb(self, msg: PointStamped):
        x = float(msg.point.x)
        y = float(msg.point.y)

        # Skip if too close to last saved point
        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            dist = math.hypot(dx, dy)
            if dist < self.min_dist_m:
                self.get_logger().info(f"Skipped (too close): dist={dist:.2f} m")
                return

        self.waypoints.append({"x": x, "y": y})
        self.last_xy = (x, y)
        self.get_logger().info(f"Added #{len(self.waypoints)}: x={x:.3f}, y={y:.3f}")
        self._save()


    def _save(self):
        data = {"frame_id": self.frame_id, "waypoints": self.waypoints}
        with open(self.output_route, "w") as f:
            json.dump(data, f, indent=2)


def main():
    rclpy.init()
    node = ClickedPointRecorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
