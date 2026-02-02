#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from qcar2_interfaces.msg import MotorCommands

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class LidarNav(Node):
    def __init__(self):
        super().__init__("lidar_nav")

        # Safe starter tuning
        self.cruise_speed = 0.18
        self.max_steer = 0.25
        self.stop_dist = 0.65
        self.hard_stop = 0.40
        self.k_center = 0.35
        self.alpha = 0.85
        self.prev_steer = 0.0

        self.scan = None
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        self.create_timer(0.05, self.step)  # 20 Hz
        self.get_logger().info("LiDAR nav started (safety stop + centering).")

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

    def step(self):
        if self.scan is None:
            self.publish_cmd(0.0, 0.0)
            return

        front = self.sector_min(self.scan, -12, 12)
        left  = self.sector_min(self.scan,  20, 70)
        right = self.sector_min(self.scan, -70, -20)

        # Emergency stop
        if front < self.hard_stop:
            self.publish_cmd(0.0, 0.0)
            return

        # If close ahead, stop and bias steer away
        if front < self.stop_dist:
            steer = self.max_steer if right < left else -self.max_steer
            self.publish_cmd(steer, 0.0)
            return

        # Center between left and right clearance
        raw = self.k_center * (right - left)  # if steering is backwards, flip to (left - right)
        raw = clamp(raw, -self.max_steer, self.max_steer)

        steer = self.alpha * self.prev_steer + (1 - self.alpha) * raw
        self.prev_steer = steer

        self.publish_cmd(steer, self.cruise_speed)

def main():
    rclpy.init()
    node = LidarNav()
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
