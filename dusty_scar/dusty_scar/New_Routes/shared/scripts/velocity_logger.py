#!/usr/bin/env python3
import csv, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('csv_path', 'velocity_demo.csv')
        self.declare_parameter('rate_hz', 20.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.csv_path = self.get_parameter('csv_path').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.last_odom = None
        self.last_cmd = None

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)

        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t_wall','odom_lin_x','odom_ang_z','cmd_lin_x','cmd_ang_z'])

        self.timer = self.create_timer(1.0/max(1.0,self.rate_hz), self.tick)
        self.get_logger().info(f"Logging to {self.csv_path} (odom={self.odom_topic}, cmd={self.cmd_topic})")

    def on_odom(self, msg): self.last_odom = msg
    def on_cmd(self, msg): self.last_cmd = msg

    def tick(self):
        t = time.time()
        odom_lin = odom_ang = float('nan')
        cmd_lin = cmd_ang = float('nan')
        if self.last_odom:
            odom_lin = float(self.last_odom.twist.twist.linear.x)
            odom_ang = float(self.last_odom.twist.twist.angular.z)
        if self.last_cmd:
            cmd_lin = float(self.last_cmd.linear.x)
            cmd_ang = float(self.last_cmd.angular.z)
        self.w.writerow([t, odom_lin, odom_ang, cmd_lin, cmd_ang])

    def destroy_node(self):
        try:
            self.f.flush(); self.f.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    n = VelocityLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
