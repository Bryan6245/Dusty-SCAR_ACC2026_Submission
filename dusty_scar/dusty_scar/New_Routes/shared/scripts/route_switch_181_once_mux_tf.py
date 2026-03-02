#!/usr/bin/env python3
import math, time, subprocess
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Twist

def zero_twist():
    t = Twist()
    t.linear.x = 0.0
    t.angular.z = 0.0
    return t

class Switch181OnceMux(Node):
    def __init__(self):
        super().__init__("route_switch_181_once_mux_tf")

        # frames
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # portals
        self.declare_parameter("p18_x", 3.055)
        self.declare_parameter("p18_y", 1.622)
        self.declare_parameter("p18_r_in", 0.55)
        self.declare_parameter("p18_r_out", 1.00)

        self.declare_parameter("p81_x", 1.480)
        self.declare_parameter("p81_y", 0.849)
        self.declare_parameter("p81_r_in", 0.65)
        self.declare_parameter("p81_r_out", 1.20)

        self.declare_parameter("min_hold_s", 2.0)

        # cmd topics
        self.declare_parameter("cmd_r1", "/cmd_vel_bc_r1")
        self.declare_parameter("cmd_r8", "/cmd_vel_bc_r8")
        self.declare_parameter("cmd_out", "/cmd_vel_bc")

        # BC params
        self.declare_parameter("linear_x", 0.12)
        self.declare_parameter("max_ang", 1.5)
        self.declare_parameter("rate_hz", 12.0)

        self.cmd_r1 = self.get_parameter("cmd_r1").value
        self.cmd_r8 = self.get_parameter("cmd_r8").value
        self.cmd_out = self.get_parameter("cmd_out").value

        self.last_r1 = zero_twist()
        self.last_r8 = zero_twist()
        self.t_r1 = 0.0
        self.t_r8 = 0.0

        self.pub = self.create_publisher(Twist, self.cmd_out, 10)
        self.create_subscription(Twist, self.cmd_r1, self.cb_r1, 10)
        self.create_subscription(Twist, self.cmd_r8, self.cb_r8, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # state
        self.active = 1          # start Route1
        self.armed = True
        self.done = False        # becomes True after we complete 1->8->1
        self.last_switch_t = 0.0

        # Start both BC controllers (no switching delay)
        self.start_bcs()

        # 20 Hz publish/switch loop
        self.create_timer(0.05, self.tick)

    def cb_r1(self, msg): self.last_r1, self.t_r1 = msg, time.time()
    def cb_r8(self, msg): self.last_r8, self.t_r8 = msg, time.time()

    def start_bcs(self):
        # kill any old bc controllers
        subprocess.run(["bash","-lc",
            "pkill -f bc_controller_route1_v8.py || true; "
            "pkill -f bc_controller_route8_auto.py || true"
        ], check=False)

        lin=float(self.get_parameter("linear_x").value)
        ang=float(self.get_parameter("max_ang").value)
        hz=float(self.get_parameter("rate_hz").value)

        # Route1 -> /cmd_vel_bc_r1
        cmd1 = (
            "CUDA_VISIBLE_DEVICES=0 python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route1_v8.py --ros-args "
            f"-p linear_x:={lin} -p max_ang:={ang} -p rate_hz:={hz} "
            f"--remap /cmd_vel_nav:={self.cmd_r1} -r __node:=bc_route1"
        )
        # Route8 -> /cmd_vel_bc_r8
        cmd8 = (
            "CUDA_VISIBLE_DEVICES=0 python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route8_auto.py --ros-args "
            f"-p linear_x:={lin} -p max_ang:={ang} -p rate_hz:={hz} "
            f"--remap /cmd_vel_nav:={self.cmd_r8} -r __node:=bc_route8"
        )

        self.get_logger().info("Starting BC Route1 + Route8 in parallel (mux selects output)...")
        subprocess.Popen(["bash","-lc", cmd1])
        subprocess.Popen(["bash","-lc", cmd8])

    def tick(self):
        # publish selected cmd (stale protection)
        now = time.time()
        stale_r1 = (now - self.t_r1) > 0.5
        stale_r8 = (now - self.t_r8) > 0.5

        if self.active == 1:
            out = zero_twist() if stale_r1 else self.last_r1
        else:
            out = zero_twist() if stale_r8 else self.last_r8

        self.pub.publish(out)

        # if completed, no more switching
        if self.done:
            return

        # switch decision based on TF distance
        world = self.get_parameter("world_frame").value
        base  = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(world, base, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
        except Exception:
            return

        hold = float(self.get_parameter("min_hold_s").value)
        if (now - self.last_switch_t) < hold:
            return

        if self.active == 1:
            sx=float(self.get_parameter("p18_x").value); sy=float(self.get_parameter("p18_y").value)
            rin=float(self.get_parameter("p18_r_in").value); rout=float(self.get_parameter("p18_r_out").value)
            d=math.hypot(x-sx, y-sy)
            if d > rout: self.armed = True
            if self.armed and d < rin:
                self.get_logger().info("SWITCH 1->8")
                self.active = 8
                self.armed = False
                self.last_switch_t = now
        else:
            sx=float(self.get_parameter("p81_x").value); sy=float(self.get_parameter("p81_y").value)
            rin=float(self.get_parameter("p81_r_in").value); rout=float(self.get_parameter("p81_r_out").value)
            d=math.hypot(x-sx, y-sy)
            if d > rout: self.armed = True
            if self.armed and d < rin:
                self.get_logger().info("SWITCH 8->1 (DONE, hold Route1)")
                self.active = 1
                self.armed = False
                self.last_switch_t = now
                self.done = True

def main():
    rclpy.init()
    n = Switch181OnceMux()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    subprocess.run(["bash","-lc",
        "pkill -f bc_controller_route1_v8.py || true; "
        "pkill -f bc_controller_route8_auto.py || true"
    ], check=False)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
