#!/usr/bin/env python3
import math, subprocess, time
import rclpy
from rclpy.node import Node
import tf2_ros

class Switch181Once(Node):
    def __init__(self):
        super().__init__("route_switch_181_once_tf")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        # P18: 1->8
        self.declare_parameter("p18_x", 3.055)
        self.declare_parameter("p18_y", 1.622)
        self.declare_parameter("p18_r_in", 0.55)
        self.declare_parameter("p18_r_out", 1.00)

        # P81: 8->1
        self.declare_parameter("p81_x", 1.480)
        self.declare_parameter("p81_y", 0.849)
        self.declare_parameter("p81_r_in", 0.35)
        self.declare_parameter("p81_r_out", 0.90)

        self.declare_parameter("linear_x", 0.12)
        self.declare_parameter("max_ang", 1.5)
        self.declare_parameter("rate_hz", 12.0)
        self.declare_parameter("min_hold_s", 2.0)
        self.last_switch_t = 0.0

        self.route = 1
        self.armed = True
        self.done = False  # becomes True after we complete 1->8->1

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.tick)  # 10 Hz

        self.stop_all_bc()
        self.start_route(1)

    def stop_all_bc(self):
        subprocess.run(["bash","-lc",
            "pkill -f bc_controller_route1_v8.py || true; "
            "pkill -f bc_controller_route8_auto.py || true"
        ], check=False)

    def _start_bc(self, script: str):
        lin=float(self.get_parameter("linear_x").value)
        ang=float(self.get_parameter("max_ang").value)
        hz=float(self.get_parameter("rate_hz").value)
        cmd=(f"CUDA_VISIBLE_DEVICES=0 python3 {script} --ros-args "
             f"-p linear_x:={lin} -p max_ang:={ang} -p rate_hz:={hz} "
             "--remap /cmd_vel_nav:=/cmd_vel_bc")
        subprocess.Popen(["bash","-lc", cmd])

    def start_route(self, rid: int):
        self.stop_all_bc()
        if rid == 1:
            self.get_logger().info("START Route1")
            self._start_bc("dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route1_v8.py")
        else:
            self.get_logger().info("SWITCH -> Route8")
            self._start_bc("dusty_scar/dusty_scar/New_Routes/shared/scripts/bc_controller_route8_auto.py")
        self.route = rid
        self.armed = False
        self.last_switch_t = time.time()

    def tick(self):
        if self.done:
            # stay on Route1, no more switching
            return

        world=self.get_parameter("world_frame").value
        base=self.get_parameter("base_frame").value
        try:
            tf=self.tf_buffer.lookup_transform(world, base, rclpy.time.Time())
            x=tf.transform.translation.x
            y=tf.transform.translation.y
        except Exception:
            return

        if self.route == 1:
            sx=float(self.get_parameter("p18_x").value); sy=float(self.get_parameter("p18_y").value)
            rin=float(self.get_parameter("p18_r_in").value); rout=float(self.get_parameter("p18_r_out").value)
            nxt=(8,"P18 1->8")
        else:
            sx=float(self.get_parameter("p81_x").value); sy=float(self.get_parameter("p81_y").value)
            rin=float(self.get_parameter("p81_r_in").value); rout=float(self.get_parameter("p81_r_out").value)
            nxt=(1,"P81 8->1")

        d=math.hypot(x-sx, y-sy)
        if d > rout:
            self.armed = True

        hold=float(self.get_parameter("min_hold_s").value)
        if self.armed and d < rin and (time.time()-self.last_switch_t) > hold:
            self.start_route(nxt[0])
            if nxt[0] == 1 and self.route == 1:
                # we just completed 8->1, so stop switching
                self.get_logger().info("DONE: completed 1->8->1. Holding Route1.")
                self.done = True

        self.get_logger().info(f"route={self.route} next={nxt[1]} d={d:.2f} armed={self.armed} done={self.done}")

def main():
    rclpy.init()
    n=Switch181Once()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.stop_all_bc()
    n.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
