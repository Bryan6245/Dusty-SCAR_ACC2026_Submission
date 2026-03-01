#!/usr/bin/env python3
import csv, sys
import matplotlib.pyplot as plt

def read_csv(path):
    t=[]; odom_v=[]; odom_w=[]; cmd_v=[]; cmd_w=[]
    with open(path, 'r') as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row['t_wall']))
            odom_v.append(float(row['odom_lin_x']))
            odom_w.append(float(row['odom_ang_z']))
            cmd_v.append(float(row['cmd_lin_x']))
            cmd_w.append(float(row['cmd_ang_z']))
    t0 = t[0] if t else 0.0
    t = [x - t0 for x in t]
    return t, odom_v, odom_w, cmd_v, cmd_w

def main():
    if len(sys.argv) < 3:
        print("Usage: plot_velocity.py <input.csv> <output.png>")
        sys.exit(2)
    inp, outp = sys.argv[1], sys.argv[2]
    t, odom_v, odom_w, cmd_v, cmd_w = read_csv(inp)

    plt.figure()
    plt.plot(t, odom_v, label='odom linear.x')
    plt.plot(t, cmd_v, label='cmd linear.x')
    plt.xlabel('time (s)'); plt.ylabel('linear velocity (m/s)')
    plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(outp, dpi=200)

    plt.figure()
    plt.plot(t, odom_w, label='odom angular.z')
    plt.plot(t, cmd_w, label='cmd angular.z')
    plt.xlabel('time (s)'); plt.ylabel('angular velocity (rad/s)')
    plt.grid(True); plt.legend(); plt.tight_layout()
    outp2 = outp.replace(".png", "_ang.png")
    plt.savefig(outp2, dpi=200)

    print("Wrote:", outp)
    print("Wrote:", outp2)

if __name__ == '__main__':
    main()
