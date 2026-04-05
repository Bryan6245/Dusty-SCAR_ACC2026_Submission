#!/usr/bin/env python3
"""
Extract Route1 training data + pose from a rosbag2.

Outputs:
- images/*.jpg
- labels.csv (BC-compatible): image,linear_x,angular_z,t,match_dt
- labels_pose.csv: image,linear_x,angular_z,x,y,yaw,t,match_dt,pose_dt

Pose is taken from /tf for transform: odom -> base_link (nearest in time, within max_pose_dt).
"""

import csv
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to rosbag2 folder (contains metadata.yaml)")
    ap.add_argument("--out", required=True, help="Output folder (creates images/, labels.csv, labels_pose.csv)")
    ap.add_argument("--image_topic", default="/camera/color_image")
    ap.add_argument("--cmd_topic", default="/cmd_vel_nav")
    ap.add_argument("--tf_topic", default="/tf")
    ap.add_argument("--odom_frame", default="odom")
    ap.add_argument("--base_frame", default="base_link")
    ap.add_argument("--max_dt", type=float, default=0.20, help="Max seconds between image and last cmd")
    ap.add_argument("--max_pose_dt", type=float, default=0.20, help="Max seconds between image and last pose")
    ap.add_argument("--every", type=int, default=2, help="Keep every Nth image message")
    args = ap.parse_args()

    bag_dir = Path(args.bag)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    img_dir = out_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    labels_path = out_dir / "labels.csv"
    labels_pose_path = out_dir / "labels_pose.csv"

    bridge = CvBridge()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for needed in (args.image_topic, args.cmd_topic, args.tf_topic):
        if needed not in topics_and_types:
            raise SystemExit(f"Missing topic in bag: {needed}\nAvailable: {list(topics_and_types)[:25]} ...")

    ImageMsg = get_message(topics_and_types[args.image_topic])
    CmdMsg   = get_message(topics_and_types[args.cmd_topic])
    TfMsg    = get_message(topics_and_types[args.tf_topic])

    last_cmd = None
    last_cmd_t = None

    # pose state from TF
    last_pose = None  # (x, y, yaw)
    last_pose_t = None

    total_imgs = 0
    kept = 0
    skipped_no_cmd = 0
    skipped_dt = 0
    skipped_no_pose = 0
    skipped_pose_dt = 0
    skipped_decode = 0
    skipped_empty = 0

    with open(labels_path, "w", newline="") as f_bc, open(labels_pose_path, "w", newline="") as f_pose:
        w_bc = csv.writer(f_bc)
        w_pose = csv.writer(f_pose)

        w_bc.writerow(["image", "linear_x", "angular_z", "t", "match_dt"])
        w_pose.writerow(["image", "linear_x", "angular_z", "x", "y", "yaw", "t", "match_dt", "pose_dt"])

        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t = t_ns * 1e-9

            # Update pose from TF
            if topic == args.tf_topic:
                try:
                    tfm = deserialize_message(data, TfMsg)
                except Exception:
                    continue
                # tfm.transforms is a list of geometry_msgs/TransformStamped
                for tr in getattr(tfm, "transforms", []):
                    try:
                        parent = tr.header.frame_id.strip("/")
                        child = tr.child_frame_id.strip("/")
                    except Exception:
                        continue
                    if parent == args.odom_frame and child == args.base_frame:
                        tx = float(tr.transform.translation.x)
                        ty = float(tr.transform.translation.y)
                        q = tr.transform.rotation
                        yaw = quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
                        # Use transform timestamp if available, else bag time
                        try:
                            ts = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                        except Exception:
                            ts = t
                        last_pose = (tx, ty, yaw)
                        last_pose_t = ts
                continue

            # Update cmd
            if topic == args.cmd_topic:
                try:
                    last_cmd = deserialize_message(data, CmdMsg)
                    last_cmd_t = t
                except Exception:
                    last_cmd = None
                    last_cmd_t = None
                continue

            # Process image
            if topic != args.image_topic:
                continue

            total_imgs += 1
            if (total_imgs - 1) % max(1, args.every) != 0:
                continue

            if last_cmd is None or last_cmd_t is None:
                skipped_no_cmd += 1
                continue
            dt = abs(t - last_cmd_t)
            if dt > args.max_dt:
                skipped_dt += 1
                continue

            if last_pose is None or last_pose_t is None:
                skipped_no_pose += 1
                continue
            pdt = abs(t - last_pose_t)
            if pdt > args.max_pose_dt:
                skipped_pose_dt += 1
                continue

            # Deserialize + convert image safely
            try:
                img_msg = deserialize_message(data, ImageMsg)
            except Exception:
                skipped_decode += 1
                continue

            try:
                cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            except Exception:
                try:
                    cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
                except Exception:
                    skipped_decode += 1
                    continue

            if cv_img is None or getattr(cv_img, "size", 0) == 0:
                skipped_empty += 1
                continue

            fname = f"frame_{kept:06d}.jpg"
            out_path = img_dir / fname
            if not cv2.imwrite(str(out_path), cv_img):
                skipped_empty += 1
                continue

            lin_x = float(last_cmd.linear.x)
            ang_z = float(last_cmd.angular.z)
            x, y, yaw = last_pose

            w_bc.writerow([f"images/{fname}", lin_x, ang_z, f"{t:.6f}", f"{dt:.6f}"])
            w_pose.writerow([f"images/{fname}", lin_x, ang_z, f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}", f"{t:.6f}", f"{dt:.6f}", f"{pdt:.6f}"])
            kept += 1

    print("=== Route1 Extract Summary (WITH POSE) ===")
    print(f"Bag: {bag_dir}")
    print(f"Out: {out_dir}")
    print(f"Image topic: {args.image_topic}")
    print(f"Cmd topic:   {args.cmd_topic}")
    print(f"TF topic:    {args.tf_topic} (pose {args.odom_frame}->{args.base_frame})")
    print(f"Total image msgs seen: {total_imgs}")
    print(f"Samples kept:          {kept}")
    print(f"Skipped (no cmd):      {skipped_no_cmd}")
    print(f"Skipped (cmd dt>max):  {skipped_dt}")
    print(f"Skipped (no pose yet): {skipped_no_pose}")
    print(f"Skipped (pose dt>max): {skipped_pose_dt}")
    print(f"Skipped (decode):      {skipped_decode}")
    print(f"Skipped (empty/write): {skipped_empty}")
    print(f"Wrote: {labels_path}")
    print(f"Wrote: {labels_pose_path}")


if __name__ == "__main__":
    main()
