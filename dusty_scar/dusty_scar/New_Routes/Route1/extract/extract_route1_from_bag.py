#!/usr/bin/env python3
"""
Route1 bag extractor (Digital Twin)

Extracts:
- images/*.jpg from an Image topic
- labels.csv with [image, linear_x, angular_z, t, match_dt]
by matching each image to the most recent cmd_vel Twist message.
Skips malformed/empty frames instead of crashing.
"""

import csv
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def extract(
    bag_dir: Path,
    out_dir: Path,
    image_topic: str,
    cmd_topic: str,
    max_dt: float,
    every: int,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    img_dir = out_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    bridge = CvBridge()

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if image_topic not in topics_and_types:
        raise RuntimeError(
            f"Image topic not found in bag: {image_topic}\n"
            f"Available topics: {list(topics_and_types.keys())}"
        )
    if cmd_topic not in topics_and_types:
        raise RuntimeError(
            f"Cmd topic not found in bag: {cmd_topic}\n"
            f"Available topics: {list(topics_and_types.keys())}"
        )

    ImageMsg = get_message(topics_and_types[image_topic])
    CmdMsg = get_message(topics_and_types[cmd_topic])

    last_cmd = None
    last_cmd_t = None  # seconds

    labels_path = out_dir / "labels.csv"

    total_imgs = 0
    kept = 0
    skipped_empty = 0
    skipped_decode = 0
    skipped_dt = 0
    skipped_no_cmd = 0

    with open(labels_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["image", "linear_x", "angular_z", "t", "match_dt"])

        msg_idx = 0
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            t = t_ns * 1e-9  # seconds
            msg_idx += 1

            if topic == cmd_topic:
                try:
                    last_cmd = deserialize_message(data, CmdMsg)
                    last_cmd_t = t
                except Exception:
                    # If a cmd message is malformed (rare), just ignore it
                    last_cmd = None
                    last_cmd_t = None
                continue

            if topic != image_topic:
                continue

            total_imgs += 1
            if (total_imgs - 1) % max(1, every) != 0:
                continue

            if last_cmd is None or last_cmd_t is None:
                skipped_no_cmd += 1
                continue

            dt = abs(t - last_cmd_t)
            if dt > max_dt:
                skipped_dt += 1
                continue

            # Deserialize image
            try:
                img_msg = deserialize_message(data, ImageMsg)
            except Exception:
                skipped_decode += 1
                continue

            # Convert safely
            cv_img = None
            try:
                cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            except Exception:
                try:
                    cv_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
                except Exception:
                    skipped_decode += 1
                    continue

            # Skip empty frames
            if cv_img is None or getattr(cv_img, "size", 0) == 0:
                skipped_empty += 1
                continue

            fname = f"frame_{kept:06d}.jpg"
            out_path = img_dir / fname

            ok = cv2.imwrite(str(out_path), cv_img)
            if not ok:
                skipped_empty += 1
                continue

            lin_x = float(last_cmd.linear.x)
            ang_z = float(last_cmd.angular.z)

            w.writerow([f"images/{fname}", lin_x, ang_z, f"{t:.6f}", f"{dt:.6f}"])
            kept += 1

    print("=== Route1 Extract Summary ===")
    print(f"Bag: {bag_dir}")
    print(f"Out: {out_dir}")
    print(f"Image topic: {image_topic}")
    print(f"Cmd topic:   {cmd_topic}")
    print(f"Total image msgs seen: {total_imgs}")
    print(f"Samples kept:          {kept}")
    print(f"Skipped (no cmd yet):  {skipped_no_cmd}")
    print(f"Skipped (dt > max):    {skipped_dt}")
    print(f"Skipped (decode):      {skipped_decode}")
    print(f"Skipped (empty/write): {skipped_empty}")
    print(f"Wrote labels:          {labels_path}")


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to rosbag2 folder (contains metadata.yaml)")
    ap.add_argument("--out", required=True, help="Output folder to write images/ and labels.csv")
    ap.add_argument("--image_topic", default="/camera/color_image")
    ap.add_argument("--cmd_topic", default="/cmd_vel_nav")
    ap.add_argument("--max_dt", type=float, default=0.20, help="Max allowed seconds between image and last cmd")
    ap.add_argument("--every", type=int, default=2, help="Keep every Nth image message")
    args = ap.parse_args()

    extract(
        bag_dir=Path(args.bag),
        out_dir=Path(args.out),
        image_topic=args.image_topic,
        cmd_topic=args.cmd_topic,
        max_dt=args.max_dt,
        every=args.every,
    )


if __name__ == "__main__":
    main()
