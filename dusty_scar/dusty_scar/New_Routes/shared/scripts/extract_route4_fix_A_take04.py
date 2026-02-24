#!/usr/bin/env python3
import csv
from pathlib import Path

import numpy as np
import cv2

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMG_TOPIC = "/camera/color_image"
CMD_TOPIC = "/cmd_vel"
TARGET_HZ = 10.0
MAX_DT = 0.12  # tighter for micro-fix

BAG_DIR = "/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route4/bags/route4_fix_A_take04"
OUT_DIR = "/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission/dusty_scar/dusty_scar/New_Routes/Route4/extracted/route4_fix_A_take04"

def rosimg_to_bgr(msg):
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        return None

    enc = (msg.encoding or "").lower()
    step = int(getattr(msg, "step", w * 3))
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    # Data should be at least h*step bytes for packed rows
    if len(buf) < h * step:
        return None

    # reshape to rows
    rows = buf[:h * step].reshape((h, step))

    if enc in ("rgb8", "bgr8"):
        row_bytes = w * 3
        if step < row_bytes:
            return None
        img = rows[:, :row_bytes].reshape((h, w, 3))
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img  # bgr8

    if enc in ("rgba8", "bgra8"):
        row_bytes = w * 4
        if step < row_bytes:
            return None
        img = rows[:, :row_bytes].reshape((h, w, 4))
        if enc == "rgba8":
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    # If it's something unexpected, skip it for now
    return None

def main():
    out = Path(OUT_DIR)
    img_dir = out / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=BAG_DIR, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                                    output_serialization_format="cdr")

    # Read topic types
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if IMG_TOPIC not in topic_types:
        raise RuntimeError(f"Missing {IMG_TOPIC}. Found: {list(topic_types.keys())}")
    if CMD_TOPIC not in topic_types:
        raise RuntimeError(f"Missing {CMD_TOPIC}. Found: {list(topic_types.keys())}")

    ImgMsg = get_message(topic_types[IMG_TOPIC])
    CmdMsg = get_message(topic_types[CMD_TOPIC])

    # Reopen for a clean pass
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    cmds = []    # (t, lin, ang)
    images = []  # (t, imgmsg)

    while reader.has_next():
        topic, data, t_bag = reader.read_next()
        t = t_bag * 1e-9
        if topic == CMD_TOPIC:
            msg = deserialize_message(data, CmdMsg)
            cmds.append((t, float(msg.linear.x), float(msg.angular.z)))
        elif topic == IMG_TOPIC:
            msg = deserialize_message(data, ImgMsg)
            images.append((t, msg))

    if not cmds or not images:
        raise RuntimeError(f"Found cmds={len(cmds)} images={len(images)}; need both > 0")

    cmd_times = np.array([c[0] for c in cmds], dtype=np.float64)
    cmd_lin  = np.array([c[1] for c in cmds], dtype=np.float32)
    cmd_ang  = np.array([c[2] for c in cmds], dtype=np.float32)

    keep_dt = 1.0 / TARGET_HZ
    last_keep_t = -1e9
    idx = 0
    rows = []
    skipped_decode = 0
    skipped_dt = 0

    for t_img, img_msg in images:
        if t_img - last_keep_t < keep_dt:
            continue

        j = int(np.argmin(np.abs(cmd_times - t_img)))
        dt = float(abs(cmd_times[j] - t_img))
        if dt > MAX_DT:
            skipped_dt += 1
            continue

        img_bgr = rosimg_to_bgr(img_msg)
        if img_bgr is None or img_bgr.size == 0:
            skipped_decode += 1
            continue

        fname = f"frame_{idx:06d}.jpg"
        ok = cv2.imwrite(str(img_dir / fname), img_bgr)
        if not ok:
            skipped_decode += 1
            continue

        rows.append((f"images/{fname}", float(cmd_lin[j]), float(cmd_ang[j]), float(t_img), dt))
        idx += 1
        last_keep_t = t_img

    with open(out / "labels.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["image", "linear_x", "angular_z", "t", "match_dt"])
        w.writerows(rows)

    print(f"✅ Wrote {len(rows)} samples to: {OUT_DIR}")
    print(f"   Skipped (dt too large): {skipped_dt}")
    print(f"   Skipped (decode/write): {skipped_decode}")
    print(f"   Images: {img_dir}")
    print(f"   Labels: {out / 'labels.csv'}")

if __name__ == "__main__":
    main()
