#!/usr/bin/env python3
import json
import math
import subprocess
import time
from pathlib import Path

VIRTUAL_CONTAINER = "virtual-qcar2"
ISAAC_CONTAINER = "isaac_ros_dev-x86_64-container"
SPAWN_CONT = "/tmp/spawn_single_pedesterian.py"

REPO = Path.home() / "Documents/ACC_Development/isaac_ros_common_quanser_copy_20260125_003516/ws/src/Dusty-SCAR_ACC2026_Submission"
THIS_DIR = REPO / "dusty_scar/dusty_scar/qlabs_setup_actors/pedestrian_spawn_pickup"
SPAWN_HOST = THIS_DIR / "spawn_single_pedesterian.py"
MISSION_JSON = THIS_DIR / "last_taxi_mission.json"
ACTIVE_TARGET_JSON = THIS_DIR / "active_target.json"

PED_ID = 799
PED_Z_DEFAULT = 0.006
SIDEWALK_DISTANCE_M = 0.205

# Set your actual taxi hub road coordinate here
TAXI_HUB_XYZ = [0.0, 0.0, 0.006]

# Pose tracking config
ARRIVAL_TOL_M = 0.18
ARRIVAL_STABLE_POLLS = 3
POSE_POLL_S = 0.5
TARGET_TIMEOUT_S = 240.0

# Optional future hook for your BC/route executor.
ROUTE_EXECUTOR = None


def run(cmd, capture_output=False):
    print("+", " ".join(str(c) for c in cmd))
    return subprocess.run(
        [str(c) for c in cmd],
        check=True,
        text=True,
        capture_output=capture_output,
    )


def sync_spawn_script():
    run(["docker", "cp", str(SPAWN_HOST), f"{VIRTUAL_CONTAINER}:{SPAWN_CONT}"])


def spawn_ped(actor_id, xyz):
    run([
        "docker", "exec", "-i", VIRTUAL_CONTAINER,
        "python3", SPAWN_CONT,
        "--no-wait",
        "--id", str(actor_id),
        "--x", str(xyz[0]),
        "--y", str(xyz[1]),
        "--z", str(xyz[2]),
    ])


def despawn_ped(actor_id):
    run([
        "docker", "exec", "-i", VIRTUAL_CONTAINER,
        "python3", SPAWN_CONT,
        "--no-wait",
        "--id", str(actor_id),
        "--x", "0",
        "--y", "0",
        "--z", str(PED_Z_DEFAULT),
        "--despawn",
    ])


def compute_offset_point(road_xyz, degrees, distance):
    x, y, z = road_xyz
    rad = math.radians(degrees)
    px = x + distance * math.cos(rad)
    py = y + distance * math.sin(rad)
    return [round(px, 3), round(py, 3), z]


def prompt_pickup():
    while True:
        raw = input("Enter PICKUP as x y z degree, or q to quit -> ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None

        parts = raw.split()
        if len(parts) != 4:
            print("Please enter exactly 4 values: x y z degree")
            continue

        try:
            x, y, z, deg = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
        except ValueError:
            print("Invalid number. Try again.")
            continue

        if not (0.0 <= deg <= 359.0):
            print("Degree must be between 0 and 359.")
            continue

        return [x, y, z], deg


def prompt_dropoff():
    while True:
        raw = input("Enter DROPOFF as x y z, or q to quit -> ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            return None

        parts = raw.split()
        if len(parts) != 3:
            print("Please enter exactly 3 values: x y z")
            continue

        try:
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("Invalid number. Try again.")
            continue

        return [x, y, z]


def save_state(mission):
    MISSION_JSON.write_text(json.dumps(mission, indent=2))
    print(f"\nSaved mission to: {MISSION_JSON}")


def dispatch_target(label, xyz):
    target = {
        "label": label,
        "target_xyz": xyz,
        "timestamp": time.time(),
    }
    ACTIVE_TARGET_JSON.write_text(json.dumps(target, indent=2))

    print(f"\n=== DISPATCH {label.upper()} TARGET ===")
    print(json.dumps(target, indent=2))

    if ROUTE_EXECUTOR is not None:
        cmd = list(ROUTE_EXECUTOR) + [
            "--label", label,
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
        ]
        run(cmd)


def get_current_pose():
    py_block = r"""
python3 - <<'__POSEPY__'
import math
import json
import time
import rclpy
import tf2_ros
from rclpy.node import Node

WORLD = "odom"
BASE = "base_link"

rclpy.init()
node = Node("taxi_pose_snap")
buf = tf2_ros.Buffer()
lis = tf2_ros.TransformListener(buf, node)

tf = None
t0 = time.time()
while time.time() - t0 < 2.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        tf = buf.lookup_transform(WORLD, BASE, rclpy.time.Time())
        break
    except Exception:
        pass

if tf is None:
    print("POSE_UNAVAILABLE")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

x = tf.transform.translation.x
y = tf.transform.translation.y
z = tf.transform.translation.z

qx = tf.transform.rotation.x
qy = tf.transform.rotation.y
qz = tf.transform.rotation.z
qw = tf.transform.rotation.w

yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
print(json.dumps({"x": x, "y": y, "z": z, "yaw": yaw}))
node.destroy_node()
rclpy.shutdown()
__POSEPY__
""".strip()

    bash_cmd = " && ".join([
        "source /opt/ros/humble/setup.bash",
        "source /workspaces/isaac_ros-dev/ws/install/setup.bash",
        "export ROS_DOMAIN_ID=0",
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        py_block,
    ])

    proc = subprocess.run(
        ["docker", "exec", ISAAC_CONTAINER, "bash", "-lc", bash_cmd],
        text=True,
        capture_output=True,
    )

    if proc.returncode != 0:
        return None

    out = proc.stdout.strip().splitlines()
    if not out:
        return None

    last = out[-1].strip()
    if last == "POSE_UNAVAILABLE":
        return None

    try:
        return json.loads(last)
    except Exception:
        return None


def wait_until_arrived(label, xyz):
    tx, ty, _ = xyz
    stable_hits = 0
    t0 = time.time()
    last_log = 0.0

    print(f"\n[WAIT] Waiting for automatic arrival at {label.upper()} ...")

    while True:
        pose = get_current_pose()
        now = time.time()

        if pose is None:
            if now - last_log > 2.0:
                print("[WAIT] Pose unavailable right now. Retrying...")
                last_log = now
            time.sleep(POSE_POLL_S)
            if now - t0 > TARGET_TIMEOUT_S:
                raise RuntimeError(f"Timed out waiting for {label} pose tracking.")
            continue

        dx = pose["x"] - tx
        dy = pose["y"] - ty
        dist = math.hypot(dx, dy)

        if now - last_log > 1.0:
            print(
                f"[WAIT] {label}: car=({pose['x']:.3f}, {pose['y']:.3f}) "
                f"target=({tx:.3f}, {ty:.3f}) dist={dist:.3f}"
            )
            last_log = now

        if dist <= ARRIVAL_TOL_M:
            stable_hits += 1
        else:
            stable_hits = 0

        if stable_hits >= ARRIVAL_STABLE_POLLS:
            print(f"[ARRIVED] {label.upper()} reached.")
            return

        if now - t0 > TARGET_TIMEOUT_S:
            raise RuntimeError(f"Timed out waiting for arrival at {label}.")

        time.sleep(POSE_POLL_S)


def stop_for_3_seconds(label):
    print(f"[STOP] {label.upper()}: waiting 3 seconds...")
    time.sleep(3)


def main():
    print("\n=== TAXI COMPETITION DAY LOOP ===")
    print("Pickup format : x y z degree")
    print("Dropoff format: x y z")
    print(f"Pickup sidewalk distance is fixed at {SIDEWALK_DISTANCE_M} m")
    print(f"Taxi hub is set to: {TAXI_HUB_XYZ}")
    print(f"Arrival tolerance: {ARRIVAL_TOL_M} m")
    print("Flow: pickup -> auto-arrival -> wait 3s -> dropoff -> auto-arrival -> wait 3s -> hub -> auto-arrival -> repeat")
    print("Type q at the pickup or dropoff prompt to end the program.\n")

    sync_spawn_script()
    mission_count = 0

    while True:
        pickup_data = prompt_pickup()
        if pickup_data is None:
            print("\nExiting program.")
            break

        pickup_road_xyz, pickup_deg = pickup_data
        pickup_sidewalk_xyz = compute_offset_point(
            pickup_road_xyz, pickup_deg, SIDEWALK_DISTANCE_M
        )

        dropoff_road_xyz = prompt_dropoff()
        if dropoff_road_xyz is None:
            print("\nExiting program.")
            break

        mission_count += 1
        mission = {
            "mission_index": mission_count,
            "pickup_road_xyz": pickup_road_xyz,
            "pickup_spawn_degree_deg": pickup_deg,
            "pickup_spawn_distance_m": SIDEWALK_DISTANCE_M,
            "pickup_sidewalk_spawn_xyz": pickup_sidewalk_xyz,
            "dropoff_road_xyz": dropoff_road_xyz,
            "taxi_hub_xyz": TAXI_HUB_XYZ,
            "arrival_tolerance_m": ARRIVAL_TOL_M,
        }

        save_state(mission)

        print("\n=== CURRENT MISSION ===")
        print(json.dumps(mission, indent=2))

        try:
            print("\nSpawning pickup pedestrian...")
            spawn_ped(PED_ID, pickup_sidewalk_xyz)

            dispatch_target("pickup", pickup_road_xyz)
            wait_until_arrived("pickup", pickup_road_xyz)
            stop_for_3_seconds("pickup")

            print("Despawning pickup pedestrian...")
            despawn_ped(PED_ID)

            dispatch_target("dropoff", dropoff_road_xyz)
            wait_until_arrived("dropoff", dropoff_road_xyz)
            stop_for_3_seconds("dropoff")

            dispatch_target("hub", TAXI_HUB_XYZ)
            wait_until_arrived("hub", TAXI_HUB_XYZ)

            print("\nMission cycle complete. Ready for the next pickup/dropoff pair.\n")

        except RuntimeError as e:
            print(f"\n[ERROR] {e}")
            print("[ERROR] Mission cycle aborted. Pedestrian will be despawned if needed.")
            try:
                despawn_ped(PED_ID)
            except Exception:
                pass


if __name__ == "__main__":
    main()
