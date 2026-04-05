#!/usr/bin/env bash
set -euo pipefail

ISAAC_CONTAINER="isaac_ros_dev-x86_64-container"
VIRTUAL_CONTAINER="virtual-qcar2"

HOST_REPO="$HOME/Documents/ACC_Development/isaac_ros_common_quanser_copy_20260125_003516/ws/src/Dusty-SCAR_ACC2026_Submission"
ISAAC_REPO="/workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission"

SPAWN_HOST="$HOST_REPO/dusty_scar/dusty_scar/qlabs_setup_actors/pedestrian_spawn_pickup/spawn_single_pedestrian.py"
SPAWN_CONT="/tmp/spawn_single_pedestrian.py"
GOTO_CONT="$ISAAC_REPO/dusty_scar/dusty_scar/New_Routes/shared/scripts/taxi_prompt/goto_pose_once.py"

PED_ID=799
PED_CFG=6
PED_SCALE=0.15
PED_YAW=0.0

run_goal () {
  local GX="$1"
  local GY="$2"
  local GYAW="$3"

  docker exec -i "$ISAAC_CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /workspaces/isaac_ros-dev/ws/install/setup.bash &&
    export ROS_DOMAIN_ID=0 &&
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
    python3 '$GOTO_CONT' --x '$GX' --y '$GY' --yaw '$GYAW'
  "
}

spawn_ped () {
  local PX="$1"
  local PY="$2"
  local PZ="$3"

  docker exec -i "$VIRTUAL_CONTAINER" python3 "$SPAWN_CONT" \
    --no-wait \
    --id "$PED_ID" \
    --x "$PX" --y "$PY" --z "$PZ" \
    --yaw "$PED_YAW" \
    --cfg "$PED_CFG" \
    --scale "$PED_SCALE"
}

despawn_ped () {
  docker exec -i "$VIRTUAL_CONTAINER" python3 "$SPAWN_CONT" \
    --no-wait \
    --id "$PED_ID" \
    --x 0 --y 0 --z 0.006 \
    --despawn
}

echo
echo "=== TAXI PROMPT MISSION ==="

echo "Enter HUB coordinates: X Y YAW"
read -r HUB_X HUB_Y HUB_YAW

echo "Enter PICKUP coordinates: X Y Z"
read -r PICKUP_X PICKUP_Y PICKUP_Z

echo "Enter DROPOFF coordinates: X Y Z"
read -r DROPOFF_X DROPOFF_Y DROPOFF_Z

docker cp "$SPAWN_HOST" "$VIRTUAL_CONTAINER:$SPAWN_CONT"

echo
echo "[1/5] Spawning pedestrian at pickup..."
spawn_ped "$PICKUP_X" "$PICKUP_Y" "$PICKUP_Z"

echo "[2/5] Driving to pickup..."
run_goal "$PICKUP_X" "$PICKUP_Y" 0.0

echo "At pickup. Waiting 3 seconds..."
sleep 3

echo "Passenger boarded. Despawning pickup pedestrian..."
despawn_ped

echo
echo "[3/5] Driving to dropoff..."
run_goal "$DROPOFF_X" "$DROPOFF_Y" 0.0

echo "[4/5] Spawning pedestrian at dropoff..."
spawn_ped "$DROPOFF_X" "$DROPOFF_Y" "$DROPOFF_Z"

echo "At dropoff. Waiting 3 seconds..."
sleep 3

echo
echo "[5/5] Returning to hub..."
run_goal "$HUB_X" "$HUB_Y" "$HUB_YAW"

echo "Mission complete. Vehicle should remain at hub."
