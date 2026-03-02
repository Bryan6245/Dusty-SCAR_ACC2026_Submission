# Quick Start — ACC 2026 Virtual Stage (Required Terminals)

This repo runs a ROS2 autonomy pipeline in Quanser QLabs CityScape/SDCS.
**Competition compliance:** `qvl` is used only inside `virtual-qcar2` for environment setup (map + actors). Vehicle control/sensing is via ROS topics.

---

## TERMINAL 1 (HOST): Containers + Map + Actors (virtual-qcar2)
Run on the host (Ubuntu), outside ISAAC:

```bash
docker start virtual-qcar2 isaac_ros_dev-x86_64-container
docker exec -it virtual-qcar2 bash -lc "cd /home/qcar2_scripts/python && python3 Base_Scenarios_Python/Setup_Competition_Map.py"

docker exec -it virtual-qcar2 python3 /tmp/infrastructure_actors.py --no-wait
docker exec -it virtual-qcar2 python3 /tmp/people_actors.py --no-wait
docker exec -it virtual-qcar2 python3 /tmp/traffic_lights.py --no-wait

## Terminal 2 (ISAAC): SLAM + NAV Bringup

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch qcar2_nodes qcar2_slam_and_nav_bringup_virtual_launch.py

##Terminal 3 (ISAAC): Disable Nav2 publishing/cmd_vel_nav 

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 param set /lifecycle_manager_navigation autostart false || true
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"

## Terminal 4 (ISAAC): YOLO Detector (GPU)

cd /workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ws/install/setup.bash

python3 -c "import ultralytics; print('ultralytics ok')" || python3 -m pip install --user -U ultralytics

LATEST="$(ls -1t dusty_scar/dusty_scar/models/yolo/*.pt | head -1)"
echo "YOLO_WEIGHTS=$LATEST"

pkill -f yolo_frontcam_detector.py || true
CUDA_VISIBLE_DEVICES=0 python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/yolo_frontcam_detector.py --ros-args \
  -p weights:="$LATEST" \
  -p image_topic:=/camera/color_image \
  -p conf:=0.25 \
  -p device_id:=0 \
  -p rate_hz:=6.0

## Terminal 5 (ISAAC): Traffic-law Supervisor

cd /workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ws/install/setup.bash

pkill -f traffic_rules_supervisor_v5_stopfix.py || true
python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/traffic_rules_supervisor_v5_stopfix.py --ros-args \
  -p cmd_in:=/cmd_vel_bc \
  -p cmd_out:=/cmd_vel_nav \
  -p dets_json:=/yolo/detections_json \
  -p cw_stop_ymax:=0.62 \
  -p cw_stop_h:=0.08 \
  -p debug:=true

## Terminal 6 (ISAAC): Annotated Live feed (Front camera)

source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ws/install/setup.bash
export QT_X11_NO_MITSHM=1
export GDK_DISABLE_SHM=1
export XDG_RUNTIME_DIR=/tmp/runtime-admin
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

ros2 run image_view image_view --ros-args -r image:=/yolo/annotated

## Terminal 7 (ISAAC): Autonomous Run from Routes 1 -> 8 -> 1

source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/ws/install/setup.bash
cd /workspaces/isaac_ros-dev/ws/src/Dusty-SCAR_ACC2026_Submission

pkill -f route_switch_181_once_mux_tf.py || true
pkill -f bc_controller_route1_v8.py || true
pkill -f bc_controller_route8_auto.py || true

python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/route_switch_181_once_mux_tf.py --ros-args \
  -p world_frame:=odom -p base_frame:=base_link \
  -p p18_x:=3.055 -p p18_y:=1.622 -p p18_r_in:=0.55 -p p18_r_out:=1.00 \
  -p p81_x:=1.480 -p p81_y:=0.849 -p p81_r_in:=0.65 -p p81_r_out:=1.20 \
  -p min_hold_s:=2.0
