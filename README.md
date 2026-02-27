# Dusty-SCAR ACC 2026 Submission
## Demo: Traffic-law Supervisor (Stop/Yield/Lights)
This command enables traffic-rule compliance using YOLO detections (stop/yield signs + traffic lights + crosswalk stopline).
Run inside the ISAAC container after YOLO + BC are running and Nav2 /controller_server is NOT publishing `/cmd_vel_nav`:

```bash
pkill -f traffic_rules_supervisor_v5_stopfix.py || true
python3 dusty_scar/dusty_scar/New_Routes/shared/scripts/traffic_rules_supervisor_v5_stopfix.py --ros-args \
  -p cmd_in:=/cmd_vel_bc \
  -p cmd_out:=/cmd_vel_nav \
  -p dets_json:=/yolo/detections_json \
  -p cw_stop_ymax:=0.62 \
  -p cw_stop_h:=0.08 \
  -p debug:=true
