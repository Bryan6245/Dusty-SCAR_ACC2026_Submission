# Route7 BC — FINAL

Status:
- Route7 completes the full route reliably.

Final:
- model (local only): New_Routes/Route7/models/route7_steer_v8.pt
- controller: New_Routes/shared/scripts/bc_controller_route7_v8.py

Runtime params:
- linear_x=0.12
- max_ang=1.5
- rate_hz=12.0

Data used (high level):
- base: route7_take01
- intersection fix: route7_fix_intersection_take02
- centered laps: route7_center_take04, route7_center_take05
- second-half focus: route7_secondhalf_take06 (weighted 2x)
- curve micro-fix: route7_fix_curve_take07
