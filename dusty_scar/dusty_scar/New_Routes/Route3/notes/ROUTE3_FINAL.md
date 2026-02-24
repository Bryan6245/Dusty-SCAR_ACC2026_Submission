# Route3 BC — FINAL

Status:
- Route3 behavior is “good to ship” (lane-centered, stable loop)

Runtime params (best):
- linear_x = 0.12
- max_ang  = 1.5
- rate_hz  = 12.0

Model (local only, not committed):
- New_Routes/Route3/models/route3_steer_v?.pt
- HOST backup: ~/Downloads/acc_models/Route3/

Datasets used:
- Base takes: take01, center_take02, take03
- Targeted micro-fix takes: fix_roundabout_entry_take04, fix_yellow_take05, fix_lane_touch_take06, fix_post_4way_take07, fix_lane_center_take08
