# Route2 BC — FINAL v2 (Roundabout)

Status:
- Runs continuous laps around the roundabout
- Stays inside lane consistently (stable repeatable behavior)

Model (local only, NOT committed):
- Route2/models/route2_steer_v2.pt
- HOST backup:
  ~/Downloads/acc_models/Route2/route2_steer_v2_FINAL_20260222_235529.pt

Dataset:
- Route2 combo: route2_combo_v2
- Built from: route2_take01 + route2_center_take02

Runtime params (verified):
- linear_x = 0.12
- max_ang  = 1.5
- rate_hz  = 12.0

Controller:
- bc_controller_route2_v2.py loads route2_steer_v2.pt
