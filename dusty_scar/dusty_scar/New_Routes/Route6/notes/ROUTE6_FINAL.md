# Route6 BC — FINAL

Status:
- Route6 completes the full route reliably.

Runtime params:
- linear_x = 0.12
- max_ang  = 1.5
- rate_hz  = 12.0

Model (local only, not committed):
- New_Routes/Route6/models/route6_steer_v?.pt
- HOST backup: ~/Downloads/acc_models/Route6/

Data used:
- take01 + center_take02
- fix1_take03 + fix1_take04
- fix2_take05

Controller:
- New_Routes/shared/scripts/bc_controller_route6_v?.py
