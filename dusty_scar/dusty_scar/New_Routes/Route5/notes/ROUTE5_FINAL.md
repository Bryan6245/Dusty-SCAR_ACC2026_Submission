# Route5 BC — FINAL

Status:
- Route5 runs reliably and follows route well.

Runtime params:
- linear_x = 0.12
- max_ang  = 1.5
- rate_hz  = 12.0

Model (local only, not committed):
- New_Routes/Route5/models/route5_steer_v?.pt
- HOST backup: ~/Downloads/acc_models/Route5/

Data used (high level):
- take01 + center_take02
- secondhalf_take03
- micro-fix take04 (final small area)
Controller:
- New_Routes/shared/scripts/bc_controller_route5_v?.py
