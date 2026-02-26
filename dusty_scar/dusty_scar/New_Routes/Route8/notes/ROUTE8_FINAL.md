# Route8 BC — FINAL

Status:
- Route8 completes the full route reliably.

Final artifacts:
- controller: New_Routes/shared/scripts/bc_controller_route8_v5.py (or latest)
- model (local only): New_Routes/Route8/models/route8_steer_v5.pt (or latest)

Runtime params:
- linear_x=0.12
- max_ang=1.5
- rate_hz=12.0

Data notes:
- base takes: route8_take01, route8_center_take02
- fix A: route8_fix_A_take03 + recovery_take04 (weighted)
- fix B: route8_fix_B_take05
