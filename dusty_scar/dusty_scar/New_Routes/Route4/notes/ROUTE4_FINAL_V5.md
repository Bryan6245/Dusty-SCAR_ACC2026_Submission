# Route4 BC — FINAL v5

Status:
- Runs the full Route4 loop reliably.
- Minor: one small spot where wheel slightly touches curb (acceptable for ship).

Runtime params (best):
- linear_x = 0.12
- max_ang  = 1.5
- rate_hz  = 12.0

Model (local only, not committed):
- New_Routes/Route4/models/route4_steer_v5.pt
- HOST backup: ~/Downloads/acc_models/Route4/route4_steer_v5.pt

Data used:
- take01, center_take02, take03
- Fix A take04 (kept)
- Roundabout→post-intersection micro-fix take06 (kept)

Controller:
- New_Routes/shared/scripts/bc_controller_route4_v5.py
