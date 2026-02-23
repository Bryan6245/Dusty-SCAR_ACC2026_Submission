# Route1 Final (BC)

Final model file (local, not committed):
- Route1/models/route1_steer_v8.pt
- Checkpoint backup: Route1/models/checkpoints/route1_steer_v8_FINAL_*.pt

Runtime params used for final test:
- linear_x: 0.12
- max_ang: 1.5
- rate_hz: 12.0

Controller script:
- New_Routes/shared/scripts/bc_controller_route1_v8.py

Training:
- combo: Route1/combo/route1_combo_v8
- epochs: 25
- train script: New_Routes/shared/scripts/train_route1_v8.py
