# Route4 BC — WIP

Data:
- route4_take01 recorded (long bag) + extracted: 4017 samples
- combo: Route4/combo/route4_combo_v1 (from take01)

Training:
- train script: New_Routes/shared/scripts/train_route4_v1.py
- epochs: 25
- model (local only, not committed): Route4/models/route4_steer_v1.pt

Controller:
- New_Routes/shared/scripts/bc_controller_route4_v1.py
Test params:
- linear_x=0.12, max_ang=1.5, rate_hz=12.0

Next:
- test v1 behavior
- likely record center_take02 + tight-turn micro-fix takes
