# Route3 BC — WIP (trained v2, pending test)

Data:
- route3_take01 extracted: 1584 samples
- route3_center_take02 extracted: 2101 samples (bag ~474s, labels from /cmd_vel)
- combo: Route3/combo/route3_combo_v2
  - total samples: 3685 (labels.csv lines: 3686 incl header)

Training:
- script: New_Routes/shared/scripts/train_route3_v2.py
- epochs: 25
- best val MSE: 0.03676 (epoch 20)
- model (local only, not committed): Route3/models/route3_steer_v2.pt

Next time:
- test with controller: bc_controller_route3_v2.py
- params: linear_x=0.12, max_ang=1.5, rate_hz=12
- if it drifts/off-route: record a short recovery take and weight it in combo_v3
