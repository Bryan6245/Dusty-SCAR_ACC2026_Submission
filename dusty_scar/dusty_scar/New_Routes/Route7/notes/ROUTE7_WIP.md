# Route7 BC — WIP

Data:
- take01 recorded (full route)
- issue: goes off trajectory at intersection
- recorded micro-fix: fix_intersection_take02 (multiple passes)

Training:
- v1: trained from take01
- v2: trained from combo_v2 (take01 + intersection fix)
Models are local only (not committed).

Runtime params:
- linear_x=0.12, max_ang=1.5, rate_hz=12

Next:
- test v2 repeatedly at the intersection
- if still off: record recovery-style intersection take and retrain v3
