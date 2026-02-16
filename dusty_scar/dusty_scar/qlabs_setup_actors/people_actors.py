#!/usr/bin/env python3
import argparse
from qvl.qlabs import QuanserInteractiveLabs
from qvl.person import QLabsPerson  # correct class

# Format: (actor_id, [x, y, z], yaw_deg, configuration, scale)
# Using configs 6–11 at least once each, then reusing a few because we have 10 total people.
SCALE_BIG = [0.15, 0.15, 0.15]

PEOPLE = [
    # Original person (you said yaw = 130)
    (700, [2.197, -1.066, 0.006], 130.0, 6, SCALE_BIG),

    # New people (configs 7–11 used once each here)
    (701, [2.408, 1.294, 0.006], 180.0, 7, SCALE_BIG),
    (702, [1.676, 0.252, 0.006], 180.0, 8, SCALE_BIG),   # fixed the typo "1.676. 0.252"
    (703, [1.728, 2.108, 0.006],  90.0, 9, SCALE_BIG),
    (704, [0.762, 1.312, 0.006], 270.0, 10, SCALE_BIG),
    (705, [0.573, 3.743, 0.006],  45.0, 11, SCALE_BIG),

    # Extra locations (reusing 6–9)
    (706, [1.415, 1.275, 0.006], 270.0, 6, SCALE_BIG),
    (707, [-0.443, 1.299, 0.006], 270.0, 7, SCALE_BIG),
    (708, [-1.464, 0.188, 0.006],   0.0, 8, SCALE_BIG),
    (709, [1.731, 2.432, 0.006],  90.0, 9, SCALE_BIG),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--wait", action="store_true", help="show QLabs confirmations")
    ap.add_argument("--no-wait", action="store_true", help="no confirmations")
    args = ap.parse_args()

    WAIT = True
    if args.no_wait:
        WAIT = False
    elif args.wait:
        WAIT = True

    qlabs = QuanserInteractiveLabs()
    print("Connecting...")
    if not qlabs.open(args.host):
        raise SystemExit("ERROR: could not connect to QLabs")

    person = QLabsPerson(qlabs)

    for (actor_id, xyz, yaw_deg, cfg, scale) in PEOPLE:
        # destroy first so reruns apply changes
        try:
            person.actorNumber = actor_id
            person.destroy()
        except Exception:
            pass

        status = person.spawn_id_degrees(
            actorNumber=actor_id,
            location=xyz,
            rotation=[0, 0, yaw_deg],
            scale=scale,
            configuration=cfg,
            waitForConfirmation=WAIT
        )
        print(f"[person] id={actor_id} status={status} xyz={xyz} yaw={yaw_deg:.1f} cfg={cfg} scale={scale}")

    qlabs.close()
    print("Done.")


if __name__ == "__main__":
    main()
