import time
import argparse

from qvl.qlabs import QuanserInteractiveLabs
from qvl.traffic_light import QLabsTrafficLight

# Put all traffic lights at a visible height (road is ~0.006)
TL_Z = 0.006  # common in QLabs examples for traffic lights

# (actor_id, [x,y,z], yaw_deg, config, scale)
TRAFFIC_LIGHTS = [
    (600, [-0.489, 0.330, TL_Z], 180.0, 0, [0.15, 0.15, 0.15]),
    (601, [-0.370, 1.441, TL_Z],  90.0, 0, [0.15, 0.15, 0.15]),
    (602, [ 0.587, 1.469, TL_Z],   0.0, 0, [0.15, 0.15, 0.15]),
    (603, [ 0.665, 0.287, TL_Z], 270.0, 0, [0.15, 0.15, 0.15]),
]

# Realistic intersection behavior:
# Group lights that should go GREEN together.
PHASE_A = [600, 602]
PHASE_B = [601, 603]

# If you prefer all lights cycle together, set this to True
CYCLE_ALL_TOGETHER = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--green", type=float, default=6.0)
    ap.add_argument("--yellow", type=float, default=1.5)
    ap.add_argument("--all_red", type=float, default=0.5)
    ap.add_argument("--wait", action="store_true", help="show QLabs confirmations")
    ap.add_argument("--no-wait", action="store_true", help="no confirmations")
    args = ap.parse_args()

    WAIT = True
    if args.no_wait:
        WAIT = False
    elif args.wait:
        WAIT = True

    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    if not qlabs.open(args.host):
        raise SystemExit("ERROR: could not connect to QLabs")

    # One object per light (reliable)
    lights = {}

    # Spawn/update
    for (actor_id, xyz, yaw_deg, cfg, scl) in TRAFFIC_LIGHTS:
        tl = QLabsTrafficLight(qlabs)
        lights[actor_id] = tl

        # destroy so reruns apply changes
        try:
            tl.actorNumber = actor_id
            tl.destroy()
        except Exception:
            pass

        status = tl.spawn_id_degrees(
            actorNumber=actor_id,
            location=xyz,
            rotation=[0, 0, yaw_deg],
            scale=scl,
            configuration=cfg,
            waitForConfirmation=WAIT
        )

        ok = tl.set_color(color=tl.COLOR_RED, waitForConfirmation=WAIT)
        print(f"[spawn] id={actor_id} status={status} set_red_ok={ok} xyz={xyz} yaw={yaw_deg} cfg={cfg} scale={scl}")

    def set_group(ids, color):
        for aid in ids:
            tl = lights.get(aid)
            if tl is None:
                continue
            tl.set_color(color=color, waitForConfirmation=False)

    def set_all(color):
        for tl in lights.values():
            tl.set_color(color=color, waitForConfirmation=False)

    print("Cycling traffic lights (Ctrl+C to stop)...")
    try:
        while True:
            # ---- Phase A green / Phase B red (or all together) ----
            if CYCLE_ALL_TOGETHER:
                set_all(QLabsTrafficLight(qlabs).COLOR_GREEN)
            else:
                set_group(PHASE_A, QLabsTrafficLight(qlabs).COLOR_GREEN)
                set_group(PHASE_B, QLabsTrafficLight(qlabs).COLOR_RED)
            time.sleep(args.green)

            # ---- Yellow for the currently-green phase (or all) ----
            if CYCLE_ALL_TOGETHER:
                set_all(QLabsTrafficLight(qlabs).COLOR_YELLOW)
            else:
                set_group(PHASE_A, QLabsTrafficLight(qlabs).COLOR_YELLOW)
            time.sleep(args.yellow)

            # ---- All red brief ----
            set_all(QLabsTrafficLight(qlabs).COLOR_RED)
            time.sleep(args.all_red)

            # ---- Swap phases ----
            if CYCLE_ALL_TOGETHER:
                set_all(QLabsTrafficLight(qlabs).COLOR_GREEN)
            else:
                set_group(PHASE_B, QLabsTrafficLight(qlabs).COLOR_GREEN)
                set_group(PHASE_A, QLabsTrafficLight(qlabs).COLOR_RED)
            time.sleep(args.green)

            if CYCLE_ALL_TOGETHER:
                set_all(QLabsTrafficLight(qlabs).COLOR_YELLOW)
            else:
                set_group(PHASE_B, QLabsTrafficLight(qlabs).COLOR_YELLOW)
            time.sleep(args.yellow)

            set_all(QLabsTrafficLight(qlabs).COLOR_RED)
            time.sleep(args.all_red)

    except KeyboardInterrupt:
        print("\nStopped cycling.")
    finally:
        # leave them red on exit
        try:
            for tl in lights.values():
                tl.set_color(color=QLabsTrafficLight(qlabs).COLOR_RED, waitForConfirmation=False)
        except Exception:
            pass
        qlabs.close()


if __name__ == "__main__":
    main()

