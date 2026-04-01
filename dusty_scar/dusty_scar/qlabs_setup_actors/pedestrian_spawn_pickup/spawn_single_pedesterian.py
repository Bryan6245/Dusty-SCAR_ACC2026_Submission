#!/usr/bin/env python3
import argparse
from qvl.qlabs import QuanserInteractiveLabs
from qvl.person import QLabsPerson

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--id", type=int, default=799)
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--z", type=float, default=0.006)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--cfg", type=int, default=6)
    ap.add_argument("--scale", type=float, default=0.15)
    ap.add_argument("--despawn", action="store_true")
    ap.add_argument("--wait", action="store_true")
    ap.add_argument("--no-wait", action="store_true")
    args = ap.parse_args()

    wait = True
    if args.no_wait:
        wait = False
    elif args.wait:
        wait = True

    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    if not qlabs.open(args.host):
        raise SystemExit("ERROR: could not connect to QLabs")

    person = QLabsPerson(qlabs)
    person.actorNumber = args.id

    try:
        person.destroy()
    except Exception:
        pass

    if args.despawn:
        print(f"[pedestrian] despawned id={args.id}")
        qlabs.close()
        return

    status = person.spawn_id_degrees(
        actorNumber=args.id,
        location=[args.x, args.y, args.z],
        rotation=[0.0, 0.0, args.yaw],
        scale=[args.scale, args.scale, args.scale],
        configuration=args.cfg,
        waitForConfirmation=wait,
    )

    print(
        f"[pedestrian] spawned id={args.id} status={status} "
        f"xyz={[args.x, args.y, args.z]} yaw={args.yaw} cfg={args.cfg}"
    )
    qlabs.close()

if __name__ == "__main__":
    main()
