import math
import argparse

from qvl.qlabs import QuanserInteractiveLabs
from qvl.crosswalk import QLabsCrosswalk

# Road signage actors
from qvl.stop_sign import QLabsStopSign
from qvl.yield_sign import QLabsYieldSign
from qvl.roundabout_sign import QLabsRoundaboutSign

# (Optional for later)
# from qvl.traffic_cone import QLabsTrafficCone


# === Crosswalk Bounds ===
Y0, Y1, Z = 0.570, 1.275, 0.006
X1 = 0.745
X2 = -0.487

# === Crosswalk Scale settings ===
base = 0.10
length_factor = 0.49
scale_v = [base * length_factor, base, base]   # vertical edges
scale_h = [base * length_factor, base, base]   # horizontal edges

CROSSWALK_CONFIG = 0

# === Sign scaling ===
YIELD_SCALE = [0.10, 0.10, 0.10]
STOP_SCALE = [0.10, 0.10, 0.10]
ROUNDABOUT_SCALE = [0.10, 0.10, 0.10]


def center_and_yaw(A, B):
    center = [(A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0, (A[2] + B[2]) / 2.0]
    yaw = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
    return center, yaw


def spawn_signs(label, actor_obj, sign_list, wait=True):
    if not sign_list:
        print(f"[{label}] No spawns configured yet.")
        return

    for (actor_id, xyz, yaw_deg, cfg, scale) in sign_list:
        if scale is None:
            scale = [1, 1, 1]

        # Remove existing actor with same ID so changes apply
        try:
            actor_obj.actorNumber = actor_id
            actor_obj.destroy()
        except Exception as e:
            print(f"[{label}] destroy id={actor_id} (ignored): {e}")

        status = actor_obj.spawn_id_degrees(
            actorNumber=actor_id,
            location=xyz,
            rotation=[0, 0, yaw_deg],
            scale=scale,
            configuration=cfg,
            waitForConfirmation=wait
        )
        print(f"[{label}] id={actor_id} status={status} xyz={xyz} yaw={yaw_deg:.2f} cfg={cfg} scale={scale}")

def spawn_box_crosswalk(cw, base_id, X1, X2, Y0, Y1, Z, scale_v, scale_h, config, wait, y_shift=0.250):
    def _spawn(actor_id, center, yaw_deg, scale):
        try:
            cw.actorNumber = actor_id
            cw.destroy()
        except Exception:
            pass
        return cw.spawn_id_degrees(actor_id, center, [0, 0, yaw_deg], scale, config, wait)

    # Right vertical (X1)
    A = (X1, Y0, Z); B = (X1, Y1, Z)
    c0, yaw0 = center_and_yaw(A, B)
    s0 = _spawn(base_id + 0, c0, yaw0, scale_v)

    # Left vertical (X2)
    A = (X2, Y0, Z); B = (X2, Y1, Z)
    c1, yaw1 = center_and_yaw(A, B)
    s1 = _spawn(base_id + 1, c1, yaw1, scale_v)

    # Bottom horizontal (shifted -y_shift)
    A = (X2, Y0, Z); B = (X1, Y0, Z)
    c2, yaw2 = center_and_yaw(A, B)
    c2 = [c2[0], c2[1] - y_shift, c2[2]]
    s2 = _spawn(base_id + 2, c2, yaw2, scale_h)

    # Top horizontal (shifted +y_shift)
    A = (X2, Y1, Z); B = (X1, Y1, Z)
    c3, yaw3 = center_and_yaw(A, B)
    c3 = [c3[0], c3[1] + y_shift, c3[2]]
    s3 = _spawn(base_id + 3, c3, yaw3, scale_h)

    print(f"[crosswalk-box] ids={base_id}-{base_id+3} status={[s0,s1,s2,s3]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--wait", action="store_true", help="Show QLabs confirmations")
    ap.add_argument("--no-wait", action="store_true", help="No confirmations")
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

    # === Crosswalks ===
    cw = QLabsCrosswalk(qlabs)

    # ORIGINAL #1 (id 200)
    A = (X1, Y0, Z); B = (X1, Y1, Z)
    c200, yaw200 = center_and_yaw(A, B)
    s200 = cw.spawn_id_degrees(200, c200, [0, 0, yaw200], scale_v, CROSSWALK_CONFIG, WAIT)

    # ORIGINAL #2 (id 201)
    A = (X2, Y0, Z); B = (X2, Y1, Z)
    c201, yaw201 = center_and_yaw(A, B)
    s201 = cw.spawn_id_degrees(201, c201, [0, 0, yaw201], scale_v, CROSSWALK_CONFIG, WAIT)

    # BOTTOM edge (id 202) with Y shift -0.250
    A = (X2, Y0, Z); B = (X1, Y0, Z)
    c202, yaw202 = center_and_yaw(A, B)
    c202 = [c202[0], c202[1] - 0.250, c202[2]]
    s202 = cw.spawn_id_degrees(202, c202, [0, 0, yaw202], scale_h, CROSSWALK_CONFIG, WAIT)

    # TOP edge (id 203) with Y shift +0.250
    A = (X2, Y1, Z); B = (X1, Y1, Z)
    c203, yaw203 = center_and_yaw(A, B)
    c203 = [c203[0], c203[1] + 0.250, c203[2]]
    s203 = cw.spawn_id_degrees(203, c203, [0, 0, yaw203], scale_h, CROSSWALK_CONFIG, WAIT)

    yaw210_new = 90.0
    c210 = [1.350, 0.922, 0.006]
    s210 = cw.spawn_id_degrees(210, c210, [0, 0, yaw210_new], scale_v, CROSSWALK_CONFIG, WAIT)

    print(f"[crosswalk] id=200 status={s200} center={c200} yaw={yaw200:.2f} scale={scale_v}")
    print(f"[crosswalk] id=201 status={s201} center={c201} yaw={yaw201:.2f} scale={scale_v}")
    print(f"[crosswalk] id=202 status={s202} center(shifted)={c202} yaw={yaw202:.2f} scale={scale_h} (Y -0.250)")
    print(f"[crosswalk] id=203 status={s203} center(shifted)={c203} yaw={yaw203:.2f} scale={scale_h} (Y +0.250)")
    print(f"[crosswalk] id=210 status={s210} center={c210} yaw={yaw210_new:.2f} scale={scale_v}")
    
    # === Signs ===
    # Format: (actor_id, [x,y,z], yaw_deg, config, scale_or_None)

    STOP_SIGNS = [
        (300, [2.444, 0.166, 0.006], -90.0, 0, STOP_SCALE),
        (301, [1.725, 1.660, 0.006], 90.0, 0, STOP_SCALE),
        (302, [-1.510, 3.615, 0.006], 315.0, 0, STOP_SCALE),
        (303, [-1.505, 2.190, 0.006], 45.0, 0, STOP_SCALE),  
    ]

    # ✅ Edit direction here: change the yaw_deg value (-90 / 90 / 180 / 0)
    YIELD_SIGNS = [
        (400, [0.045, -1.3, 0.006], 180.0, 0, YIELD_SCALE),
        (401, [2.430, 3.150, 0.006], -90.0, 0, YIELD_SCALE),
        (402, [1.060, 2.750, 0.006], -135.0, 0, YIELD_SCALE),
        (403, [0.510, 3.838, 0.006], 135.0, 0, YIELD_SCALE),
    ]

    ROUNDABOUT_SIGNS = [
        (500, [2.430, 2.630, 0.006], -90.0, 0, ROUNDABOUT_SCALE),
        (501, [0.660, 2.380, 0.006], -135.0, 0, ROUNDABOUT_SCALE),
        (502, [0.015, 3.960, 0.006], 135.0, 0, ROUNDABOUT_SCALE),
    ]

    stop = QLabsStopSign(qlabs)
    yld = QLabsYieldSign(qlabs)
    rnd = QLabsRoundaboutSign(qlabs)

    spawn_signs("stop_sign", stop, STOP_SIGNS, wait=WAIT)
    spawn_signs("yield_sign", yld, YIELD_SIGNS, wait=WAIT)
    spawn_signs("roundabout_sign", rnd, ROUNDABOUT_SIGNS, wait=WAIT)

    qlabs.close()
    print("Done.")


if __name__ == "__main__":
    main()
