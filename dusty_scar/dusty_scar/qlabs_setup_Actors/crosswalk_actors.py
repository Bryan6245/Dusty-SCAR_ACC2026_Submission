import math
from qvl.qlabs import QuanserInteractiveLabs
from qvl.crosswalk import QLabsCrosswalk

# === Bounds ===
Y0, Y1, Z = 0.570, 1.275, 0.006
X1 = 0.745
X2 = -0.487

# === Scale settings ===
base = 0.10
length_factor = 0.49
scale_v = [base * length_factor, base, base]   # vertical edges
scale_h = [base * length_factor, base, base]   # horizontal edges

config = 0
WAIT = True  # set False if you don’t want confirmations popping up

def center_and_yaw(A, B):
    center = [(A[0]+B[0])/2.0, (A[1]+B[1])/2.0, (A[2]+B[2])/2.0]
    yaw = math.degrees(math.atan2(B[1]-A[1], B[0]-A[0]))
    return center, yaw

def main():
    qlabs = QuanserInteractiveLabs()
    print("Connecting...")
    if not qlabs.open("localhost"):
        raise SystemExit("ERROR: could not connect to QLabs")

    cw = QLabsCrosswalk(qlabs)

    # === Spawn ORIGINAL #1 (id 200) ===
    A = (X1, Y0, Z); B = (X1, Y1, Z)
    c200, yaw200 = center_and_yaw(A, B)
    s200 = cw.spawn_id_degrees(200, c200, [0, 0, yaw200], scale_v, config, WAIT)

    # === Spawn ORIGINAL #2 (id 201) ===
    A = (X2, Y0, Z); B = (X2, Y1, Z)
    c201, yaw201 = center_and_yaw(A, B)
    s201 = cw.spawn_id_degrees(201, c201, [0, 0, yaw201], scale_v, config, WAIT)

    # === Spawn BOTTOM edge (id 202) with Y shift -0.250 ===
    A = (X2, Y0, Z); B = (X1, Y0, Z)
    c202, yaw202 = center_and_yaw(A, B)
    c202 = [c202[0], c202[1] - 0.250, c202[2]]
    s202 = cw.spawn_id_degrees(202, c202, [0, 0, yaw202], scale_h, config, WAIT)

    # === Spawn TOP edge (id 203) with Y shift +0.250 ===
    A = (X2, Y1, Z); B = (X1, Y1, Z)
    c203, yaw203 = center_and_yaw(A, B)
    c203 = [c203[0], c203[1] + 0.250, c203[2]]
    s203 = cw.spawn_id_degrees(203, c203, [0, 0, yaw203], scale_h, config, WAIT)

    print(f"id=200 status={s200} center={c200} yaw={yaw200:.2f} scale={scale_v}")
    print(f"id=201 status={s201} center={c201} yaw={yaw201:.2f} scale={scale_v}")
    print(f"id=202 status={s202} center(shifted)={c202} yaw={yaw202:.2f} scale={scale_h} (Y -0.250)")
    print(f"id=203 status={s203} center(shifted)={c203} yaw={yaw203:.2f} scale={scale_h} (Y +0.250)")

    qlabs.close()

if __name__ == "__main__":
    main()
