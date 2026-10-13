#!/usr/bin/env python3
"""Calibrate the IMU's accelerometer and gyro without equipment (docs/calibration.md).

    calibrate_imu.py [--port /dev/ttyACM0] [--location LAT,LON,HEIGHT] [--save PREFIX]
    calibrate_imu.py --from PREFIX [--location LAT,LON,HEIGHT | --gravity G] [--initial-static 10]

The first form guides a session with the board: it asks for the location (local gravity is the
reference), then for one pose after another, waits until the board rests in each, records everything
in between, and calibrates. The recording is kept (PREFIX_imu.csv, PREFIX_mag.csv). The second form
calibrates a recording again, or one made with pellicanus_reader.py --save. The result goes to
PREFIX_imu_calibration.json.

The method is Tedaldi, Pretto and Menegatti, "A robust and easy to implement method for IMU
calibration without external equipments" (ICRA 2014):

- the board is held still in many orientations, and moved between them;
- the accelerometer is calibrated first: in every still pose its corrected reading must have the
  magnitude of gravity;
- then the gyro: the rotation it integrates between two still poses must carry the gravity direction
  of the first pose onto that of the second;
- the gyro bias is the mean over the still period at the start.

Models (raw: the sensor's output; calibrated: what the host should use):

    accel_cal = Ca @ (accel_raw + ba),   Ca = Ta @ diag(ka)   (Ta: 3 misalignment terms)
    gyro_cal  = Cg @ (gyro_raw - bg),    Cg = Tg @ diag(kg)   (Tg: 6 misalignment terms)

Needs numpy and scipy.
"""
import json
import sys

import numpy as np
from scipy.optimize import least_squares

from calibration_common import arg, load_csv, static_intervals
from live_capture import guided_pose, normal_gravity, open_session


def accel_matrix(p):
    ayz, azy, azx, kx, ky, kz = p[:6]
    t = np.array([[1.0, -ayz, azy], [0.0, 1.0, -azx], [0.0, 0.0, 1.0]])
    return t @ np.diag([kx, ky, kz]), p[6:9]


def gyro_matrix(p):
    gyz, gzy, gxz, gzx, gxy, gyx, kx, ky, kz = p
    t = np.array([[1.0, -gyz, gzy], [gxz, 1.0, -gzx], [-gxy, gyx, 1.0]])
    return t @ np.diag([kx, ky, kz])


def rotate(q, v):
    """Rotate vector v by unit quaternion q (w, x, y, z)."""
    w, x, y, z = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return r @ v


def integrate(omega, dt):
    """The rotation (quaternion, sensor frame at the start -> at the end) of a rate history."""
    q = np.array([1.0, 0.0, 0.0, 0.0])
    phi = 0.5 * (omega[:-1] + omega[1:]) * dt[:, None]
    angle = np.linalg.norm(phi, axis=1)
    half = 0.5 * angle
    s = np.where(angle > 1e-12, np.sin(half) / np.where(angle > 1e-12, angle, 1.0), 0.5)
    dq = np.column_stack([np.cos(half), phi * s[:, None]])
    for w2, x2, y2, z2 in dq:
        w1, x1, y1, z1 = q
        q = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return q / np.linalg.norm(q)


# The guided session: the first pose is held long (the gyro bias and the noise the still poses are
# judged against), then the six faces, then the twelve edges. Each move turns the board about a
# different axis, which the gyro's scale and misalignment need.
POSES = [(0, 0, 1), (1, 0, 0), (0, 0, -1), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
         (1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 0, -1), (-1, 0, -1), (0, 1, -1), (0, -1, -1),
         (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0)]
INITIAL_S, HOLD_S = 10.0, 3.0


def calibrate(imu, gravity, initial):
    """The calibration (a dict) from a recording's IMU columns."""
    t = imu["time_us"] * 1e-6
    accel = np.column_stack([imu["ax_m_s2"], imu["ay_m_s2"], imu["az_m_s2"]])
    gyro = np.column_stack([imu["gx_rad_s"], imu["gy_rad_s"], imu["gz_rad_s"]])

    intervals = static_intervals(t, accel, gyro, initial)
    if len(intervals) < 9:
        sys.exit(f"only {len(intervals)} still poses found: 9 or more are needed (docs/calibration.md)")
    first = intervals[0]
    bg = gyro[first[0]:first[1]].mean(axis=0)
    means = np.array([accel[a:b].mean(axis=0) for a, b in intervals])
    print(f"{len(intervals)} still poses over {t[-1] - t[0]:.0f} s; gyro bias {np.degrees(bg)} deg/s "
          f"(first {t[first[1]] - t[first[0]]:.1f} s still)")

    # --- accelerometer: |Ca (a + ba)| = g in every still pose
    def accel_residual(p):
        c, b = accel_matrix(p)
        return np.linalg.norm((means + b) @ c.T, axis=1) - gravity

    pa0 = np.r_[0, 0, 0, 1, 1, 1, 0, 0, 0.0]
    fa = least_squares(accel_residual, pa0, x_scale=np.r_[1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 0.1, 0.1, 0.1])
    ca, ba = accel_matrix(fa.x)
    before_a = accel_residual(pa0)
    after_a = accel_residual(fa.x)

    # --- gyro: the integrated rotation between still poses carries gravity from one to the next
    up = (means + ba) @ ca.T
    up /= np.linalg.norm(up, axis=1, keepdims=True)
    moves = [(intervals[i][1], intervals[i + 1][0]) for i in range(len(intervals) - 1)]

    def gyro_residual(p):
        cg = gyro_matrix(p)
        res = []
        for i, (a, b) in enumerate(moves):
            omega = (gyro[a:b + 1] - bg) @ cg.T
            q = integrate(omega, np.diff(t[a:b + 1]))
            # a vector fixed in the world, seen from the frame after the rotation: R^T u
            qc = q * np.array([1, -1, -1, -1])
            res.append(rotate(qc, up[i]) - up[i + 1])
        return np.concatenate(res)

    pg0 = np.r_[0, 0, 0, 0, 0, 0, 1, 1, 1.0]
    fg = least_squares(gyro_residual, pg0, x_scale=1e-2)
    cg = gyro_matrix(fg.x)
    angle = lambda r: np.degrees(np.linalg.norm(r.reshape(-1, 3), axis=1))
    before_g = angle(gyro_residual(pg0))
    after_g = angle(gyro_residual(fg.x))

    print(f"accelerometer: |a| - g over the poses, RMS {np.sqrt(np.mean(before_a ** 2)) / gravity * 1e3:.2f} mg "
          f"before, {np.sqrt(np.mean(after_a ** 2)) / gravity * 1e3:.2f} mg after")
    print(f"  bias {ba} m/s2, scale {fa.x[3:6]}, misalignment {np.degrees(fa.x[:3])} deg")
    print(f"gyro: gravity carried between poses, error RMS {np.sqrt(np.mean(before_g ** 2)):.3f} deg "
          f"before, {np.sqrt(np.mean(after_g ** 2)):.3f} deg after ({len(moves)} moves)")
    print(f"  scale {fg.x[6:9]}, misalignment {np.degrees(fg.x[:6])} deg")

    result = {
        "model": "accel_cal = accel_matrix @ (accel_raw + accel_bias); gyro_cal = gyro_matrix @ (gyro_raw - gyro_bias)",
        "accel_matrix": ca.tolist(), "accel_bias_m_s2": ba.tolist(),
        "gyro_matrix": cg.tolist(), "gyro_bias_rad_s": bg.tolist(),
        "gravity_m_s2": gravity, "poses": len(intervals),
        "accel_rms_error_mg": float(np.sqrt(np.mean(after_a ** 2)) / gravity * 1e3),
        "gyro_rms_error_deg": float(np.sqrt(np.mean(after_g ** 2))),
        "temperature_c": float(np.mean(imu["temp_c"])),
    }
    return result


def location():
    """(latitude, longitude, height) from --location, else asked for."""
    text = arg("--location") or input("Location: latitude (deg), longitude (deg), height above sea level (m), "
                                      "e.g. 39.92,32.85,900: ")
    lat, lon, height = (float(v) for v in text.replace(" ", "").split(","))
    return lat, lon, height


def guided():
    lat, lon, height = location()
    gravity = normal_gravity(lat, height)
    print(f"local gravity {gravity:.5f} m/s2 (WGS84 normal gravity at {lat:.4f} deg, {height:.0f} m)")
    cap, prefix = open_session(arg("--port", "/dev/ttyACM0"), "imu")
    prefix = arg("--save", prefix)
    print(f"\n{len(POSES)} poses. Put the board on a steady table; in each pose let go of it and keep the cable "
          "slack.\nMove it to the next pose by hand: the turns between poses are part of the calibration.")
    try:
        for i, up in enumerate(POSES):
            hold = INITIAL_S if i == 0 else HOLD_S
            extra = f" (this one {INITIAL_S:.0f} s: it sets the gyro bias)" if i == 0 else ""
            guided_pose(cap, i + 1, len(POSES), up, hold, extra=extra)
            if i == 0:
                cap.learn_quiet(INITIAL_S)
    finally:
        cap.stop()
        cap.save(prefix)
        print(f"\nrecording written to {prefix}_imu.csv and {prefix}_mag.csv")
    imu, _ = cap.tables()
    return prefix, calibrate(imu, gravity, INITIAL_S), (lat, lon, height)


def main() -> None:
    if arg("--from"):
        prefix = arg("--from")
        place = location() if arg("--location") else None
        gravity = normal_gravity(place[0], place[2]) if place else float(arg("--gravity", 9.80665))
        result = calibrate(load_csv(f"{prefix}_imu.csv"), gravity, float(arg("--initial-static", INITIAL_S)))
    else:
        prefix, result, place = guided()
    if place:
        result["location_lat_lon_height"] = list(place)
    out = arg("--out", f"{prefix}_imu_calibration.json")
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"written {out}")


if __name__ == "__main__":
    main()
