#!/usr/bin/env python3
"""Calibrate the magnetometer (docs/calibration.md): hard iron, soft iron, and the rotation from the
magnetometer's axes to the IMU's.

    calibrate_mag.py [--port /dev/ttyACM0] [--field-gauss B] [--imu-calibration FILE.json] [--save PREFIX]
    calibrate_mag.py --from PREFIX [--field-gauss B] [--imu-calibration FILE.json] [--initial-static 10]

The first form guides a session with the board, in three parts: it rests still for a while, then is
turned slowly through every direction while the share of the sphere covered is shown, then rests in a
few poses, each at a different heading. The recording is kept (PREFIX_imu.csv, PREFIX_mag.csv). The
second form calibrates a recording again, or one made with pellicanus_reader.py --save. The result
goes to PREFIX_mag_calibration.json.

- **Hard and soft iron:** an ellipsoid is fitted to every magnetometer sample. In a uniform field the
  corrected reading must have a constant magnitude:

      mag_cal = soft_iron @ (mag_raw - hard_iron),   |mag_cal| = B

  B is --field-gauss when given (the local field, e.g. from a world magnetic model), else the mean
  radius, which keeps the sensor's own scale. The board must be turned through as many directions as
  possible, away from steel and from computers.
- **Rotation to the IMU:** in every still pose, the angle between gravity and the field (the dip) is
  the same. The rotation that keeps it constant is the rotation from the magnetometer's axes to the
  IMU's. This needs the IMU recording, with the still poses calibrate_imu.py also uses, and is better
  with its calibration (--imu-calibration).

Needs numpy and scipy.
"""
import json
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from calibration_common import arg, load_csv, static_intervals
from live_capture import enter_pressed, guided_pose, open_session


def ellipsoid_fit(m):
    """Algebraic fit of a general ellipsoid, then the centre and the matrix mapping it to a sphere."""
    x, y, z = m.T
    d = np.column_stack([x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z])
    v, *_ = np.linalg.lstsq(d, np.ones(len(m)), rcond=None)
    a = np.array([[v[0], v[5], v[4]], [v[5], v[1], v[3]], [v[4], v[3], v[2]]])
    centre = -np.linalg.solve(a, v[6:9])
    scale = 1.0 + centre @ a @ centre
    w, u = np.linalg.eigh(a / scale)
    if np.any(w <= 0):
        sys.exit("the samples do not describe an ellipsoid: turn the board through more directions")
    return centre, u @ np.diag(np.sqrt(w)) @ u.T  # symmetric: |s (m - c)| = 1 on the fit


def coverage(directions, bins=(6, 12)):
    """Share of latitude/longitude bins of the sphere the field directions fall in."""
    lat = np.degrees(np.arcsin(np.clip(directions[:, 2], -1, 1)))
    lon = np.degrees(np.arctan2(directions[:, 1], directions[:, 0]))
    h, *_ = np.histogram2d(lat, lon, bins=bins, range=[[-90, 90], [-180, 180]])
    return float((h > 0).mean())


def sphere_centre(m):
    """Least-squares centre of a sphere through the samples (|m|^2 = 2 c.m + k): a quick hard iron."""
    a = np.column_stack([2 * m, np.ones(len(m))])
    x, *_ = np.linalg.lstsq(a, (m * m).sum(axis=1), rcond=None)
    return x[:3]


def calibrate(mag, imu, field, initial, imu_calibration=None):
    """The calibration (a dict) from a recording's magnetometer and IMU columns."""
    raw = np.column_stack([mag["x_gauss"], mag["y_gauss"], mag["z_gauss"]])
    tm = mag["time_us"] * 1e-6

    centre, shape = ellipsoid_fit(raw)
    radius = float(np.mean(np.linalg.norm((raw - centre) @ shape.T, axis=1)))
    field = field or float(np.mean(np.linalg.norm(raw - centre, axis=1)))

    # refine: a symmetric soft-iron matrix and the centre, |S (m - c)| = B
    def residual(p):
        s = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
        return np.linalg.norm((raw - p[6:9]) @ s.T, axis=1) - field

    s0 = shape * field / radius
    p0 = np.r_[s0[0, 0], s0[1, 1], s0[2, 2], s0[0, 1], s0[0, 2], s0[1, 2], centre]
    fit = least_squares(residual, p0)
    p = fit.x
    soft = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
    hard = p[6:9]
    cal = (raw - hard) @ soft.T
    before = np.linalg.norm(raw, axis=1)
    after = np.linalg.norm(cal, axis=1)
    cover = coverage(cal / after[:, None])

    print(f"{len(raw)} samples over {tm[-1] - tm[0]:.0f} s; directions cover {cover * 100:.0f} % of the sphere")
    if cover < 0.5:
        print("  WARNING: too few directions for a trustworthy soft-iron matrix: turn the board through more of them")
    print(f"magnitude: {before.mean():.4f} G +- {before.std() / before.mean() * 100:.2f} % before, "
          f"{after.mean():.4f} G +- {after.std() / after.mean() * 100:.2f} % after "
          f"(residual RMS {np.sqrt(np.mean(fit.fun ** 2)) * 1e5:.0f} nT)")
    print(f"  hard iron {hard} G")
    print(f"  soft iron\n{np.array2string(soft, precision=5)}")

    result = {
        "model": "mag_cal = soft_iron @ (mag_raw - hard_iron), in gauss, in the magnetometer's axes",
        "hard_iron_gauss": hard.tolist(), "soft_iron": soft.tolist(), "field_gauss": field,
        "samples": len(raw), "sphere_coverage": cover,
        "residual_rms_nt": float(np.sqrt(np.mean(fit.fun ** 2)) * 1e5),
    }

    # --- rotation to the IMU: the dip is the same in every still pose
    if imu is not None:
        t = imu["time_us"] * 1e-6
        accel = np.column_stack([imu["ax_m_s2"], imu["ay_m_s2"], imu["az_m_s2"]])
        gyro = np.column_stack([imu["gx_rad_s"], imu["gy_rad_s"], imu["gz_rad_s"]])
        if imu_calibration:
            c = json.load(open(imu_calibration))
            accel = (accel + np.array(c["accel_bias_m_s2"])) @ np.array(c["accel_matrix"]).T
        intervals = static_intervals(t, accel, gyro, initial)
        ups, fields = [], []
        for a, b in intervals:
            sel = (tm >= t[a]) & (tm <= t[b - 1])
            if sel.sum() >= 10:
                u = accel[a:b].mean(axis=0)
                f = cal[sel].mean(axis=0)
                ups.append(u / np.linalg.norm(u))
                fields.append(f / np.linalg.norm(f))
        ups, fields = np.array(ups), np.array(fields)
        if len(ups) >= 6:
            def dip_residual(q):
                r = Rotation.from_rotvec(q[:3]).as_matrix()
                return np.einsum("ij,ij->i", ups, fields @ r.T) - q[3]

            best = None
            for guess in Rotation.from_euler("zyx", [[a, b, c] for a in (0, 90, 180, 270)
                                                     for b in (0, 90, 180, 270) for c in (0, 90)], degrees=True):
                f2 = least_squares(dip_residual, np.r_[guess.as_rotvec(), 0.0])
                if best is None or f2.cost < best.cost:
                    best = f2
            rot = Rotation.from_rotvec(best.x[:3])
            spread = np.degrees(np.std(np.arcsin(np.clip(best.fun + best.x[3], -1, 1))))
            dip = np.degrees(np.arcsin(np.clip(best.x[3], -1, 1)))
            w, x, y, z = np.roll(rot.as_quat(), 1)
            print(f"rotation magnetometer -> IMU: {np.round(rot.as_euler('zyx', degrees=True), 2)} deg (z, y, x); "
                  f"dip {dip:.2f} deg, spread over {len(ups)} poses {spread:.2f} deg")
            result.update({"rotation_mag_to_imu_wxyz": [w, x, y, z], "dip_deg": dip,
                           "dip_spread_deg": spread, "poses": len(ups)})
            print("  (the dip is measured from the accelerometer's 'up', so its sign follows the IMU's axes)")
        else:
            print(f"rotation to the IMU: only {len(ups)} still poses with magnetometer data, 6 needed")

    return result


# the guided session's still poses after the turning: each at a new heading, so the field turns
# about gravity in the board's axes
POSES = [(1, 0, 0), (0, 0, -1), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (1, 0, 1), (0, 1, 1), (0, 0, 1)]
INITIAL_S, HOLD_S, COVERAGE_GOAL = 10.0, 3.0, 0.8


def turn_through_directions(cap):
    """The middle part: turning, with the sphere's coverage shown, until the user presses Enter."""
    print("\nTurn the board slowly through every direction: roll it, flip it, point each face down in turn.\n"
          "Keep away from steel and computers. Press Enter when the coverage is past "
          f"{COVERAGE_GOAL * 100:.0f} %.")
    start = len(cap.recent_mag())
    cover = 0.0
    while True:
        m = cap.recent_mag()[start:, 1:4]
        if len(m) > 50:
            d = m - sphere_centre(m)
            cover = coverage(d / np.linalg.norm(d, axis=1, keepdims=True))
        bar = "#" * int(cover * 30)
        print(f"\r        coverage [{bar:30s}] {cover * 100:3.0f} %  {len(m)} samples   ", end="", flush=True)
        if enter_pressed():
            if cover >= COVERAGE_GOAL or input("\n        coverage is low: stop anyway? [y/N] ").lower() == "y":
                print()
                return
        time.sleep(0.25)


def guided():
    cap, prefix = open_session(arg("--port", "/dev/ttyACM0"), "mag")
    prefix = arg("--save", prefix)
    total = len(POSES) + 1
    try:
        guided_pose(cap, 1, total, (0, 0, 1), INITIAL_S, extra=f" (this one {INITIAL_S:.0f} s)")
        cap.learn_quiet(INITIAL_S)
        turn_through_directions(cap)
        print("\nNow a few still poses, for the rotation to the IMU: turn the board to a new heading "
              "for each one.")
        for i, up in enumerate(POSES):
            guided_pose(cap, i + 2, total, up, HOLD_S)
    finally:
        cap.stop()
        cap.save(prefix)
        print(f"\nrecording written to {prefix}_imu.csv and {prefix}_mag.csv")
    imu, mag = cap.tables()
    return prefix, mag, imu


def main() -> None:
    if arg("--from"):
        prefix = arg("--from")
        mag = load_csv(f"{prefix}_mag.csv")
        try:
            imu = load_csv(f"{prefix}_imu.csv")
        except (OSError, ValueError):
            imu = None
        initial = float(arg("--initial-static", INITIAL_S))
    else:
        prefix, mag, imu = guided()
        initial = INITIAL_S
    if mag is None:
        sys.exit("no magnetometer data")
    result = calibrate(mag, imu, float(arg("--field-gauss", 0.0)), initial, arg("--imu-calibration"))
    out = arg("--out", f"{prefix}_mag_calibration.json")
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    print(f"written {out}")


if __name__ == "__main__":
    main()
