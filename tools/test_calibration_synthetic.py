#!/usr/bin/env python3
"""Check calibrate_imu.py and calibrate_mag.py on a synthetic recording with known errors.

    test_calibration_synthetic.py [DIRECTORY]

Writes a recording in pellicanus_reader.py --save's format (18 still poses with turns between them,
then a minute of free tumbling), runs both tools on it and compares what they found with the truth.
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

here = os.path.dirname(os.path.abspath(__file__))
folder = sys.argv[1] if len(sys.argv) > 1 else "synthetic_calibration"
os.makedirs(folder, exist_ok=True)
prefix = os.path.join(folder, "session")
rng = np.random.default_rng(1)
rate, g = 250.0, 9.80665

# --- the motion: still poses, turns between them, then tumbling
poses = [Rotation.identity()] + list(Rotation.random(17, random_state=3))
keys, key_t, now = [], [], 0.0
for i, pose in enumerate(poses):
    hold = 12.0 if i == 0 else 3.0
    keys += [pose, pose]
    key_t += [now, now + hold]
    now += hold + 2.0  # a 2 s turn to the next pose
tumble = list(Rotation.random(30, random_state=5))
for r in tumble:
    keys.append(r)
    key_t.append(now)
    now += 2.0
t = np.arange(0.0, key_t[-1], 1.0 / rate)
slerp = Slerp(key_t, Rotation.concatenate(keys))
attitude = slerp(t)  # sensor -> world

# body rates from consecutive attitudes; specific force with no linear motion (world z up)
delta = attitude[:-1].inv() * attitude[1:]
omega = np.vstack([delta.as_rotvec() * rate, np.zeros((1, 3))])
force = attitude.inv().apply(np.array([0.0, 0.0, g]))
field_world = 0.5 * np.array([np.cos(np.radians(60)), 0.0, -np.sin(np.radians(60))])

# --- the errors, as the tools model them (raw = inverse model)
ca = np.array([[1.012, -0.004, 0.003], [0.0, 0.991, -0.002], [0.0, 0.0, 1.006]])
ba = np.array([0.05, -0.03, 0.08])
cg = np.array([[1.006, -0.003, 0.002], [0.004, 0.996, -0.002], [-0.001, 0.003, 1.003]])
bg = np.radians([0.2, -0.6, 0.05])
mag_to_imu = Rotation.from_euler("zyx", [90, 0, 180], degrees=True)  # the chips' layout
soft = np.array([[1.04, 0.02, -0.01], [0.02, 0.97, 0.015], [-0.01, 0.015, 1.01]])
hard = np.array([0.20, -0.10, 0.30])

accel_raw = force @ np.linalg.inv(ca).T - ba + rng.normal(0, 0.01, force.shape)
gyro_raw = omega @ np.linalg.inv(cg).T + bg + rng.normal(0, 6e-4, omega.shape)
mag_t = t[::5]  # 50 Hz
field_imu = attitude[::5].inv().apply(field_world)
field_mag = mag_to_imu.inv().apply(field_imu)
mag_raw = field_mag @ np.linalg.inv(soft).T + hard + rng.normal(0, 3e-4, field_mag.shape)

with open(f"{prefix}_imu.csv", "w") as f:
    f.write("seq,time_us,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,temp_c\n")
    for i in range(len(t)):
        f.write(f"{i},{int(t[i] * 1e6)},{','.join(f'{v:.7g}' for v in accel_raw[i])},"
                f"{','.join(f'{v:.7g}' for v in gyro_raw[i])},25.0\n")
with open(f"{prefix}_mag.csv", "w") as f:
    f.write("seq,time_us,x_gauss,y_gauss,z_gauss\n")
    for i in range(len(mag_t)):
        f.write(f"{i},{int(mag_t[i] * 1e6)},{','.join(f'{v:.7g}' for v in mag_raw[i])}\n")

run = lambda *a: subprocess.run([sys.executable, *a], check=True, cwd=here)
run("calibrate_imu.py", "--from", os.path.abspath(prefix))
run("calibrate_mag.py", "--from", os.path.abspath(prefix), "--field-gauss", "0.5",
    "--imu-calibration", os.path.abspath(f"{prefix}_imu_calibration.json"))

imu = json.load(open(f"{prefix}_imu_calibration.json"))
mag = json.load(open(f"{prefix}_mag_calibration.json"))
r_found = Rotation.from_quat(np.roll(mag["rotation_mag_to_imu_wxyz"], -1))
checks = {
    "accel matrix": np.abs(np.array(imu["accel_matrix"]) - ca).max(),
    "accel bias (m/s2)": np.abs(np.array(imu["accel_bias_m_s2"]) - ba).max(),
    "gyro matrix": np.abs(np.array(imu["gyro_matrix"]) - cg).max(),
    "gyro bias (deg/s)": np.degrees(np.abs(np.array(imu["gyro_bias_rad_s"]) - bg).max()),
    "hard iron (G)": np.abs(np.array(mag["hard_iron_gauss"]) - hard).max(),
    "soft iron": np.abs(np.array(mag["soft_iron"]) - soft).max(),
    "mag -> IMU rotation (deg)": np.degrees((r_found * mag_to_imu.inv()).magnitude()),
}
limits = {"accel matrix": 2e-3, "accel bias (m/s2)": 0.01, "gyro matrix": 2e-3, "gyro bias (deg/s)": 0.02,
          "hard iron (G)": 2e-3, "soft iron": 5e-3, "mag -> IMU rotation (deg)": 0.5}
print("\nfound against the truth:")
ok = True
for name, error in checks.items():
    good = error <= limits[name]
    ok &= good
    print(f"  {name:28s} max error {error:.5f}  (limit {limits[name]})  {'ok' if good else 'FAIL'}")
sys.exit(0 if ok else 1)
