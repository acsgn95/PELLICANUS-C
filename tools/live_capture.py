"""The board's stream read in the background, and the prompts the guided calibration tools share.

LiveCapture keeps every IMU and magnetometer packet, tells whether the board is still, and writes
the session in pellicanus_reader.py --save's format, so it can be analysed again later.
"""
import os
import select
import struct
import sys
import threading
import time

import numpy as np

import pellicanus_reader as reader

IMU_COLUMNS = ["time_us", "ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s", "temp_c"]
MAG_COLUMNS = ["time_us", "x_gauss", "y_gauss", "z_gauss"]


class LiveCapture:
    def __init__(self, port: str):
        self.imu = []  # rows of IMU_COLUMNS
        self.mag = []  # rows of MAG_COLUMNS
        self.lock = threading.Lock()
        self.running = True
        self.stats = {"crc_errors": 0}
        self.quiet_accel = None
        self.quiet_gyro = None
        read = reader.open_port(port)
        self.thread = threading.Thread(target=self._run, args=(read,), daemon=True)
        self.thread.start()

    def _run(self, read):
        for kind, _, _, time_us, payload in reader.packets(read, 1e9, self.stats):
            if not self.running:
                break
            with self.lock:
                if kind == 1:
                    self.imu.append((time_us, *struct.unpack("<7f", payload)))
                elif kind == 2:
                    self.mag.append((time_us, *struct.unpack("<3f", payload)))

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)

    def wait_for_data(self, seconds=5.0):
        end = time.time() + seconds
        while time.time() < end:
            with self.lock:
                if len(self.imu) > 50:
                    return True
            time.sleep(0.05)
        return False

    def recent_imu(self, seconds):
        with self.lock:
            n = min(len(self.imu), int(seconds * 260) + 1)
            rows = self.imu[-n:] if n else []
        d = np.array(rows) if rows else np.empty((0, 8))
        return d[d[:, 0] >= d[-1, 0] - seconds * 1e6] if len(d) else d

    def recent_mag(self, seconds=None):
        with self.lock:
            rows = list(self.mag) if seconds is None else self.mag[-int(seconds * 55) - 1:]
        return np.array(rows) if rows else np.empty((0, 4))

    def learn_quiet(self, seconds):
        """The noise at rest, from the last `seconds`: stillness is judged against it from now on."""
        d = self.recent_imu(seconds)
        self.quiet_accel = float(np.linalg.norm(d[:, 1:4].std(axis=0)))
        self.quiet_gyro = float(np.linalg.norm(d[:, 4:7].std(axis=0)))

    def still(self, seconds):
        """Whether the board has been still over the last `seconds`."""
        d = self.recent_imu(seconds)
        if len(d) < seconds * 200:
            return False
        accel_spread = np.linalg.norm(d[:, 1:4].std(axis=0))
        gyro_spread = np.linalg.norm(d[:, 4:7].std(axis=0))
        return (accel_spread < max(4 * (self.quiet_accel or 0.0), 0.04)
                and gyro_spread < max(5 * (self.quiet_gyro or 0.0), 0.01))

    def up(self, seconds=0.3):
        """The accelerometer's mean direction over the last `seconds` (the board's 'up')."""
        d = self.recent_imu(seconds)
        if not len(d):
            return None
        a = d[:, 1:4].mean(axis=0)
        return a / np.linalg.norm(a)

    def tables(self):
        """The session so far as load_csv() returns a recording: two dicts of numpy columns."""
        with self.lock:
            imu, mag = np.array(self.imu), np.array(self.mag)
        return ({c: imu[:, i] for i, c in enumerate(IMU_COLUMNS)},
                {c: mag[:, i] for i, c in enumerate(MAG_COLUMNS)} if len(mag) else None)

    def save(self, prefix):
        """The session in pellicanus_reader.py --save's format (IMU and magnetometer files)."""
        os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
        with self.lock:
            imu, mag = list(self.imu), list(self.mag)
        for name, columns, rows in (("imu", IMU_COLUMNS, imu), ("mag", MAG_COLUMNS, mag)):
            with open(f"{prefix}_{name}.csv", "w") as f:
                f.write("seq," + ",".join(columns) + "\n")
                for i, r in enumerate(rows):
                    f.write(f"{i},{r[0]},{','.join(f'{v:.7g}' for v in r[1:])}\n")


def enter_pressed():
    """True once the user has pressed Enter (the line is consumed); never blocks."""
    if select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def describe(up):
    """How to place the board so that `up` (in the IMU's axes) faces up."""
    names = [f"{'+' if v > 0 else '-'}{axis}" for v, axis in zip(up, "XYZ") if abs(v) > 0.1]
    if len(names) == 1:
        return f"IMU {names[0]} axis up"
    return f"IMU {' and '.join(names)} axes up, tilted half way (rest it on an edge, against a book)"


def guided_pose(cap, number, total, up, hold_s, tolerance_deg=25.0, extra=""):
    """Ask for a pose, show how far the board is from it, and return once it has been still in it
    for `hold_s` seconds (its mean accelerometer)."""
    up = np.asarray(up, float) / np.linalg.norm(up)
    print(f"\n[{number}/{total}] {describe(up)}{extra}.\n        Let go of it, then press Enter.")
    while True:
        while not enter_pressed():
            u = cap.up()
            angle = np.degrees(np.arccos(np.clip(u @ up, -1, 1))) if u is not None else float("nan")
            state = "still" if cap.still(0.5) else "moving"
            now = f"{'+' if u[np.argmax(abs(u))] > 0 else '-'}{'XYZ'[np.argmax(abs(u))]}" if u is not None else "?"
            print(f"\r        {angle:5.1f} deg from the pose (now {now} is up), {state}    ", end="", flush=True)
            time.sleep(0.1)
        print(f"        holding still for {hold_s:.0f} s ...", flush=True)
        end = time.time() + hold_s + 20
        while time.time() < end and not cap.still(hold_s):
            time.sleep(0.1)
        if not cap.still(hold_s):
            print("        it did not stay still: let it rest and press Enter again")
            continue
        u = cap.up(hold_s)
        angle = np.degrees(np.arccos(np.clip(u @ up, -1, 1)))
        if angle > tolerance_deg:
            print(f"        {angle:.0f} deg from the pose (limit {tolerance_deg:.0f}): adjust it and press Enter")
            continue
        print(f"        ok ({angle:.0f} deg from the pose)")
        return cap.recent_imu(hold_s)[:, 1:4].mean(axis=0)


def open_session(port, kind):
    """Start capturing; the default recording prefix is calibration/<kind>_<date>_<time>."""
    cap = LiveCapture(port)
    if not cap.wait_for_data():
        cap.stop()
        sys.exit(f"no IMU data on {port}: is the board connected, and no other reader running?")
    return cap, time.strftime(f"calibration/{kind}_%Y%m%d_%H%M%S")


def normal_gravity(latitude_deg: float, height_m: float) -> float:
    """WGS84 normal gravity (Somigliana) at the latitude, reduced to the height with WGS84's
    second-order free-air formula (NIMA TR8350.2, 4-3)."""
    a, f, m = 6378137.0, 1 / 298.257223563, 0.00344978650684
    s2 = np.sin(np.radians(latitude_deg)) ** 2
    g0 = 9.7803253359 * (1 + 0.00193185265241 * s2) / np.sqrt(1 - 0.00669437999013 * s2)
    return float(g0 * (1 - 2 / a * (1 + f + m - 2 * f * s2) * height_m + 3 * height_m ** 2 / a ** 2))
