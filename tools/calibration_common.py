"""What calibrate_imu.py and calibrate_mag.py share: reading a recording and finding the still poses."""
import sys

import numpy as np


def arg(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


def load_csv(path):
    """A recording file from pellicanus_reader.py --save, as a dict of numpy columns."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        sys.exit(f"{path} is empty")
    return {name: np.atleast_1d(data[name]) for name in data.dtype.names}


def _rolling_mean(x, n):
    c = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), axis=0)
    out = np.full(x.shape, np.nan)
    out[n - 1:] = (c[n:] - c[:-n]) / n
    return out


def static_intervals(t, accel, gyro, initial_s, window_s=0.5, min_s=1.0, trim_s=0.1):
    """Index ranges [start, end) where the board was still.

    The recording must hold the board at rest for `initial_s` seconds before the poses: the first
    such stretch sets the noise the thresholds are scaled from and the gyro bias; what comes before it
    (the board being placed) is skipped. A sample is still when, over the window ending at it, the
    accelerometer's spread and the gyro's rate (bias removed) stay within a few times that noise.
    """
    rate = (len(t) - 1) / (t[-1] - t[0])
    n = max(3, int(window_s * rate))
    a_mean = _rolling_mean(accel, n)
    a_var = _rolling_mean(accel * accel, n) - a_mean * a_mean
    a_spread = np.sqrt(np.clip(a_var, 0, None).sum(axis=1))

    # the first stretch at rest, judged against fixed limits while the noise is still unknown
    g_mean = _rolling_mean(gyro, n)
    g_spread = np.sqrt(np.clip(_rolling_mean(gyro * gyro, n) - g_mean * g_mean, 0, None).sum(axis=1))
    rest = (a_spread < 0.04) & (g_spread < 0.01)
    need = max(1, int(initial_s * rate) - n + 1)
    run, start = 0, None
    for i, r in enumerate(rest):
        run = run + 1 if r else 0
        if run >= need:
            start = max(0, i - run + 1 - (n - 1))
            break
    if start is None:
        sys.exit(f"no {initial_s:.0f} s at rest found: the board must rest still for that long before the poses")
    if t[start] - t[0] > 0.5:
        print(f"the first {t[start] - t[0]:.1f} s (before the board came to rest) are skipped")
    first = (t >= t[start]) & (t < t[start] + initial_s)
    bias = np.median(gyro[first], axis=0)
    g_rate = np.linalg.norm(_rolling_mean(gyro - bias, n), axis=1)
    quiet_a = np.nanmedian(a_spread[first])
    quiet_g = np.nanmedian(g_rate[first])
    still = (a_spread < max(4 * quiet_a, 0.04)) & (g_rate < max(5 * quiet_g, 0.01))
    # a window describes the samples it covers: a still window makes all of them still
    flag = np.zeros(len(t), dtype=bool)
    for end in np.flatnonzero(still):
        flag[end - n + 1:end + 1] = True
    flag[:start] = False

    intervals = []
    start = None
    for i, s in enumerate(np.r_[flag, False]):
        if s and start is None:
            start = i
        elif not s and start is not None:
            a = np.searchsorted(t, t[start] + trim_s)
            b = np.searchsorted(t, t[i - 1] - trim_s)
            if t[i - 1] - t[start] >= min_s and b > a:
                intervals.append((int(a), int(b)))
            start = None
    if not intervals or t[intervals[0][1] - 1] - t[intervals[0][0]] < 0.8 * initial_s - 2 * trim_s:
        sys.exit(f"the first {initial_s:.0f} s at rest were not found again: rest the board longer at the start")
    return intervals
