#!/usr/bin/env python3
"""Analyse a static recording made with pellicanus_reader.py --save PREFIX: continuity of every stream
and the IMU's Allan deviation (white noise and bias instability), barometer and magnetometer noise.

    static_analysis.py PREFIX [--plot OUT.png]

Record with the board on a steady surface, untouched, for 30 minutes or more: the bias instability
shows at averaging times of minutes. Needs numpy and pandas, and matplotlib for --plot.
"""
import sys

import numpy as np
import pandas as pd

prefix = sys.argv[1]
imu = pd.read_csv(f"{prefix}_imu.csv")
mag = pd.read_csv(f"{prefix}_mag.csv")
baro = pd.read_csv(f"{prefix}_baro.csv")
gnss = pd.read_csv(f"{prefix}_gnss.csv")
status = pd.read_csv(f"{prefix}_status.csv")

print("== continuity")
for name, df in (("IMU", imu), ("MAG", mag), ("BARO", baro), ("GNSS", gnss), ("STATUS", status)):
    seq = df.seq.to_numpy()
    gaps = int((np.diff(seq) != 1).sum())
    line = f"  {name:6s} {len(df):7d} packets, sequence gaps {gaps}"
    if name in ("IMU", "MAG", "BARO"):
        dt = np.diff(df.time_us.to_numpy()) / 1e3
        line += (f", {1e3 / dt.mean():.3f} Hz, interval {dt.mean():.4f} ms, std {dt.std() * 1e3:.1f} us, "
                 f"min {dt.min():.3f}, max {dt.max():.3f} ms")
    print(line)
first, last = status.iloc[0], status.iloc[-1]
span = (last.time_us - first.time_us) / 1e6
print(f"  board over {span / 60:.1f} min: IMU produced {last.imu_samples - first.imu_samples}, "
      f"dropped {last.imu_dropped - first.imu_dropped}, missed {last.imu_missed - first.imu_missed}; "
      f"USB packets dropped {last.usb_dropped - first.usb_dropped}")


def allan(x, rate, taus):
    """Overlapping Allan deviation of a rate signal x sampled at `rate`."""
    cs = np.cumsum(np.r_[0.0, x]) / rate
    out = []
    for tau in taus:
        m = int(round(tau * rate))
        if m < 1 or 2 * m >= len(x):
            out.append(np.nan)
            continue
        avg = (cs[m:] - cs[:-m]) / (m / rate)
        d = avg[m:] - avg[:-m]
        out.append(np.sqrt(0.5 * np.mean(d * d)))
    return np.array(out)


t = (imu.time_us.to_numpy() - imu.time_us.iloc[0]) / 1e6
rate = (len(t) - 1) / t[-1]
taus = np.logspace(np.log10(4 / rate), np.log10(t[-1] / 8), 40)
print(f"\n== IMU Allan deviation ({t[-1] / 60:.1f} min at {rate:.2f} Hz)")
results = {}
for cols, kind, unit, white_unit, conv in (
        (("gx_rad_s", "gy_rad_s", "gz_rad_s"), "gyro", "rad/s", "deg/sqrt(h)", np.degrees(1) * 60),
        (("ax_m_s2", "ay_m_s2", "az_m_s2"), "accel", "m/s2", "ug/sqrt(Hz)", 1e6 / 9.80665)):
    for c in cols:
        ad = allan(imu[c].to_numpy(), rate, taus)
        results[c] = ad
        # white noise: the -1/2 slope read at tau = 1 s (fit over 0.05-1 s)
        sel = (taus >= 0.05) & (taus <= 1.0) & np.isfinite(ad)
        n = np.exp(np.mean(np.log(ad[sel]) + 0.5 * np.log(taus[sel])))  # sigma(1 s)
        b = np.nanmin(ad) / 0.664
        tb = taus[np.nanargmin(ad)]
        if kind == "gyro":
            print(f"  {c}: ARW {n * conv:.3f} {white_unit} ({n:.2e} rad/s/sqrt(Hz)), "
                  f"bias instability {np.degrees(b) * 3600:.2f} deg/h at tau {tb:.0f} s")
        else:
            print(f"  {c}: VRW {n * conv:.0f} {white_unit} ({n:.2e} m/s2/sqrt(Hz)), "
                  f"bias instability {b / 9.80665 * 1e6:.1f} ug at tau {tb:.0f} s")

print("\n== temperature")
print(f"  IMU die {imu.temp_c.iloc[:2500].mean():.2f} -> {imu.temp_c.iloc[-2500:].mean():.2f} C; "
      f"barometer {baro.temp_c.iloc[:400].mean():.2f} -> {baro.temp_c.iloc[-400:].mean():.2f} C")

# gyro bias against the die temperature, from 30 s means (only meaningful if the temperature moved)
blocks = imu.groupby(imu.time_us // 30_000_000).mean()
spread = blocks.temp_c.max() - blocks.temp_c.min()
if spread > 0.5:
    slopes = [np.degrees(np.polyfit(blocks.temp_c, blocks[c], 1)[0]) for c in ("gx_rad_s", "gy_rad_s", "gz_rad_s")]
    print(f"  gyro bias against temperature ({spread:.1f} C range, 30 s means): "
          + ", ".join(f"{c} {s * 1e3:+.1f}" for c, s in zip("xyz", slopes)) + " mdeg/s per C")

print("\n== barometer and magnetometer noise (sample-to-sample, so drift does not count)")
for name, df, cols, k, unit in (("BARO", baro, ("pressure_pa",), 1.0, "Pa"),
                                ("MAG", mag, ("x_gauss", "y_gauss", "z_gauss"), 1e5, "nT")):
    for c in cols:
        d = np.diff(df[c].to_numpy())
        drift = df[c].iloc[-200:].mean() - df[c].iloc[:200].mean()
        print(f"  {name} {c}: white {d.std() / np.sqrt(2) * k:.2f} {unit}, drift over the run {drift * k:+.1f} {unit}")

if "--plot" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = sys.argv[sys.argv.index("--plot") + 1]
    t0 = imu.time_us.iloc[0]
    minutes = lambda df: (df.time_us - t0) / 60e6

    # 1. Allan deviation
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    for c in ("gx_rad_s", "gy_rad_s", "gz_rad_s"):
        ax[0].loglog(taus, np.degrees(results[c]) * 3600, label=c[:2])
    ax[0].set(xlabel="tau (s)", ylabel="Allan deviation (deg/h)", title="Gyro Allan deviation")
    for c in ("ax_m_s2", "ay_m_s2", "az_m_s2"):
        ax[1].loglog(taus, results[c] / 9.80665 * 1e6, label=c[:2])
    ax[1].set(xlabel="tau (s)", ylabel="Allan deviation (ug)", title="Accelerometer Allan deviation")
    for a in ax:
        a.grid(True, which="both", alpha=0.3)
        a.legend()
    fig.tight_layout()
    fig.savefig(out.replace(".png", "_allan.png"), dpi=110)

    # 2. Time series over the whole recording, each axis about its mean
    fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    every = max(1, len(imu) // 20000)  # thin the 250 Hz series for drawing
    for c in ("ax_m_s2", "ay_m_s2", "az_m_s2"):
        ax[0].plot(minutes(imu)[::every], (imu[c] - imu[c].mean())[::every] * 1e3, lw=0.4, label=f"{c[:2]} - {imu[c].mean():+.3f}")
    ax[0].set(ylabel="accel - mean (mm/s2)", title="Accelerometer")
    for c in ("gx_rad_s", "gy_rad_s", "gz_rad_s"):
        ax[1].plot(minutes(imu)[::every], np.degrees(imu[c] - imu[c].mean())[::every], lw=0.4,
                   label=f"{c[:2]} - {np.degrees(imu[c].mean()):+.3f} deg/s")
    ax[1].set(ylabel="gyro - mean (deg/s)", title="Gyro")
    for c in ("x_gauss", "y_gauss", "z_gauss"):
        ax[2].plot(minutes(mag), (mag[c] - mag[c].mean()) * 1e5, lw=0.5, label=f"{c[0]} - {mag[c].mean():+.4f} G")
    ax[2].set(ylabel="field - mean (nT)", title="Magnetometer")
    ax[3].plot(minutes(baro), baro.pressure_pa - baro.pressure_pa.mean(), lw=0.5,
               label=f"pressure - {baro.pressure_pa.mean():.0f} Pa")
    ax[3].set(ylabel="pressure - mean (Pa)", title="Barometer")
    ax[4].plot(minutes(imu)[::every], imu.temp_c[::every], lw=0.8, label="IMU die")
    ax[4].plot(minutes(baro), baro.temp_c, lw=0.8, label="barometer")
    ax[4].set(ylabel="C", xlabel="minutes", title="Temperature")
    for a in ax:
        a.grid(True, alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out.replace(".png", "_series.png"), dpi=110)
    print(f"\nplots: {out.replace('.png', '_allan.png')}, {out.replace('.png', '_series.png')}")
