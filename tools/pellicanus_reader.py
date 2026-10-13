#!/usr/bin/env python3
"""Read and check the PELLICANUS binary stream (docs/protocol.md).

    pellicanus_reader.py [PORT] [--seconds N] [--print] [--save PREFIX]
    pellicanus_reader.py [PORT] --live

--live shows every stream's latest values and rate on one screen, refreshed four times a second,
until Ctrl-C.

PORT defaults to /dev/ttyACM0. Uses pyserial when installed (any OS); otherwise reads the port
directly (Linux and macOS). Prints the board's messages, then, after N seconds (default 10), a
summary per stream: rate, sequence gaps, the intervals between samples on the board's own clock,
and the CRC errors seen. --print also prints every decoded packet. --save writes every packet to
PREFIX_imu.csv, PREFIX_mag.csv, PREFIX_baro.csv, PREFIX_gnss.csv and PREFIX_status.csv as it
arrives, for a recording.
"""
import os
import struct
import sys
import time

SYNC = b"\xa5\x5a"
HEADER = struct.Struct("<2sBBHIQ")  # sync, type, version, length, seq, time_us
TYPES = {1: "IMU", 2: "MAG", 3: "BARO", 4: "GNSS", 5: "STATUS", 6: "TEXT"}
STATUS = struct.Struct("<12sBBHHHHIIIIIII")


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def open_port(path: str):
    try:
        import serial  # pyserial

        port = serial.Serial(path, timeout=0.05)
        return lambda: port.read(4096)
    except ImportError:
        import tty

        fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(fd)  # binary: no CR/LF translation, no flow control characters

        def read() -> bytes:
            try:
                return os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.002)
                return b""

        return read


def packets(read, seconds: float, stats: dict):
    """Yield (type, version, seq, time_us, payload) for every packet whose CRC checks."""
    buf = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        buf += read()
        while True:
            start = buf.find(SYNC)
            if start < 0:
                del buf[:-1]
                break
            del buf[:start]
            if len(buf) < HEADER.size:
                break
            _, kind, version, length, seq, time_us = HEADER.unpack_from(buf)
            total = HEADER.size + length + 2
            if length > 255:
                del buf[:2]  # not a header: resynchronise
                continue
            if len(buf) < total:
                break
            body = bytes(buf[2:HEADER.size + length])
            (crc,) = struct.unpack_from("<H", buf, HEADER.size + length)
            if crc != crc16(body):
                stats["crc_errors"] += 1
                del buf[:2]
                continue
            yield kind, version, seq, time_us, bytes(buf[HEADER.size:HEADER.size + length])
            del buf[:total]


def live(port: str) -> None:
    """The latest of everything, redrawn in place."""
    stats = {"crc_errors": 0}
    latest: dict = {}
    arrivals: dict = {}
    text = ""
    gnss = ""
    status = None
    start = last_draw = time.time()
    try:
        for kind, version, seq, time_us, payload in packets(open_port(port), 1e9, stats):
            now = time.time()
            name = TYPES.get(kind, f"type {kind}")
            arrivals.setdefault(name, []).append(now)
            arrivals[name] = [t for t in arrivals[name] if now - t <= 1.0]
            if name == "IMU":
                latest[name] = (seq, time_us, struct.unpack("<7f", payload))
            elif name == "MAG":
                latest[name] = (seq, time_us, struct.unpack("<3f", payload))
            elif name == "BARO":
                latest[name] = (seq, time_us, struct.unpack("<2f", payload))
            elif name == "GNSS":
                gnss = payload.decode(errors="replace")
            elif name == "TEXT":
                text = payload.decode(errors="replace")
            elif name == "STATUS":
                status = STATUS.unpack(payload)
            if now - last_draw < 0.25:
                continue
            last_draw = now
            rate = {k: len(v) for k, v in arrivals.items()}
            lines = [f"PELLICANUS live  {port}  {now - start:7.1f} s  CRC errors {stats['crc_errors']}   (Ctrl-C to stop)", ""]
            if "IMU" in latest:
                seq, t, v = latest["IMU"]
                lines.append(f"IMU   {rate.get('IMU', 0):4d}/s  seq {seq:9d}  board time {t / 1e6:11.6f} s")
                lines.append(f"      accel  {v[0]:+9.4f} {v[1]:+9.4f} {v[2]:+9.4f}  m/s2")
                lines.append(f"      gyro   {v[3]:+9.5f} {v[4]:+9.5f} {v[5]:+9.5f}  rad/s   temp {v[6]:5.2f} C")
            if "MAG" in latest:
                seq, t, v = latest["MAG"]
                lines.append(f"MAG   {rate.get('MAG', 0):4d}/s  field  {v[0]:+8.5f} {v[1]:+8.5f} {v[2]:+8.5f}  gauss")
            if "BARO" in latest:
                seq, t, v = latest["BARO"]
                lines.append(f"BARO  {rate.get('BARO', 0):4d}/s  pressure {v[0]:10.1f} Pa   temp {v[1]:5.2f} C")
            lines.append(f"GNSS  {rate.get('GNSS', 0):4d}/s  {gnss[:90]}")
            if status:
                fw = status[0].rstrip(b"\0").decode()
                lines.append(f"BOARD firmware {fw}  sensors 0x{status[1]:02x}  IMU dropped {status[8]} missed {status[9]}"
                             f"  USB dropped {status[13]}")
            lines.append(f"TEXT  {text[:100]}")
            sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else "/dev/ttyACM0"
    if "--live" in sys.argv:
        live(port)
        return
    seconds = float(sys.argv[sys.argv.index("--seconds") + 1]) if "--seconds" in sys.argv else 10.0
    show = "--print" in sys.argv
    files = {}
    if "--save" in sys.argv:
        prefix = sys.argv[sys.argv.index("--save") + 1]
        os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
        headers = {
            "IMU": "seq,time_us,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,temp_c",
            "MAG": "seq,time_us,x_gauss,y_gauss,z_gauss",
            "BARO": "seq,time_us,pressure_pa,temp_c",
            "GNSS": "seq,time_us,sentence",
            "STATUS": "seq,time_us,sensors_ok,imu_samples,imu_dropped,imu_missed,mag_samples,"
                      "baro_samples,gnss_sentences,usb_dropped",
        }
        for name, header in headers.items():
            # Line-buffered: a recording stopped halfway keeps what it had.
            files[name] = open(f"{prefix}_{name.lower()}.csv", "w", buffering=1)
            files[name].write(header + "\n")
    stats = {"crc_errors": 0}
    seen: dict = {}
    status = None
    for kind, version, seq, time_us, payload in packets(open_port(port), seconds, stats):
        name = TYPES.get(kind, f"type {kind}")
        s = seen.setdefault(name, {"n": 0, "gaps": 0, "seq": None, "t": [], "values": []})
        if s["seq"] is not None and seq != s["seq"] + 1:
            s["gaps"] += 1
        s["seq"] = seq
        s["n"] += 1
        s["t"].append(time_us)
        if name == "IMU":
            s["values"].append(struct.unpack("<7f", payload))
        elif name == "MAG":
            s["values"].append(struct.unpack("<3f", payload))
        elif name == "BARO":
            s["values"].append(struct.unpack("<2f", payload))
        elif name in ("TEXT", "GNSS") and (show or name == "TEXT"):
            print(f"{name:6s} {time_us / 1e6:10.3f} s  {payload.decode(errors='replace')}")
        elif name == "STATUS":
            status = STATUS.unpack(payload)
        if name in files:
            if name in ("IMU", "MAG", "BARO"):
                row = ",".join(f"{v:.7g}" for v in s["values"][-1])
            elif name == "GNSS":
                row = '"' + payload.decode(errors="replace") + '"'
            else:
                row = f"{status[1]}," + ",".join(str(v) for v in status[7:])
            files[name].write(f"{seq},{time_us},{row}\n")
        if show and name not in ("TEXT", "GNSS"):
            print(f"{name:6s} seq {seq} t {time_us} {s['values'][-1] if s['values'] else ''}")

    for f in files.values():
        f.close()
    print(f"\nsummary over {seconds:.0f} s, CRC errors {stats['crc_errors']}")
    for name, s in seen.items():
        t = s["t"]
        line = f"  {name:6s} {s['n']:6d} packets, sequence gaps {s['gaps']}"
        if len(t) > 2 and name in ("IMU", "MAG", "BARO"):
            dt = [(b - a) / 1000.0 for a, b in zip(t, t[1:])]
            mean = sum(dt) / len(dt)
            std = (sum((d - mean) ** 2 for d in dt) / len(dt)) ** 0.5
            line += (f", {1000.0 / mean:.2f} Hz on the board's clock, interval {mean:.4f} ms "
                     f"(std {std * 1000:.1f} us, max {max(dt):.3f} ms)")
        print(line)
        if s["values"] and name in ("IMU", "MAG", "BARO"):
            cols = list(zip(*s["values"]))
            means = " ".join(f"{sum(c) / len(c):.5g}" for c in cols)
            stds = " ".join(f"{(sum((x - sum(c) / len(c)) ** 2 for x in c) / len(c)) ** 0.5:.3g}" for c in cols)
            print(f"         mean [{means}]\n         std  [{stds}]")
    if status:
        fw = status[0].rstrip(b"\0").decode()
        (sensors, decimation, rate, sensor_hz, acc_g, gyro_dps, produced, dropped, missed,
         mag_n, baro_n, gnss_n, usb_dropped) = status[1:]
        print(f"  board: firmware {fw}, sensors 0x{sensors:02x}, IMU {rate} Hz ({sensor_hz} Hz / {decimation}), "
              f"+-{acc_g} g, +-{gyro_dps} deg/s; IMU produced {produced}, dropped {dropped}, missed {missed}; "
              f"USB packets dropped {usb_dropped}")


if __name__ == "__main__":
    main()
