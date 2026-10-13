# PELLICANUS USB protocol, version 1

The board sends one binary stream over its USB CDC serial port: every measurement as a packet with
the RP2040's time in microseconds, a sequence number per stream and a CRC. `tools/pellicanus_reader.py`
reads and checks it; its code is also the reference decoder.

Open the port in **raw mode** (no CR/LF translation). On Linux, `tty.setraw` or `stty raw`; pyserial
does this by itself.

## Packet

All fields little-endian.

| Offset | Size | Field | |
|---|---|---|---|
| 0 | 2 | sync | `0xA5 0x5A` |
| 2 | 1 | type | see below |
| 3 | 1 | version | payload version of this type (1) |
| 4 | 2 | length | payload bytes (at most 255) |
| 6 | 4 | seq | packet number within this type, consecutive from 0 |
| 10 | 8 | time_us | the RP2040's `time_us_64()` for the measurement |
| 18 | length | payload | |
| 18 + length | 2 | crc | CRC-16/CCITT-FALSE over bytes 2 .. 17 + length |

The CRC uses polynomial 0x1021 and initial value 0xFFFF, with no reflection and no final XOR.
`crc16(b"123456789") == 0x29B1`.

### Reading the stream

- To find a packet, look for the sync bytes, then check the CRC. If the CRC fails, skip the two sync
  bytes and search again: the sync bytes can occur inside a payload.
- A gap in `seq` is a packet that was not sent. The board drops whole packets when the host is not
  connected or not reading fast enough. It never waits for the host, so sampling goes on.

## Types

### `0x01` IMU, 28 bytes, 250 Hz

| Field | Type | Unit |
|---|---|---|
| accel x, y, z | float32 × 3 | m/s² |
| gyro x, y, z | float32 × 3 | rad/s |
| temperature | float32 | °C (IMU die) |

- **Rate:** the IAM-20680 samples at 1 kHz. Each packet is the mean of 4 consecutive samples.
- **Filtering:** the sensor's low-pass filters are at 41 Hz (gyro) and 44.8 Hz (accelerometer), below
  the 125 Hz Nyquist frequency of the output.
- **`time_us`:** the mean of the four samples' data-ready times. The sensor reads `INT` at each
  sample, and the second core timestamps it. The motion is older than that by the low-pass filters'
  delay: a few milliseconds, which a camera–IMU time calibration absorbs.
- **`seq`:** a gap means a sample the board produced but could not send.

### `0x02` MAG, 12 bytes

| Field | Type | Unit |
|---|---|---|
| field x, y, z | float32 × 3 | gauss |

50 Hz continuous measurement, sent as each is ready. Automatic SET/RESET is on, with a SET pulse
every 100 measurements (2 s), so the sensor's offset does not drift with temperature or after a strong
field. `time_us` is when it was read, within 2 ms of the measurement.

### `0x03` BARO, 8 bytes, 40 Hz

| Field | Type | Unit |
|---|---|---|
| pressure | float32 | Pa |
| temperature | float32 | °C |

Pressure at OSR 8192 (about 1.6 Pa RMS), temperature at OSR 1024, with the MS5637's datasheet
compensation, second order included. `time_us` is when the conversion pair completed.

### `0x04` GNSS, variable

One NMEA sentence from the SL-871, ASCII, without CR LF, sent as it completes. The receiver is set
to 4 fixes a second (PMTK220,250) at start; its acknowledgement, `$PMTK001,220,3`, comes through
here too.

### `0x05` STATUS, 50 bytes, once a second

| Field | Type | |
|---|---|---|
| firmware | char × 12 | version, NUL-padded |
| sensors_ok | uint8 | bit 0 IMU, bit 1 magnetometer, bit 2 barometer initialised |
| imu_decimation | uint8 | sensor samples per IMU packet |
| imu_rate_hz | uint16 | IMU packet rate |
| imu_sensor_hz | uint16 | the sensor's own rate |
| accel_range_g | uint16 | ± full scale |
| gyro_range_dps | uint16 | ± full scale |
| imu_samples | uint32 | IMU samples produced |
| imu_dropped | uint32 | IMU samples lost between the cores |
| imu_missed | uint32 | sensor samples not read before the next one |
| mag_samples | uint32 | |
| baro_samples | uint32 | |
| gnss_sentences | uint32 | |
| usb_dropped | uint32 | packets of any type not sent: host not connected or not reading |

`imu_dropped` and `imu_missed` should stay 0. `usb_dropped` grows whenever nobody reads.

### `0x06` TEXT, variable

A message, ASCII: the boot report (which sensors were found) and errors. The boot report is only
seen by a host that has the port open by then, about four seconds after power-up. `sensors_ok` in
STATUS carries the same information every second.

## Axes

Each sensor reports in **its own axes**, right-handed, as its datasheet draws them. The board
applies no mounting rotation and no calibration. Turning the axes into the vehicle's, and biases,
scale factors and soft and hard iron, are for the host to apply.
