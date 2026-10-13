# PELLICANUS-C

PELLICANUS-C is the firmware for the PELLICANUS hardware platform. It runs on a Raspberry Pi Pico / RP2040 target and streams sensor telemetry from the onboard IMU, magnetometer, barometer, and GNSS module over USB CDC serial.

## Features

- C firmware built with the Raspberry Pi Pico SDK and CMake.
- IAM-20680 IMU support for acceleration and gyroscope data.
- MMC5983MA magnetometer support.
- MS5637 barometer support for pressure, temperature, and approximate altitude.
- SL-871 GNSS support for position, speed, heading, altitude, and fix data.
- Binary USB CDC output with timestamps, sequence numbers and CRC (docs/protocol.md).
- IMU sampled at 1 kHz on the second core, from its data-ready interrupt, and averaged to 250 Hz;
  every sample carries the RP2040's time in microseconds and a sequence number.
- Magnetometer at 50 Hz with automatic SET/RESET; barometer at 40 Hz with pressure at OSR 8192;
  both read without blocking.
- GNSS received by interrupt, so no byte is lost; its baud rate is found at start (the receiver
  is moved from 9600 to 115200), it is set to 4 fixes a second, and every NMEA sentence is
  forwarded.

## Hardware Connections

### Shared I2C1 Bus

MMC5983MA and MS5637 share the same I2C bus.

| Signal | Pico GPIO |
| --- | --- |
| SDA | GPIO26 |
| SCL | GPIO27 |
| Speed | 400 kHz |

### IAM-20680 SPI

| Signal | Pico GPIO |
| --- | --- |
| CS | GPIO17 |
| SCK | GPIO18 |
| MOSI | GPIO19 |
| MISO | GPIO20 |
| Speed | 1 MHz |

### SL-871 GNSS UART

| Signal | Pico GPIO |
| --- | --- |
| TX | GPIO28 |
| RX | GPIO29 |
| Initial baud rate | 9600 |
| Fast baud rate | 115200 |

### LEDs

| LED | Net | Driven by |
| --- | --- | --- |
| D2 | GPIO0 | the firmware: the board's health |
| D4 | TIMEPULSE (also GPIO5) | the GNSS receiver's 1PPS: a pulse a second once it has a fix |

D2:

| Pattern | Meaning |
| --- | --- |
| solid | starting the sensors |
| one short flash a second | healthy, and a host is reading the stream |
| two short flashes a second | healthy, but nobody is reading (not connected, or packets dropped) |
| five flashes a second | a fault: a sensor did not start, the GNSS receiver was not heard, IMU samples were lost, or the GNSS receive buffer overflowed |

GPIO5 is only ever an input: it shares D4's net with the receiver's TIMEPULSE output.

## Requirements

- Raspberry Pi Pico SDK
- CMake 3.13 or newer
- ARM GNU Toolchain (`arm-none-eabi`)
- Python
- Ninja or another CMake-supported generator

This project uses `pico_sdk_import.cmake`, so `PICO_SDK_PATH` must point to your local Pico SDK installation before configuring the build.

PowerShell example:

```powershell
$env:PICO_SDK_PATH="C:\path\to\pico-sdk"
```

## Build

Configure the build directory:

```powershell
cmake -S . -B build -G Ninja
```

Build the firmware:

```powershell
cmake --build build
```

A regular CMake build produces `build\pellicanus.elf`. The included `flash.ps1` script also converts the firmware to BIN and UF2 formats before copying it to the Pico in BOOTSEL mode.

## Flashing

Run the flashing helper from PowerShell:

```powershell
.\flash.ps1
```

The script:

1. Removes old firmware outputs from `build`.
2. Rebuilds the project.
3. Converts `pellicanus.elf` to `pellicanus.bin`.
4. Uses `build\bin2uf2.py` to create `pellicanus_final.uf2`.
5. Prompts you to connect the Pico in BOOTSEL mode.
6. Copies the UF2 file to `D:\pellicanus_final.uf2`.

The script currently uses local tool paths:

```powershell
$CMAKE   = "C:\Program Files\CMake\bin\cmake.exe"
$OBJCOPY = "C:\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe"
```

Update `flash.ps1` if these paths are different on your machine.

## USB Output

The firmware sends a binary stream over USB CDC serial: every measurement as a packet with the
RP2040's time in microseconds, a sequence number and a CRC. The format is in
[docs/protocol.md](docs/protocol.md). Open the port in raw mode.

`tools/pellicanus_reader.py` decodes and checks it (Python 3, pyserial optional on Linux and macOS):

```text
python3 tools/pellicanus_reader.py /dev/ttyACM0 --seconds 10
```

It prints the board's messages, then a summary per stream: rate and sample intervals on the
board's clock, sequence gaps, CRC errors, and the board's own counters. Typical boot messages:

```text
[OK] IAM-20680HT initialised (WHO_AM_I 0xFA)
[OK] MMC5983MA initialised
[OK] MS5637 initialised
[OK] SL-871 GNSS heard at 115200 baud
PELLICANUS 2.1.0 ready: IMU 250 Hz (1000 Hz sensor, mean of 4), protocol 1
```

## Build and Flash on Linux

With the Pico SDK (`PICO_SDK_PATH`) and `gcc-arm-none-eabi`, `libnewlib-arm-none-eabi` and
`libusb-1.0-0-dev` installed:

```text
cmake -S . -B build -G Ninja -DPICO_BOARD=pico
cmake --build build
picotool load -f -x build/pellicanus.uf2
```

`picotool load -f` asks the running firmware to reboot into BOOTSEL itself, so no button is needed.
The `picotool` the SDK builds during the build has no USB support; build a full one from its
source (or install it) for flashing, and install its `udev/60-picotool.rules` for USB access.

`tools/static_analysis.py` analyses a recording of the board at rest (`--save PREFIX`): stream
continuity, the IMU's Allan deviation (white noise and bias instability), gyro bias against
temperature, barometer and magnetometer noise; `--plot OUT.png` draws the Allan deviation and the
time series. Record 30 minutes or more, untouched:

```text
python3 tools/pellicanus_reader.py /dev/ttyACM0 --seconds 1800 --save static/run
python3 tools/static_analysis.py static/run --plot static/run.png
```

## Calibration

`tools/calibrate_imu.py` and `tools/calibrate_mag.py` calibrate the IMU and the magnetometer by hand,
without equipment. Each one guides a session with the board, pose by pose, and writes the result as
JSON. See [docs/calibration.md](docs/calibration.md).

```text
python3 tools/calibrate_imu.py --port /dev/ttyACM0
python3 tools/calibrate_mag.py --port /dev/ttyACM0 --imu-calibration calibration/imu_..._imu_calibration.json
```

## Project Structure

```text
.
|-- CMakeLists.txt
|-- flash.ps1
|-- include/
|   |-- pellicanus.h
|   |-- protocol.h
|   |-- imu_stream.h
|   |-- iam20680.h
|   |-- mmc5983ma.h
|   |-- ms5637.h
|   `-- sl871_gnss.h
|-- docs/
|   |-- calibration.md
|   `-- protocol.md
|-- tools/
|   |-- pellicanus_reader.py
|   |-- static_analysis.py
|   |-- calibrate_imu.py
|   |-- calibrate_mag.py
|   |-- calibration_common.py
|   |-- live_capture.py
|   `-- test_calibration_synthetic.py
`-- src/
    |-- main.c
    |-- protocol.c
    |-- imu_stream.c
    |-- iam20680.c
    |-- mmc5983ma.c
    |-- ms5637.c
    `-- sl871_gnss.c
```

## Source Overview

- `src/main.c`: System initialization, the main loop and the USB output.
- `src/imu_stream.c`: The IMU on the second core: data-ready sampling at 1 kHz, averaging, timestamps.
- `src/protocol.c`: The binary packets and CRC; packets are dropped, never waited for, when the host does not read.
- `src/iam20680.c`: IAM-20680 SPI driver.
- `src/mmc5983ma.c`: MMC5983MA I2C magnetometer driver.
- `src/ms5637.c`: MS5637 I2C barometer driver.
- `src/sl871_gnss.c`: SL-871 UART/NMEA GNSS reader.
- `include/pellicanus.h`: Shared pin, bus speed, and sampling-rate configuration.

## Configuration

Main firmware settings are defined in `include/pellicanus.h`:

```c
#define PELLICANUS_I2C_SPEED 100000
#define PELLICANUS_IMU_HZ    100
#define PELLICANUS_GNSS_HZ   4
```

The IMU SPI pin mapping is defined in `include/iam20680.h`, and the GNSS UART pin mapping is defined in `include/sl871_gnss.h`.

## Notes

- `build/` and generated firmware files are ignored by Git.
- `flash_nuke.uf2` can be used as a helper UF2 for clearing the Pico flash memory.
- `pico_add_extra_outputs` is disabled in `CMakeLists.txt`; UF2 generation is handled by `flash.ps1` in the current workflow.
