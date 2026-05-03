# PELLICANUS-C

PELLICANUS-C is the firmware for the PELLICANUS hardware platform. It runs on a Raspberry Pi Pico / RP2040 target and streams sensor telemetry from the onboard IMU, magnetometer, barometer, and GNSS module over USB CDC serial.

## Features

- C firmware built with the Raspberry Pi Pico SDK and CMake.
- IAM-20680 IMU support for acceleration and gyroscope data.
- MMC5983MA magnetometer support.
- MS5637 barometer support for pressure, temperature, and approximate altitude.
- SL-871 GNSS support for position, speed, heading, altitude, and fix data.
- USB CDC serial output enabled.
- IMU, magnetometer, and barometer sampling at 100 Hz.
- GNSS update loop at 4 Hz.

## Hardware Connections

### Shared I2C1 Bus

MMC5983MA and MS5637 share the same I2C bus.

| Signal | Pico GPIO |
| --- | --- |
| SDA | GPIO26 |
| SCL | GPIO27 |
| Speed | 100 kHz |

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

### Status LED

| Purpose | Pico GPIO |
| --- | --- |
| Sensor status LED | GPIO0 |

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

## Serial Output

After startup, the firmware prints initialization status and telemetry over USB CDC serial.

Example startup output:

```text
[OK] IAM-20680 initialised
[OK] MMC5983MA initialised
[OK] MS5637 initialised
[OK] SL-871 GNSS initialised
PELLICANUS v2 ready - streaming at 100 Hz IMU / 4 Hz GNSS
```

Telemetry lines contain:

```text
PELLICANUS v2 AX:<accel_x> AY:<accel_y> AZ:<accel_z> GX:<gyro_x> GY:<gyro_y> GZ:<gyro_z> MX:<mag_x> MY:<mag_y> MZ:<mag_z> P:<pressure> T:<temperature> ALT:<altitude> LAT:<latitude> LON:<longitude> HGNSS:<gnss_altitude> SPD:<speed> HDG:<heading>
```

If the GNSS fix is not valid yet, the `LAT`, `LON`, `HGNSS`, `SPD`, and `HDG` fields are omitted.

## Project Structure

```text
.
|-- CMakeLists.txt
|-- flash.ps1
|-- include/
|   |-- pellicanus.h
|   |-- iam20680.h
|   |-- mmc5983ma.h
|   |-- ms5637.h
|   `-- sl871_gnss.h
`-- src/
    |-- main.c
    |-- iam20680.c
    |-- mmc5983ma.c
    |-- ms5637.c
    `-- sl871_gnss.c
```

## Source Overview

- `src/main.c`: System initialization, timers, sensor read loop, and serial telemetry output.
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
