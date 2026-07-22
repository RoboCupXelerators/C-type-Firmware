# C-type-Firmware

STM32F407 firmware for the RoboMaster C board used by the RoboCup Xelerators.
The primary target is a bare-metal C firmware for USB CDC communication with a
Pi and CAN control of DM-S2325 motors.

## Repository layout

- `firmware/robomaster_c_xelerator_t2/`: primary production firmware.
- `firmware/robomaster_c_cdc_recovery/`: minimal USB CDC recovery image.
- `tests/`: Python and C host tests for protocols, IMU math, diagnostics, and
  chassis behavior.
- `tools/`: host-side protocol and chassis utilities.
- `config/`: canonical chassis configuration and generated firmware settings.
- `docs/`: current build, flash, hardware, protocol, and acceptance records.
- `reference/source_manifest.txt`: provenance for official board and motor
  references.

## Hardware target

- MCU: STM32F407VGH6, Cortex-M4F, 168 MHz.
- IMU: BMI088 over SPI.
- Motor bus: classic CAN at 1 Mbps by default.
- Host link: USB OTG FS CDC.

## Build

Install CMake and the GNU Arm Embedded toolchain (`arm-none-eabi-gcc`), then:

```sh
cd firmware/robomaster_c_xelerator_t2
./build.sh
```

The main image is written to the ignored `build/` directory. The script emits
ELF, BIN, HEX, MAP, and SHA256 information.

To build the recovery image:

```sh
cd firmware/robomaster_c_cdc_recovery
./build.sh
```

## Test

```sh
pytest -q
```

The tests run on the host and do not enable motors or access hardware.

## Commit policy

Build directories, compiler outputs, Python caches, macOS metadata, packet
captures, archived history, and vendor manuals are kept out of the initial
source commit. Add those files explicitly only when a reproducible release or
an audit requires them.
