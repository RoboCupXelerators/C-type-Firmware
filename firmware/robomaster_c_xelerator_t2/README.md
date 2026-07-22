# Xelerator T2 C Board Firmware

Based on official RoboMaster C Board example 20.standard_robot.
Built with arm-none-eabi-gcc + CMake.

## Build

    bash build.sh

## Output

- build/xelerator_t2_c_board.elf
- build/xelerator_t2_c_board.bin
- build/xelerator_t2_c_board.hex
- build/xelerator_t2_c_board.map

## MCU

- STM32F407VGH6, 168 MHz, 1 MB Flash, 128 KB SRAM + 64 KB CCM
- HSE = 12 MHz (from official .ioc)

## Peripherals

- CAN1: PD0(RX)/PD1(TX), 1 Mbps classic CAN -> DM-S2325 motor
- USB OTG FS: PA11(DM)/PA12(DP), CDC virtual serial port -> Pi5
- No FreeRTOS (bare metal)

## Firmware Features (first version)

- USB CDC enumeration
- Pi-C PING/PONG
- GET_STATUS/STATUS
- CAN init + error status read
- DM-S2325 feedback receive + decode
- Single motor velocity command (VEL mode)
- Enable / 0 velocity / Disable
- Pi heartbeat timeout: 300ms -> auto zero + disable
- Motor feedback timeout
- Bus-off handling (HAL CAN auto recovery)
- E-stop interface placeholder

## NOT in first version

- Four-wheel chassis kinematics
- Multi-motor sync
- PID tuning
- MIT mode / position mode / torque mode
- Motor parameter write
- Motor ID change
- CAN baud change
- Firmware upgrade
- Kicker / dribbler / lidar / camera

## Official Source

- Base: reference/vendor/robomaster-c-official/20.standard_robot (commit 59d12b1a)
- HAL: STM32F4xx_HAL_Driver from official project
- USB: STM32_USB_Device_Library CDC class
- CAN: Official bsp_can.c filter config
- Startup: GCC-compatible replacement for Keil startup_stm32f407xx.s
- Linker: GCC .ld derived from Keil project memory layout
