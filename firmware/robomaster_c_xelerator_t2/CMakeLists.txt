cmake_minimum_required(VERSION 3.13)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake"
    CACHE STRING "Path to toolchain file")

project(xelerator_t2_c_board C CXX ASM)

option(CBOARD_DIAG_PARSE_ONLY "Accept MIT frames without enabling or sending CAN" OFF)

# --- Target MCU ---
set(MCU_ARCH cortex-m4)
set(MCU_FPU "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_FLAGS "-mcpu=${MCU_ARCH} -mthumb ${MCU_FPU}")

set(CMAKE_C_FLAGS "${MCU_FLAGS} -Wall -Wextra -Os -ffunction-sections -fdata-sections -fno-common -fno-exceptions" CACHE INTERNAL "C flags")
set(CMAKE_CXX_FLAGS "${MCU_FLAGS} -Wall -Wextra -Os -ffunction-sections -fdata-sections -fno-common -fno-exceptions" CACHE INTERNAL "CXX flags")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/Startup/stm32f407vg_flash.ld -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/xelerator_t2_c_board.map --specs=nano.specs --specs=nosys.specs -lc -lm -lnosys" CACHE INTERNAL "linker flags")

# --- Include paths ---
set(INC_DIRS
    Core/Inc
    Core/Inc/bsp
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

# --- Source files ---
set(SRC_FILES
    # Core application
    Core/Src/main.c
    Core/Src/can.c
    Core/Src/gpio.c
    Core/Src/adc.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/dma.c
    Core/Src/crc.c
    Core/Src/rtc.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
    # USB CDC
    Core/Src/usb_device.c
    Core/Src/usb_otg.c
    Core/Src/usbd_cdc_if.c
    Core/Src/usbd_conf.c
    Core/Src/usbd_desc.c
    # BSP
    Core/Src/bsp_can.c
    # Motor control (Xelerator T2)
    Core/Src/dm2325_can.c
    Core/Src/motor_safety.c
    Core/Src/pi_c_protocol.c
    Core/Src/main_loop.c
    Core/Src/cboard_diag.c
    Core/Src/dm_can_discovery.c
    Core/Src/chassis_kinematics.c
    Core/Src/chassis_controller.c
    Core/Src/chassis_runtime.c
    Core/Src/bmi088_driver.c
    Core/Src/cboard_imu_math.c
    Core/Src/cboard_imu.c
    # USB Device Library
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # HAL drivers (subset needed for CAN + USB + GPIO + RCC + DMA)
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    # Startup
    Startup/startup_stm32f407xx.s
)

# --- Build defines ---
add_definitions(
    -DSTM32F407xx
    -DUSE_HAL_DRIVER
    -DARM_MATH_CM4
)

if(CBOARD_DIAG_PARSE_ONLY)
    add_definitions(-DCBOARD_DIAG_PARSE_ONLY=1)
endif()

# --- Target ---
add_executable(xelerator_t2_c_board ${SRC_FILES})

# Keep the produced artifact aligned with build.sh and flashing tooling.
set_target_properties(xelerator_t2_c_board PROPERTIES SUFFIX ".elf")

target_include_directories(xelerator_t2_c_board PRIVATE ${INC_DIRS})

target_link_libraries(xelerator_t2_c_board PRIVATE m)

# --- Post-build: generate bin, hex, and print size ---
add_custom_command(TARGET xelerator_t2_c_board POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:xelerator_t2_c_board>
            ${CMAKE_BINARY_DIR}/xelerator_t2_c_board.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:xelerator_t2_c_board>
            ${CMAKE_BINARY_DIR}/xelerator_t2_c_board.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:xelerator_t2_c_board>
    COMMENT "Generating BIN, HEX, and printing size"
)
