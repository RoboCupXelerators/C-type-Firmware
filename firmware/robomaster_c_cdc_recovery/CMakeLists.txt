cmake_minimum_required(VERSION 3.13)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake"
    CACHE STRING "Path to toolchain file")

project(xelerator_cdc_recovery C ASM)

# --- Target MCU (same as main firmware: STM32F407xx, Cortex-M4F) ---
set(MCU_ARCH cortex-m4)
set(MCU_FPU "-mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_FLAGS "-mcpu=${MCU_ARCH} -mthumb ${MCU_FPU}")

set(CMAKE_C_FLAGS "${MCU_FLAGS} -Wall -Wextra -Os -ffunction-sections -fdata-sections -fno-common" CACHE INTERNAL "C flags")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS}" CACHE INTERNAL "ASM flags")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/Startup/stm32f407vg_flash.ld -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/xelerator_cdc_recovery.map --specs=nano.specs --specs=nosys.specs -lc -lm -lnosys" CACHE INTERNAL "linker flags")

set(INC_DIRS
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
)

set(SRC_FILES
    Core/Src/main.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    # USB CDC
    Core/Src/usb_device.c
    Core/Src/usb_otg.c
    Core/Src/usbd_cdc_if.c
    Core/Src/usbd_conf.c
    Core/Src/usbd_desc.c
    # USB Device Library
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # HAL drivers (minimal subset: RCC, GPIO, PWR, PCD, USB, Cortex, Flash)
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    # CMSIS system
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    # Startup
    Startup/startup_stm32f407xx.s
)

add_definitions(
    -DSTM32F407xx
    -DUSE_HAL_DRIVER
    -DARM_MATH_CM4
)

add_executable(xelerator_cdc_recovery ${SRC_FILES})
target_include_directories(xelerator_cdc_recovery PRIVATE ${INC_DIRS})

add_custom_command(TARGET xelerator_cdc_recovery POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:xelerator_cdc_recovery>
            ${CMAKE_BINARY_DIR}/xelerator_cdc_recovery.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:xelerator_cdc_recovery>
            ${CMAKE_BINARY_DIR}/xelerator_cdc_recovery.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:xelerator_cdc_recovery>
    COMMENT "Generating BIN, HEX, and printing size"
)
