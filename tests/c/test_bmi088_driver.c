#include "bmi088_driver.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t accel_regs[256];
static uint8_t gyro_regs[256];
static bmi088_device_t selected_device;
static bool selected;
static bool force_bus_error;
static bool discard_next_accel_chip_read;
static bool discard_next_gyro_chip_read;

static bool mock_select(bmi088_device_t device, bool active) {
    selected_device = device;
    selected = active;
    return !force_bus_error;
}

static bool mock_transfer(const uint8_t *tx, uint8_t *rx, size_t length) {
    if (force_bus_error || !selected || tx == NULL || rx == NULL || length == 0U) return false;
    uint8_t *regs = selected_device == BMI088_DEVICE_ACCEL ? accel_regs : gyro_regs;
    memset(rx, 0, length);
    if ((tx[0] & 0x80U) != 0U) {
        size_t offset = selected_device == BMI088_DEVICE_ACCEL ? 2U : 1U;
        uint8_t reg = tx[0] & 0x7FU;
        for (size_t index = offset; index < length; ++index) {
            if (reg == 0x00U && selected_device == BMI088_DEVICE_ACCEL &&
                discard_next_accel_chip_read) {
                rx[index] = 0xFFU;
                discard_next_accel_chip_read = false;
                reg++;
            } else if (reg == 0x00U && selected_device == BMI088_DEVICE_GYRO &&
                       discard_next_gyro_chip_read) {
                rx[index] = 0xFFU;
                discard_next_gyro_chip_read = false;
                reg++;
            } else {
                rx[index] = regs[reg++];
            }
        }
    } else if (length >= 2U) {
        uint8_t reg = tx[0];
        if (selected_device == BMI088_DEVICE_ACCEL && reg == 0x7EU) {
            discard_next_accel_chip_read = true;
        } else if (selected_device == BMI088_DEVICE_GYRO && reg == 0x14U) {
            discard_next_gyro_chip_read = true;
        } else {
            regs[reg] = tx[1];
        }
    }
    return true;
}

static void mock_delay(uint32_t delay_ms) {
    (void)delay_ms;
}

static bmi088_bus_t mock_bus(void) {
    bmi088_bus_t bus = {mock_select, mock_transfer, mock_delay};
    return bus;
}

static void put_i16(uint8_t *regs, uint8_t address, int16_t value) {
    regs[address] = (uint8_t)((uint16_t)value & 0xFFU);
    regs[(uint8_t)(address + 1U)] = (uint8_t)((uint16_t)value >> 8);
}

static void reset_registers(void) {
    memset(accel_regs, 0, sizeof(accel_regs));
    memset(gyro_regs, 0, sizeof(gyro_regs));
    accel_regs[0x00] = BMI088_ACCEL_CHIP_ID_VALUE;
    gyro_regs[0x00] = BMI088_GYRO_CHIP_ID_VALUE;
    selected = false;
    force_bus_error = false;
    discard_next_accel_chip_read = false;
    discard_next_gyro_chip_read = false;
}

static void test_init_and_read(void) {
    reset_registers();
    bmi088_t device;
    bmi088_bus_t bus = mock_bus();
    assert(bmi088_init(&device, &bus) == BMI088_RESULT_OK);
    assert(device.initialized);
    assert(device.accel_chip_id == BMI088_ACCEL_CHIP_ID_VALUE);
    assert(device.gyro_chip_id == BMI088_GYRO_CHIP_ID_VALUE);
    assert(accel_regs[0x40] == 0xABU);
    assert(accel_regs[0x41] == 0x00U);
    assert(gyro_regs[0x10] == 0x82U);

    accel_regs[0x03] = 0x80U;
    gyro_regs[0x0A] = 0x80U;
    put_i16(accel_regs, 0x12, 1000);
    put_i16(accel_regs, 0x14, -2000);
    put_i16(accel_regs, 0x16, 32767);
    put_i16(gyro_regs, 0x02, 100);
    put_i16(gyro_regs, 0x04, -200);
    put_i16(gyro_regs, 0x06, 300);
    /* Raw temperature 80 -> 33 C. */
    accel_regs[0x22] = 10U;
    accel_regs[0x23] = 0U;
    bmi088_sample_t sample;
    assert(bmi088_read_sample(&device, &sample) == BMI088_RESULT_OK);
    assert(fabsf(sample.accel_mps2[0] - 1000.0f * BMI088_ACCEL_LSB_TO_MPS2) < 1e-6f);
    assert(fabsf(sample.accel_mps2[1] + 2000.0f * BMI088_ACCEL_LSB_TO_MPS2) < 1e-6f);
    assert(fabsf(sample.gyro_rad_s[2] - 300.0f * BMI088_GYRO_LSB_TO_RAD_S) < 1e-6f);
    assert(fabsf(sample.temperature_c - 33.0f) < 1e-6f);
}

static void test_failures(void) {
    reset_registers();
    accel_regs[0x00] = 0U;
    bmi088_t device;
    bmi088_bus_t bus = mock_bus();
    assert(bmi088_init(&device, &bus) == BMI088_RESULT_ACCEL_NOT_FOUND);

    reset_registers();
    force_bus_error = true;
    assert(bmi088_init(&device, &bus) == BMI088_RESULT_BUS_ERROR);
    assert(device.transfer_error_count > 0U);
}

static void test_accel_data_ready_paces_fusion(void) {
    reset_registers();
    bmi088_t device;
    bmi088_bus_t bus = mock_bus();
    assert(bmi088_init(&device, &bus) == BMI088_RESULT_OK);
    bmi088_sample_t sample;
    accel_regs[0x03] = 0x00U;
    gyro_regs[0x0A] = 0x80U;
    assert(bmi088_read_sample(&device, &sample) == BMI088_RESULT_NOT_READY);
    accel_regs[0x03] = 0x80U;
    gyro_regs[0x0A] = 0x00U;
    assert(bmi088_read_sample(&device, &sample) == BMI088_RESULT_OK);
}

int main(void) {
    test_init_and_read();
    test_failures();
    test_accel_data_ready_paces_fusion();
    puts("bmi088 driver host tests passed");
    return 0;
}
