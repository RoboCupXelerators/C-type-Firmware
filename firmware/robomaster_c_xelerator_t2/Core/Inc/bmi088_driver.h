#ifndef BMI088_DRIVER_H
#define BMI088_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BMI088_ACCEL_CHIP_ID_VALUE 0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU
#define BMI088_ACCEL_LSB_TO_MPS2 0.0008974358974f
#define BMI088_GYRO_LSB_TO_RAD_S 0.0010652644360f

typedef enum {
    BMI088_DEVICE_ACCEL = 0,
    BMI088_DEVICE_GYRO = 1,
} bmi088_device_t;

typedef struct {
    bool (*select)(bmi088_device_t device, bool active);
    bool (*transfer)(const uint8_t *tx, uint8_t *rx, size_t length);
    void (*delay_ms)(uint32_t delay_ms);
} bmi088_bus_t;

typedef enum {
    BMI088_RESULT_OK = 0,
    BMI088_RESULT_BAD_ARGUMENT,
    BMI088_RESULT_BUS_ERROR,
    BMI088_RESULT_ACCEL_NOT_FOUND,
    BMI088_RESULT_GYRO_NOT_FOUND,
    BMI088_RESULT_ACCEL_CONFIG_ERROR,
    BMI088_RESULT_GYRO_CONFIG_ERROR,
    BMI088_RESULT_SENSOR_ERROR,
    BMI088_RESULT_NONFINITE,
    BMI088_RESULT_NOT_READY,
} bmi088_result_t;

typedef struct {
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
    uint8_t accel_status;
    uint8_t gyro_status;
    uint8_t accel_error;
} bmi088_sample_t;

typedef struct {
    bmi088_bus_t bus;
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint32_t transfer_error_count;
    bool initialized;
} bmi088_t;

bmi088_result_t bmi088_init(bmi088_t *device, const bmi088_bus_t *bus);
bmi088_result_t bmi088_read_sample(bmi088_t *device, bmi088_sample_t *sample);
bmi088_result_t bmi088_read_register(
    bmi088_t *device,
    bmi088_device_t sensor,
    uint8_t reg,
    uint8_t *value);

#endif
