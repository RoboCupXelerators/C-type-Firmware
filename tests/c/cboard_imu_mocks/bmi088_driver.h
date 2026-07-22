#ifndef TEST_BMI088_DRIVER_H
#define TEST_BMI088_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { BMI088_DEVICE_ACCEL = 0, BMI088_DEVICE_GYRO = 1 } bmi088_device_t;
typedef enum {
    BMI088_RESULT_OK = 0,
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
    bool (*select)(bmi088_device_t, bool);
    bool (*transfer)(const uint8_t *, uint8_t *, size_t);
    void (*delay_ms)(uint32_t);
} bmi088_bus_t;
typedef struct {
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint32_t transfer_error_count;
} bmi088_t;
typedef struct {
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
} bmi088_sample_t;

bmi088_result_t bmi088_init(bmi088_t *device, const bmi088_bus_t *bus);
bmi088_result_t bmi088_read_sample(bmi088_t *device, bmi088_sample_t *sample);

#endif
