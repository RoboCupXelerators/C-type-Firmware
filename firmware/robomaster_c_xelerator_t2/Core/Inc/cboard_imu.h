#ifndef CBOARD_IMU_H
#define CBOARD_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IMU_STATE_UNINITIALIZED = 0,
    IMU_STATE_INITIALIZING = 1,
    IMU_STATE_CALIBRATING = 2,
    IMU_STATE_READY = 3,
    IMU_STATE_DEGRADED = 4,
    IMU_STATE_FAULT = 5,
} imu_state_code_t;

typedef enum {
    IMU_CAL_NOT_STARTED = 0,
    IMU_CAL_COLLECTING = 1,
    IMU_CAL_VALID = 2,
    IMU_CAL_REJECTED_MOVING = 3,
    IMU_CAL_TIMEOUT = 4,
} imu_calibration_state_t;

typedef enum {
    IMU_SOURCE_GYRO_RELATIVE = 1,
    IMU_SOURCE_MAG_FUSED = 2,
    IMU_SOURCE_EXTERNAL_CORRECTED = 3,
} imu_heading_source_t;

enum {
    IMU_FLAG_ACCEL_INIT_FAILED = 1UL << 0,
    IMU_FLAG_GYRO_INIT_FAILED = 1UL << 1,
    IMU_FLAG_MAG_INIT_FAILED = 1UL << 2,
    IMU_FLAG_SPI_ERROR = 1UL << 3,
    IMU_FLAG_SAMPLE_STALE = 1UL << 4,
    IMU_FLAG_SAMPLE_OVERRUN = 1UL << 5,
    IMU_FLAG_CALIBRATION_NOT_VALID = 1UL << 6,
    IMU_FLAG_CALIBRATION_MOVEMENT_DETECTED = 1UL << 7,
    IMU_FLAG_TEMPERATURE_NOT_STABLE = 1UL << 8,
    IMU_FLAG_MAG_DISTURBANCE = 1UL << 9,
    IMU_FLAG_ATTITUDE_NOT_VALID = 1UL << 10,
    IMU_FLAG_TIMESTAMP_ERROR = 1UL << 11,
    IMU_FLAG_AXIS_MAPPING_INVALID = 1UL << 12,
    IMU_FLAG_NONFINITE_SAMPLE = 1UL << 13,
    IMU_FLAG_AXIS_MAPPING_UNVERIFIED = 1UL << 14,
    IMU_FLAG_ACCEL_SENSOR_ERROR = 1UL << 15,
    IMU_FLAG_TEMPERATURE_OVERTEMP = 1UL << 16,
    IMU_FLAG_HEATER_UNAVAILABLE = 1UL << 17,
};

typedef struct {
    int8_t source_axis[3];
    int8_t axis_sign[3];
    float yaw_mount_offset_rad;
    bool hardware_verified;
} imu_mount_transform_t;

typedef struct {
    uint32_t sequence;
    uint32_t timestamp_us;

    float accel_mps2[3];
    float gyro_rad_s[3];
    float mag[3];
    float quaternion_wxyz[4];

    float roll_rad;
    float pitch_rad;
    float yaw_wrapped_rad;
    float yaw_continuous_rad;
    float gyro_z_rad_s;
    float temperature_c;
    float gyro_bias_rad_s[3];

    uint32_t sample_count;
    uint32_t error_count;
    uint32_t missed_sample_count;
    uint32_t age_us;
    uint32_t flags;
    imu_state_code_t state;
    imu_calibration_state_t calibration_state;
    imu_heading_source_t source;

    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint16_t calibration_sample_count;
    float calibration_gyro_variance[3];
    uint32_t sample_interval_min_us;
    uint32_t sample_interval_max_us;
    uint32_t update_execution_max_us;
    uint32_t spi_error_count;
    uint32_t movement_rejection_count;
} cboard_imu_state_t;

void cboard_imu_init(void);
void cboard_imu_update(void);
bool cboard_imu_get_snapshot(cboard_imu_state_t *out);
bool cboard_imu_is_healthy(void);
bool cboard_imu_zero_yaw(void);
bool cboard_imu_set_yaw_reference(float angle_rad);
uint32_t cboard_imu_get_age_us(void);
uint32_t cboard_imu_get_error_flags(void);

#ifdef CBOARD_IMU_TEST
void cboard_imu_test_publish_state(const cboard_imu_state_t *state);
#endif

#endif
