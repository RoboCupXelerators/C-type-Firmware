#ifndef CBOARD_IMU_MATH_H
#define CBOARD_IMU_MATH_H

#include "cboard_imu.h"

#define CBOARD_IMU_GRAVITY_MPS2 9.80665f
#define CBOARD_IMU_CALIBRATION_SAMPLES 1000U

typedef struct {
    uint32_t count;
    uint32_t movement_rejections;
    float gyro_mean[3];
    float gyro_m2[3];
    imu_calibration_state_t state;
} cboard_imu_calibrator_t;

typedef struct {
    float q[4];
    float integral_error[3];
    float yaw_previous_wrapped;
    float yaw_continuous;
    bool initialized;
} cboard_imu_attitude_t;

bool cboard_imu_transform_valid(const imu_mount_transform_t *transform);
bool cboard_imu_transform_vector(
    const imu_mount_transform_t *transform,
    const float sensor[3],
    float robot[3]);
float cboard_imu_wrap_0_2pi(float angle_rad);
float cboard_imu_wrap_minus_pi_pi(float angle_rad);

void cboard_imu_calibrator_reset(cboard_imu_calibrator_t *calibrator);
bool cboard_imu_calibrator_add(
    cboard_imu_calibrator_t *calibrator,
    const float accel_mps2[3],
    const float gyro_rad_s[3]);
bool cboard_imu_calibrator_result(
    const cboard_imu_calibrator_t *calibrator,
    float bias_rad_s[3],
    float variance[3]);

void cboard_imu_attitude_reset(cboard_imu_attitude_t *attitude);
bool cboard_imu_attitude_update(
    cboard_imu_attitude_t *attitude,
    const float gyro_rad_s[3],
    const float accel_mps2[3],
    float dt_s,
    float quaternion_wxyz[4],
    float *roll_rad,
    float *pitch_rad,
    float *yaw_wrapped_rad,
    float *yaw_continuous_rad);

#endif
