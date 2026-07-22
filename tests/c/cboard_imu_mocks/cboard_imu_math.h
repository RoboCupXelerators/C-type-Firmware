#ifndef TEST_CBOARD_IMU_MATH_H
#define TEST_CBOARD_IMU_MATH_H

#include "cboard_imu.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t count;
    uint32_t movement_rejections;
    imu_calibration_state_t state;
} cboard_imu_calibrator_t;
typedef struct {
    bool initialized;
    float yaw_continuous;
} cboard_imu_attitude_t;

bool cboard_imu_transform_valid(const imu_mount_transform_t *transform);
bool cboard_imu_transform_vector(
    const imu_mount_transform_t *transform, const float sensor[3], float robot[3]);
void cboard_imu_calibrator_reset(cboard_imu_calibrator_t *calibrator);
bool cboard_imu_calibrator_add(
    cboard_imu_calibrator_t *calibrator, const float accel[3], const float gyro[3]);
bool cboard_imu_calibrator_result(
    const cboard_imu_calibrator_t *calibrator, float bias[3], float variance[3]);
void cboard_imu_attitude_reset(cboard_imu_attitude_t *attitude);
bool cboard_imu_attitude_update(
    cboard_imu_attitude_t *attitude, const float gyro[3], const float accel[3], float dt_s,
    float quaternion[4], float *roll, float *pitch, float *yaw_wrapped, float *yaw_continuous);
float cboard_imu_wrap_0_2pi(float angle_rad);

#endif
