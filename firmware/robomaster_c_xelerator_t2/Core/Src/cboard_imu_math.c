#include "cboard_imu_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CBOARD_IMU_PI 3.14159265358979323846f
#define CBOARD_IMU_TWO_PI (2.0f * CBOARD_IMU_PI)
#define CBOARD_IMU_CAL_ACCEL_MIN (0.85f * CBOARD_IMU_GRAVITY_MPS2)
#define CBOARD_IMU_CAL_ACCEL_MAX (1.15f * CBOARD_IMU_GRAVITY_MPS2)
#define CBOARD_IMU_CAL_GYRO_MAX 0.12f
#define CBOARD_IMU_CAL_VARIANCE_MAX 0.0004f
#define CBOARD_IMU_MAHONY_KP 1.8f

static bool vector_finite(const float value[3]) {
    return value != NULL && isfinite(value[0]) && isfinite(value[1]) && isfinite(value[2]);
}

static float clamp_unit(float value) {
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}

bool cboard_imu_transform_valid(const imu_mount_transform_t *transform) {
    if (transform == NULL || !isfinite(transform->yaw_mount_offset_rad)) return false;
    uint8_t used = 0U;
    for (uint8_t index = 0; index < 3U; ++index) {
        int8_t axis = transform->source_axis[index];
        if (axis < 0 || axis > 2 || (transform->axis_sign[index] != 1 && transform->axis_sign[index] != -1)) {
            return false;
        }
        uint8_t bit = (uint8_t)(1U << (uint8_t)axis);
        if ((used & bit) != 0U) return false;
        used |= bit;
    }
    return used == 0x07U;
}

bool cboard_imu_transform_vector(
    const imu_mount_transform_t *transform,
    const float sensor[3],
    float robot[3]) {
    if (!cboard_imu_transform_valid(transform) || !vector_finite(sensor) || robot == NULL) return false;
    float mapped[3];
    for (uint8_t index = 0; index < 3U; ++index) {
        mapped[index] = sensor[(uint8_t)transform->source_axis[index]] * (float)transform->axis_sign[index];
    }
    float cosine = cosf(transform->yaw_mount_offset_rad);
    float sine = sinf(transform->yaw_mount_offset_rad);
    robot[0] = cosine * mapped[0] - sine * mapped[1];
    robot[1] = sine * mapped[0] + cosine * mapped[1];
    robot[2] = mapped[2];
    return vector_finite(robot);
}

float cboard_imu_wrap_0_2pi(float angle_rad) {
    if (!isfinite(angle_rad)) return NAN;
    float wrapped = fmodf(angle_rad, CBOARD_IMU_TWO_PI);
    if (wrapped < 0.0f) wrapped += CBOARD_IMU_TWO_PI;
    if (wrapped >= CBOARD_IMU_TWO_PI) wrapped = 0.0f;
    return wrapped;
}

float cboard_imu_wrap_minus_pi_pi(float angle_rad) {
    float wrapped = cboard_imu_wrap_0_2pi(angle_rad + CBOARD_IMU_PI);
    return isfinite(wrapped) ? wrapped - CBOARD_IMU_PI : NAN;
}

void cboard_imu_calibrator_reset(cboard_imu_calibrator_t *calibrator) {
    if (calibrator == NULL) return;
    memset(calibrator, 0, sizeof(*calibrator));
    calibrator->state = IMU_CAL_COLLECTING;
}

bool cboard_imu_calibrator_add(
    cboard_imu_calibrator_t *calibrator,
    const float accel_mps2[3],
    const float gyro_rad_s[3]) {
    if (calibrator == NULL || !vector_finite(accel_mps2) || !vector_finite(gyro_rad_s)) return false;
    float accel_norm = sqrtf(accel_mps2[0] * accel_mps2[0] + accel_mps2[1] * accel_mps2[1] +
                             accel_mps2[2] * accel_mps2[2]);
    float gyro_norm = sqrtf(gyro_rad_s[0] * gyro_rad_s[0] + gyro_rad_s[1] * gyro_rad_s[1] +
                            gyro_rad_s[2] * gyro_rad_s[2]);
    if (accel_norm < CBOARD_IMU_CAL_ACCEL_MIN || accel_norm > CBOARD_IMU_CAL_ACCEL_MAX ||
        gyro_norm > CBOARD_IMU_CAL_GYRO_MAX) {
        calibrator->movement_rejections++;
        calibrator->count = 0U;
        memset(calibrator->gyro_mean, 0, sizeof(calibrator->gyro_mean));
        memset(calibrator->gyro_m2, 0, sizeof(calibrator->gyro_m2));
        calibrator->state = IMU_CAL_REJECTED_MOVING;
        return false;
    }

    calibrator->state = IMU_CAL_COLLECTING;
    calibrator->count++;
    for (uint8_t index = 0; index < 3U; ++index) {
        float delta = gyro_rad_s[index] - calibrator->gyro_mean[index];
        calibrator->gyro_mean[index] += delta / (float)calibrator->count;
        float delta2 = gyro_rad_s[index] - calibrator->gyro_mean[index];
        calibrator->gyro_m2[index] += delta * delta2;
    }
    return calibrator->count >= CBOARD_IMU_CALIBRATION_SAMPLES;
}

bool cboard_imu_calibrator_result(
    const cboard_imu_calibrator_t *calibrator,
    float bias_rad_s[3],
    float variance[3]) {
    if (calibrator == NULL || bias_rad_s == NULL || variance == NULL ||
        calibrator->count < CBOARD_IMU_CALIBRATION_SAMPLES) return false;
    bool valid = true;
    for (uint8_t index = 0; index < 3U; ++index) {
        bias_rad_s[index] = calibrator->gyro_mean[index];
        variance[index] = calibrator->gyro_m2[index] / (float)(calibrator->count - 1U);
        valid = valid && isfinite(bias_rad_s[index]) && isfinite(variance[index]) &&
                variance[index] <= CBOARD_IMU_CAL_VARIANCE_MAX;
    }
    return valid;
}

void cboard_imu_attitude_reset(cboard_imu_attitude_t *attitude) {
    if (attitude == NULL) return;
    memset(attitude, 0, sizeof(*attitude));
    attitude->q[0] = 1.0f;
}

static void quaternion_from_roll_pitch(float roll, float pitch, float q[4]) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

bool cboard_imu_attitude_update(
    cboard_imu_attitude_t *attitude,
    const float gyro_rad_s[3],
    const float accel_mps2[3],
    float dt_s,
    float quaternion_wxyz[4],
    float *roll_rad,
    float *pitch_rad,
    float *yaw_wrapped_rad,
    float *yaw_continuous_rad) {
    if (attitude == NULL || !vector_finite(gyro_rad_s) || !vector_finite(accel_mps2) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f || quaternion_wxyz == NULL ||
        roll_rad == NULL || pitch_rad == NULL || yaw_wrapped_rad == NULL || yaw_continuous_rad == NULL) {
        return false;
    }

    float accel_norm = sqrtf(accel_mps2[0] * accel_mps2[0] + accel_mps2[1] * accel_mps2[1] +
                             accel_mps2[2] * accel_mps2[2]);
    if (!isfinite(accel_norm) || accel_norm < 0.1f) return false;
    float ax = accel_mps2[0] / accel_norm;
    float ay = accel_mps2[1] / accel_norm;
    float az = accel_mps2[2] / accel_norm;

    if (!attitude->initialized) {
        float roll = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
        quaternion_from_roll_pitch(roll, pitch, attitude->q);
        attitude->yaw_previous_wrapped = 0.0f;
        attitude->yaw_continuous = 0.0f;
        attitude->initialized = true;
    }

    float q0 = attitude->q[0];
    float q1 = attitude->q[1];
    float q2 = attitude->q[2];
    float q3 = attitude->q[3];
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float error[3] = {
        ay * vz - az * vy,
        az * vx - ax * vz,
        ax * vy - ay * vx,
    };
    float corrected[3];
    for (uint8_t index = 0; index < 3U; ++index) {
        /* In this 6DoF mode gravity constrains tilt but cannot observe yaw.
         * Persistent integral feedback can therefore wind up around gravity
         * and inject a fictitious yaw rate after long uptime.  The explicit
         * stationary gyro calibration supplies the bias estimate; retain only
         * proportional accelerometer feedback for roll/pitch correction. */
        attitude->integral_error[index] = 0.0f;
        corrected[index] = gyro_rad_s[index] + CBOARD_IMU_MAHONY_KP * error[index];
    }

    float half_dt = 0.5f * dt_s;
    attitude->q[0] += (-q1 * corrected[0] - q2 * corrected[1] - q3 * corrected[2]) * half_dt;
    attitude->q[1] += ( q0 * corrected[0] + q2 * corrected[2] - q3 * corrected[1]) * half_dt;
    attitude->q[2] += ( q0 * corrected[1] - q1 * corrected[2] + q3 * corrected[0]) * half_dt;
    attitude->q[3] += ( q0 * corrected[2] + q1 * corrected[1] - q2 * corrected[0]) * half_dt;
    float q_norm = sqrtf(attitude->q[0] * attitude->q[0] + attitude->q[1] * attitude->q[1] +
                         attitude->q[2] * attitude->q[2] + attitude->q[3] * attitude->q[3]);
    if (!isfinite(q_norm) || q_norm < 0.1f) return false;
    for (uint8_t index = 0; index < 4U; ++index) attitude->q[index] /= q_norm;

    q0 = attitude->q[0]; q1 = attitude->q[1]; q2 = attitude->q[2]; q3 = attitude->q[3];
    *roll_rad = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    *pitch_rad = asinf(clamp_unit(2.0f * (q0 * q2 - q3 * q1)));
    float yaw_signed = atan2f(2.0f * (q0 * q3 + q1 * q2),
                              1.0f - 2.0f * (q2 * q2 + q3 * q3));
    float delta_yaw = cboard_imu_wrap_minus_pi_pi(yaw_signed - attitude->yaw_previous_wrapped);
    attitude->yaw_continuous += delta_yaw;
    attitude->yaw_previous_wrapped = yaw_signed;
    *yaw_wrapped_rad = cboard_imu_wrap_0_2pi(attitude->yaw_continuous);
    *yaw_continuous_rad = attitude->yaw_continuous;
    memcpy(quaternion_wxyz, attitude->q, sizeof(attitude->q));
    return isfinite(*roll_rad) && isfinite(*pitch_rad) && isfinite(*yaw_wrapped_rad) &&
           isfinite(*yaw_continuous_rad);
}
