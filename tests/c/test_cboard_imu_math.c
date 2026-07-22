#include "cboard_imu_math.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI_F 3.14159265358979323846f

static int near(float actual, float expected, float tolerance) {
    return fabsf(actual - expected) <= tolerance;
}

static void test_transform_and_wrap(void) {
    imu_mount_transform_t identity = {{0, 1, 2}, {1, 1, 1}, 0.0f, false};
    float source[3] = {1.0f, -2.0f, 3.0f};
    float output[3] = {0};
    assert(cboard_imu_transform_valid(&identity));
    assert(cboard_imu_transform_vector(&identity, source, output));
    assert(near(output[0], 1.0f, 1e-6f));
    assert(near(output[1], -2.0f, 1e-6f));
    assert(near(output[2], 3.0f, 1e-6f));

    imu_mount_transform_t rotated = {{1, 0, 2}, {-1, 1, 1}, 0.0f, true};
    assert(cboard_imu_transform_vector(&rotated, source, output));
    assert(near(output[0], 2.0f, 1e-6f));
    assert(near(output[1], 1.0f, 1e-6f));

    imu_mount_transform_t yaw_offset = {{0, 1, 2}, {1, 1, 1}, PI_F / 2.0f, true};
    assert(cboard_imu_transform_vector(&yaw_offset, source, output));
    assert(near(output[0], 2.0f, 1e-5f));
    assert(near(output[1], 1.0f, 1e-5f));

    imu_mount_transform_t invalid = {{0, 0, 2}, {1, 1, 1}, 0.0f, false};
    assert(!cboard_imu_transform_valid(&invalid));
    assert(near(cboard_imu_wrap_0_2pi(-0.1f), 2.0f * PI_F - 0.1f, 1e-5f));
    assert(near(cboard_imu_wrap_0_2pi(2.0f * PI_F + 0.2f), 0.2f, 1e-5f));
    assert(near(cboard_imu_wrap_minus_pi_pi(PI_F + 0.2f), -PI_F + 0.2f, 1e-5f));
    assert(isnan(cboard_imu_wrap_0_2pi(NAN)));
}

static void test_calibration(void) {
    cboard_imu_calibrator_t calibrator;
    cboard_imu_calibrator_reset(&calibrator);
    float accel[3] = {0.0f, 0.0f, CBOARD_IMU_GRAVITY_MPS2};
    float gyro[3] = {0.01f, -0.02f, 0.005f};
    for (uint32_t index = 0; index < CBOARD_IMU_CALIBRATION_SAMPLES; ++index) {
        bool done = cboard_imu_calibrator_add(&calibrator, accel, gyro);
        assert(done == (index + 1U == CBOARD_IMU_CALIBRATION_SAMPLES));
    }
    float bias[3];
    float variance[3];
    assert(cboard_imu_calibrator_result(&calibrator, bias, variance));
    assert(near(bias[0], gyro[0], 1e-6f));
    assert(near(bias[1], gyro[1], 1e-6f));
    assert(near(bias[2], gyro[2], 1e-6f));
    assert(variance[0] < 1e-9f && variance[1] < 1e-9f && variance[2] < 1e-9f);

    float moving_gyro[3] = {0.0f, 0.0f, 0.5f};
    assert(!cboard_imu_calibrator_add(&calibrator, accel, moving_gyro));
    assert(calibrator.state == IMU_CAL_REJECTED_MOVING);
    assert(calibrator.count == 0U);
    assert(calibrator.movement_rejections == 1U);
    float nan_gyro[3] = {NAN, 0.0f, 0.0f};
    assert(!cboard_imu_calibrator_add(&calibrator, accel, nan_gyro));

    cboard_imu_calibrator_reset(&calibrator);
    for (uint32_t index = 0; index < CBOARD_IMU_CALIBRATION_SAMPLES; ++index) {
        float noisy_gyro[3] = {(index & 1U) ? 0.03f : -0.03f, 0.0f, 0.0f};
        (void)cboard_imu_calibrator_add(&calibrator, accel, noisy_gyro);
    }
    assert(!cboard_imu_calibrator_result(&calibrator, bias, variance));
}

static void test_attitude_yaw_continuity(void) {
    cboard_imu_attitude_t attitude;
    cboard_imu_attitude_reset(&attitude);
    float accel[3] = {0.0f, 0.0f, CBOARD_IMU_GRAVITY_MPS2};
    float gyro[3] = {0.0f, 0.0f, PI_F / 2.0f};
    float q[4], roll, pitch, yaw_wrapped, yaw_continuous;
    for (uint32_t index = 0; index < 5000U; ++index) {
        assert(cboard_imu_attitude_update(&attitude, gyro, accel, 0.001f, q, &roll, &pitch,
                                          &yaw_wrapped, &yaw_continuous));
    }
    assert(near(roll, 0.0f, 0.01f));
    assert(near(pitch, 0.0f, 0.01f));
    assert(near(yaw_continuous, 2.5f * PI_F, 0.02f));
    assert(near(yaw_wrapped, 0.5f * PI_F, 0.02f));
    float q_norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    assert(near(q_norm, 1.0f, 1e-5f));

    gyro[0] = INFINITY;
    assert(!cboard_imu_attitude_update(&attitude, gyro, accel, 0.001f, q, &roll, &pitch,
                                       &yaw_wrapped, &yaw_continuous));
}

static void test_six_dof_filter_does_not_inject_unobservable_yaw_feedback(void) {
    cboard_imu_attitude_t attitude;
    cboard_imu_attitude_reset(&attitude);
    float accel[3] = {0.0f, 0.0f, CBOARD_IMU_GRAVITY_MPS2};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    float q[4], roll, pitch, yaw_wrapped, yaw_continuous;

    assert(cboard_imu_attitude_update(&attitude, gyro, accel, 0.001f, q, &roll, &pitch,
                                      &yaw_wrapped, &yaw_continuous));
    /* Accelerometer-only fusion cannot observe rotation around gravity.  A
     * stale Mahony integral term must therefore never steer relative yaw. */
    attitude.integral_error[2] = 0.01f;
    for (uint32_t index = 0; index < 10000U; ++index) {
        assert(cboard_imu_attitude_update(&attitude, gyro, accel, 0.001f, q, &roll, &pitch,
                                          &yaw_wrapped, &yaw_continuous));
    }
    assert(near(yaw_continuous, 0.0f, 1e-4f));
}

int main(void) {
    test_transform_and_wrap();
    test_calibration();
    test_attitude_yaw_continuity();
    test_six_dof_filter_does_not_inject_unobservable_yaw_feedback();
    puts("cboard_imu_math host tests passed");
    return 0;
}
