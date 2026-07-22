#include "chassis_controller.h"
#include "chassis_motor_config_generated.h"
#include "pi_c_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void candidate_wheels(wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT]) {
    const float positions[4] = {0.6981317008f, -0.6981317008f, 2.268928028f, -2.268928028f};
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheels[i].x_m = 0.0605f * cosf(positions[i]);
        wheels[i].y_m = 0.0605f * sinf(positions[i]);
        wheels[i].drive_angle_rad = positions[i] + 1.5707963268f;
        wheels[i].wheel_radius_m = 0.030f;
        wheels[i].gear_ratio = 1.0f;
        wheels[i].motor_sign = 1.0f;
    }
}

static chassis_control_limits_t limits(void) {
    chassis_control_limits_t result = {
        .max_vx_mps = 0.3f,
        .max_vy_mps = 0.3f,
        .max_wz_rad_s = 2.0f,
        .feedback_timeout_ms = 100U,
        .command_timeout_ms = 100U,
    };
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) result.max_motor_rad_s[i] = 20.0f;
    return result;
}

static void healthy_feedback(chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT], uint32_t now, bool enabled) {
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        feedback[i].assigned = true;
        feedback[i].online = true;
        feedback[i].enabled = enabled;
        feedback[i].fault_code = 0U;
        feedback[i].received_tick = now;
    }
}

static void test_production_80_100_geometry_translates_instead_of_rotating(void) {
    wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT];
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheels[i].x_m = CHASSIS_WHEEL_X_M[i];
        wheels[i].y_m = CHASSIS_WHEEL_Y_M[i];
        wheels[i].drive_angle_rad = CHASSIS_DRIVE_ANGLE_RAD[i];
        wheels[i].wheel_radius_m = CHASSIS_WHEEL_RADIUS_M[i];
        wheels[i].gear_ratio = CHASSIS_GEAR_RATIO[i];
        wheels[i].motor_sign = CHASSIS_MOTOR_SIGN[i];
    }
    const float rad_to_deg = 57.2957795131f;
    const float front_included_deg = acosf(cosf(
        wheels[0].drive_angle_rad - wheels[1].drive_angle_rad)) * rad_to_deg;
    const float rear_included_deg = acosf(cosf(
        wheels[2].drive_angle_rad - wheels[3].drive_angle_rad)) * rad_to_deg;
    assert(fabsf(front_included_deg - 80.0f) < 1.0e-4f);
    assert(fabsf(rear_included_deg - 100.0f) < 1.0e-4f);

    chassis_kinematics_t kinematics;
    assert(chassis_kinematics_init(&kinematics, wheels, 10000.0f));
    const chassis_twist_t forward = {0.05f, 0.0f, 0.0f};
    wheel_targets_t targets;
    assert(chassis_inverse_kinematics(&kinematics, &forward, NULL, &targets));
    /* Physical positive motor rotation rolls the left wheels backward and the
     * right wheels forward. Translation therefore needs [-,+,-,+], not four
     * equal positive commands, which make the chassis rotate in place. */
    assert(targets.motor_rad_s[0] < 0.0f);
    assert(targets.motor_rad_s[1] > 0.0f);
    assert(targets.motor_rad_s[2] < 0.0f);
    assert(targets.motor_rad_s[3] > 0.0f);
}

static void test_protocol_decode(void) {
    uint8_t payload[PI_C_CHASSIS_TWIST_PAYLOAD_LEN] = {0};
    const uint32_t host_ms = 0x12345678U;
    const float vx = 0.03f, vy = -0.02f, wz = 0.2f;
    payload[0] = 0x78; payload[1] = 0x56; payload[2] = 0x34; payload[3] = 0x12;
    memcpy(payload + 4, &vx, sizeof(vx));
    memcpy(payload + 8, &vy, sizeof(vy));
    memcpy(payload + 12, &wz, sizeof(wz));
    payload[16] = 100U; payload[18] = PI_C_CHASSIS_MODE_VELOCITY;
    pi_c_chassis_twist_t decoded;
    assert(pi_c_decode_chassis_twist(payload, sizeof(payload), &decoded));
    assert(decoded.host_monotonic_ms == host_ms);
    assert(fabsf(decoded.vx_mps - vx) < 1.0e-6f);
    payload[18] = 99U;
    assert(!pi_c_decode_chassis_twist(payload, sizeof(payload), &decoded));
}

static void test_imu_snapshot_protocol(void) {
    cboard_imu_state_t state;
    memset(&state, 0, sizeof(state));
    state.sequence = 0x12345678U;
    state.timestamp_us = 999U;
    state.state = IMU_STATE_READY;
    state.calibration_state = IMU_CAL_VALID;
    state.source = IMU_SOURCE_GYRO_RELATIVE;
    state.accel_chip_id = 0x1EU;
    state.gyro_chip_id = 0x0FU;
    state.quaternion_wxyz[0] = 1.0f;
    state.accel_mps2[2] = 9.80665f;
    state.temperature_c = 45.0f;
    uint8_t payload[PI_C_IMU_STATUS_PAYLOAD_LEN];
    assert(pi_c_encode_imu_status(payload, sizeof(payload), &state) == PI_C_IMU_STATUS_PAYLOAD_LEN);
    assert(payload[0] == 0x78U && payload[1] == 0x56U);
    assert(payload[48] == IMU_STATE_READY);
    assert(payload[51] == 0x1EU && payload[52] == 0x0FU);
    state.yaw_wrapped_rad = NAN;
    assert(pi_c_encode_imu_status(payload, sizeof(payload), &state) == 0);
}

int main(void) {
    test_production_80_100_geometry_translates_instead_of_rotating();
    wheel_geometry_t wheels[CHASSIS_WHEEL_COUNT];
    candidate_wheels(wheels);
    chassis_kinematics_t kinematics;
    assert(chassis_kinematics_init(&kinematics, wheels, 10000.0f));
    assert(kinematics.rank == 3U);

    chassis_controller_t controller;
    chassis_control_limits_t control_limits = limits();
    assert(chassis_controller_init(&controller, &kinematics, &control_limits));
    assert(controller.state == CHASSIS_DISABLED);

    chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT];
    healthy_feedback(feedback, 1000U, false);
    feedback[2].assigned = false;
    assert(!chassis_controller_arm(&controller, feedback, 1000U));
    assert(controller.state == CHASSIS_DISABLED);

    healthy_feedback(feedback, 1000U, false);
    assert(chassis_controller_arm(&controller, feedback, 1000U));
    healthy_feedback(feedback, 1010U, true);
    chassis_controller_observe_feedback(&controller, feedback, 1010U);
    assert(controller.state == CHASSIS_READY);

    chassis_twist_t command = {0.03f, -0.01f, 0.1f};
    assert(chassis_controller_accept_twist(&controller, 1U, &command, 1020U));
    assert(!chassis_controller_accept_twist(&controller, 1U, &command, 1021U));
    wheel_targets_t targets;
    assert(chassis_controller_update(&controller, 1030U, &targets));
    assert(targets.motor_rad_s[0] != 0.0f);

    assert(chassis_controller_update(&controller, 1201U, &targets));
    assert(controller.state == CHASSIS_STOPPING);
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) assert(targets.motor_rad_s[i] == 0.0f);

    healthy_feedback(feedback, 1210U, false);
    assert(chassis_controller_disabled_confirmed(&controller, feedback, 1210U));
    assert(controller.state == CHASSIS_DISABLED);

    assert(chassis_controller_arm(&controller, feedback, 1210U));
    healthy_feedback(feedback, 1220U, true);
    chassis_controller_observe_feedback(&controller, feedback, 1220U);
    feedback[3].fault_code = 0xAU;
    chassis_controller_observe_feedback(&controller, feedback, 1221U);
    assert(controller.state == CHASSIS_FAULT);
    assert(chassis_controller_update(&controller, 1222U, &targets));
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) assert(targets.motor_rad_s[i] == 0.0f);

    test_protocol_decode();
    test_imu_snapshot_protocol();
    return 0;
}
