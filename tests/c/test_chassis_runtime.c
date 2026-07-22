#include "chassis_runtime.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static dm2325_motor_t *g_motors;
static uint32_t g_tick;
static uint32_t g_time_us;
static uint8_t g_fail_enable_id;
static uint8_t g_fail_control_id;
static uint8_t g_fail_disable_once_id;
static bool g_suppress_first_disable_feedback;
static uint32_t g_enable_calls[4];
static uint32_t g_disable_calls[4];
static uint32_t g_control_calls[4];
static chassis_kinematics_t g_kinematics;

uint32_t hal_get_tick(void) { return g_tick; }
int hal_can_init(uint32_t baudrate) { (void)baudrate; return 0; }
uint32_t cboard_diag_time_us(void) { g_time_us += 40U; return g_time_us; }

static int index_for_id(uint32_t can_id) {
    for (int index = 0; index < 4; ++index) if (g_motors[index].slave_id == can_id) return index;
    return -1;
}

int hal_can_send(uint32_t can_id, const uint8_t *data, uint8_t len) {
    int index = index_for_id(can_id);
    if (index < 0 || len != 8U) return -1;
    dm2325_motor_t *motor = &g_motors[index];
    if (data[7] == DM_CMD_ENABLE && data[0] == 0xFFU) {
        g_enable_calls[index]++;
        if (motor->slave_id == g_fail_enable_id) return -1;
        motor->feedback.error_code = DM_ERR_ENABLED;
        motor->feedback.fault = false;
        motor->feedback.online = true;
        motor->feedback.last_feedback_tick = g_tick;
        motor->feedback.feedback_sequence++;
    } else if (data[7] == DM_CMD_DISABLE && data[0] == 0xFFU) {
        g_disable_calls[index]++;
        if (motor->slave_id == g_fail_disable_once_id && g_disable_calls[index] == 1U) return -1;
        if (g_suppress_first_disable_feedback && g_disable_calls[index] == 1U) return 0;
        motor->feedback.error_code = DM_ERR_DISABLED;
        motor->feedback.fault = false;
        motor->feedback.online = true;
        motor->feedback.last_feedback_tick = g_tick;
        motor->feedback.feedback_sequence++;
    } else {
        g_control_calls[index]++;
        if (motor->slave_id == g_fail_control_id) return -1;
        motor->feedback.last_feedback_tick = g_tick;
        motor->feedback.feedback_sequence++;
    }
    return 0;
}

static void setup(dm2325_motor_t motors[4], chassis_runtime_t *runtime) {
    memset(motors, 0, sizeof(dm2325_motor_t) * 4U);
    memset(g_enable_calls, 0, sizeof(g_enable_calls));
    memset(g_disable_calls, 0, sizeof(g_disable_calls));
    memset(g_control_calls, 0, sizeof(g_control_calls));
    g_motors = motors; g_tick = 1000U; g_time_us = 0U;
    g_fail_enable_id = 0U; g_fail_control_id = 0U; g_fail_disable_once_id = 0U;
    g_suppress_first_disable_feedback = false;
    for (uint8_t index = 0; index < 4U; ++index) {
        motors[index].slave_id = index + 1U;
        motors[index].master_id = 0x11U + index;
        motors[index].pmax = 12.5f; motors[index].vmax = 200.0f; motors[index].tmax = 10.0f;
        motors[index].feedback.error_code = DM_ERR_DISABLED;
        motors[index].feedback.online = true;
        motors[index].feedback.feedback_sequence = 1U;
        motors[index].feedback.last_feedback_tick = g_tick;
    }
    assert(chassis_runtime_init(runtime, motors));
    wheel_geometry_t wheels[4];
    const float angles[4] = {2.268928028f, 0.872664626f, 3.839724354f, -0.698131701f};
    const float x[4] = {0.046345f, 0.046345f, -0.038889f, -0.038889f};
    const float y[4] = {0.038889f, -0.038889f, 0.046345f, -0.046345f};
    for (uint8_t index = 0; index < 4U; ++index) {
        wheels[index] = (wheel_geometry_t){x[index], y[index], angles[index], 0.03f, 1.0f, 1.0f};
    }
    assert(chassis_kinematics_init(&g_kinematics, wheels, 10000.0f));
    chassis_runtime_set_kinematics(runtime, &g_kinematics);
    assert(runtime->state == CHASSIS_RUNTIME_DISABLED);
}

static void finish_arm(chassis_runtime_t *runtime) {
    for (uint8_t index = 0; index < 4U; ++index) {
        chassis_runtime_tick(runtime, g_tick++);
    }
    assert(runtime->state == CHASSIS_RUNTIME_READY);
    for (uint8_t index = 0; index < 4U; ++index) assert(g_enable_calls[index] == 1U);
}

static void test_success_path(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick));
    finish_arm(&runtime);
    assert(chassis_runtime_accept_zero(&runtime, g_tick));
    for (uint32_t step = 0; step < 20U; ++step) {
        g_tick += 10U;
        assert(chassis_runtime_accept_zero(&runtime, g_tick));
        chassis_runtime_tick(&runtime, g_tick);
    }
    assert(runtime.control_tick_count == 20U);
    assert(runtime.max_command_skew_us > 0U);
    assert(chassis_runtime_request_stop(&runtime, g_tick));
    assert(runtime.state == CHASSIS_RUNTIME_READY);
    for (uint8_t index = 0; index < 4U; ++index) assert(motors[index].enabled);
    assert(chassis_runtime_request_disable(&runtime, g_tick));
    for (uint32_t step = 0; step < 25U; ++step) {
        g_tick += 10U;
        chassis_runtime_tick(&runtime, g_tick);
    }
    assert(runtime.state == CHASSIS_RUNTIME_DISABLED);
    for (uint8_t index = 0; index < 4U; ++index) assert(g_disable_calls[index] == 1U);
}

static void test_each_enable_failure_rolls_back(void) {
    for (uint8_t failed_id = 1U; failed_id <= 4U; ++failed_id) {
        dm2325_motor_t motors[4]; chassis_runtime_t runtime;
        setup(motors, &runtime);
        g_fail_enable_id = failed_id;
        bool accepted = chassis_runtime_request_arm(&runtime, g_tick);
        if (failed_id == 1U) assert(!accepted);
        for (uint8_t step = 0; step < 5U && runtime.state == CHASSIS_RUNTIME_ARMING; ++step) {
            chassis_runtime_tick(&runtime, g_tick++);
        }
        assert(runtime.state == CHASSIS_RUNTIME_FAULT);
        for (uint8_t index = 0; index < 4U; ++index) assert(!motors[index].enabled);
        for (uint8_t index = failed_id; index < 4U; ++index) assert(g_enable_calls[index] == 0U);
    }
}

static void test_disable_confirmation_retry(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    g_suppress_first_disable_feedback = true;
    assert(chassis_runtime_request_disable(&runtime, g_tick));
    for (uint32_t step = 0; step < 30U && runtime.state == CHASSIS_RUNTIME_STOPPING; ++step) {
        g_tick += 10U;
        chassis_runtime_tick(&runtime, g_tick);
    }
    assert(runtime.state == CHASSIS_RUNTIME_DISABLED);
    assert(runtime.fault == CHASSIS_RUNTIME_FAULT_NONE);
    for (uint8_t index = 0; index < 4U; ++index) assert(g_disable_calls[index] == 2U);
}

static void test_stale_and_watchdog_fault(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    motors[2].feedback.last_feedback_tick = g_tick - 101U;
    chassis_runtime_tick(&runtime, g_tick);
    assert(runtime.state == CHASSIS_RUNTIME_FAULT);
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    g_tick += 101U;
    for (uint8_t index = 0; index < 4U; ++index) motors[index].feedback.last_feedback_tick = g_tick;
    chassis_runtime_tick(&runtime, g_tick);
    assert(runtime.state == CHASSIS_RUNTIME_FAULT);
    assert(runtime.fault == CHASSIS_RUNTIME_FAULT_HOST_WATCHDOG);
}

static void test_duplicate_config_rejected(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    motors[3].master_id = motors[2].master_id;
    assert(!chassis_runtime_init(&runtime, motors));
    assert(runtime.state == CHASSIS_RUNTIME_FAULT);
}

static void test_nonzero_twist_is_limited_and_sequence_checked(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    chassis_twist_t command = {1.0f, -1.0f, 9.0f};
    assert(chassis_runtime_accept_twist(&runtime, 10U, &command, g_tick));
    assert(!chassis_runtime_accept_twist(&runtime, 10U, &command, g_tick));
    assert(runtime.requested_twist.vx_mps == 1.0f);
    assert(runtime.requested_twist.vy_mps == -1.0f);
    assert(runtime.requested_twist.wz_rad_s == 2.0f);
    g_tick += 10U;
    chassis_runtime_tick(&runtime, g_tick);
    assert(runtime.state == CHASSIS_RUNTIME_RUNNING);
    for (uint8_t index = 0; index < 4U; ++index) assert(fabsf(runtime.targets_rad_s[index]) <= 20.0f);
}

static void refresh_enabled_feedback(dm2325_motor_t motors[4], const chassis_runtime_t *runtime) {
    for (uint8_t index = 0U; index < 4U; ++index) {
        motors[index].feedback.velocity = runtime->targets_rad_s[index];
        motors[index].feedback.torque = 0.25f;
        motors[index].feedback.error_code = DM_ERR_ENABLED;
        motors[index].feedback.fault = false;
        motors[index].feedback.online = true;
        motors[index].feedback.last_feedback_tick = g_tick;
        motors[index].feedback.feedback_sequence++;
    }
}

static void test_profiles_are_distinct_and_safe_by_default(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(runtime.profile == CHASSIS_PROFILE_NORMAL);
    const chassis_profile_limits_t *normal = chassis_runtime_get_profile_limits(CHASSIS_PROFILE_NORMAL);
    const chassis_profile_limits_t *competition =
        chassis_runtime_get_profile_limits(CHASSIS_PROFILE_COMPETITION_CANDIDATE);
    assert(normal != NULL && competition != NULL);
    assert(normal->reverse_brake_rad_s2 > normal->normal_decel_rad_s2);
    assert(normal->normal_decel_rad_s2 >= normal->normal_accel_rad_s2);
    assert(normal->reverse_jerk_rad_s3 > normal->normal_jerk_rad_s3);
    assert(competition->reverse_brake_rad_s2 > normal->reverse_brake_rad_s2);
    assert(isnan(normal->torque_limit_nm));
    assert(isnan(normal->current_limit_a));
    assert(chassis_runtime_set_profile(&runtime, CHASSIS_PROFILE_SAFE));
    assert(runtime.profile == CHASSIS_PROFILE_SAFE);
    assert(chassis_runtime_request_arm(&runtime, g_tick));
    assert(!chassis_runtime_set_profile(&runtime, CHASSIS_PROFILE_COMPETITION_CANDIDATE));
}

static void test_four_wheel_reversal_brakes_next_tick_and_crosses_zero(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    chassis_twist_t forward = {0.30f, 0.0f, 0.0f};
    chassis_twist_t reverse = {-0.30f, 0.0f, 0.0f};
    uint8_t sequence = 1U;
    for (uint32_t step = 0U; step < 80U; ++step) {
        assert(chassis_runtime_accept_twist(&runtime, sequence++, &forward, g_tick));
        g_tick += 10U;
        refresh_enabled_feedback(motors, &runtime);
        chassis_runtime_tick(&runtime, g_tick);
    }

    float before[4];
    memcpy(before, runtime.targets_rad_s, sizeof(before));
    assert(chassis_runtime_accept_twist(&runtime, sequence++, &reverse, g_tick));
    g_tick += 10U;
    refresh_enabled_feedback(motors, &runtime);
    chassis_runtime_tick(&runtime, g_tick);

    assert(runtime.reverse_event_count == 4U);
    for (uint8_t wheel = 0U; wheel < 4U; ++wheel) {
        assert(runtime.requested_targets_rad_s[wheel] * before[wheel] < 0.0f);
        assert((runtime.targets_rad_s[wheel] - before[wheel]) *
               runtime.requested_targets_rad_s[wheel] > 0.0f);
        assert(runtime.limiter_state[wheel] == CHASSIS_LIMITER_REVERSE_BRAKE);
        assert(runtime.reverse_state[wheel] == CHASSIS_REVERSE_BRAKING);
        assert(runtime.target_acceleration_rad_s2[wheel] *
               runtime.requested_targets_rad_s[wheel] > 0.0f);
    }

    uint32_t exact_zero_cycles[4] = {0U, 0U, 0U, 0U};
    bool crossed[4] = {false, false, false, false};
    for (uint32_t step = 0U; step < 160U; ++step) {
        assert(chassis_runtime_accept_twist(&runtime, sequence++, &reverse, g_tick));
        g_tick += 10U;
        refresh_enabled_feedback(motors, &runtime);
        chassis_runtime_tick(&runtime, g_tick);
        for (uint8_t wheel = 0U; wheel < 4U; ++wheel) {
            if (runtime.targets_rad_s[wheel] == 0.0f) exact_zero_cycles[wheel]++;
            if (runtime.targets_rad_s[wheel] * runtime.requested_targets_rad_s[wheel] > 0.0f) {
                crossed[wheel] = true;
            }
        }
    }
    for (uint8_t wheel = 0U; wheel < 4U; ++wheel) {
        assert(crossed[wheel]);
        assert(exact_zero_cycles[wheel] <= 1U);
        assert(runtime.targets_rad_s[wheel] * runtime.requested_targets_rad_s[wheel] > 0.0f);
    }
    const chassis_reverse_event_t *latest = chassis_runtime_latest_reverse_event(&runtime);
    assert(latest != NULL);
    assert(latest->first_brake_command_ms == latest->command_received_ms + 10U);
    assert(latest->zero_cross_ms != 0U);
    assert(latest->reverse_velocity_reached_ms >= latest->zero_cross_ms);
    assert(latest->result == 1U);
}

static void test_rearm_resets_host_sequence_baseline(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    chassis_twist_t zero = {0.0f, 0.0f, 0.0f};
    assert(chassis_runtime_accept_twist(&runtime, 100U, &zero, g_tick));
    assert(chassis_runtime_request_disable(&runtime, g_tick));
    for (uint32_t step = 0; step < 25U; ++step) {
        g_tick += 10U;
        chassis_runtime_tick(&runtime, g_tick);
    }
    assert(runtime.state == CHASSIS_RUNTIME_DISABLED);
    memset(g_enable_calls, 0, sizeof(g_enable_calls));
    assert(chassis_runtime_request_arm(&runtime, g_tick));
    assert(!runtime.has_command_sequence);
    finish_arm(&runtime);
    /* Distance 140 would be rejected if sequence 100 leaked across Arm. */
    assert(chassis_runtime_accept_twist(&runtime, 240U, &zero, g_tick));
}

static void test_control_failure_disables_all_and_disable_retries_once(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    assert(chassis_runtime_accept_zero(&runtime, g_tick));
    g_fail_control_id = 4U;
    g_fail_disable_once_id = 2U;
    g_tick += 10U;
    chassis_runtime_tick(&runtime, g_tick);
    assert(runtime.state == CHASSIS_RUNTIME_FAULT);
    for (uint8_t index = 0; index < 4U; ++index) assert(!motors[index].enabled);
    assert(g_disable_calls[1] == 2U);
}

static void test_user_emergency_stop_latches_fault(void) {
    dm2325_motor_t motors[4]; chassis_runtime_t runtime;
    setup(motors, &runtime);
    assert(chassis_runtime_request_arm(&runtime, g_tick)); finish_arm(&runtime);
    chassis_runtime_emergency_stop(&runtime, CHASSIS_RUNTIME_FAULT_USER_ESTOP, g_tick);
    assert(runtime.state == CHASSIS_RUNTIME_FAULT);
    assert(runtime.fault == CHASSIS_RUNTIME_FAULT_USER_ESTOP);
    assert(!chassis_runtime_request_arm(&runtime, g_tick));
    for (uint8_t index = 0; index < 4U; ++index) {
        assert(!motors[index].enabled);
        assert(runtime.targets_rad_s[index] == 0.0f);
    }
    chassis_runtime_emergency_stop(&runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, g_tick);
    assert(runtime.fault == CHASSIS_RUNTIME_FAULT_USER_ESTOP);
    assert(runtime.secondary_fault == CHASSIS_RUNTIME_FAULT_CAN_TX);
    assert(runtime.secondary_fault_count == 1U);
    assert(chassis_runtime_clear_fault(&runtime, g_tick));
    assert(runtime.state == CHASSIS_RUNTIME_DISABLED);
}

int main(void) {
    test_success_path();
    test_disable_confirmation_retry();
    test_each_enable_failure_rolls_back();
    test_stale_and_watchdog_fault();
    test_duplicate_config_rejected();
    test_nonzero_twist_is_limited_and_sequence_checked();
    test_profiles_are_distinct_and_safe_by_default();
    test_four_wheel_reversal_brakes_next_tick_and_crosses_zero();
    test_rearm_resets_host_sequence_baseline();
    test_control_failure_disables_all_and_disable_retries_once();
    test_user_emergency_stop_latches_fault();
    return 0;
}
