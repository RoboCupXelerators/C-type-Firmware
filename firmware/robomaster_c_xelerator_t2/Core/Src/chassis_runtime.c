#include "chassis_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define FEEDBACK_TIMEOUT_MS 100U
#define ARM_STEP_TIMEOUT_MS 100U
#define HOST_WATCHDOG_MS 100U
#define CONTROL_PERIOD_MS 10U
#define ZERO_BEFORE_DISABLE_MS 200U
#define DISABLE_CONFIRM_TIMEOUT_MS 150U
#define DISABLE_CONFIRM_RETRY_MS 75U
#define SAFE_KD 1.0f
/* DM2325: 560 rpm no-load output speed at 24 V = 58.64 rad/s. */
#define MAX_VX_MPS 2.29f
#define MAX_VY_MPS 2.29f
#define MAX_WZ_RAD_S 2.0f
#define MAX_MOTOR_RAD_S 58.64f

/* Motor-side rad/s limits. Competition values are candidates and are not the
 * default until staged hardware testing proves current, voltage and traction. */
static const chassis_profile_limits_t PROFILE_LIMITS[] = {
    [CHASSIS_PROFILE_SAFE] = {
        8.0f, 12.0f, 18.0f, 40.0f, 120.0f, 0.20f, 0.50f, 0.05f, NAN, NAN
    },
    [CHASSIS_PROFILE_NORMAL] = {
        16.7f, 25.0f, 40.0f, 66.7f, 240.0f, 0.20f, 0.50f, 0.05f, NAN, NAN
    },
    [CHASSIS_PROFILE_COMPETITION_CANDIDATE] = {
        40.0f, 60.0f, 100.0f, 200.0f, 600.0f, 0.20f, 0.50f, 0.05f, NAN, NAN
    }
};

extern uint32_t cboard_diag_time_us(void);

static bool elapsed(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

static bool fresh(const dm2325_motor_t *motor, uint32_t now) {
    return motor->feedback.feedback_sequence != 0U &&
           (uint32_t)(now - motor->feedback.last_feedback_tick) <= FEEDBACK_TIMEOUT_MS;
}

static bool all_feedback_state(const chassis_runtime_t *runtime, uint32_t now, uint8_t state) {
    for (uint8_t index = 0; index < CHASSIS_RUNTIME_WHEEL_COUNT; ++index) {
        const dm2325_motor_t *motor = &runtime->motors[index];
        if (!fresh(motor, now) || motor->feedback.fault || motor->feedback.error_code != state) return false;
    }
    return true;
}

static void clear_targets(chassis_runtime_t *runtime) {
    memset(runtime->targets_rad_s, 0, sizeof(runtime->targets_rad_s));
    memset(runtime->requested_targets_rad_s, 0, sizeof(runtime->requested_targets_rad_s));
    memset(runtime->target_acceleration_rad_s2, 0, sizeof(runtime->target_acceleration_rad_s2));
    memset(runtime->reverse_state, 0, sizeof(runtime->reverse_state));
    memset(runtime->limiter_state, 0, sizeof(runtime->limiter_state));
    memset(runtime->active_reverse_event, UINT8_MAX, sizeof(runtime->active_reverse_event));
    memset(&runtime->requested_twist, 0, sizeof(runtime->requested_twist));
    memset(&runtime->limited_twist, 0, sizeof(runtime->limited_twist));
    memset(runtime->twist_acceleration, 0, sizeof(runtime->twist_acceleration));
}

static float clampf(float value, float limit) {
    return fmaxf(-limit, fminf(limit, value));
}

const chassis_profile_limits_t *chassis_runtime_get_profile_limits(chassis_control_profile_t profile) {
    if (profile > CHASSIS_PROFILE_COMPETITION_CANDIDATE) return NULL;
    return &PROFILE_LIMITS[profile];
}

bool chassis_runtime_set_profile(chassis_runtime_t *runtime, chassis_control_profile_t profile) {
    const chassis_profile_limits_t *limits = chassis_runtime_get_profile_limits(profile);
    if (runtime == NULL || limits == NULL || runtime->state != CHASSIS_RUNTIME_DISABLED) return false;
    runtime->profile = profile;
    runtime->profile_limits = *limits;
    return true;
}

const chassis_reverse_event_t *chassis_runtime_latest_reverse_event(const chassis_runtime_t *runtime) {
    if (runtime == NULL || runtime->reverse_event_count == 0U) return NULL;
    uint8_t index = (uint8_t)((runtime->reverse_event_head + CHASSIS_REVERSE_EVENT_CAPACITY - 1U) %
                              CHASSIS_REVERSE_EVENT_CAPACITY);
    return &runtime->reverse_events[index];
}

static float step_motor_target(float target, float measured, float *current, float *acceleration,
                               const chassis_profile_limits_t *limits,
                               chassis_reverse_state_t *reverse_state,
                               chassis_limiter_state_t *limiter_state) {
    const float dt = (float)CONTROL_PERIOD_MS / 1000.0f;
    if (fabsf(target) < limits->velocity_deadband_rad_s) target = 0.0f;
    bool reversing = fabsf(target) > limits->reverse_detection_epsilon_rad_s &&
                     fabsf(measured) > limits->reverse_detection_epsilon_rad_s &&
                     target * measured < 0.0f;
    bool decelerating = !reversing && fabsf(target) < fabsf(measured);
    float max_accel = limits->normal_accel_rad_s2;
    float max_jerk = limits->normal_jerk_rad_s3;
    *limiter_state = CHASSIS_LIMITER_NORMAL_ACCEL;
    if (reversing) {
        max_accel = limits->reverse_brake_rad_s2;
        max_jerk = limits->reverse_jerk_rad_s3;
        *limiter_state = CHASSIS_LIMITER_REVERSE_BRAKE;
        if (*reverse_state == CHASSIS_REVERSE_IDLE || *reverse_state == CHASSIS_REVERSE_COMPLETE) {
            *reverse_state = CHASSIS_REVERSE_BRAKING;
        }
    } else if (decelerating) {
        max_accel = limits->normal_decel_rad_s2;
        *limiter_state = CHASSIS_LIMITER_NORMAL_DECEL;
    }

    float desired_accel = clampf((target - *current) / dt, max_accel);
    /* A stale same-direction acceleration must never survive a reversal. The
     * reverse jerk limit still shapes the new braking acceleration from zero. */
    if (reversing && *acceleration * desired_accel < 0.0f) *acceleration = 0.0f;
    float accel_delta = clampf(desired_accel - *acceleration, max_jerk * dt);
    *acceleration = clampf(*acceleration + accel_delta, max_accel);
    float next = *current + *acceleration * dt;
    if ((target - *current) * (target - next) <= 0.0f) {
        next = target;
        *acceleration = 0.0f;
    }
    *current = next;
    return next;
}

static void start_reverse_event(chassis_runtime_t *runtime, uint8_t wheel,
                                float old_target, float new_target, uint32_t now) {
    uint8_t slot = runtime->reverse_event_head;
    for (uint8_t index = 0U; index < CHASSIS_RUNTIME_WHEEL_COUNT; ++index) {
        if (runtime->active_reverse_event[index] == slot) runtime->active_reverse_event[index] = UINT8_MAX;
    }
    chassis_reverse_event_t *event = &runtime->reverse_events[slot];
    memset(event, 0, sizeof(*event));
    event->event_id = ++runtime->reverse_event_sequence;
    event->command_received_ms = runtime->latest_command_ms;
    event->first_brake_command_ms = now;
    event->old_target_velocity_rad_s = old_target;
    event->new_target_velocity_rad_s = new_target;
    event->measured_velocity_at_command_rad_s = runtime->motors[wheel].feedback.velocity;
    event->motor_id = runtime->motors[wheel].slave_id;
    event->profile = (uint8_t)runtime->profile;
    runtime->active_reverse_event[wheel] = slot;
    runtime->reverse_event_head = (uint8_t)((slot + 1U) % CHASSIS_REVERSE_EVENT_CAPACITY);
    if (runtime->reverse_event_count < CHASSIS_REVERSE_EVENT_CAPACITY) runtime->reverse_event_count++;
}

static void update_reverse_events(chassis_runtime_t *runtime, uint32_t now) {
    for (uint8_t wheel = 0U; wheel < CHASSIS_RUNTIME_WHEEL_COUNT; ++wheel) {
        uint8_t slot = runtime->active_reverse_event[wheel];
        if (slot == UINT8_MAX) continue;
        chassis_reverse_event_t *event = &runtime->reverse_events[slot];
        float measured = runtime->motors[wheel].feedback.velocity;
        float torque = fabsf(runtime->motors[wheel].feedback.torque);
        if (torque > event->peak_measured_torque_nm) event->peak_measured_torque_nm = torque;
        event->fault_code = runtime->motors[wheel].feedback.error_code;
        if (event->fault_code != DM_ERR_ENABLED) {
            event->result = 2U;
            runtime->active_reverse_event[wheel] = UINT8_MAX;
            continue;
        }
        if (event->zero_cross_ms == 0U &&
            (fabsf(measured) <= runtime->profile_limits.zero_speed_epsilon_rad_s ||
             measured * event->new_target_velocity_rad_s > 0.0f)) {
            event->zero_cross_ms = now;
            runtime->reverse_state[wheel] = CHASSIS_REVERSE_ZERO_CROSSED;
        }
        float reached = fmaxf(runtime->profile_limits.reverse_detection_epsilon_rad_s,
                              fabsf(event->new_target_velocity_rad_s) * 0.25f);
        if (event->zero_cross_ms != 0U && measured * event->new_target_velocity_rad_s > 0.0f &&
            fabsf(measured) >= reached) {
            event->reverse_velocity_reached_ms = now;
            event->result = 1U;
            runtime->reverse_state[wheel] = CHASSIS_REVERSE_COMPLETE;
            runtime->active_reverse_event[wheel] = UINT8_MAX;
        }
    }
}

static bool update_motion_targets(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime->kinematics == NULL) return false;
    float limits[4] = {MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S, MAX_MOTOR_RAD_S};
    wheel_targets_t requested;
    if (!chassis_inverse_kinematics(runtime->kinematics, &runtime->requested_twist, limits, &requested)) return false;
    update_reverse_events(runtime, now);
    for (uint8_t wheel = 0U; wheel < CHASSIS_RUNTIME_WHEEL_COUNT; ++wheel) {
        float old_requested = runtime->requested_targets_rad_s[wheel];
        float new_requested = requested.motor_rad_s[wheel];
        if (fabsf(old_requested) > runtime->profile_limits.reverse_detection_epsilon_rad_s &&
            fabsf(new_requested) > runtime->profile_limits.reverse_detection_epsilon_rad_s &&
            old_requested * new_requested < 0.0f) {
            start_reverse_event(runtime, wheel, old_requested, new_requested, now);
        }
        runtime->requested_targets_rad_s[wheel] = new_requested;
        runtime->targets_rad_s[wheel] = step_motor_target(
            new_requested, runtime->motors[wheel].feedback.velocity,
            &runtime->targets_rad_s[wheel], &runtime->target_acceleration_rad_s2[wheel],
            &runtime->profile_limits, &runtime->reverse_state[wheel], &runtime->limiter_state[wheel]);
        if (requested.scale < 1.0f &&
            runtime->limiter_state[wheel] != CHASSIS_LIMITER_REVERSE_BRAKE) {
            runtime->limiter_state[wheel] = CHASSIS_LIMITER_SPEED_SATURATION;
        }
    }
    (void)chassis_forward_kinematics(runtime->kinematics, runtime->targets_rad_s,
                                     &runtime->limited_twist);
    return true;
}

static bool send_control_tick(chassis_runtime_t *runtime) {
    uint32_t started_us = cboard_diag_time_us();
    bool success = true;
    for (uint8_t index = 0; index < CHASSIS_RUNTIME_WHEEL_COUNT; ++index) {
        if (dm2325_can_mit_ctrl(&runtime->motors[index], 0.0f, runtime->targets_rad_s[index],
                                0.0f, SAFE_KD, 0.0f) != DM2325_OK) {
            runtime->can_tx_failure_count++;
            success = false;
        }
    }
    uint32_t skew = cboard_diag_time_us() - started_us;
    if (skew > runtime->max_command_skew_us) runtime->max_command_skew_us = skew;
    runtime->control_sequence++;
    runtime->control_tick_count++;
    return success;
}

static bool disable_all(chassis_runtime_t *runtime) {
    bool success = true;
    for (uint8_t index = 0; index < CHASSIS_RUNTIME_WHEEL_COUNT; ++index) {
        if (dm2325_can_mit_disable(&runtime->motors[index]) != DM2325_OK) {
            runtime->can_tx_failure_count++;
            success = false;
        }
    }
    return success;
}

static void enter_fault(chassis_runtime_t *runtime, chassis_runtime_fault_t fault, uint32_t now) {
    clear_targets(runtime);
    if (runtime->fault == CHASSIS_RUNTIME_FAULT_NONE) {
        runtime->fault = fault;
    } else {
        runtime->secondary_fault = fault;
        runtime->secondary_fault_count++;
    }
    for (uint8_t index = 0; index < CHASSIS_RUNTIME_WHEEL_COUNT; ++index) {
        if (runtime->motors[index].enabled) {
            dm2325_can_mit_ctrl(&runtime->motors[index], 0.0f, 0.0f, 0.0f, SAFE_KD, 0.0f);
        }
    }
    if (!disable_all(runtime)) {
        /* The safety contract permits exactly one retry of failed Disable. */
        disable_all(runtime);
    }
    runtime->state = CHASSIS_RUNTIME_FAULT;
    runtime->transition_started_ms = now;
}

bool chassis_runtime_init(chassis_runtime_t *runtime, dm2325_motor_t motors[4]) {
    if (runtime == NULL || motors == NULL) return false;
    memset(runtime, 0, sizeof(*runtime));
    runtime->motors = motors;
    runtime->profile = CHASSIS_PROFILE_NORMAL;
    runtime->profile_limits = PROFILE_LIMITS[CHASSIS_PROFILE_NORMAL];
    memset(runtime->active_reverse_event, UINT8_MAX, sizeof(runtime->active_reverse_event));
    for (uint8_t left = 0; left < CHASSIS_RUNTIME_WHEEL_COUNT; ++left) {
        if (motors[left].slave_id > 0x0FU || motors[left].master_id == 0U) {
            runtime->fault = CHASSIS_RUNTIME_FAULT_CONFIG;
            runtime->state = CHASSIS_RUNTIME_FAULT;
            return false;
        }
        for (uint8_t right = left + 1U; right < CHASSIS_RUNTIME_WHEEL_COUNT; ++right) {
            if (motors[left].slave_id == motors[right].slave_id ||
                motors[left].master_id == motors[right].master_id) {
                runtime->fault = CHASSIS_RUNTIME_FAULT_CONFIG;
                runtime->state = CHASSIS_RUNTIME_FAULT;
                return false;
            }
        }
    }
    runtime->state = CHASSIS_RUNTIME_DISABLED;
    return true;
}

void chassis_runtime_set_kinematics(chassis_runtime_t *runtime, chassis_kinematics_t *kinematics) {
    if (runtime != NULL) runtime->kinematics = kinematics;
}

bool chassis_runtime_request_arm(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime == NULL || runtime->state != CHASSIS_RUNTIME_DISABLED ||
        !all_feedback_state(runtime, now, DM_ERR_DISABLED)) return false;
    clear_targets(runtime);
    runtime->fault = CHASSIS_RUNTIME_FAULT_NONE;
    runtime->state = CHASSIS_RUNTIME_ARMING;
    /* Arm starts a new host-control session. Sequence anti-replay must not
     * compare its first Twist against the final sequence of the prior run. */
    runtime->has_command_sequence = false;
    runtime->last_command_sequence = 0U;
    runtime->arm_index = 0U;
    runtime->transition_started_ms = now;
    runtime->transition_deadline_ms = now + ARM_STEP_TIMEOUT_MS;
    runtime->awaited_feedback_sequence = runtime->motors[0].feedback.feedback_sequence;
    if (dm2325_can_mit_enable(&runtime->motors[0]) != DM2325_OK) {
        enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
        return false;
    }
    return true;
}

bool chassis_runtime_accept_zero(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime == NULL || (runtime->state != CHASSIS_RUNTIME_READY &&
                            runtime->state != CHASSIS_RUNTIME_RUNNING)) return false;
    clear_targets(runtime);
    runtime->latest_command_ms = now;
    runtime->state = CHASSIS_RUNTIME_READY;
    return true;
}

bool chassis_runtime_accept_twist(chassis_runtime_t *runtime, uint8_t sequence,
                                  const chassis_twist_t *twist, uint32_t now) {
    if (runtime == NULL || twist == NULL || runtime->kinematics == NULL ||
        (runtime->state != CHASSIS_RUNTIME_READY && runtime->state != CHASSIS_RUNTIME_RUNNING) ||
        !isfinite(twist->vx_mps) || !isfinite(twist->vy_mps) || !isfinite(twist->wz_rad_s)) return false;
    if (runtime->has_command_sequence) {
        uint8_t distance = (uint8_t)(sequence - runtime->last_command_sequence);
        if (distance == 0U || distance > 127U) return false;
    }
    runtime->requested_twist.vx_mps = clampf(twist->vx_mps, MAX_VX_MPS);
    runtime->requested_twist.vy_mps = clampf(twist->vy_mps, MAX_VY_MPS);
    runtime->requested_twist.wz_rad_s = clampf(twist->wz_rad_s, MAX_WZ_RAD_S);
    runtime->last_command_sequence = sequence;
    runtime->has_command_sequence = true;
    runtime->latest_command_ms = now;
    runtime->state = (runtime->requested_twist.vx_mps != 0.0f ||
                      runtime->requested_twist.vy_mps != 0.0f ||
                      runtime->requested_twist.wz_rad_s != 0.0f)
        ? CHASSIS_RUNTIME_RUNNING : CHASSIS_RUNTIME_READY;
    return true;
}

bool chassis_runtime_request_stop(chassis_runtime_t *runtime, uint32_t now) {
    return chassis_runtime_accept_zero(runtime, now);
}

bool chassis_runtime_request_disable(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime == NULL || (runtime->state != CHASSIS_RUNTIME_READY &&
                            runtime->state != CHASSIS_RUNTIME_RUNNING)) return false;
    clear_targets(runtime);
    runtime->state = CHASSIS_RUNTIME_STOPPING;
    runtime->transition_started_ms = now;
    runtime->last_control_tick_ms = now - CONTROL_PERIOD_MS;
    runtime->disable_sent = false;
    runtime->disable_confirmation_retry_sent = false;
    return true;
}

void chassis_runtime_emergency_stop(chassis_runtime_t *runtime, chassis_runtime_fault_t fault, uint32_t now) {
    if (runtime != NULL) enter_fault(runtime, fault, now);
}

bool chassis_runtime_clear_fault(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime == NULL || runtime->state != CHASSIS_RUNTIME_FAULT ||
        !all_feedback_state(runtime, now, DM_ERR_DISABLED)) return false;
    runtime->fault = CHASSIS_RUNTIME_FAULT_NONE;
    runtime->state = CHASSIS_RUNTIME_DISABLED;
    return true;
}

void chassis_runtime_tick(chassis_runtime_t *runtime, uint32_t now) {
    if (runtime == NULL) return;
    if (runtime->state == CHASSIS_RUNTIME_ARMING) {
        dm2325_motor_t *motor = &runtime->motors[runtime->arm_index];
        if (motor->feedback.feedback_sequence != runtime->awaited_feedback_sequence) {
            if (!fresh(motor, now) || motor->feedback.fault || motor->feedback.error_code != DM_ERR_ENABLED) {
                enter_fault(runtime, CHASSIS_RUNTIME_FAULT_ARM_FEEDBACK, now);
                return;
            }
            runtime->arm_index++;
            if (runtime->arm_index == CHASSIS_RUNTIME_WHEEL_COUNT) {
                runtime->state = CHASSIS_RUNTIME_READY;
                runtime->latest_command_ms = now;
                runtime->last_control_tick_ms = now - CONTROL_PERIOD_MS;
                return;
            }
            motor = &runtime->motors[runtime->arm_index];
            runtime->awaited_feedback_sequence = motor->feedback.feedback_sequence;
            runtime->transition_deadline_ms = now + ARM_STEP_TIMEOUT_MS;
            if (dm2325_can_mit_enable(motor) != DM2325_OK) {
                enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
            }
        } else if ((int32_t)(now - runtime->transition_deadline_ms) >= 0) {
            enter_fault(runtime, CHASSIS_RUNTIME_FAULT_ARM_TIMEOUT, now);
        }
        return;
    }

    if (runtime->state == CHASSIS_RUNTIME_READY || runtime->state == CHASSIS_RUNTIME_RUNNING) {
        if (!all_feedback_state(runtime, now, DM_ERR_ENABLED)) {
            enter_fault(runtime, CHASSIS_RUNTIME_FAULT_MOTOR_FEEDBACK, now);
            return;
        }
        if (elapsed(now, runtime->latest_command_ms, HOST_WATCHDOG_MS + 1U)) {
            enter_fault(runtime, CHASSIS_RUNTIME_FAULT_HOST_WATCHDOG, now);
            return;
        }
        if (elapsed(now, runtime->last_control_tick_ms, CONTROL_PERIOD_MS)) {
            uint32_t periods = (uint32_t)(now - runtime->last_control_tick_ms) / CONTROL_PERIOD_MS;
            uint32_t lateness_us = (periods > 1U ? periods - 1U : 0U) * CONTROL_PERIOD_MS * 1000U;
            runtime->last_scheduler_lateness_us = lateness_us;
            if (lateness_us > runtime->scheduler_lateness_max_us) runtime->scheduler_lateness_max_us = lateness_us;
            runtime->last_control_tick_ms += periods * CONTROL_PERIOD_MS;
            if (!update_motion_targets(runtime, now) || !send_control_tick(runtime)) {
                enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
            }
        }
        return;
    }

    if (runtime->state == CHASSIS_RUNTIME_STOPPING) {
        if (!runtime->disable_sent && elapsed(now, runtime->last_control_tick_ms, CONTROL_PERIOD_MS)) {
            runtime->last_control_tick_ms = now;
            clear_targets(runtime);
            if (!send_control_tick(runtime)) {
                enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
                return;
            }
        }
        if (!runtime->disable_sent && elapsed(now, runtime->transition_started_ms, ZERO_BEFORE_DISABLE_MS)) {
            if (!disable_all(runtime)) {
                if (!disable_all(runtime)) {
                    enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
                    return;
                }
            }
            runtime->disable_sent = true;
            runtime->transition_deadline_ms = now + DISABLE_CONFIRM_TIMEOUT_MS;
        }
        if (runtime->disable_sent) {
            if (all_feedback_state(runtime, now, DM_ERR_DISABLED)) {
                runtime->state = CHASSIS_RUNTIME_DISABLED;
            } else if (!runtime->disable_confirmation_retry_sent &&
                       elapsed(now, runtime->transition_deadline_ms - DISABLE_CONFIRM_TIMEOUT_MS,
                               DISABLE_CONFIRM_RETRY_MS)) {
                if (!disable_all(runtime)) {
                    enter_fault(runtime, CHASSIS_RUNTIME_FAULT_CAN_TX, now);
                    return;
                }
                runtime->disable_confirmation_retry_sent = true;
            } else if ((int32_t)(now - runtime->transition_deadline_ms) >= 0) {
                enter_fault(runtime, CHASSIS_RUNTIME_FAULT_DISABLE_TIMEOUT, now);
            }
        }
    }
}
