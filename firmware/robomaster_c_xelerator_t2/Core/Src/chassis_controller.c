#include "chassis_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool elapsed_within(uint32_t now, uint32_t then, uint32_t limit) {
    return (uint32_t)(now - then) <= limit;
}

static bool feedback_healthy(
    const chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now,
    bool require_enabled) {
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (!feedback[i].assigned || !feedback[i].online || feedback[i].fault_code != 0U ||
            (require_enabled && !feedback[i].enabled) ||
            !elapsed_within(now, feedback[i].received_tick, controller->limits.feedback_timeout_ms)) return false;
    }
    return true;
}

static void enter_fault(chassis_controller_t *controller, chassis_fault_t fault) {
    memset(&controller->latest_command, 0, sizeof(controller->latest_command));
    controller->has_command = false;
    controller->fault = fault;
    controller->state = CHASSIS_FAULT;
}

bool chassis_controller_init(
    chassis_controller_t *controller,
    chassis_kinematics_t *kinematics,
    const chassis_control_limits_t *limits) {
    if (controller == NULL || kinematics == NULL || limits == NULL) return false;
    memset(controller, 0, sizeof(*controller));
    controller->kinematics = kinematics;
    controller->limits = *limits;
    if (!kinematics->valid || kinematics->rank != 3U) {
        enter_fault(controller, CHASSIS_FAULT_INVALID_GEOMETRY);
        return false;
    }
    if (!isfinite(limits->max_vx_mps) || !isfinite(limits->max_vy_mps) ||
        !isfinite(limits->max_wz_rad_s) || limits->max_vx_mps <= 0.0f ||
        limits->max_vy_mps <= 0.0f || limits->max_wz_rad_s <= 0.0f ||
        limits->feedback_timeout_ms == 0U || limits->command_timeout_ms == 0U) return false;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (!isfinite(limits->max_motor_rad_s[i]) || limits->max_motor_rad_s[i] <= 0.0f) return false;
    }
    controller->state = CHASSIS_DISABLED;
    return true;
}

bool chassis_controller_arm(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now) {
    if (controller == NULL || feedback == NULL || controller->state != CHASSIS_DISABLED) return false;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) if (!feedback[i].assigned) return false;
    if (!feedback_healthy(controller, feedback, now, false)) {
        enter_fault(controller, CHASSIS_FAULT_ARM_FEEDBACK);
        return false;
    }
    controller->state = CHASSIS_ARMING;
    return true;
}

void chassis_controller_observe_feedback(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now) {
    if (controller == NULL || feedback == NULL) return;
    if (controller->state == CHASSIS_ARMING || controller->state == CHASSIS_READY || controller->state == CHASSIS_RUNNING) {
        if (!feedback_healthy(controller, feedback, now, true)) {
            enter_fault(controller, CHASSIS_FAULT_MOTOR_FEEDBACK);
        } else if (controller->state == CHASSIS_ARMING) {
            controller->state = CHASSIS_READY;
        }
    }
}

bool chassis_controller_accept_twist(
    chassis_controller_t *controller,
    uint8_t sequence,
    const chassis_twist_t *twist,
    uint32_t now) {
    if (controller == NULL || twist == NULL ||
        (controller->state != CHASSIS_READY && controller->state != CHASSIS_RUNNING)) return false;
    if (controller->has_sequence) {
        const uint8_t distance = (uint8_t)(sequence - controller->last_sequence);
        if (distance == 0U || distance > 127U) return false;
    }
    if (!isfinite(twist->vx_mps) || !isfinite(twist->vy_mps) || !isfinite(twist->wz_rad_s)) {
        enter_fault(controller, CHASSIS_FAULT_NONFINITE_COMMAND);
        return false;
    }
    chassis_twist_t limits = {
        controller->limits.max_vx_mps,
        controller->limits.max_vy_mps,
        controller->limits.max_wz_rad_s
    };
    if (!chassis_limit_twist(twist, &limits, &controller->latest_command)) return false;
    controller->latest_command_tick = now;
    controller->last_sequence = sequence;
    controller->has_command = true;
    controller->has_sequence = true;
    controller->state = (controller->latest_command.vx_mps != 0.0f ||
        controller->latest_command.vy_mps != 0.0f || controller->latest_command.wz_rad_s != 0.0f)
        ? CHASSIS_RUNNING : CHASSIS_READY;
    return true;
}

bool chassis_controller_update(
    chassis_controller_t *controller,
    uint32_t now,
    wheel_targets_t *targets) {
    if (controller == NULL || targets == NULL) return false;
    if (controller->state == CHASSIS_RUNNING &&
        (!controller->has_command || !elapsed_within(now, controller->latest_command_tick, controller->limits.command_timeout_ms))) {
        memset(&controller->latest_command, 0, sizeof(controller->latest_command));
        controller->has_command = false;
        controller->fault = CHASSIS_FAULT_HOST_WATCHDOG;
        controller->state = CHASSIS_STOPPING;
    }
    chassis_twist_t command = {0.0f, 0.0f, 0.0f};
    if (controller->state == CHASSIS_READY || controller->state == CHASSIS_RUNNING) command = controller->latest_command;
    return chassis_inverse_kinematics(controller->kinematics, &command, controller->limits.max_motor_rad_s, targets);
}

void chassis_controller_stop(chassis_controller_t *controller) {
    if (controller == NULL) return;
    memset(&controller->latest_command, 0, sizeof(controller->latest_command));
    controller->has_command = false;
    if (controller->state != CHASSIS_DISABLED && controller->state != CHASSIS_FAULT) controller->state = CHASSIS_STOPPING;
}

bool chassis_controller_disabled_confirmed(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now) {
    if (controller == NULL || feedback == NULL || controller->state != CHASSIS_STOPPING) return false;
    for (uint8_t i = 0; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (!feedback[i].assigned || !feedback[i].online || feedback[i].enabled || feedback[i].fault_code != 0U ||
            !elapsed_within(now, feedback[i].received_tick, controller->limits.feedback_timeout_ms)) return false;
    }
    controller->state = CHASSIS_DISABLED;
    return true;
}
