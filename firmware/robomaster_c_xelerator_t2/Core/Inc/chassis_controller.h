#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include "chassis_kinematics.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CHASSIS_BOOT = 0,
    CHASSIS_DISABLED,
    CHASSIS_ARMING,
    CHASSIS_READY,
    CHASSIS_RUNNING,
    CHASSIS_STOPPING,
    CHASSIS_FAULT
} chassis_state_t;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INVALID_GEOMETRY,
    CHASSIS_FAULT_ARM_FEEDBACK,
    CHASSIS_FAULT_MOTOR_FEEDBACK,
    CHASSIS_FAULT_NONFINITE_COMMAND,
    CHASSIS_FAULT_HOST_WATCHDOG
} chassis_fault_t;

typedef struct {
    bool assigned;
    bool online;
    bool enabled;
    uint16_t fault_code;
    uint32_t received_tick;
} chassis_motor_feedback_t;

typedef struct {
    float max_vx_mps;
    float max_vy_mps;
    float max_wz_rad_s;
    float max_motor_rad_s[CHASSIS_WHEEL_COUNT];
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
} chassis_control_limits_t;

typedef struct {
    chassis_kinematics_t *kinematics;
    chassis_control_limits_t limits;
    chassis_state_t state;
    chassis_fault_t fault;
    chassis_twist_t latest_command;
    uint32_t latest_command_tick;
    uint8_t last_sequence;
    bool has_command;
    bool has_sequence;
} chassis_controller_t;

bool chassis_controller_init(
    chassis_controller_t *controller,
    chassis_kinematics_t *kinematics,
    const chassis_control_limits_t *limits);
bool chassis_controller_arm(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now);
void chassis_controller_observe_feedback(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now);
bool chassis_controller_accept_twist(
    chassis_controller_t *controller,
    uint8_t sequence,
    const chassis_twist_t *twist,
    uint32_t now);
bool chassis_controller_update(
    chassis_controller_t *controller,
    uint32_t now,
    wheel_targets_t *targets);
void chassis_controller_stop(chassis_controller_t *controller);
bool chassis_controller_disabled_confirmed(
    chassis_controller_t *controller,
    const chassis_motor_feedback_t feedback[CHASSIS_WHEEL_COUNT],
    uint32_t now);

#endif
