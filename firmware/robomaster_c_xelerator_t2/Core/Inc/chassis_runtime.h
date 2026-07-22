#ifndef CHASSIS_RUNTIME_H
#define CHASSIS_RUNTIME_H

#include "dm2325_can.h"
#include "chassis_kinematics.h"

#include <stdbool.h>
#include <stdint.h>

#define CHASSIS_RUNTIME_WHEEL_COUNT 4U
#define CHASSIS_REVERSE_EVENT_CAPACITY 16U

typedef enum {
    CHASSIS_PROFILE_SAFE = 0,
    CHASSIS_PROFILE_NORMAL = 1,
    CHASSIS_PROFILE_COMPETITION_CANDIDATE = 2
} chassis_control_profile_t;

typedef enum {
    CHASSIS_REVERSE_IDLE = 0,
    CHASSIS_REVERSE_BRAKING = 1,
    CHASSIS_REVERSE_ZERO_CROSSED = 2,
    CHASSIS_REVERSE_ACCELERATING = 3,
    CHASSIS_REVERSE_COMPLETE = 4
} chassis_reverse_state_t;

typedef enum {
    CHASSIS_LIMITER_NORMAL_ACCEL = 0,
    CHASSIS_LIMITER_NORMAL_DECEL = 1,
    CHASSIS_LIMITER_REVERSE_BRAKE = 2,
    CHASSIS_LIMITER_SPEED_SATURATION = 3
} chassis_limiter_state_t;

typedef struct {
    float normal_accel_rad_s2;
    float normal_decel_rad_s2;
    float reverse_brake_rad_s2;
    float normal_jerk_rad_s3;
    float reverse_jerk_rad_s3;
    float zero_speed_epsilon_rad_s;
    float reverse_detection_epsilon_rad_s;
    float velocity_deadband_rad_s;
    float torque_limit_nm;
    float current_limit_a;
} chassis_profile_limits_t;

typedef struct {
    uint32_t event_id;
    uint32_t command_received_ms;
    uint32_t first_brake_command_ms;
    uint32_t zero_cross_ms;
    uint32_t reverse_velocity_reached_ms;
    float old_target_velocity_rad_s;
    float new_target_velocity_rad_s;
    float measured_velocity_at_command_rad_s;
    float peak_measured_torque_nm;
    uint8_t motor_id;
    uint8_t profile;
    uint8_t result;
    uint8_t fault_code;
} chassis_reverse_event_t;

typedef enum {
    CHASSIS_RUNTIME_BOOT = 0,
    CHASSIS_RUNTIME_DISABLED = 1,
    CHASSIS_RUNTIME_ARMING = 2,
    CHASSIS_RUNTIME_READY = 3,
    CHASSIS_RUNTIME_RUNNING = 4,
    CHASSIS_RUNTIME_STOPPING = 5,
    CHASSIS_RUNTIME_FAULT = 6
} chassis_runtime_state_t;

typedef enum {
    CHASSIS_RUNTIME_FAULT_NONE = 0,
    CHASSIS_RUNTIME_FAULT_CONFIG = 1,
    CHASSIS_RUNTIME_FAULT_ARM_TIMEOUT = 2,
    CHASSIS_RUNTIME_FAULT_ARM_FEEDBACK = 3,
    CHASSIS_RUNTIME_FAULT_MOTOR_FEEDBACK = 4,
    CHASSIS_RUNTIME_FAULT_CAN_TX = 5,
    CHASSIS_RUNTIME_FAULT_HOST_WATCHDOG = 6,
    CHASSIS_RUNTIME_FAULT_DISABLE_TIMEOUT = 7,
    CHASSIS_RUNTIME_FAULT_USER_ESTOP = 8
} chassis_runtime_fault_t;

typedef struct {
    dm2325_motor_t *motors;
    chassis_kinematics_t *kinematics;
    chassis_runtime_state_t state;
    chassis_runtime_fault_t fault;
    chassis_runtime_fault_t secondary_fault;
    uint32_t secondary_fault_count;
    uint8_t arm_index;
    uint32_t transition_started_ms;
    uint32_t transition_deadline_ms;
    uint32_t awaited_feedback_sequence;
    uint32_t latest_command_ms;
    uint32_t last_control_tick_ms;
    uint32_t control_sequence;
    uint32_t control_tick_count;
    uint32_t can_tx_failure_count;
    uint32_t max_command_skew_us;
    uint32_t scheduler_lateness_max_us;
    uint32_t last_scheduler_lateness_us;
    bool disable_sent;
    bool disable_confirmation_retry_sent;
    float targets_rad_s[CHASSIS_RUNTIME_WHEEL_COUNT];
    float requested_targets_rad_s[CHASSIS_RUNTIME_WHEEL_COUNT];
    float target_acceleration_rad_s2[CHASSIS_RUNTIME_WHEEL_COUNT];
    chassis_reverse_state_t reverse_state[CHASSIS_RUNTIME_WHEEL_COUNT];
    chassis_limiter_state_t limiter_state[CHASSIS_RUNTIME_WHEEL_COUNT];
    uint8_t active_reverse_event[CHASSIS_RUNTIME_WHEEL_COUNT];
    chassis_twist_t requested_twist;
    chassis_twist_t limited_twist;
    float twist_acceleration[3];
    chassis_control_profile_t profile;
    chassis_profile_limits_t profile_limits;
    chassis_reverse_event_t reverse_events[CHASSIS_REVERSE_EVENT_CAPACITY];
    uint8_t reverse_event_head;
    uint8_t reverse_event_count;
    uint32_t reverse_event_sequence;
    uint8_t last_command_sequence;
    bool has_command_sequence;
} chassis_runtime_t;

bool chassis_runtime_init(chassis_runtime_t *runtime, dm2325_motor_t motors[4]);
bool chassis_runtime_request_arm(chassis_runtime_t *runtime, uint32_t now_ms);
void chassis_runtime_set_kinematics(chassis_runtime_t *runtime, chassis_kinematics_t *kinematics);
bool chassis_runtime_set_profile(chassis_runtime_t *runtime, chassis_control_profile_t profile);
const chassis_profile_limits_t *chassis_runtime_get_profile_limits(chassis_control_profile_t profile);
const chassis_reverse_event_t *chassis_runtime_latest_reverse_event(const chassis_runtime_t *runtime);
bool chassis_runtime_accept_twist(chassis_runtime_t *runtime, uint8_t sequence,
                                  const chassis_twist_t *twist, uint32_t now_ms);
bool chassis_runtime_accept_zero(chassis_runtime_t *runtime, uint32_t now_ms);
bool chassis_runtime_request_stop(chassis_runtime_t *runtime, uint32_t now_ms);
bool chassis_runtime_request_disable(chassis_runtime_t *runtime, uint32_t now_ms);
bool chassis_runtime_clear_fault(chassis_runtime_t *runtime, uint32_t now_ms);
void chassis_runtime_emergency_stop(chassis_runtime_t *runtime, chassis_runtime_fault_t fault, uint32_t now_ms);
void chassis_runtime_tick(chassis_runtime_t *runtime, uint32_t now_ms);

#endif
