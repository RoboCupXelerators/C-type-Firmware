#include "motor_safety.h"

void motor_safety_init(motor_state_t *m, uint8_t motor_id) {
    m->motor_id = motor_id;
    m->enabled = false;
    m->online = false;
    m->fault = false;
    m->link_timeout = false;
    m->actual_speed = 0.0f;
    m->target_speed = 0.0f;
    m->temperature_c = 0.0f;
    m->bus_voltage = 0.0f;
    m->error_code = 0;
    m->last_command_tick = 0;
    m->command_expire_tick = 0;
    m->last_heartbeat_tick = 0;
}

bool motor_safety_valid_id(uint8_t motor_id) {
    return motor_id >= MOTOR_ID_MIN && motor_id <= MOTOR_ID_MAX;
}

bool motor_safety_enable(motor_state_t *m) {
    if (!motor_safety_valid_id(m->motor_id)) return false;
    m->enabled = true;
    m->fault = false;
    m->link_timeout = false;
    return true;
}

bool motor_safety_disable(motor_state_t *m) {
    m->enabled = false;
    m->target_speed = 0.0f;
    m->link_timeout = false;
    return true;
}

float motor_safety_clamp_speed(float speed) {
    if (speed > SAFE_SPEED_MAX) return SAFE_SPEED_MAX;
    if (speed < SAFE_SPEED_MIN) return SAFE_SPEED_MIN;
    return speed;
}

speed_result_t motor_safety_set_speed(motor_state_t *m, float target, uint32_t duration_ms) {
    if (!motor_safety_valid_id(m->motor_id)) return SPEED_BAD_ID;
    if (!m->enabled) return SPEED_NOT_ENABLED;

    m->target_speed = motor_safety_clamp_speed(target);

    if (duration_ms > MAX_DURATION_MS) duration_ms = MAX_DURATION_MS;
    if (duration_ms < MIN_DURATION_MS) duration_ms = MIN_DURATION_MS;

    m->link_timeout = false;
    return SPEED_OK;
}

void motor_safety_stop(motor_state_t *m) {
    m->target_speed = 0.0f;
}

void motor_safety_watchdog(motor_state_t *m, uint32_t current_tick) {
    /* Check command duration expiry */
    if (m->command_expire_tick > 0 && current_tick >= m->command_expire_tick) {
        m->target_speed = 0.0f;
        m->command_expire_tick = 0;
    }

    /* Motion watchdog: 300ms without heartbeat or motion command
     * -> immediately zero target, mark link_timeout
     * Caller (main_loop) is responsible for sending 0 velocity + Disable */
    if (m->enabled && m->last_command_tick > 0 &&
        (current_tick - m->last_command_tick) > MOTION_WATCHDOG_MS) {
        m->target_speed = 0.0f;
        m->actual_speed = 0.0f;
        m->link_timeout = true;
    }
}
