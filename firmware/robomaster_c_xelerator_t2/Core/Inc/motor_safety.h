#ifndef MOTOR_SAFETY_H
#define MOTOR_SAFETY_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Safety limits — must match Pi-side protocol.py
 *
 * Timeout design (Xelerator T2 first version, NOT Damiao official params):
 *   HEARTBEAT_INTERVAL_MS      = 100   (recommended Pi heartbeat period)
 *   REQUEST_RESPONSE_TIMEOUT_MS = 2000  (for PING/STATUS/ACK waits)
 *   MOTION_WATCHDOG_MS         = 300   (motion link loss protection)
 *
 * If no heartbeat or motion command is received within 300ms during
 * active motor control, immediately zero target, send 0 velocity,
 * send Disable, enter LINK_TIMEOUT state.
 */

#define SAFE_SPEED_MIN          (-10.0f)
#define SAFE_SPEED_MAX          (10.0f)
#define MAX_DURATION_MS         5000
#define MIN_DURATION_MS         100

#define HEARTBEAT_INTERVAL_MS       100
#define REQUEST_RESPONSE_TIMEOUT_MS 2000
#define MOTION_WATCHDOG_MS          300

/* Motor ID whitelist */
#define MOTOR_ID_MIN    1
#define MOTOR_ID_MAX    4   /* Formal chassis ESC IDs are 1..4 */

/* Motor state */
typedef struct {
    uint8_t  motor_id;
    bool     enabled;
    bool     online;
    bool     fault;
    bool     link_timeout;
    float    actual_speed;
    float    target_speed;
    float    temperature_c;
    float    bus_voltage;
    uint16_t error_code;
    uint32_t last_command_tick;   /* HAL_GetTick() */
    uint32_t command_expire_tick; /* when target_speed should auto-zero */
    uint32_t last_heartbeat_tick;
} motor_state_t;

/* Initialize motor state */
void motor_safety_init(motor_state_t *m, uint8_t motor_id);

/* Check if motor ID is valid */
bool motor_safety_valid_id(uint8_t motor_id);

/* Enable motor */
bool motor_safety_enable(motor_state_t *m);

/* Disable motor (also zeroes target) */
bool motor_safety_disable(motor_state_t *m);

/* Set speed (clamped, only if enabled) */
typedef enum {
    SPEED_OK = 0,
    SPEED_NOT_ENABLED = 1,
    SPEED_BAD_ID = 2,
    SPEED_OUT_OF_RANGE = 3,
} speed_result_t;

speed_result_t motor_safety_set_speed(motor_state_t *m, float target, uint32_t duration_ms);

/* Stop motor */
void motor_safety_stop(motor_state_t *m);

/* Watchdog check — call in main loop; auto-zeros if timeout */
void motor_safety_watchdog(motor_state_t *m, uint32_t current_tick);

/* Clamp speed to safe range */
float motor_safety_clamp_speed(float speed);

#endif /* MOTOR_SAFETY_H */
