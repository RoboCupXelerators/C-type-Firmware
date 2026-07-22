/*
 * main_loop.c — C board main loop for Xelerator T2 DM2325 motor control.
 *
 * Architecture:
 * 1. USB CDC receive callback enqueues data into a ring buffer
 * 2. Main loop parses Pi-C protocol frames
 * 3. Frames dispatch through safety state machine and DM2325 CAN driver
 * 4. Responses (ACK/NACK/STATUS) sent back via USB CDC
 * 5. Watchdog runs in main loop; motor auto-zeros on 300ms timeout
 * 6. DM2325 feedback from CAN RX is reported in STATUS messages
 *
 * Integration: Uses real STM32 HAL_CAN_* functions and USB CDC.
 */

#include "main.h"
#include "can.h"
#include "pi_c_protocol.h"
#include "motor_safety.h"
#include "dm2325_can.h"
#include "dm_can_discovery.h"
#include "usbd_cdc_if.h"
#include "cboard_diag.h"
#include "chassis_runtime.h"
#include "chassis_motor_config_generated.h"
#include "cboard_imu.h"
#include <math.h>
#include <string.h>

/* --- Ring buffer for USB CDC RX --- */

#define RX_BUF_SIZE 512
static uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* USB CDC receive callback — called from interrupt context.
 * Enqueues data, does NOT parse. */
void cboard_usb_rx_callback(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next == rx_tail) {
            g_cboard_diag.usb_rx_ring_overflow_count++;
            g_cboard_diag.usb_rx_dropped_bytes += len - i;
            cboard_diag_event(CBOARD_DIAG_RX_OVERFLOW, 0, len - i, rx_head);
            break;
        }
        rx_buf[rx_head] = data[i];
        rx_head = next;
    }
    g_cboard_diag.usb_rx_ring_depth = (rx_head + RX_BUF_SIZE - rx_tail) % RX_BUF_SIZE;
    cboard_diag_observe_max(&g_cboard_diag.usb_rx_ring_high_water, g_cboard_diag.usb_rx_ring_depth);
}

void cboard_diag_usb_rx(uint16_t len) {
    uint32_t now = cboard_diag_time_us();
    if (g_cboard_diag.usb_rx_last_us) {
        cboard_diag_observe_max(&g_cboard_diag.usb_rx_max_gap_us, now - g_cboard_diag.usb_rx_last_us);
    }
    g_cboard_diag.usb_rx_callback_count++;
    g_cboard_diag.usb_rx_packet_count++;
    g_cboard_diag.usb_rx_bytes += len;
    g_cboard_diag.usb_rx_last_us = now;
    cboard_diag_event(CBOARD_DIAG_USB_RX_ENTER, len, 0, 0);
}

void cboard_diag_usb_rearm(void) {
    g_cboard_diag.usb_rx_rearm_count++;
    cboard_diag_event(CBOARD_DIAG_USB_RX_REARM, 0, 0, 0);
}

static int rx_buf_pop(uint8_t *byte) {
    if (rx_head == rx_tail) return 0;
    *byte = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

/* --- Motor and parser state --- */

#define CHASSIS_MOTOR_COUNT 4U
static dm2325_motor_t motors[CHASSIS_MOTOR_COUNT];
/* Legacy single-motor diagnostics remain bound to FL. */
#define motor motors[0]
static dm2325_motor_t *engineering_motor = &motors[0];
static motor_state_t  safety_state;
static chassis_runtime_t chassis_runtime;
static chassis_kinematics_t chassis_kinematics;
static pi_c_parser_t  parser;
static uint8_t tx_buf[PI_C_MAX_FRAME_SIZE];

/* CAN diagnostic counters (section 39 requirement) */
static uint32_t can_tx_count = 0;
static uint32_t can_tx_fail_count = 0;
static uint32_t can_rx_count = 0;
static uint32_t can_rx_error_count = 0;
static uint32_t motor_id_mismatch_count = 0;

typedef struct {
    volatile bool active;
    volatile bool ready;
    uint8_t expected_opcode;
    uint8_t expected_rid;
    uint16_t expected_esc_id;
    uint16_t expected_master_id;
    uint16_t reply_can_id;
    uint8_t data[8];
} dm_param_capture_t;

static dm_param_capture_t dm_param_capture;

static dm2325_motor_t *motor_by_esc_id(uint8_t esc_id) {
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        if (motors[index].slave_id == esc_id) return &motors[index];
    }
    return NULL;
}

static bool all_motors_disabled(void) {
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        if (motors[index].enabled || motors[index].feedback.error_code != DM_ERR_DISABLED) return false;
    }
    return true;
}

#define DM2325_SLAVE_ID_DEFAULT  0x01
#define DM2325_MASTER_ID_DEFAULT 0x11

/* --- HAL tick and USB TX wrappers --- */
extern uint32_t cboard_hal_get_tick(void);
extern void cboard_usb_cdc_tx(const uint8_t *data, uint16_t len);
extern void cboard_usb_cdc_tx_poll(void);

/* --- Real HAL CAN integration --- */
extern CAN_HandleTypeDef hcan1;

int hal_can_init(uint32_t baudrate) {
    (void)baudrate;  /* CAN already initialized by MX_CAN1_Init in main.c */
    return 0;
}

int hal_can_send(uint32_t can_id, const uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status;
    int ok = 0;
    uint32_t started_us = cboard_diag_time_us();

    g_cboard_diag.can_tx_attempt_count++;
    cboard_diag_event(CBOARD_DIAG_CAN_TX_BEGIN, len, can_id, 0);

    tx_header.StdId = can_id & 0x7FF;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        g_cboard_diag.can_tx_no_mailbox_count++;
        cboard_diag_event(CBOARD_DIAG_CAN_MAILBOX_FULL, 1, can_id, 0);
        uint32_t mailbox_deadline_us = cboard_diag_time_us() + 2000U;
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U &&
               (int32_t)(cboard_diag_time_us() - mailbox_deadline_us) < 0) {
        }
    }
    status = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U
        ? HAL_TIMEOUT
        : HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox);
    if (status == HAL_OK) {
        can_tx_count++;
        g_cboard_diag.can_tx_success_count++;
        ok = 1;
    }

    if (ok) {
        cboard_diag_observe_max(&g_cboard_diag.can_tx_duration_max_us,
                                 cboard_diag_time_us() - started_us);
        cboard_diag_event(CBOARD_DIAG_CAN_TX_END, 1, can_id, cboard_diag_time_us() - started_us);
        return 0;
    }
    can_tx_fail_count++;
    g_cboard_diag.can_tx_fail_count++;
    cboard_diag_observe_max(&g_cboard_diag.can_tx_duration_max_us,
                             cboard_diag_time_us() - started_us);
    cboard_diag_event(CBOARD_DIAG_CAN_TX_END, 0, can_id, cboard_diag_time_us() - started_us);
    return -1;
}

uint32_t hal_get_tick(void) {
    return cboard_hal_get_tick();
}

/* --- CAN RX callback — called from HAL CAN RX interrupt --- */
void cboard_can_rx_callback(uint32_t can_id, const uint8_t *data, uint8_t len) {
    /* ONLY accept feedback from MST_ID (motor feedback CAN ID).
     * Do NOT accept slave_id -- that matches our own TX frames (CAN1 TX echo
     * received by CAN2 RX on shared bus), which would corrupt feedback data. */
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        dm2325_motor_t *candidate = &motors[index];
        if (can_id != candidate->master_id) continue;
        /* Parameter replies share MST_ID. Identify them by the echoed full
         * target ID as well as opcode; D[2] alone is a valid position byte. */
        dm_discovery_state_t discovery_state = DmCanDiscovery_GetState();
        bool direct_parameter_reply = (dm_param_capture.active || dm_param_capture.ready) &&
            can_id == dm_param_capture.expected_master_id && len == 8U &&
            data[0] == (uint8_t)(dm_param_capture.expected_esc_id & 0xFFU) &&
            data[1] == (uint8_t)(dm_param_capture.expected_esc_id >> 8) &&
            data[2] == dm_param_capture.expected_opcode && data[3] == dm_param_capture.expected_rid;
        bool discovery_parameter_reply =
            (discovery_state == DISCOVERY_SCANNING || discovery_state == DISCOVERY_READING_PARAMS) &&
            len == 8U && data[0] == candidate->slave_id && data[1] == 0U &&
            (data[2] == 0x33U || data[2] == 0x55U || data[2] == 0xAAU);
        bool parameter_reply = direct_parameter_reply || discovery_parameter_reply;
        if (!parameter_reply && len == 8U &&
            (data[0] & 0x0FU) == (candidate->slave_id & 0x0FU)) {
            candidate->feedback.last_raw_can_id = (uint16_t)can_id;
            dm2325_can_decode_feedback(candidate, data, len);
        } else if (!parameter_reply) {
            motor_id_mismatch_count++;
            if (candidate->feedback.invalid_count != UINT16_MAX) candidate->feedback.invalid_count++;
        }
        return;
    }
}

/* HAL CAN RX0 interrupt callback (called by stm32f4xx_it.c) */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        uint32_t can_id = rx_header.IDE == CAN_ID_STD ?
                          rx_header.StdId : rx_header.ExtId;
        if (dm_param_capture.active && rx_header.DLC == 8U &&
            can_id == dm_param_capture.expected_master_id &&
            rx_data[0] == (uint8_t)(dm_param_capture.expected_esc_id & 0xFFU) &&
            rx_data[1] == (uint8_t)(dm_param_capture.expected_esc_id >> 8) &&
            rx_data[2] == dm_param_capture.expected_opcode &&
            rx_data[3] == dm_param_capture.expected_rid) {
            dm_param_capture.reply_can_id = (uint16_t)can_id;
            memcpy(dm_param_capture.data, rx_data, sizeof(dm_param_capture.data));
            dm_param_capture.ready = true;
            dm_param_capture.active = false;
        }
        cboard_can_rx_callback(can_id, rx_data, rx_header.DLC);
        DmCanDiscovery_OnRx(can_id, rx_data, rx_header.DLC);
        can_rx_count++;
    }
}

/* --- Build STATUS response --- */
static void send_status(uint8_t seq) {
    uint8_t buf[41];
    uint8_t flags = (safety_state.enabled ? 1 : 0) |
                    (motor.feedback.online ? 2 : 0) |
                    (motor.feedback.fault ? 4 : 0);

    buf[0] = motor.slave_id;
    buf[1] = flags;
    float speed = motor.feedback.velocity;
    memcpy(buf + 2, &speed, 4);
    float temp = (float)motor.feedback.temp_mos;
    memcpy(buf + 6, &temp, 4);
    float volt = 0.0f;
    memcpy(buf + 10, &volt, 4);
    uint16_t err = motor.feedback.error_code;
    buf[14] = (uint8_t)(err & 0xFF);
    buf[15] = (uint8_t)(err >> 8);
    buf[16] = PI_C_PROTOCOL_VERSION;
    /* Added position and torque for rotation verification */
    float position = motor.feedback.position;
    memcpy(buf + 17, &position, 4);
    float torque = motor.feedback.torque;
    memcpy(buf + 21, &torque, 4);
    memcpy(buf + 25, motor.feedback.raw_can_data, 8);
    memcpy(buf + 33, &motor.feedback.feedback_sequence, 4);
    memcpy(buf + 37, &motor.feedback.last_feedback_tick, 4);

    int total = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_STATUS, seq, buf, 41);
    if (total > 0) {
        g_cboard_diag.cdc_tx_status_count++;
        cboard_diag_event(CBOARD_DIAG_STATUS_TX_ENQUEUE, seq, total, 0);
        cboard_usb_cdc_tx(tx_buf, total);
    }
}

static void send_chassis_status(uint8_t sequence) {
    uint8_t payload[248];
    memset(payload, 0, sizeof(payload));
    uint32_t now = cboard_hal_get_tick();
    uint8_t online_mask = 0U, enabled_mask = 0U, fault_mask = 0U;
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        dm2325_motor_t *item = &motors[index];
        bool online = item->feedback.feedback_sequence != 0U &&
                      (uint32_t)(now - item->feedback.last_feedback_tick) <= 100U;
        if (!online && item->feedback.feedback_sequence != 0U && !item->feedback.stale_latched) {
            if (item->feedback.stale_count != UINT16_MAX) item->feedback.stale_count++;
            item->feedback.stale_latched = true;
        }
        if (online) online_mask |= (uint8_t)(1U << index);
        if (item->enabled) enabled_mask |= (uint8_t)(1U << index);
        if (item->feedback.fault) fault_mask |= (uint8_t)(1U << index);
    }
    payload[0] = (uint8_t)chassis_runtime.state;
    payload[1] = online_mask;
    payload[2] = enabled_mask;
    payload[3] = fault_mask;
    memcpy(payload + 4, &now, 4);
    uint32_t esr = hcan1.Instance->ESR;
    memcpy(payload + 8, &esr, 4);
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        dm2325_motor_t *item = &motors[index];
        uint8_t *record = payload + 12U + (uint16_t)index * 52U;
        uint16_t esc_id = item->slave_id, master_id = item->master_id;
        memcpy(record, &esc_id, 2);
        memcpy(record + 2, &master_id, 2);
        record[4] = (item->enabled ? 1U : 0U) |
                    (item->feedback.online ? 2U : 0U) |
                    (item->feedback.fault ? 4U : 0U);
        record[5] = item->feedback.error_code;
        record[6] = item->feedback.temp_mos;
        record[7] = item->feedback.temp_rotor;
        memcpy(record + 8, &item->feedback.position, 4);
        memcpy(record + 12, &item->feedback.velocity, 4);
        memcpy(record + 16, &item->feedback.torque, 4);
        memcpy(record + 20, &item->feedback.position_raw, 2);
        memcpy(record + 22, &item->feedback.velocity_raw, 2);
        memcpy(record + 24, &item->feedback.torque_raw, 2);
        memcpy(record + 28, &item->feedback.feedback_sequence, 4);
        memcpy(record + 32, &item->feedback.last_feedback_tick, 4);
        uint16_t feedback_age_ms = item->feedback.feedback_sequence == 0U
            ? UINT16_MAX
            : (uint16_t)(((uint32_t)(now - item->feedback.last_feedback_tick) > UINT16_MAX)
                ? UINT16_MAX : (uint32_t)(now - item->feedback.last_feedback_tick));
        memcpy(record + 36, &item->feedback.position_unwrapped, 4);
        memcpy(record + 40, &feedback_age_ms, 2);
        memcpy(record + 42, &item->feedback.rx_count, 4);
        memcpy(record + 46, &item->feedback.invalid_count, 2);
        memcpy(record + 48, &item->feedback.stale_count, 2);
        memcpy(record + 50, &item->feedback.last_raw_can_id, 2);
    }
    memcpy(payload + 220, &chassis_runtime.control_sequence, 4);
    memcpy(payload + 224, &chassis_runtime.control_tick_count, 4);
    memcpy(payload + 228, &chassis_runtime.can_tx_failure_count, 4);
    memcpy(payload + 232, &chassis_runtime.max_command_skew_us, 4);
    memcpy(payload + 236, &chassis_runtime.scheduler_lateness_max_us, 4);
    uint32_t runtime_fault = (uint32_t)chassis_runtime.fault;
    memcpy(payload + 240, &runtime_fault, 4);
    memcpy(payload + 244, &motor_id_mismatch_count, 4);
    int length = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_CHASSIS_STATUS,
                                  sequence, payload, sizeof(payload));
    if (length > 0) cboard_usb_cdc_tx(tx_buf, length);
}

static void send_chassis_diagnostics(uint8_t sequence) {
    uint8_t payload[PI_C_CHASSIS_DIAGNOSTICS_PAYLOAD_LEN];
    memset(payload, 0, sizeof(payload));
    uint32_t now = cboard_hal_get_tick();
    payload[0] = 1U;
    payload[1] = (uint8_t)chassis_runtime.profile;
    payload[2] = DM_CTRL_MODE_MIT;
    for (uint8_t wheel = 0U; wheel < CHASSIS_MOTOR_COUNT; ++wheel) {
        if (chassis_runtime.reverse_state[wheel] == CHASSIS_REVERSE_BRAKING ||
            chassis_runtime.reverse_state[wheel] == CHASSIS_REVERSE_ZERO_CROSSED) {
            payload[3] |= (uint8_t)(1U << wheel);
        }
    }
    memcpy(payload + 4, &now, 4);
    uint32_t command_age = now - chassis_runtime.latest_command_ms;
    uint16_t command_age_ms = command_age > UINT16_MAX ? UINT16_MAX : (uint16_t)command_age;
    uint16_t control_period_ms = 10U;
    memcpy(payload + 8, &command_age_ms, 2);
    memcpy(payload + 10, &control_period_ms, 2);
    const float profile_values[8] = {
        chassis_runtime.profile_limits.normal_accel_rad_s2,
        chassis_runtime.profile_limits.normal_decel_rad_s2,
        chassis_runtime.profile_limits.reverse_brake_rad_s2,
        chassis_runtime.profile_limits.normal_jerk_rad_s3,
        chassis_runtime.profile_limits.reverse_jerk_rad_s3,
        chassis_runtime.profile_limits.zero_speed_epsilon_rad_s,
        chassis_runtime.profile_limits.reverse_detection_epsilon_rad_s,
        chassis_runtime.profile_limits.velocity_deadband_rad_s
    };
    memcpy(payload + 12, profile_values, sizeof(profile_values));
    for (uint8_t wheel = 0U; wheel < CHASSIS_MOTOR_COUNT; ++wheel) {
        dm2325_motor_t *motor_item = &motors[wheel];
        uint8_t *record = payload + 44U + (uint16_t)wheel * 28U;
        record[0] = motor_item->slave_id;
        record[1] = (uint8_t)chassis_runtime.reverse_state[wheel];
        record[2] = (uint8_t)chassis_runtime.limiter_state[wheel];
        record[3] = 0x03U; /* measured velocity and torque are present in this record */
        memcpy(record + 4, &chassis_runtime.targets_rad_s[wheel], 4);
        memcpy(record + 8, &chassis_runtime.requested_targets_rad_s[wheel], 4);
        memcpy(record + 12, &motor_item->feedback.velocity, 4);
        float velocity_error = chassis_runtime.targets_rad_s[wheel] - motor_item->feedback.velocity;
        memcpy(record + 16, &velocity_error, 4);
        memcpy(record + 20, &motor_item->feedback.torque, 4);
        uint32_t age = now - motor_item->feedback.last_feedback_tick;
        uint16_t feedback_age_ms = age > UINT16_MAX ? UINT16_MAX : (uint16_t)age;
        memcpy(record + 24, &feedback_age_ms, 2);
        record[26] = motor_item->feedback.error_code;
        record[27] = motor_item->enabled ? 1U : 0U;
    }
    const chassis_reverse_event_t *event = chassis_runtime_latest_reverse_event(&chassis_runtime);
    if (event != NULL) {
        uint8_t *record = payload + 156U;
        memcpy(record, &event->event_id, 4);
        memcpy(record + 4, &event->command_received_ms, 4);
        memcpy(record + 8, &event->first_brake_command_ms, 4);
        memcpy(record + 12, &event->zero_cross_ms, 4);
        memcpy(record + 16, &event->reverse_velocity_reached_ms, 4);
        memcpy(record + 20, &event->old_target_velocity_rad_s, 4);
        memcpy(record + 24, &event->new_target_velocity_rad_s, 4);
        memcpy(record + 28, &event->measured_velocity_at_command_rad_s, 4);
        memcpy(record + 32, &event->peak_measured_torque_nm, 4);
        record[36] = event->motor_id;
        record[37] = event->profile;
        record[38] = event->result;
        record[39] = event->fault_code;
        uint32_t command_to_brake = event->first_brake_command_ms - event->command_received_ms;
        uint32_t brake_to_zero = event->zero_cross_ms == 0U ? UINT32_MAX :
            event->zero_cross_ms - event->first_brake_command_ms;
        uint32_t zero_to_reverse = event->reverse_velocity_reached_ms == 0U || event->zero_cross_ms == 0U
            ? UINT32_MAX : event->reverse_velocity_reached_ms - event->zero_cross_ms;
        memcpy(record + 40, &command_to_brake, 4);
        memcpy(record + 44, &brake_to_zero, 4);
        memcpy(record + 48, &zero_to_reverse, 4);
    }
    int length = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_CHASSIS_DIAGNOSTICS,
                                  sequence, payload, sizeof(payload));
    if (length > 0) cboard_usb_cdc_tx(tx_buf, length);
}

static void send_imu_status(uint8_t sequence) {
    cboard_imu_state_t state;
    uint8_t payload[PI_C_IMU_STATUS_PAYLOAD_LEN];
    (void)cboard_imu_get_snapshot(&state);
    int payload_length = pi_c_encode_imu_status(payload, sizeof(payload), &state);
    if (payload_length <= 0) return;
    int frame_length = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_IMU_STATUS,
                                        sequence, payload, (uint8_t)payload_length);
    if (frame_length > 0) cboard_usb_cdc_tx(tx_buf, frame_length);
}

static bool dm_param_exchange(
    uint16_t esc_id,
    uint8_t opcode,
    uint8_t rid,
    uint32_t value,
    uint8_t sequence) {
    uint8_t request[8] = {
        (uint8_t)(esc_id & 0xFFU), (uint8_t)(esc_id >> 8), opcode, rid,
        (uint8_t)(value & 0xFFU), (uint8_t)(value >> 8),
        (uint8_t)(value >> 16), (uint8_t)(value >> 24)
    };
    memset(&dm_param_capture, 0, sizeof(dm_param_capture));
    dm_param_capture.expected_opcode = opcode;
    dm_param_capture.expected_rid = rid;
    dm_param_capture.expected_esc_id = esc_id;
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        if (motors[index].slave_id == esc_id) {
            dm_param_capture.expected_master_id = motors[index].master_id;
            break;
        }
    }
    if (dm_param_capture.expected_master_id == 0U) return false;
    dm_param_capture.active = true;
    if (hal_can_send(DM_PARAM_CAN_ID, request, sizeof(request)) != 0) {
        dm_param_capture.active = false;
        return false;
    }
    uint32_t deadline = cboard_hal_get_tick() + 100U;
    while (!dm_param_capture.ready &&
           (int32_t)(cboard_hal_get_tick() - deadline) < 0) {
    }
    dm_param_capture.active = false;
    if (!dm_param_capture.ready) return false;

    uint8_t result[10];
    uint16_t response_target = (uint16_t)dm_param_capture.data[0] |
                               ((uint16_t)dm_param_capture.data[1] << 8);
    memcpy(result, &dm_param_capture.reply_can_id, 2);
    memcpy(result + 2, &response_target, 2);
    result[4] = dm_param_capture.data[2];
    result[5] = dm_param_capture.data[3];
    memcpy(result + 6, dm_param_capture.data + 4, 4);
    int length = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_DM_PARAM_RESULT,
                                  sequence, result, sizeof(result));
    if (length > 0) cboard_usb_cdc_tx(tx_buf, length);
    return length > 0;
}

/* --- Frame handler --- */
static void handle_frame(pi_c_frame_t *frame) {
    uint32_t tick = cboard_hal_get_tick();
    cboard_diag_event(CBOARD_DIAG_FRAME_DISPATCH, frame->msg_type, frame->seq, frame->payload_len);

    if (frame->version != PI_C_PROTOCOL_VERSION) {
        int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                  frame->msg_type, NACK_BAD_VERSION);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        return;
    }

    switch (frame->msg_type) {
    case MSG_PING: {
        int len = pi_c_build_pong(tx_buf, sizeof(tx_buf), frame->seq);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_GET_STATUS: {
        g_cboard_diag.status_request_count++;
        cboard_diag_event(CBOARD_DIAG_STATUS_REQUEST, frame->seq, 0, 0);
        send_status(frame->seq);
        break;
    }
    case MSG_GET_CHASSIS_STATUS: {
        send_chassis_status(frame->seq);
        break;
    }
    case MSG_GET_CHASSIS_DIAGNOSTICS: {
        bool accepted = frame->payload_len == 0U;
        if (accepted) {
            send_chassis_diagnostics(frame->seq);
        } else {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        }
        break;
    }
    case MSG_CHASSIS_SET_PROFILE: {
        bool accepted = frame->payload_len == 1U &&
            chassis_runtime_set_profile(&chassis_runtime,
                (chassis_control_profile_t)frame->payload[0]);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type,
                              frame->payload_len == 1U ? NACK_BUSY : NACK_BAD_PAYLOAD);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_GET_IMU_STATUS: {
        if (frame->payload_len != 0U) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        send_imu_status(frame->seq);
        break;
    }
    case MSG_IMU_ZERO_YAW: {
        bool accepted = frame->payload_len == 0U && cboard_imu_zero_yaw();
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type,
                              frame->payload_len == 0U ? NACK_BUSY : NACK_BAD_PAYLOAD);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_IMU_SET_REFERENCE: {
        float angle_degrees = 0.0f;
        if (frame->payload_len == 4U) memcpy(&angle_degrees, frame->payload, 4U);
        bool decoded = frame->payload_len == 4U && isfinite(angle_degrees);
        bool accepted = decoded && cboard_imu_set_yaw_reference(
            angle_degrees * 0.01745329251994329577f);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type,
                              decoded ? NACK_BUSY : NACK_BAD_PAYLOAD);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_ARM: {
        bool accepted = frame->payload_len == 0U &&
            chassis_runtime_request_arm(&chassis_runtime, tick);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type, NACK_BUSY);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_TWIST: {
        pi_c_chassis_twist_t command = {0};
        bool decoded = pi_c_decode_chassis_twist(frame->payload, frame->payload_len, &command);
        bool zero = decoded && command.vx_mps == 0.0f && command.vy_mps == 0.0f && command.wz_rad_s == 0.0f;
        chassis_twist_t twist = {command.vx_mps, command.vy_mps, command.wz_rad_s};
        bool allowed = zero || CHASSIS_NONZERO_HARDWARE_ALLOWED;
        bool accepted = allowed && decoded && chassis_runtime_accept_twist(
            &chassis_runtime, frame->seq, &twist, tick);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type,
                              decoded ? NACK_NOT_ENABLED : NACK_BAD_PAYLOAD);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_STOP: {
        bool accepted = chassis_runtime_request_stop(&chassis_runtime, tick);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type, NACK_BUSY);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_DISABLE: {
        bool accepted = chassis_runtime_request_disable(&chassis_runtime, tick);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type, NACK_BUSY);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_CLEAR_FAULT: {
        bool accepted = frame->payload_len == 0U &&
            chassis_runtime_clear_fault(&chassis_runtime, tick);
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type, NACK_BUSY);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_EMERGENCY_STOP: {
        bool accepted = frame->payload_len == 0U;
        if (accepted) {
            /* Latch a user E-stop in FAULT. Motion cannot resume until a
             * fresh four-wheel Disabled state is explicitly cleared. */
            chassis_runtime_emergency_stop(
                &chassis_runtime, CHASSIS_RUNTIME_FAULT_USER_ESTOP, tick);
        }
        int len = accepted
            ? pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type)
            : pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type,
                              NACK_BAD_PAYLOAD);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_CHASSIS_REFRESH_DISABLED: {
        bool success = !safety_state.enabled;
        for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT && success; ++index) {
            uint32_t before = motors[index].feedback.feedback_sequence;
            success = dm2325_can_mit_disable(&motors[index]) == DM2325_OK;
            uint32_t deadline = cboard_hal_get_tick() + 20U;
            while (success && motors[index].feedback.feedback_sequence == before &&
                   (int32_t)(cboard_hal_get_tick() - deadline) < 0) {
            }
            success = success && motors[index].feedback.feedback_sequence != before;
        }
        if (!success) {
            for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
                dm2325_can_mit_disable(&motors[index]);
            }
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_WATCHDOG_TIMEOUT);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        send_chassis_status(frame->seq);
        break;
    }
    case MSG_MOTOR_REFRESH_STATUS: {
        if (frame->payload_len != 1U || frame->payload[0] != motor.slave_id) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint32_t before = motor.feedback.feedback_sequence;
        dm2325_result_t result = dm2325_can_refresh_status(&motor);
        if (result != DM2325_OK) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_INTERNAL_ERROR);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint32_t deadline = tick + 20U;
        while (motor.feedback.feedback_sequence == before &&
               (int32_t)(cboard_hal_get_tick() - deadline) < 0) {
        }
        if (motor.feedback.feedback_sequence == before) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_WATCHDOG_TIMEOUT);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_DM_PARAM_READ: {
        if (frame->payload_len != 3U || safety_state.enabled || motor.enabled) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint16_t esc_id = (uint16_t)frame->payload[0] |
                          ((uint16_t)frame->payload[1] << 8);
        if (!dm_param_exchange(esc_id, 0x33U, frame->payload[2], 0U, frame->seq)) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_WATCHDOG_TIMEOUT);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        }
        break;
    }
    case MSG_DM_PARAM_WRITE: {
        if (frame->payload_len != 7U || safety_state.enabled || motor.enabled) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint16_t esc_id = (uint16_t)frame->payload[0] |
                          ((uint16_t)frame->payload[1] << 8);
        uint32_t value;
        memcpy(&value, frame->payload + 3, 4);
        if (!dm_param_exchange(esc_id, 0x55U, frame->payload[2], value, frame->seq)) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_WATCHDOG_TIMEOUT);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        }
        break;
    }
    case MSG_DM_PARAM_SAVE: {
        if (frame->payload_len != 2U || safety_state.enabled || motor.enabled) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint16_t esc_id = (uint16_t)frame->payload[0] |
                          ((uint16_t)frame->payload[1] << 8);
        if (!dm_param_exchange(esc_id, 0xAAU, 0x01U, 0U, frame->seq)) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_WATCHDOG_TIMEOUT);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        }
        break;
    }
    case MSG_MOTOR_ENABLE: {
        g_cboard_diag.enable_count++;
        if (frame->payload_len < 1) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint8_t mid = frame->payload[0];
        dm2325_motor_t *selected = motor_by_esc_id(mid);
        if (!motor_safety_valid_id(mid) || selected == NULL ||
            chassis_runtime.state != CHASSIS_RUNTIME_DISABLED || !all_motors_disabled()) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        safety_state.motor_id = mid;
        motor_safety_enable(&safety_state);
        /* Start the watchdog from this enable command rather than retaining
         * a timestamp from a prior run. */
        safety_state.last_command_tick = tick;
        engineering_motor = selected;
        dm2325_can_mit_enable(engineering_motor);

        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_MOTOR_DISABLE: {
        g_cboard_diag.disable_count++;
        if (frame->payload_len < 1) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint8_t mid = frame->payload[0];
        dm2325_motor_t *selected = motor_by_esc_id(mid);
        if (selected == NULL || selected != engineering_motor) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        motor_safety_disable(&safety_state);
        dm2325_can_mit_disable(selected);

        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_MOTOR_SET_SPEED: {
        if (frame->payload_len < 9) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint8_t mid = frame->payload[0];
        if (!motor_safety_valid_id(mid)) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        float target;
        uint32_t duration;
        memcpy(&target, frame->payload + 1, 4);
        memcpy(&duration, frame->payload + 5, 4);

        speed_result_t sr = motor_safety_set_speed(&safety_state, target, duration);
        if (sr != SPEED_OK) {
            uint8_t reason = (sr == SPEED_NOT_ENABLED) ? NACK_NOT_ENABLED :
                             (sr == SPEED_BAD_ID) ? NACK_BAD_MOTOR_ID :
                             NACK_OUT_OF_RANGE;
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, reason);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        safety_state.command_expire_tick = tick + duration;

        /* MIT mode: v_des=target, p_des=0, kp=0, kd=0, t_ff=0 */
        dm2325_result_t r = dm2325_can_mit_ctrl(&motor, 0.0f, safety_state.target_speed,
                                                 0.0f, 0.0f, 0.0f);
        if (r != DM2325_OK) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_INTERNAL_ERROR);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }

        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_MOTOR_STOP: {
        motor_safety_stop(&safety_state);
        /* MIT zero output then disable */
        dm2325_can_mit_ctrl(&motor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        dm2325_can_mit_disable(&motor);

        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_HEARTBEAT: {
        safety_state.last_command_tick = tick;
        safety_state.last_heartbeat_tick = tick;
        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_GET_CAN_STATUS: {
        uint8_t buf[29];
        uint8_t flags = 1; /* can_init_ok */
        if (hcan1.Instance->ESR & CAN_ESR_BOFF) flags |= 2;

        /* Read HAL CAN error and bus status registers */
        uint32_t esr = hcan1.Instance->ESR;
        uint32_t bsr = 0; /* BSR not directly available, use ESR fields */

        extern int hal_can_send(uint32_t, const uint8_t*, uint8_t);
        /* Pack: flags(B) + tx_count(I) + tx_fail(I) + rx_count(I) + rx_err(I) + esr(I) + bsr(I) + reserved(I) */
        memcpy(buf, &flags, 1);
        memcpy(buf + 1, &can_tx_count, 4);
        memcpy(buf + 5, &can_tx_fail_count, 4);
        memcpy(buf + 9, &can_rx_count, 4);
        memcpy(buf + 13, &can_rx_error_count, 4);
        memcpy(buf + 17, &esr, 4);
        memcpy(buf + 21, &bsr, 4);
        memset(buf + 25, 0, 4);

        int len = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_CAN_STATUS, frame->seq, buf, 29);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_GET_BOARD_INFO: {
        uint8_t buf[20];
        memset(buf, 0, sizeof(buf));
        uint8_t fw_ver = PI_C_PROTOCOL_VERSION;
        uint8_t baud_code = 4; /* 1 Mbps */
        uint8_t can1_used = 1;
        uint8_t can2_used = 0;
        extern uint32_t cboard_hal_get_tick(void);
        uint32_t uptime = cboard_hal_get_tick();
        uint16_t sid = motor.slave_id;
        uint16_t mid = motor.master_id;

        memcpy(buf, &fw_ver, 1);
        memcpy(buf + 1, &baud_code, 1);
        memcpy(buf + 2, &can1_used, 1);
        memcpy(buf + 3, &can2_used, 1);
        memcpy(buf + 4, &uptime, 4);
        memcpy(buf + 8, &sid, 2);
        memcpy(buf + 10, &mid, 2);

        int len = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_BOARD_INFO, frame->seq, buf, 20);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    case MSG_DISCOVERY_START: {
        if (frame->payload_len >= 4) {
            uint16_t id_start = (uint16_t)frame->payload[0] | ((uint16_t)frame->payload[1] << 8);
            uint16_t id_end = (uint16_t)frame->payload[2] | ((uint16_t)frame->payload[3] << 8);
            DmCanDiscovery_Init();
            DmCanDiscovery_Start(id_start, id_end);
        }
        int dl = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (dl > 0) cboard_usb_cdc_tx(tx_buf, dl);
        break;
    }
    case MSG_DISCOVERY_STOP: {
        DmCanDiscovery_Stop();
        int dl = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (dl > 0) cboard_usb_cdc_tx(tx_buf, dl);
        break;
    }
    case MSG_SET_CAN_CHANNEL: {
        if (frame->payload_len < 1) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint8_t ch = frame->payload[0];
        if (ch == 1U) {
            int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        } else {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_OUT_OF_RANGE);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        }
        break;
    }
    case MSG_MIT_CTRL: {
        g_cboard_diag.mit_command_count++;
        cboard_diag_event(CBOARD_DIAG_MIT_RECEIVED, frame->seq, 0, 0);
        if (frame->payload_len < 21) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_PAYLOAD);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        uint8_t mid = frame->payload[0];
        dm2325_motor_t *selected = motor_by_esc_id(mid);
        if (!motor_safety_valid_id(mid) || selected == NULL || selected != engineering_motor ||
            chassis_runtime.state != CHASSIS_RUNTIME_DISABLED || !safety_state.enabled) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        /* Only an accepted MIT control refreshes the motion watchdog.
         * STATUS/PING traffic must never keep an enabled motor alive. */
        safety_state.last_command_tick = tick;
        float p_des, v_des, kp, kd, t_ff;
        memcpy(&p_des, frame->payload + 1, 4);
        memcpy(&v_des, frame->payload + 5, 4);
        memcpy(&kp, frame->payload + 9, 4);
        memcpy(&kd, frame->payload + 13, 4);
        memcpy(&t_ff, frame->payload + 17, 4);

        /* Engineering direction-test envelope. Production motion uses
         * CHASSIS_TWIST and cannot reach this single-wheel path. */
        if (p_des != 0.0f || kp != 0.0f || kd != 1.0f || t_ff != 0.0f ||
            !isfinite(v_des) || fabsf(v_des) > 1.0f) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_OUT_OF_RANGE);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }

        if (CBOARD_DIAG_PARSE_ONLY) break;
        dm2325_result_t r = dm2325_can_mit_ctrl(selected, p_des, v_des, kp, kd, t_ff);
        if (r != DM2325_OK) {
            uint8_t reason = (r == DM2325_ERR_NOT_ENABLED) ? NACK_NOT_ENABLED : NACK_INTERNAL_ERROR;
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, reason);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        /* MIT controls are streamed at 5-10 ms and the Pi does not await
         * their ACK. Sending one CDC ACK per control frame can starve STATUS
         * replies; retain NACKs above for rejected controls. */
        break;
    }
    case MSG_MIT_STOP: {
        if (frame->payload_len != 1U || motor_by_esc_id(frame->payload[0]) != engineering_motor) {
            int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                      frame->msg_type, NACK_BAD_MOTOR_ID);
            if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
            break;
        }
        /* Send MIT zero output, then disable */
        dm2325_can_mit_ctrl(engineering_motor, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
        dm2325_can_mit_disable(engineering_motor);
        motor_safety_disable(&safety_state);
        int len = pi_c_build_ack(tx_buf, sizeof(tx_buf), frame->seq, frame->msg_type);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    default: {
        int len = pi_c_build_nack(tx_buf, sizeof(tx_buf), frame->seq,
                                  frame->msg_type, NACK_UNKNOWN_MSG);
        if (len > 0) cboard_usb_cdc_tx(tx_buf, len);
        break;
    }
    }
}

/* --- Main loop --- */
void cboard_main_loop(void) {
    motor_safety_init(&safety_state, DM2325_SLAVE_ID_DEFAULT);
    pi_c_parser_init(&parser);

    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        motors[index].slave_id = (uint8_t)CHASSIS_ESC_IDS[index];
        motors[index].master_id = (uint8_t)CHASSIS_MASTER_IDS[index];
        motors[index].pmax = DM_S2325_PMAX_DEFAULT;
        motors[index].vmax = DM_S2325_VMAX_DEFAULT;
        motors[index].tmax = DM_S2325_TMAX_DEFAULT;
        motors[index].enabled = false;
        motors[index].feedback.online = false;
        motors[index].feedback.fault = false;
        motors[index].feedback.error_code = 0U;
    }

    dm2325_result_t can_init = dm2325_can_init(0);
    if (can_init != DM2325_OK) {
        motor.feedback.online = false;
    }

    safety_state.enabled = false;
    chassis_runtime_init(&chassis_runtime, motors);
    wheel_geometry_t wheel_geometry[CHASSIS_MOTOR_COUNT];
    for (uint8_t index = 0; index < CHASSIS_MOTOR_COUNT; ++index) {
        wheel_geometry[index].x_m = CHASSIS_WHEEL_X_M[index];
        wheel_geometry[index].y_m = CHASSIS_WHEEL_Y_M[index];
        wheel_geometry[index].drive_angle_rad = CHASSIS_DRIVE_ANGLE_RAD[index];
        wheel_geometry[index].wheel_radius_m = CHASSIS_WHEEL_RADIUS_M[index];
        wheel_geometry[index].gear_ratio = CHASSIS_GEAR_RATIO[index];
        wheel_geometry[index].motor_sign = CHASSIS_MOTOR_SIGN[index];
    }
    if (chassis_kinematics_init(&chassis_kinematics, wheel_geometry, 10000.0f)) {
        chassis_runtime_set_kinematics(&chassis_runtime, &chassis_kinematics);
    } else {
        chassis_runtime_emergency_stop(&chassis_runtime, CHASSIS_RUNTIME_FAULT_CONFIG, cboard_hal_get_tick());
    }

    while (1) {
        uint32_t tick = cboard_hal_get_tick();
        uint32_t loop_start_us = cboard_diag_time_us();
        uint32_t frames_processed = 0;
        g_cboard_diag.main_loop_iterations++;

        /* High-rate local IMU service publishes a snapshot. USB handlers only
         * read that snapshot and never trigger a synchronous sensor sample. */
        cboard_imu_update();

        uint8_t byte;
        while (rx_buf_pop(&byte)) {
            pi_c_frame_t frame;
            if (pi_c_parser_feed(&parser, &byte, 1, &frame)) {
                g_cboard_diag.parser_frames_ok++;
                g_cboard_diag.parser_last_frame_us = cboard_diag_time_us();
                frames_processed++;
                cboard_diag_event(CBOARD_DIAG_FRAME_PARSED, frame.msg_type, frame.seq, frame.payload_len);
                handle_frame(&frame);
                /* A continuous MIT stream can keep rx_buf non-empty. Service
                 * queued CDC replies per complete request so STATUS cannot be
                 * starved until the host pauses its control writes. */
                cboard_usb_cdc_tx_poll();
            }
        }

        /* Frame handling can cross a millisecond boundary. Refresh the tick
         * after it updates last_command_tick so subtraction never underflows. */
        tick = cboard_hal_get_tick();
        chassis_runtime_tick(&chassis_runtime, tick);
        bool watchdog_was_expired = safety_state.link_timeout;
        motor_safety_watchdog(&safety_state, tick);
        if (!watchdog_was_expired && safety_state.link_timeout) {
            g_cboard_diag.watchdog_timeout_count++;
            cboard_diag_event(CBOARD_DIAG_WATCHDOG_TIMEOUT, 0, tick,
                               safety_state.last_command_tick);
        }

        DmCanDiscovery_Process();
        dm_discovery_state_t dstate = DmCanDiscovery_GetState();
        if (dstate == DISCOVERY_DONE || dstate == DISCOVERY_TIMEOUT) {
            dm_discovery_result_t dr;
            DmCanDiscovery_GetResult(&dr);
            uint8_t dbuf[28];
            uint8_t dflags = dr.found ? 1 : 0;
            uint16_t desc = dr.esc_id;
            uint16_t dmid = dr.mst_id;
            uint8_t dcm = dr.ctrl_mode;
            uint8_t dcb = dr.can_baud_code;
            uint32_t dsw = dr.sw_version;
            float dpmax = dr.pmax;
            float dvmax = dr.vmax;
            float dtmax = dr.tmax;
            memcpy(dbuf, &dflags, 1);
            memcpy(dbuf + 1, &desc, 2);
            memcpy(dbuf + 3, &dmid, 2);
            memcpy(dbuf + 5, &dcm, 1);
            memcpy(dbuf + 6, &dcb, 1);
            memcpy(dbuf + 7, &dsw, 4);
            memcpy(dbuf + 11, &dpmax, 4);
            memcpy(dbuf + 15, &dvmax, 4);
            memcpy(dbuf + 19, &dtmax, 4);
            memset(dbuf + 23, 0, 5);
            int dlen = pi_c_build_frame(tx_buf, sizeof(tx_buf), MSG_DISCOVERY_RESULT, 0, dbuf, 28);
            if (dlen > 0) cboard_usb_cdc_tx(tx_buf, dlen);
            DmCanDiscovery_Init();
        }
        /* If watchdog expired: zero target, send 0 velocity, send Disable */
        if (safety_state.enabled && safety_state.link_timeout) {
            dm2325_can_mit_ctrl(engineering_motor, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
            dm2325_can_mit_disable(engineering_motor);
            motor_safety_disable(&safety_state);
        }

        cboard_usb_cdc_tx_poll();
        g_cboard_diag.frames_processed_per_iteration = frames_processed;
        cboard_diag_observe_max(&g_cboard_diag.max_frames_processed_per_iteration, frames_processed);
        cboard_diag_observe_max(&g_cboard_diag.main_loop_busy_max_us,
                                 cboard_diag_time_us() - loop_start_us);

        /* Note: MIT zero frames are only sent on watchdog timeout (link_timeout above).
         * Normal MIT control frames come from Pi via MSG_MIT_CTRL.
         * Do NOT auto-send zero here -- it would override Pi-side MIT commands. */
    }
}
