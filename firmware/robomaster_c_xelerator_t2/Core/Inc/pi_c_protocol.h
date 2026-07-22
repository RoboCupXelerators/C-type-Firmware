#ifndef PI_C_PROTOCOL_H
#define PI_C_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "cboard_imu.h"

/*
 * Pi-C binary protocol (version 1).
 *
 * Frame layout (all fields little-endian unless noted):
 *   Offset  Size  Field
 *   ------  ----  -----
 *   0       2     Frame header: 0x55 0xAA
 *   2       1     Protocol version (currently 1)
 *   3       1     Message type (see pi_c_msg_type_t)
 *   4       1     Sequence number (wraps 0-255)
 *   5       1     Payload length (0-250)
 *   6       N     Payload (N bytes, N = payload length)
 *   6+N     2     CRC16-CCITT (0x1021, init 0xFFFF) over bytes [0 .. 6+N-1]
 *   8+N     1     Frame tail: 0x0D
 *
 * Maximum frame size = 259 bytes.
 *
 * NOTE: The frame tail (0x0D) was added in version 1 for improved
 * stream synchronization. The original spec called for header + version
 * + msg_type + seq + payload_len + payload + CRC without a tail.
 * The tail provides an additional sync boundary that helps the parser
 * reject corrupted frames faster. Pi, C board, mock, and tests are all
 * unified on this format.
 */

#define PI_C_PROTOCOL_VERSION   1

#define PI_C_FRAME_HEADER_0     0x55
#define PI_C_FRAME_HEADER_1     0xAA
#define PI_C_FRAME_TAIL         0x0D

#define PI_C_MAX_PAYLOAD        250
#define PI_C_MAX_FRAME_SIZE     259

/* Message types */
typedef enum {
    MSG_PING            = 0x01,
    MSG_PONG            = 0x02,
    MSG_GET_STATUS      = 0x03,
    MSG_STATUS          = 0x04,
    MSG_MOTOR_ENABLE    = 0x05,
    MSG_MOTOR_DISABLE   = 0x06,
    MSG_MOTOR_SET_SPEED = 0x07,
    MSG_MOTOR_STOP      = 0x08,
    MSG_HEARTBEAT       = 0x09,
    MSG_ACK             = 0x0A,
    MSG_NACK            = 0x0B,
    MSG_ERROR           = 0x0C,
    MSG_GET_CAN_STATUS  = 0x0D,
    MSG_CAN_STATUS      = 0x0E,
    MSG_GET_BOARD_INFO  = 0x0F,
    MSG_BOARD_INFO      = 0x10,
    MSG_DISCOVERY_START  = 0x11,
    MSG_DISCOVERY_STATUS = 0x12,
    MSG_DISCOVERY_RESULT = 0x13,
    MSG_DISCOVERY_STOP   = 0x14,
    MSG_SET_CAN_CHANNEL  = 0x15,  /* payload: 1 byte; only CAN1 (1) is accepted */
    MSG_MIT_CTRL         = 0x16,  /* payload: motor_id(1) + p_des(4) + v_des(4) + kp(4) + kd(4) + t_ff(4) = 21 bytes */
    MSG_MIT_STOP         = 0x17,  /* payload: motor_id(1) - sends MIT zero + disable */
    MSG_CHASSIS_TWIST    = 0x18,
    MSG_CHASSIS_ARM      = 0x19,
    MSG_CHASSIS_STOP     = 0x1A,
    MSG_CHASSIS_DISABLE  = 0x1B,
    MSG_GET_CHASSIS_STATUS = 0x1C,
    MSG_CHASSIS_STATUS   = 0x1D,
    MSG_MOTOR_REFRESH_STATUS = 0x1E,
    MSG_DM_PARAM_READ     = 0x1F,
    MSG_DM_PARAM_WRITE    = 0x20,
    MSG_DM_PARAM_SAVE     = 0x21,
    MSG_DM_PARAM_RESULT   = 0x22,
    MSG_CHASSIS_REFRESH_DISABLED = 0x23,
    MSG_CHASSIS_CLEAR_FAULT = 0x24,
    MSG_CHASSIS_EMERGENCY_STOP = 0x25,
    MSG_GET_IMU_STATUS   = 0x26,
    MSG_IMU_STATUS       = 0x27,
    MSG_IMU_ZERO_YAW     = 0x28,
    MSG_IMU_SET_REFERENCE = 0x29, /* payload: float32 degrees, little-endian */
    MSG_CHASSIS_SET_PROFILE = 0x2A, /* payload: chassis_control_profile_t */
    MSG_GET_CHASSIS_DIAGNOSTICS = 0x2B,
    MSG_CHASSIS_DIAGNOSTICS = 0x2C,

} pi_c_msg_type_t;

/* NACK reason codes */
typedef enum {
    NACK_CRC_ERROR        = 0x01,
    NACK_BAD_VERSION      = 0x02,
    NACK_UNKNOWN_MSG      = 0x03,
    NACK_BAD_PAYLOAD      = 0x04,
    NACK_BAD_MOTOR_ID     = 0x05,
    NACK_NOT_ENABLED      = 0x06,
    NACK_OUT_OF_RANGE     = 0x07,
    NACK_BUSY             = 0x08,
    NACK_INTERNAL_ERROR   = 0x09,
    NACK_WATCHDOG_TIMEOUT = 0x0A,
} pi_c_nack_reason_t;

/* Parsed frame — includes a payload copy to avoid dangling pointers */
typedef struct {
    uint8_t  version;
    uint8_t  msg_type;
    uint8_t  seq;
    uint8_t  payload_len;
    uint8_t  payload_buf[PI_C_MAX_PAYLOAD];
    const uint8_t *payload;  /* points to payload_buf after successful parse */
} pi_c_frame_t;

/* Parser state */
typedef struct {
    uint8_t  buf[PI_C_MAX_FRAME_SIZE];
    uint16_t buf_len;
    uint16_t discard_count;
} pi_c_parser_t;

#define PI_C_CHASSIS_TWIST_PAYLOAD_LEN 20U
#define PI_C_CHASSIS_MODE_VELOCITY 1U
#define PI_C_IMU_STATUS_PAYLOAD_LEN 156U
#define PI_C_CHASSIS_DIAGNOSTICS_PAYLOAD_LEN 208U

typedef struct {
    uint32_t host_monotonic_ms;
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
    uint16_t valid_for_ms;
    uint8_t control_mode;
    uint8_t flags;
} pi_c_chassis_twist_t;

/* CRC16-CCITT (0x1021, init 0xFFFF) */
uint16_t pi_c_crc16(const uint8_t *data, size_t len);

/* Initialize parser */
void pi_c_parser_init(pi_c_parser_t *p);

/* Feed bytes into parser; returns 1 if a complete frame is available */
int pi_c_parser_feed(pi_c_parser_t *p, const uint8_t *data, size_t len, pi_c_frame_t *out_frame);

/* Build a frame into buffer; returns total frame length or 0 on error */
int pi_c_build_frame(uint8_t *out, size_t out_size, uint8_t msg_type, uint8_t seq,
                     const uint8_t *payload, uint8_t payload_len);

/* Build ACK frame */
int pi_c_build_ack(uint8_t *out, size_t out_size, uint8_t seq, uint8_t acked_msg_type);

/* Build NACK frame */
int pi_c_build_nack(uint8_t *out, size_t out_size, uint8_t seq, uint8_t acked_msg_type, uint8_t reason);

/* Build PONG frame */
int pi_c_build_pong(uint8_t *out, size_t out_size, uint8_t seq);

/* Decode a validated little-endian CHASSIS_TWIST payload. */
int pi_c_decode_chassis_twist(
    const uint8_t *payload,
    size_t payload_len,
    pi_c_chassis_twist_t *out);

/* Encode one internally consistent IMU snapshot, little-endian IEEE-754. */
int pi_c_encode_imu_status(
    uint8_t *out,
    size_t out_size,
    const cboard_imu_state_t *state);

#endif /* PI_C_PROTOCOL_H */
