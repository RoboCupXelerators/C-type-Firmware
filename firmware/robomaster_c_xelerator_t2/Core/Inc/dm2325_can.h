#ifndef DM_S2325_CAN_H
#define DM_S2325_CAN_H

#include <stdint.h>
#include <stdbool.h>

/*
 * DM-S2325-1EC CAN driver — based on official Damiao V1.1 manual
 * (2026-05-13) and official Gitee SDK.
 *
 * Manual: reference/vendor/damiao-official/电机产品/分立系列/DM-S2325-1EC/说明书/
 *   DM-S2325-1EC separable motor User Manual V1.1.pdf
 * SDK:    reference/vendor/damiao-official/SDK/电机SDK/Python例程/u2can/DM_CAN.py
 * STM32:  reference/vendor/damiao-official/SDK/电机控制例程/stm32例程/
 *   dm_ctrl(f4) v1.1 裸机/User/dm_motor_drv.c
 *
 * CAN ID encoding (V1.1 manual page 13-17):
 *   MIT mode:      motor_id + 0x000
 *   POS_VEL mode:  motor_id + 0x100
 *   VEL mode:      motor_id + 0x200  <-- first version uses this
 *   POS_FORCE mode: motor_id + 0x300
 *   Parameter R/W: 0x7FF
 *
 * Velocity Mode control frame (V1.1 manual page 17):
 *   CAN ID = motor_id + 0x200
 *   Data = 4 bytes: float32 LE (v_des in rad/s)
 *   NOTE: Only 4 bytes, NOT 8! (confirmed by manual and STM32 example)
 *
 * Enable (V1.1 manual page 17):
 *   Data = 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC (8 bytes)
 *
 * Disable (V1.1 manual page 17):
 *   Data = 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD (8 bytes)
 *
 * Clear Faults (V1.1 manual page 17):
 *   Data = 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFB (8 bytes)
 *
 * Feedback (V1.1 manual page 14):
 *   D[0]: [7:4]=ERR, [3:0]=ID
 *   D[1:2]: POS uint16 -> [-PMAX, PMAX] (16-bit)
 *   D[3:4]: VEL uint12 (top 12 bits) -> [-VMAX, VMAX]
 *   D[4:5]: T uint12 (low nibble D[4] + D[5]) -> [-TMAX, TMAX]
 *   D[6]: T_MOS (Celsius)
 *   D[7]: T_Rotor (Celsius)
 *
 * DM-S2325-1EC specifications (V1.1 manual page 7):
 *   Rated voltage: 24V (driver supports 24-48V)
 *   Rated torque: 1.35 Nm, Peak: 5 Nm
 *   Rated speed: 380 rpm, Max: 560 rpm (output shaft)
 *   Gear ratio: 1:25, Pole pairs: 7
 *   CAN: 1 Mbps default (classic CAN), max 5 Mbps (CAN FD)
 *
 * MIT mapping defaults (V1.1 manual page 15):
 *   PMAX = 12.5 rad, VMAX = 200 rad/s, TMAX = 20 Nm
 *   These are MIT mode mapping ranges, NOT mechanical limits.
 *   Velocity Mode sends raw float32 without mapping.
 */

/* Mode offsets */
#define DM_MODE_OFFSET_MIT       0x000
#define DM_MODE_OFFSET_POS_VEL   0x100
#define DM_MODE_OFFSET_VEL       0x200
#define DM_MODE_OFFSET_POS_FORCE 0x300

/* MIT mode Kp/Kd encoding range (official dm_motor_drv.h) */
#define DM_MIT_KP_MIN             0.0f
#define DM_MIT_KP_MAX             500.0f
#define DM_MIT_KD_MIN             0.0f
#define DM_MIT_KD_MAX             5.0f

/* Parameter CAN ID */
#define DM_PARAM_CAN_ID          0x7FF

/* Command bytes */
#define DM_CMD_ENABLE            0xFC
#define DM_CMD_DISABLE           0xFD
#define DM_CMD_SET_ZERO          0xFE
#define DM_CMD_CLEAR_ERROR       0xFB

/* Control modes (register 0x0A) */
#define DM_CTRL_MODE_MIT         1
#define DM_CTRL_MODE_POS_VEL     2
#define DM_CTRL_MODE_VEL         3
#define DM_CTRL_MODE_POS_FORCE   4

/* CAN baud rate codes (register 0x23) */
#define DM_CAN_BAUD_1M           4

/* Data lengths */
#define DM_VEL_DATA_LEN          4   /* float32 only */
#define DM_CMD_DATA_LEN          8   /* 0xFF*7 + cmd */

/* DM-S2325-1EC MIT mapping defaults */
#define DM_S2325_PMAX_DEFAULT    12.5f
#define DM_S2325_VMAX_DEFAULT    200.0f
#define DM_S2325_TMAX_DEFAULT    10.0f  /* Actual motor value, read via 0x33 RID 23 */

/* DM-S2325-1EC mechanical limits */
#define DM_S2325_RATED_SPEED_RPM 380.0f
#define DM_S2325_MAX_SPEED_RPM   560.0f
#define DM_S2325_RATED_TORQUE    1.35f
#define DM_S2325_PEAK_TORQUE     5.0f
#define DM_S2325_GEAR_RATIO      25.0f

/* Error codes (V1.1 manual page 14, LED page 11) */
#define DM_ERR_DISABLED          0x0
#define DM_ERR_ENABLED           0x1
#define DM_ERR_SENSOR            0x5
#define DM_ERR_PARAM             0x6
#define DM_ERR_OVER_VOLTAGE      0x8
#define DM_ERR_UNDER_VOLTAGE     0x9
#define DM_ERR_OVER_CURRENT      0xA
#define DM_ERR_MOS_OVER_TEMP     0xB
#define DM_ERR_COIL_OVER_TEMP    0xC
#define DM_ERR_COMM_LOST         0xD
#define DM_ERR_OVERLOAD          0xE

typedef enum {
    DM2325_OK                     = 0,
    DM2325_ERR_CAN_INIT           = 1,
    DM2325_ERR_CAN_TX             = 2,
    DM2325_ERR_CAN_RX             = 3,
    DM2325_ERR_MOTOR_OFFLINE      = 4,
    DM2325_ERR_PARAM              = 5,
    DM2325_ERR_TIMEOUT            = 6,
    DM2325_ERR_NOT_ENABLED        = 7,
    DM2325_ERR_NAN_INF            = 8,
} dm2325_result_t;

typedef struct {
    float    position;     /* rad (output shaft) */
    float    position_unwrapped; /* continuous position across MIT mapping wrap */
    float    velocity;     /* rad/s (output shaft) */
    float    torque;       /* Nm (output shaft) */
    uint8_t  error_code;
    uint8_t  temp_mos;     /* Celsius */
    uint8_t  temp_rotor;   /* Celsius */
    bool     online;
    bool     fault;
    uint32_t last_feedback_tick;
    uint32_t feedback_sequence;
    uint32_t rx_count;
    uint16_t invalid_count;
    uint16_t stale_count;
    uint16_t last_raw_can_id;
    bool     stale_latched;
    bool     position_initialized;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    uint8_t  raw_can_data[8];
} dm2325_feedback_t;

typedef struct {
    uint8_t  slave_id;     /* CAN motor ID (ESC_ID) */
    uint8_t  master_id;    /* Feedback CAN ID (MST_ID) */
    float    pmax;         /* MIT position mapping max (rad) */
    float    vmax;         /* MIT velocity mapping max (rad/s) */
    float    tmax;         /* MIT torque mapping max (Nm) */
    bool     enabled;
    dm2325_feedback_t feedback;
} dm2325_motor_t;

/* Initialize CAN peripheral */
dm2325_result_t dm2325_can_init(uint32_t can_baudrate);

/* Enable motor: send 0xFF*7 + 0xFC to slave_id + 0x200 */
dm2325_result_t dm2325_can_enable(dm2325_motor_t *motor);

/* Disable motor: send 0xFF*7 + 0xFD to slave_id + 0x200 */
dm2325_result_t dm2325_can_disable(dm2325_motor_t *motor);

/*
 * Set velocity in VEL mode (V1.1 manual page 17).
 * CAN ID = slave_id + 0x200, data = 4 bytes float32 LE.
 * Uses memcpy for safe float-to-bytes conversion (no pointer punning).
 */
dm2325_result_t dm2325_can_set_velocity(dm2325_motor_t *motor, float velocity_rad_s);

/* Stop motor: send velocity=0, then disable */
dm2325_result_t dm2325_can_stop(dm2325_motor_t *motor);

/* Clear faults: send 0xFF*7 + 0xFB to slave_id + 0x200 */
dm2325_result_t dm2325_can_clear_error(dm2325_motor_t *motor);

/* Decode feedback frame (8 bytes) per V1.1 manual page 14 */
void dm2325_can_decode_feedback(dm2325_motor_t *motor, const uint8_t *data, uint8_t len);

/* Check if motor is online */
bool dm2325_can_is_online(const dm2325_motor_t *motor, uint32_t current_tick);

/*
 * MIT mode control (official dm_motor_drv.c mit_ctrl).
 * CAN ID = slave_id + 0x000, data = 8 bytes packed MIT.
 * p_des: position target (rad), v_des: velocity target (rad/s)
 * kp: position stiffness, kd: velocity damping, t_ff: feedforward torque (Nm)
 */
dm2325_result_t dm2325_can_mit_enable(dm2325_motor_t *motor);
dm2325_result_t dm2325_can_mit_disable(dm2325_motor_t *motor);
dm2325_result_t dm2325_can_mit_ctrl(dm2325_motor_t *motor,
                                     float p_des, float v_des,
                                     float kp, float kd, float t_ff);

/* Request one fresh feedback frame without enabling or changing parameters. */
dm2325_result_t dm2325_can_refresh_status(dm2325_motor_t *motor);

/* Damiao float<->uint conversion (V1.1 manual page 15) */
int dm_float_to_uint(float x, float x_min, float x_max, int bits);
float dm_uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif /* DM_S2325_CAN_H */
