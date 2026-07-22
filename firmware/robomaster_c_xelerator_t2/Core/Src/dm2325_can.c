/*
 * DM-S2325-1EC CAN driver — based on official Damiao V1.1 manual.
 *
 * Velocity Mode: 4-byte float32 LE at CAN_ID = slave_id + 0x200.
 * Enable/Disable: 8-byte 0xFF*7 + cmd at CAN_ID = slave_id + 0x200.
 * Feedback: 8-byte packed uint fields per V1.1 manual page 14.
 *
 * Source: reference/vendor/damiao-official
 * Manual: DM-S2325-1EC separable motor User Manual V1.1.pdf (2026-05-13)
 * SDK:    SDK/电机SDK/Python例程/u2can/DM_CAN.py
 * STM32:  SDK/电机控制例程/stm32例程/dm_ctrl(f4) v1.1 裸机/User/dm_motor_drv.c
 */

#include "dm2325_can.h"
#include <string.h>
#include <math.h>

/* Stub HAL functions — connect to real STM32 HAL_CAN_* in CubeMX project */
extern int hal_can_init(uint32_t baudrate);
extern int hal_can_send(uint32_t can_id, const uint8_t *data, uint8_t len);
extern uint32_t hal_get_tick(void);

/* Damiao float<->uint conversion (V1.1 manual page 15) */
int dm_float_to_uint(float x, float x_min, float x_max, int bits) {
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    float span = x_max - x_min;
    return (int)((x - x_min) * (float)((1 << bits) - 1) / span);
}

float dm_uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + x_min;
}

dm2325_result_t dm2325_can_init(uint32_t can_baudrate) {
    if (hal_can_init(can_baudrate) != 0) {
        return DM2325_ERR_CAN_INIT;
    }
    return DM2325_OK;
}

dm2325_result_t dm2325_can_enable(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[DM_CMD_DATA_LEN] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_ENABLE
    };
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_VEL;
    if (hal_can_send(can_id, data, DM_CMD_DATA_LEN) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    motor->enabled = true;
    return DM2325_OK;
}

dm2325_result_t dm2325_can_disable(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[DM_CMD_DATA_LEN] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_DISABLE
    };
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_VEL;
    if (hal_can_send(can_id, data, DM_CMD_DATA_LEN) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    motor->enabled = false;
    return DM2325_OK;
}

dm2325_result_t dm2325_can_set_velocity(dm2325_motor_t *motor, float velocity_rad_s) {
    if (!motor) return DM2325_ERR_PARAM;
    if (!motor->enabled) return DM2325_ERR_NOT_ENABLED;

    /* Reject NaN and Inf (safety requirement) */
    if (isnan(velocity_rad_s) || isinf(velocity_rad_s)) {
        return DM2325_ERR_NAN_INF;
    }

    /* VEL mode: CAN_ID = slave_id + 0x200, data = 4 bytes float32 LE.
     * Per V1.1 manual page 17 and official STM32F4 example spd_ctrl().
     * Use memcpy for safe float-to-bytes conversion (no pointer punning). */
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_VEL;
    uint8_t data[DM_VEL_DATA_LEN] = {0};
    uint32_t raw = 0;
    memcpy(&raw, &velocity_rad_s, sizeof(raw));
    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24) & 0xFFU);

    if (hal_can_send(can_id, data, DM_VEL_DATA_LEN) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    return DM2325_OK;
}

dm2325_result_t dm2325_can_stop(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    if (motor->enabled) {
        dm2325_can_set_velocity(motor, 0.0f);
    }
    return dm2325_can_disable(motor);
}

dm2325_result_t dm2325_can_clear_error(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[DM_CMD_DATA_LEN] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_CLEAR_ERROR
    };
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_VEL;
    if (hal_can_send(can_id, data, DM_CMD_DATA_LEN) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    return DM2325_OK;
}

/* --- MIT mode functions (official dm_motor_drv.c) --- */

dm2325_result_t dm2325_can_mit_enable(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_ENABLE
    };
    /* MIT mode: CAN_ID = slave_id + 0x000 */
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_MIT;
    if (hal_can_send(can_id, data, 8) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    motor->enabled = true;
    return DM2325_OK;
}

dm2325_result_t dm2325_can_mit_disable(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_DISABLE
    };
    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_MIT;
    if (hal_can_send(can_id, data, 8) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    motor->enabled = false;
    return DM2325_OK;
}

dm2325_result_t dm2325_can_mit_ctrl(dm2325_motor_t *motor,
                                     float p_des, float v_des,
                                     float kp, float kd, float t_ff) {
    if (!motor) return DM2325_ERR_PARAM;
    if (!motor->enabled) return DM2325_ERR_NOT_ENABLED;

    /* Reject NaN/Inf (safety) */
    if (isnan(p_des) || isinf(p_des) ||
        isnan(v_des) || isinf(v_des) ||
        isnan(kp) || isinf(kp) ||
        isnan(kd) || isinf(kd) ||
        isnan(t_ff) || isinf(t_ff)) {
        return DM2325_ERR_NAN_INF;
    }

    /* Official Damiao MIT encoding (dm_motor_drv.c mit_ctrl):
     * pos: 16-bit, vel: 12-bit, kp: 12-bit, kd: 12-bit, tor: 12-bit */
    uint16_t pos_tmp = (uint16_t)dm_float_to_uint(p_des, -motor->pmax, motor->pmax, 16);
    uint16_t vel_tmp = (uint16_t)dm_float_to_uint(v_des, -motor->vmax, motor->vmax, 12);
    uint16_t tor_tmp = (uint16_t)dm_float_to_uint(t_ff, -motor->tmax, motor->tmax, 12);
    uint16_t kp_tmp  = (uint16_t)dm_float_to_uint(kp, DM_MIT_KP_MIN, DM_MIT_KP_MAX, 12);
    uint16_t kd_tmp  = (uint16_t)dm_float_to_uint(kd, DM_MIT_KD_MIN, DM_MIT_KD_MAX, 12);

    uint8_t data[8];
    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp & 0xFF);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp & 0xFF);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp & 0xFF);

    uint32_t can_id = motor->slave_id + DM_MODE_OFFSET_MIT;
    if (hal_can_send(can_id, data, 8) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    return DM2325_OK;
}

dm2325_result_t dm2325_can_refresh_status(dm2325_motor_t *motor) {
    if (!motor) return DM2325_ERR_PARAM;
    uint8_t data[8] = {
        (uint8_t)(motor->slave_id & 0xFFU), 0x00U, 0xCCU, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U
    };
    if (hal_can_send(DM_PARAM_CAN_ID, data, sizeof(data)) != 0) {
        return DM2325_ERR_CAN_TX;
    }
    return DM2325_OK;
}

void dm2325_can_decode_feedback(dm2325_motor_t *motor, const uint8_t *data, uint8_t len) {
    if (!motor || !data) return;
    if (len != 8U) {
        if (motor->feedback.invalid_count != UINT16_MAX) motor->feedback.invalid_count++;
        return;
    }

    /* D[0]: [7:4]=ERR, [3:0]=ID (V1.1 manual page 14) */
    motor->feedback.error_code = (data[0] >> 4) & 0x0F;

    /* D[1:2]: POS uint16 -> [-PMAX, PMAX] (16-bit) */
    uint16_t pos_uint = ((uint16_t)data[1] << 8) | data[2];
    motor->feedback.position_raw = pos_uint;
    float wrapped_position = dm_uint_to_float(
        pos_uint, -motor->pmax, motor->pmax, 16);
    if (!motor->feedback.position_initialized) {
        motor->feedback.position_unwrapped = wrapped_position;
        motor->feedback.position_initialized = true;
    } else {
        float delta = wrapped_position - motor->feedback.position;
        float period = 2.0f * motor->pmax;
        if (delta > motor->pmax) delta -= period;
        if (delta < -motor->pmax) delta += period;
        motor->feedback.position_unwrapped += delta;
    }
    motor->feedback.position = wrapped_position;

    /* D[3:4]: VEL uint12 (top 12 bits) -> [-VMAX, VMAX] */
    uint16_t vel_uint = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    motor->feedback.velocity_raw = vel_uint;
    motor->feedback.velocity = dm_uint_to_float(
        vel_uint, -motor->vmax, motor->vmax, 12);

    /* D[4:5]: T uint12 (low nibble D[4] + D[5]) -> [-TMAX, TMAX] */
    uint16_t tau_uint = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];
    motor->feedback.torque_raw = tau_uint;
    motor->feedback.torque = dm_uint_to_float(
        tau_uint, -motor->tmax, motor->tmax, 12);

    /* D[6]: T_MOS, D[7]: T_Rotor (Celsius) */
    motor->feedback.temp_mos = data[6];
    motor->feedback.temp_rotor = data[7];

    motor->feedback.online = true;
    motor->feedback.fault = (motor->feedback.error_code != DM_ERR_ENABLED
                             && motor->feedback.error_code != DM_ERR_DISABLED);
    motor->feedback.last_feedback_tick = hal_get_tick();
    motor->feedback.stale_latched = false;
    memcpy(motor->feedback.raw_can_data, data, 8);
    motor->feedback.feedback_sequence++;
    motor->feedback.rx_count++;
}

bool dm2325_can_is_online(const dm2325_motor_t *motor, uint32_t current_tick) {
    if (!motor || !motor->feedback.online) return false;
    /* Consider offline if no feedback for 1 second */
    return (current_tick - motor->feedback.last_feedback_tick) < 1000;
}
