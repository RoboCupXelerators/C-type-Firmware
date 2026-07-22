
/*
 * DM-S2325-1EC CAN read-only discovery implementation.
 *
 * Sends 0x33 Read Parameters frames to CAN ID 0x7FF with candidate motor IDs.
 * Only reads -- never writes, never enables, never sends velocity.
 *
 * Source: DM-S2325-1EC User Manual V1.1 (2026-05-13), pages 20-21.
 */

#include "dm_can_discovery.h"
#include <string.h>

extern int hal_can_send(uint32_t can_id, const uint8_t *data, uint8_t len);
extern uint32_t hal_get_tick(void);

static dm_discovery_state_t s_state = DISCOVERY_IDLE;
static uint16_t s_first_id = 0;
static uint16_t s_last_id = 0;
static uint16_t s_current_id = 0;
static uint8_t  s_retry_count = 0;
static uint32_t s_last_send_tick = 0;
static dm_discovery_result_t s_result;
static bool s_reply_received = false;
static uint8_t s_reply_data[8];
static uint32_t s_reply_id = 0;

static bool s_reading_params = false;
static uint8_t s_param_index = 0;
static const uint8_t s_param_rids[] = {
    DM_REG_ESC_ID, DM_REG_MST_ID, DM_REG_CTRL_MODE,
    DM_REG_CAN_BAUD, DM_REG_SW_VERSION,
    DM_REG_PMAX, DM_REG_VMAX, DM_REG_TMAX
};
#define NUM_PARAMS (sizeof(s_param_rids) / sizeof(s_param_rids[0]))

void DmCanDiscovery_Init(void) {
    s_state = DISCOVERY_IDLE;
    s_reading_params = false;
    s_param_index = 0;
    memset(&s_result, 0, sizeof(s_result));
}

bool DmCanDiscovery_Start(uint16_t first_id, uint16_t last_id) {
    if (s_state != DISCOVERY_IDLE && s_state != DISCOVERY_DONE
        && s_state != DISCOVERY_TIMEOUT) return false;
    if (last_id < first_id) return false;
    if (first_id > 0x7FE) return false;

    s_first_id = first_id;
    s_last_id = last_id;
    s_current_id = first_id;
    s_retry_count = 0;
    s_last_send_tick = 0;
    s_reply_received = false;
    s_reading_params = false;
    s_param_index = 0;
    memset(&s_result, 0, sizeof(s_result));
    s_state = DISCOVERY_SCANNING;
    return true;
}

static void send_read_param(uint16_t candidate_id, uint8_t rid) {
    /* Official Damiao read_motor_data: DLC=4, can_id_h masked to 3 bits */
    uint8_t data[4];
    data[0] = (uint8_t)(candidate_id & 0xFF);
    data[1] = (uint8_t)((candidate_id >> 8) & 0x07);
    data[2] = DM_DISCOVERY_CMD_READ;
    data[3] = rid;
    hal_can_send(DM_DISCOVERY_PARAM_CAN_ID, data, 4);
}

void DmCanDiscovery_Process(void) {
    uint32_t tick = hal_get_tick();

    if (s_state == DISCOVERY_SCANNING) {
        if (s_last_send_tick == 0) {
            send_read_param(s_current_id, DM_REG_ESC_ID);
            s_last_send_tick = tick;
            return;
        }
        if (s_reply_received) {
            if (s_reply_data[2] == 0x33
                && s_reply_data[3] == DM_REG_ESC_ID
                && s_reply_data[0] == (uint8_t)(s_current_id & 0xFF)
                && s_reply_data[1] == (uint8_t)((s_current_id >> 8) & 0x07)) {
                uint32_t esc_u32 = (uint32_t)s_reply_data[4]
                    | ((uint32_t)s_reply_data[5] << 8)
                    | ((uint32_t)s_reply_data[6] << 16)
                    | ((uint32_t)s_reply_data[7] << 24);
                s_result.esc_id = (uint16_t)(esc_u32 & 0x7FF);
                s_result.found = true;
                s_result.mst_id = (uint16_t)s_reply_id;
                memcpy(s_result.raw_reply, s_reply_data, 8);
                s_reading_params = true;
                s_param_index = 0;
                s_reply_received = false;
                s_last_send_tick = tick;
                s_state = DISCOVERY_READING_PARAMS;
                return;
            }
            s_reply_received = false;
        }
        if ((tick - s_last_send_tick) >= 50) {
            if (s_retry_count < DM_DISCOVERY_MAX_RETRIES) {
                s_retry_count++;
                send_read_param(s_current_id, DM_REG_ESC_ID);
                s_last_send_tick = tick;
            } else {
                s_retry_count = 0;
                s_current_id++;
                if (s_current_id > s_last_id) {
                    s_state = DISCOVERY_TIMEOUT;
                    return;
                }
                send_read_param(s_current_id, DM_REG_ESC_ID);
                s_last_send_tick = tick;
            }
        }
    } else if (s_state == DISCOVERY_READING_PARAMS) {
        if (s_reply_received) {
            if (s_reply_data[2] == 0x33) {
                uint8_t rid = s_reply_data[3];
                uint32_t val = (uint32_t)s_reply_data[4]
                    | ((uint32_t)s_reply_data[5] << 8)
                    | ((uint32_t)s_reply_data[6] << 16)
                    | ((uint32_t)s_reply_data[7] << 24);
                switch (rid) {
                    case DM_REG_ESC_ID: s_result.esc_id = (uint16_t)(val & 0x7FF); break;
                    case DM_REG_MST_ID: s_result.mst_id = (uint16_t)(val & 0x7FF); break;
                    case DM_REG_CTRL_MODE: s_result.ctrl_mode = (uint8_t)(val & 0xFF); break;
                    case DM_REG_CAN_BAUD: s_result.can_baud_code = (uint8_t)(val & 0xFF); break;
                    case DM_REG_SW_VERSION: s_result.sw_version = val; break;
                    case DM_REG_PMAX: memcpy(&s_result.pmax, &val, 4); break;
                    case DM_REG_VMAX: memcpy(&s_result.vmax, &val, 4); break;
                    case DM_REG_TMAX: memcpy(&s_result.tmax, &val, 4); break;
                    default: break;
                }
            }
            s_reply_received = false;
            s_param_index++;
            s_last_send_tick = tick;
            if (s_param_index >= NUM_PARAMS) {
                s_state = DISCOVERY_DONE;
                return;
            }
        }
        if ((tick - s_last_send_tick) >= 50) {
            if (s_param_index < NUM_PARAMS) {
                send_read_param(s_result.esc_id, s_param_rids[s_param_index]);
                s_last_send_tick = tick;
            }
        }
    }
}

void DmCanDiscovery_OnRx(uint32_t std_id, const uint8_t data[8], uint8_t dlc) {
    if (dlc < 4) return;
    if (s_state != DISCOVERY_SCANNING && s_state != DISCOVERY_READING_PARAMS) return;
    if (s_reply_received) return;
    s_reply_id = std_id;
    memcpy(s_reply_data, data, 8);
    s_reply_received = true;
}

void DmCanDiscovery_Stop(void) {
    s_state = DISCOVERY_IDLE;
}

bool DmCanDiscovery_GetResult(dm_discovery_result_t *out_result) {
    if (out_result) memcpy(out_result, &s_result, sizeof(s_result));
    return s_result.found;
}

dm_discovery_state_t DmCanDiscovery_GetState(void) {
    return s_state;
}

uint8_t DmCanDiscovery_GetProgress(void) {
    if (s_state == DISCOVERY_DONE || s_state == DISCOVERY_TIMEOUT) return 100;
    if (s_state == DISCOVERY_READING_PARAMS) return 100;
    if (s_last_id == s_first_id) return 50;
    uint16_t total = s_last_id - s_first_id;
    uint16_t done = s_current_id - s_first_id;
    return (uint8_t)((done * 100) / (total + 1));
}
