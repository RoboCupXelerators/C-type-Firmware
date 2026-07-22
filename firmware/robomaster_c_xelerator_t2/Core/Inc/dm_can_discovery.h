#ifndef DM_CAN_DISCOVERY_H
#define DM_CAN_DISCOVERY_H

#include <stdint.h>
#include <stdbool.h>

/*
 * DM-S2325-1EC CAN read-only discovery module.
 *
 * Uses official Damiao 0x33 Read Parameters command (V1.1 manual page 20-21)
 * to find the ESC_ID of a connected motor. Sends parameter-read frames to
 * CAN ID 0x7FF with candidate motor IDs, and checks for valid replies.
 *
 * This module ONLY sends 0x33 (read) commands. It never sends:
 *   - 0x55 (write parameter)
 *   - 0xAA (save parameters)
 *   - Enable (0xFC)
 *   - Disable (0xFD)
 *   - Velocity commands
 *   - Any motor control frame
 */

#define DM_DISCOVERY_PARAM_CAN_ID   0x7FF
#define DM_DISCOVERY_CMD_READ       0x33

/* Register IDs to read (V1.1 manual page 20-21) */
#define DM_REG_MST_ID       0x07
#define DM_REG_ESC_ID       0x08
#define DM_REG_CTRL_MODE    0x0A
#define DM_REG_CAN_BAUD     0x23
#define DM_REG_SW_VERSION   0x0E
#define DM_REG_PMAX         0x15
#define DM_REG_VMAX         0x16
#define DM_REG_TMAX         0x17

#define DM_DISCOVERY_MAX_RETRIES   1
#define DM_DISCOVERY_QUERY_INTERVAL_MS  5

typedef struct {
    uint16_t esc_id;        /* discovered ESC_ID */
    uint16_t mst_id;        /* discovered MST_ID (feedback CAN ID) */
    uint8_t  ctrl_mode;     /* current control mode */
    uint8_t  can_baud_code; /* current CAN baud rate code */
    uint32_t sw_version;    /* software version */
    float    pmax;            /* MIT position mapping max */
    float    vmax;            /* MIT velocity mapping max */
    float    tmax;            /* MIT torque mapping max */
    bool     found;         /* true if a valid motor was found */
    uint8_t  raw_reply[8];  /* raw reply frame that confirmed discovery */
} dm_discovery_result_t;

typedef enum {
    DISCOVERY_IDLE = 0,
    DISCOVERY_SCANNING,
    DISCOVERY_READING_PARAMS,
    DISCOVERY_DONE,
    DISCOVERY_TIMEOUT,
} dm_discovery_state_t;

/* Initialize discovery module */
void DmCanDiscovery_Init(void);

/* Start scanning candidate IDs [first_id, last_id] */
bool DmCanDiscovery_Start(uint16_t first_id, uint16_t last_id);

/* Call periodically from main loop; sends queries, manages timeouts */
void DmCanDiscovery_Process(void);

/* Called from CAN RX callback to feed received frames */
void DmCanDiscovery_OnRx(uint32_t std_id, const uint8_t data[8], uint8_t dlc);

/* Stop discovery */
void DmCanDiscovery_Stop(void);

/* Get current result */
bool DmCanDiscovery_GetResult(dm_discovery_result_t *out_result);

/* Get current state */
dm_discovery_state_t DmCanDiscovery_GetState(void);

/* Get current scan progress (0-100) */
uint8_t DmCanDiscovery_GetProgress(void);

#endif /* DM_CAN_DISCOVERY_H */
