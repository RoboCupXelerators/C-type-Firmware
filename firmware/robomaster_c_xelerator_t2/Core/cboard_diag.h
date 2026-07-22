#ifndef CBOARD_DIAG_H
#define CBOARD_DIAG_H

#include <stdint.h>

#ifndef CBOARD_DIAG_PARSE_ONLY
#define CBOARD_DIAG_PARSE_ONLY 0
#endif

#define CBOARD_DIAG_EVENT_CAPACITY 128U

typedef enum {
    CBOARD_DIAG_USB_RX_ENTER = 1,
    CBOARD_DIAG_USB_RX_EXIT,
    CBOARD_DIAG_USB_RX_REARM,
    CBOARD_DIAG_FRAME_PARSED,
    CBOARD_DIAG_FRAME_DISPATCH,
    CBOARD_DIAG_MIT_RECEIVED,
    CBOARD_DIAG_STATUS_REQUEST,
    CBOARD_DIAG_STATUS_TX_ENQUEUE,
    CBOARD_DIAG_CDC_TX_START,
    CBOARD_DIAG_CDC_TX_COMPLETE,
    CBOARD_DIAG_CAN_TX_BEGIN,
    CBOARD_DIAG_CAN_TX_END,
    CBOARD_DIAG_MAIN_LOOP_BEGIN,
    CBOARD_DIAG_MAIN_LOOP_END,
    CBOARD_DIAG_WATCHDOG_SERVICE,
    CBOARD_DIAG_RX_OVERFLOW,
    CBOARD_DIAG_TX_QUEUE_FULL,
    CBOARD_DIAG_CAN_MAILBOX_FULL,
} cboard_diag_event_type_t;

typedef struct {
    uint32_t timestamp_us;
    uint16_t event;
    uint16_t arg0;
    uint32_t arg1;
    uint32_t arg2;
} cboard_diag_event_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t parse_only;
    uint32_t cpu_hz;
    uint32_t usb_rx_callback_count;
    uint32_t usb_rx_packet_count;
    uint32_t usb_rx_bytes;
    uint32_t usb_rx_last_us;
    uint32_t usb_rx_max_gap_us;
    uint32_t usb_rx_rearm_count;
    uint32_t usb_rx_ring_depth;
    uint32_t usb_rx_ring_high_water;
    uint32_t usb_rx_ring_overflow_count;
    uint32_t usb_rx_dropped_bytes;
    uint32_t parser_frames_ok;
    uint32_t parser_partial_count;
    uint32_t parser_bytes_pending;
    uint32_t parser_last_frame_us;
    uint32_t mit_command_count;
    uint32_t status_request_count;
    uint32_t enable_count;
    uint32_t disable_count;
    uint32_t main_loop_iterations;
    uint32_t main_loop_period_max_us;
    uint32_t main_loop_busy_max_us;
    uint32_t main_loop_stall_count;
    uint32_t frames_processed_per_iteration;
    uint32_t max_frames_processed_per_iteration;
    uint32_t watchdog_service_gap_max_us;
    uint32_t can_tx_attempt_count;
    uint32_t can_tx_success_count;
    uint32_t can_tx_fail_count;
    uint32_t can_tx_no_mailbox_count;
    uint32_t can_tx_duration_max_us;
    uint32_t cdc_tx_enqueue_count;
    uint32_t cdc_tx_complete_count;
    uint32_t cdc_tx_busy_count;
    uint32_t cdc_tx_fail_count;
    uint32_t cdc_tx_queue_depth;
    uint32_t cdc_tx_queue_high_water;
    uint32_t cdc_tx_status_count;
    uint32_t cdc_tx_status_latency_max_us;
    uint32_t can1_rx_irq_count;
    uint32_t can1_rx_irq_duration_max_us;
    uint32_t usb_irq_count;
    uint32_t usb_irq_duration_max_us;
    uint32_t event_write_index;
    uint32_t event_overflow_count;
    cboard_diag_event_t events[CBOARD_DIAG_EVENT_CAPACITY];
} cboard_diag_state_t;

extern volatile cboard_diag_state_t g_cboard_diag;

void cboard_diag_init(uint32_t cpu_hz);
uint32_t cboard_diag_time_us(void);
void cboard_diag_event(cboard_diag_event_type_t event, uint16_t arg0,
                       uint32_t arg1, uint32_t arg2);
void cboard_diag_observe_max(volatile uint32_t *target, uint32_t value);

#endif
