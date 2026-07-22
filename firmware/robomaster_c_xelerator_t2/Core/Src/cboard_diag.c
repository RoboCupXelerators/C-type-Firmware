#include "cboard_diag.h"
#include "stm32f4xx.h"
#include <string.h>

volatile cboard_diag_state_t g_cboard_diag;
static uint32_t diag_cycles_per_us = 168U;
static uint32_t diag_last_cycle_count;
static uint32_t diag_cycle_remainder;
static uint32_t diag_time_us;

void cboard_diag_init(uint32_t cpu_hz) {
    memset((void *)&g_cboard_diag, 0, sizeof(g_cboard_diag));
    diag_cycles_per_us = cpu_hz / 1000000U;
    if (diag_cycles_per_us == 0U) diag_cycles_per_us = 1U;
    g_cboard_diag.magic = 0x43424447U;
    g_cboard_diag.version = 1U;
    g_cboard_diag.parse_only = CBOARD_DIAG_PARSE_ONLY;
    g_cboard_diag.cpu_hz = cpu_hz;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    diag_last_cycle_count = DWT->CYCCNT;
    diag_cycle_remainder = 0U;
    diag_time_us = 0U;
}

uint32_t cboard_diag_time_us(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t cycles = DWT->CYCCNT;
    uint32_t delta_cycles = cycles - diag_last_cycle_count;
    diag_last_cycle_count = cycles;
    uint32_t accumulated_cycles = diag_cycle_remainder + delta_cycles;
    diag_time_us += accumulated_cycles / diag_cycles_per_us;
    diag_cycle_remainder = accumulated_cycles % diag_cycles_per_us;
    if (primask == 0U) __enable_irq();
    return diag_time_us;
}

void cboard_diag_observe_max(volatile uint32_t *target, uint32_t value) {
    if (value > *target) *target = value;
}

void cboard_diag_event(cboard_diag_event_type_t event, uint16_t arg0,
                       uint32_t arg1, uint32_t arg2) {
    uint32_t index = g_cboard_diag.event_write_index++;
    cboard_diag_event_t *slot = (cboard_diag_event_t *)&g_cboard_diag.events[index % CBOARD_DIAG_EVENT_CAPACITY];
    if (index >= CBOARD_DIAG_EVENT_CAPACITY) g_cboard_diag.event_overflow_count++;
    slot->timestamp_us = cboard_diag_time_us();
    slot->event = (uint16_t)event;
    slot->arg0 = arg0;
    slot->arg1 = arg1;
    slot->arg2 = arg2;
}
