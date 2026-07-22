#include "dm2325_can.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t tick_ms = 100U;
uint32_t hal_get_tick(void) { return tick_ms; }
int hal_can_init(uint32_t baudrate) { (void)baudrate; return 0; }
int hal_can_send(uint32_t can_id, const uint8_t *data, uint8_t len) {
    (void)can_id; (void)data; (void)len; return 0;
}

int main(void) {
    dm2325_motor_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.pmax = 12.5f; motor.vmax = 200.0f; motor.tmax = 10.0f;

    uint8_t near_positive[8] = {0x11, 0xFF, 0xF0, 0x7F, 0xF7, 0xFF, 30, 31};
    uint8_t near_negative[8] = {0x11, 0x00, 0x10, 0x7F, 0xF7, 0xFF, 30, 31};
    dm2325_can_decode_feedback(&motor, near_positive, 8U);
    float first = motor.feedback.position_unwrapped;
    tick_ms++;
    dm2325_can_decode_feedback(&motor, near_negative, 8U);
    assert(motor.feedback.rx_count == 2U);
    assert(motor.feedback.feedback_sequence == 2U);
    assert(motor.feedback.position_unwrapped > first);
    assert(fabsf(motor.feedback.position_unwrapped - first) < 0.1f);

    dm2325_can_decode_feedback(&motor, near_negative, 7U);
    assert(motor.feedback.invalid_count == 1U);
    assert(motor.feedback.rx_count == 2U);
    return 0;
}
