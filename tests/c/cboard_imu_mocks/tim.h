#ifndef TEST_CBOARD_IMU_TIM_H
#define TEST_CBOARD_IMU_TIM_H
#include <stdbool.h>
#include <stdint.h>
bool TIM10_IMU_Heater_IsReady(void);
void TIM10_IMU_Heater_Set(uint16_t compare);
#endif
