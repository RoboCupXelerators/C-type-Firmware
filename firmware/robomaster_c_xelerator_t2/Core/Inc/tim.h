#ifndef __TIM_H__
#define __TIM_H__

#include <stdbool.h>
#include "main.h"

extern TIM_HandleTypeDef htim10;

bool MX_TIM10_Init(void);
void TIM10_IMU_Heater_Set(uint16_t pwm);
bool TIM10_IMU_Heater_IsReady(void);

#endif
