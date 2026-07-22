#ifndef __ADC_H__
#define __ADC_H__

#include <stdbool.h>
#include "main.h"

extern ADC_HandleTypeDef hadc1;

bool MX_ADC1_Temperature_Init(void);
bool ADC1_ReadMCUTemperature(float *temperature_c);

#endif
