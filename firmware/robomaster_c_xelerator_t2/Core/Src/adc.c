#include "adc.h"

#include <math.h>

ADC_HandleTypeDef hadc1;
static bool adc1_temperature_ready;

bool MX_ADC1_Temperature_Init(void) {
    ADC_ChannelConfTypeDef channel = {0};
    adc1_temperature_ready = false;
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) return false;
    channel.Channel = ADC_CHANNEL_TEMPSENSOR;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adc1_temperature_ready = HAL_ADC_ConfigChannel(&hadc1, &channel) == HAL_OK;
    return adc1_temperature_ready;
}

bool ADC1_ReadMCUTemperature(float *temperature_c) {
    if (!adc1_temperature_ready || temperature_c == NULL) return false;
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return false;
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 10U);
    uint32_t raw = status == HAL_OK ? HAL_ADC_GetValue(&hadc1) : 0U;
    (void)HAL_ADC_Stop(&hadc1);
    if (status != HAL_OK || raw == 0U || raw >= 4095U) return false;
    float voltage = (float)raw * 3.3f / 4095.0f;
    float measured = (voltage - 0.76f) / 0.0025f + 25.0f;
    if (!isfinite(measured) || measured < -20.0f || measured > 100.0f) return false;
    *temperature_c = measured;
    return true;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc_handle) {
    if (adc_handle == NULL || adc_handle->Instance != ADC1) return;
    __HAL_RCC_ADC1_CLK_ENABLE();
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adc_handle) {
    if (adc_handle == NULL || adc_handle->Instance != ADC1) return;
    __HAL_RCC_ADC1_CLK_DISABLE();
}
