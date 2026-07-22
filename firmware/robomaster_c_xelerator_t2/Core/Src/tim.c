#include "tim.h"

TIM_HandleTypeDef htim10;
static bool imu_heater_ready;

bool MX_TIM10_Init(void) {
    imu_heater_ready = false;
    TIM_OC_InitTypeDef output = {0};
    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 0U;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 4999U;
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) return false;
    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0U;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &output, TIM_CHANNEL_1) != HAL_OK) return false;
    imu_heater_ready = HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) == HAL_OK;
    return imu_heater_ready;
}

void TIM10_IMU_Heater_Set(uint16_t pwm) {
    if (!imu_heater_ready) return;
    if (pwm > 4999U) pwm = 4999U;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}

bool TIM10_IMU_Heater_IsReady(void) {
    return imu_heater_ready;
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_handle) {
    if (tim_handle == NULL || tim_handle->Instance != TIM10) return;
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOF, &gpio);
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_handle) {
    if (tim_handle == NULL || tim_handle->Instance != TIM10) return;
    __HAL_RCC_TIM10_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6);
}
