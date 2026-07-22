#include "spi.h"

SPI_HandleTypeDef hspi1;
static bool spi1_ready;

bool MX_SPI1_Init(void) {
    spi1_ready = false;
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10U;
    spi1_ready = HAL_SPI_Init(&hspi1) == HAL_OK;
    return spi1_ready;
}

bool SPI1_SetFast(void) {
    if (!spi1_ready) return false;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spi1_ready = HAL_SPI_Init(&hspi1) == HAL_OK;
    return spi1_ready;
}

bool SPI1_IsReady(void) {
    return spi1_ready;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spi_handle) {
    if (spi_handle == NULL || spi_handle->Instance != SPI1) return;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spi_handle) {
    if (spi_handle == NULL || spi_handle->Instance != SPI1) return;
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
}
