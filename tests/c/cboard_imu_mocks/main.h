#ifndef TEST_CBOARD_IMU_MAIN_H
#define TEST_CBOARD_IMU_MAIN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct { int unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;

extern GPIO_TypeDef test_gpio_a;
extern GPIO_TypeDef test_gpio_b;

#define GPIOA (&test_gpio_a)
#define GPIOB (&test_gpio_b)
#define GPIO_PIN_0 ((uint16_t)0x0001U)
#define GPIO_PIN_4 ((uint16_t)0x0010U)
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_GYRO_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef *handle, uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout);
void HAL_Delay(uint32_t delay_ms);

#endif
