#ifndef TEST_CBOARD_IMU_SPI_H
#define TEST_CBOARD_IMU_SPI_H
#include "main.h"
#include <stdbool.h>
extern SPI_HandleTypeDef hspi1;
bool SPI1_IsReady(void);
bool SPI1_SetFast(void);
#endif
