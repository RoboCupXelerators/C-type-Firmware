#ifndef __SPI_H__
#define __SPI_H__

#include <stdbool.h>
#include "main.h"

extern SPI_HandleTypeDef hspi1;

bool MX_SPI1_Init(void);
bool SPI1_SetFast(void);
bool SPI1_IsReady(void);

#endif
