#ifndef WEBOTS_STM32F4XX_HAL_H
#define WEBOTS_STM32F4XX_HAL_H

#include <stdint.h>

/* The firmware interface passes these handles by pointer only. */
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;

#endif
