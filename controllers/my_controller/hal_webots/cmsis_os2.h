#ifndef WEBOTS_CMSIS_OS2_H
#define WEBOTS_CMSIS_OS2_H

#include <stdint.h>

typedef enum { osOK = 0 } osStatus_t;

uint32_t osKernelGetTickFreq(void);
uint32_t osKernelGetTickCount(void);
osStatus_t osDelayUntil(uint32_t deadline);
osStatus_t osDelay(uint32_t ticks);
_Noreturn void osThreadExit(void);

#endif
