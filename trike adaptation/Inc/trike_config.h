#ifndef TRIKE_CONFIG_H
#define TRIKE_CONFIG_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRIKE_MODE_M1 1U
#define TRIKE_MODE_M2 2U
#define TRIKE_MODE_M3 3U

/* Two reserved 2-KB pages at the end of STM32F103RCT6 256-KB Flash. */
#define TRIKE_CONFIG_PAGE_A 0x0803F000UL
#define TRIKE_CONFIG_PAGE_B 0x0803F800UL
#define TRIKE_CONFIG_PAGE_SIZE 0x800UL

uint8_t trikeConfigLoadMode(void);
HAL_StatusTypeDef trikeConfigSaveMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif
