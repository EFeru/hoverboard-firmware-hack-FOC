#ifndef TRIKE_CONFIG_H
#define TRIKE_CONFIG_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRIKE_MODE_M1 1U
#define TRIKE_MODE_M2 2U
#define TRIKE_MODE_M3 3U

#define TRIKE_CONFIG_PAGE_A 0x0803F000UL
#define TRIKE_CONFIG_PAGE_B 0x0803F800UL
#define TRIKE_CONFIG_PAGE_SIZE 0x800UL

#define TRIKE_CAL_ZERO_VALID  0x0001U
#define TRIKE_CAL_RANGE_VALID 0x0002U

typedef struct {
  uint16_t zero;
  uint16_t minimum;
  uint16_t maximum;
  uint16_t flags;
} TrikeThrottleCalibration;

uint8_t trikeConfigLoadMode(void);
HAL_StatusTypeDef trikeConfigSaveMode(uint8_t mode);
uint8_t trikeConfigLoadThrottle(TrikeThrottleCalibration *calibration);
HAL_StatusTypeDef trikeConfigSaveThrottle(const TrikeThrottleCalibration *calibration);

#ifdef __cplusplus
}
#endif

#endif
