#ifndef BLDC_H
#define BLDC_H

#include <stdint.h>

uint8_t bldc_getMotorsEnable(void);
void bldc_setMotorsEnable(uint8_t enable);

#endif