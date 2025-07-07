/**
  **************************************************************************
  * @file     wk_system.h
  * @brief    workbench system header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __WK_SYSTEM_H
#define __WK_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "at32f413.h"

void wk_timebase_init(void);
void wk_delay_us(uint32_t delay);
void wk_delay_ms(uint32_t delay);

#ifdef __cplusplus
}
#endif

#endif
