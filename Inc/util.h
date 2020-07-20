/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


// Rx Structures USART
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
  #ifdef CONTROL_IBUS    
    typedef struct{
      uint8_t  start;
      uint8_t  type; 
      uint8_t  channels[IBUS_NUM_CHANNELS*2];
      uint8_t  checksuml;
      uint8_t  checksumh;    
    } SerialCommand;
  #else
    typedef struct{
      uint16_t  start; 
      int16_t   steer;
      int16_t   speed;
      uint16_t  checksum;    
    } SerialCommand;
  #endif
#endif
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
    typedef struct{
      uint16_t	start;
      int16_t  	roll;
      int16_t  	pitch;
      int16_t  	yaw;
      uint16_t  sensors;
      uint16_t 	checksum;
    } SerialSideboard;
#endif

// Initialization Functions
void BLDC_Init(void);
void Input_Lim_Init(void);
void Input_Init(void);
void UART_DisableRxErrors(UART_HandleTypeDef *huart);

// General Functions
void poweronMelody(void);
void shortBeep(uint8_t freq);
void shortBeepMany(uint8_t cnt);
void longBeep(uint8_t freq);
void calcAvgSpeed(void);
void adcCalibLim(void);
void updateCurSpdLim(void);
void saveConfig(void);
int  addDeadBand(int16_t u, int16_t deadBand, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
void standstillHold(int16_t *speedCmd);
void electricBrake(uint16_t speedBlend, uint8_t reverseDir);

// Poweroff Functions
void poweroff(void);
void poweroffPressCheck(void);

// Read Command Function
void readCommand(void);
void usart2_rx_check(void);
void usart3_rx_check(void);
#if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
void usart_process_debug(uint8_t *userCommand, uint32_t len);
#endif
#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3)
void usart_process_command(SerialCommand *command_in, SerialCommand *command_out, uint8_t usart_idx);
#endif
#if defined(SIDEBOARD_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART3)
void usart_process_sideboard(SerialSideboard *Sideboard_in, SerialSideboard *Sideboard_out, uint8_t usart_idx);
#endif

// Sideboard functions
void sideboardLeds(uint8_t *leds);
void sideboardSensors(uint8_t sensors);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL);

// Multiple Tap Function
typedef struct {
  uint32_t 	t_timePrev;
  uint8_t 	z_pulseCntPrev;
  uint8_t 	b_hysteresis;
  uint8_t 	b_multipleTap;
} MultipleTap;
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x);

#endif

