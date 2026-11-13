/**
  * RemoteUartBus + CANSimple over UART (VARIANT_UARTBUS), see Src/uartbus.c and docs/uartbus.md
  */
#ifndef UARTBUS_H
#define UARTBUS_H

#include <stdint.h>

void    uartbusRx(const uint8_t *data, uint32_t len);   // received bytes, any chunk size (USART interrupt)
void    uartbusUpdate(int16_t *cmdL, int16_t *cmdR);    // main loop: per motor command [-1000, 1000], control modes, watchdog
uint8_t uartbusActive(void);                            // 1 while a motor is armed and its set points arrive

#endif
