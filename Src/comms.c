#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static volatile uint8_t uart_buf[100];
static volatile int16_t ch_buf[8];
//volatile char char_buf[300];

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope(void) {
  #if defined DEBUG_SERIAL_SERVOTERM && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    uart_buf[0] = 0xff;
    uart_buf[1] = CLAMP(ch_buf[0]+127, 0, 255);
    uart_buf[2] = CLAMP(ch_buf[1]+127, 0, 255);
    uart_buf[3] = CLAMP(ch_buf[2]+127, 0, 255);
    uart_buf[4] = CLAMP(ch_buf[3]+127, 0, 255);
    uart_buf[5] = CLAMP(ch_buf[4]+127, 0, 255);
    uart_buf[6] = CLAMP(ch_buf[5]+127, 0, 255);
    uart_buf[7] = CLAMP(ch_buf[6]+127, 0, 255);
    uart_buf[8] = CLAMP(ch_buf[7]+127, 0, 255);
    uart_buf[9] = '\n';

    #ifdef DEBUG_SERIAL_USART2
    if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)uart_buf, strLength);	 
    }
    #endif
    #ifdef DEBUG_SERIAL_USART3
    if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)uart_buf, strLength);	 
    }
    #endif
  #endif

  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    // memset((void *)(uintptr_t)uart_buf, 0, sizeof(uart_buf));
    int strLength;
    strLength = sprintf((char *)(uintptr_t)uart_buf,
                "1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i\r\n",
                ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);
                
    #ifdef DEBUG_SERIAL_USART2
    if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)uart_buf, strLength);	 
    }
    #endif
    #ifdef DEBUG_SERIAL_USART3
    if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)uart_buf, strLength);	 
    }
    #endif
  #endif


}

void consoleLog(char *message)
{
  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    #ifdef DEBUG_SERIAL_USART2
    if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)message, strlen((char *)(uintptr_t)message));	 
    }
    #endif
    #ifdef DEBUG_SERIAL_USART3
    if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, strlen((char *)(uintptr_t)message));	 
    }
    #endif
  #endif
}
