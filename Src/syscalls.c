#include <reent.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"

extern volatile uint8_t uart_buf[200];

/*
* printf sends its output to this function, this function sends it to the uart dma output buffer
*/
__attribute__((__used__)) int _write(int fd, const char *ptr, int len){
  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    #ifdef DEBUG_SERIAL_USART2
      while(DMA1_Channel7->CNDTR != 0);  // wait
      memcpy(uart_buf,ptr,len);  // copy to buffer
      DMA1_Channel7->CCR    &= ~DMA_CCR_EN;
      DMA1_Channel7->CNDTR   = len;  // set number of bytes to read
      DMA1_Channel7->CMAR    = (uint32_t)uart_buf;  // set buffer to read from
      DMA1_Channel7->CCR    |= DMA_CCR_EN;
    #endif
    #ifdef DEBUG_SERIAL_USART3
      while(DMA1_Channel2->CNDTR != 0);  // wait
      memcpy(uart_buf,ptr,len);  // copy to buffer
      DMA1_Channel2->CCR    &= ~DMA_CCR_EN;
      DMA1_Channel2->CNDTR   = len;  // set number of bytes to read
      DMA1_Channel2->CMAR    = (uint32_t)uart_buf;  // set buffer to read from
      DMA1_Channel2->CCR    |= DMA_CCR_EN;
    #endif
  #endif
  return len;
}
