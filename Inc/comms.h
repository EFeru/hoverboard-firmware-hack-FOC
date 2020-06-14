#pragma once

#define SERIAL_USART_BUFFER_SIZE 1024 // TODO: implement send_wait routine..
typedef struct tag_serial_usart_buffer {
    SERIAL_USART_IT_BUFFERTYPE buff[SERIAL_USART_BUFFER_SIZE];
    int head;
    int tail;

    // count of buffer overflows
    unsigned int overflow;

} SERIAL_USART_BUFFER;

#if defined(SERIAL_USART2_IT)

    extern volatile SERIAL_USART_BUFFER usart2_it_TXbuffer;
    extern volatile SERIAL_USART_BUFFER usart2_it_RXbuffer;

    int   USART2_IT_starttx();
    int   USART2_IT_send(unsigned char *data, int len);
    void  USART2_IT_IRQ(USART_TypeDef *us);

#endif

#if defined(SERIAL_USART3_IT)

    extern volatile SERIAL_USART_BUFFER usart3_it_TXbuffer;
    extern volatile SERIAL_USART_BUFFER usart3_it_RXbuffer;

    int   USART3_IT_starttx();
    int   USART3_IT_send(unsigned char *data, int len);
    void  USART3_IT_IRQ(USART_TypeDef *us);

#endif

int                        serial_usart_buffer_count(volatile SERIAL_USART_BUFFER *usart_buf);
void                       serial_usart_buffer_push (volatile SERIAL_USART_BUFFER *usart_buf, SERIAL_USART_IT_BUFFERTYPE value);
SERIAL_USART_IT_BUFFERTYPE serial_usart_buffer_pop  (volatile SERIAL_USART_BUFFER *usart_buf);



void setScopeChannel(uint8_t ch, int16_t val);
void consoleScope();
/////////////////////////////////////////////////////////


#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))