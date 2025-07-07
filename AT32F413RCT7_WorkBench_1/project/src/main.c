/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f413_wk_config.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/**
 * @brief Отправляет символ через USART2
 * @param ch: символ для отправки
 * @retval none
 */
//void usart2_send_char(uint8_t ch);

/**
 * @brief Отправляет строку через USART2
 * @param str: указатель на строку для отправки
 * @retval none
 */
//void usart2_send_string(const char* str);

/**
 * @brief Отправляет символ через USART3
 * @param ch: символ для отправки
 * @retval none
 */
void usart3_send_char(uint8_t ch);

/**
 * @brief Отправляет строку через USART3
 * @param str: указатель на строку для отправки
 * @retval none
 */
void usart3_send_string(const char* str);

/**
 * @brief Генерирует простой beep звук
 * @param frequency: частота в Гц
 * @param duration: длительность в мс
 * @retval none
 */
void speaker_beep(uint32_t frequency, uint32_t duration);

/**
 * @brief Короткий beep
 * @param none
 * @retval none
 */
void speaker_short_beep(void);

/**
 * @brief Длинный beep
 * @param none
 * @retval none
 */
void speaker_long_beep(void);

/**
 * @brief Выводит информацию о частотах системы
 * @param none
 * @retval none
 */
void print_system_info(void);

/**
 * @brief Выводит информацию о температуре и напряжении
 * @param none
 * @retval none
 */
void print_adc_info(void);

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/**
 * @brief Отправляет символ через USART2
 * @param ch: символ для отправки
 * @retval none
 */
//void usart2_send_char(uint8_t ch)
//{
//  /* Ждем, пока буфер передачи не будет пуст */
//  while(usart_flag_get(USART2, USART_TDBE_FLAG) == RESET);
//  
//  /* Отправляем символ */
//  usart_data_transmit(USART2, (uint16_t)ch);
//  
//  /* Ждем, пока передача не завершится */
//  while(usart_flag_get(USART2, USART_TDC_FLAG) == RESET);
//}

/**
 * @brief Отправляет строку через USART2
 * @param str: указатель на строку для отправки
 * @retval none
 */
//void usart2_send_string(const char* str)
//{
//  while(*str)
//  {
//    usart2_send_char(*str++);
//  }
//}

/**
 * @brief Отправляет символ через USART3
 * @param ch: символ для отправки
 * @retval none
 */
void usart3_send_char(uint8_t ch)
{
  /* Ждем, пока буфер передачи не будет пуст */
  while(usart_flag_get(USART3, USART_TDBE_FLAG) == RESET);
  
  /* Отправляем символ */
  usart_data_transmit(USART3, (uint16_t)ch);
  
  /* Ждем, пока передача не завершится */
  while(usart_flag_get(USART3, USART_TDC_FLAG) == RESET);
}

/**
 * @brief Отправляет строку через USART3
 * @param str: указатель на строку для отправки
 * @retval none
 */
void usart3_send_string(const char* str)
{
  while(*str)
  {
    usart3_send_char(*str++);
  }
}

/**
 * @brief Генерирует простой beep звук
 * @param frequency: частота в Гц
 * @param duration: длительность в мс
 * @retval none
 */
void speaker_beep(uint32_t frequency, uint32_t duration)
{
  uint32_t half_period_us = 500000 / frequency; // полпериода в микросекундах
  uint32_t cycles = (duration * 1000) / (half_period_us * 2);
  
	uint32_t i;
  for(i = 0; i < cycles; i++)
  {
    gpio_bits_set(GPIOA, GPIO_PINS_4);    // HIGH
    wk_delay_us(half_period_us);
    gpio_bits_reset(GPIOA, GPIO_PINS_4);  // LOW
    wk_delay_us(half_period_us);
  }
}

/**
 * @brief Короткий beep
 * @param none
 * @retval none
 */
void speaker_short_beep(void)
{
  speaker_beep(1200, 50); // 1.2 кГц, 50 мс
}

/**
 * @brief Длинный beep
 * @param none
 * @retval none
 */
void speaker_long_beep(void)
{
  speaker_beep(1500, 100); // 1.5 кГц, 100 мс
}

/**
 * @brief Выводит информацию о частотах системы
 * @param none
 * @retval none
 */
void print_system_info(void)
{
  crm_clocks_freq_type crm_clocks;
  char buffer[128];
  
  /* Получаем информацию о частотах */
  crm_clocks_freq_get(&crm_clocks);
  
  usart3_send_string("\r\n=== SYSTEM CLOCK INFORMATION ===\r\n");
  
  sprintf(buffer, "System Clock (SCLK): %u MHz\r\n", (unsigned int)(crm_clocks.sclk_freq / 1000000));
  usart3_send_string(buffer);
  
  sprintf(buffer, "AHB Bus Clock: %u MHz\r\n", (unsigned int)(crm_clocks.ahb_freq / 1000000));
  usart3_send_string(buffer);
  
  sprintf(buffer, "APB1 Bus Clock: %u MHz\r\n", (unsigned int)(crm_clocks.apb1_freq / 1000000));
  usart3_send_string(buffer);
  
  sprintf(buffer, "APB2 Bus Clock: %u MHz\r\n", (unsigned int)(crm_clocks.apb2_freq / 1000000));
  usart3_send_string(buffer);
  
  sprintf(buffer, "ADC Clock: %u MHz\r\n", (unsigned int)(crm_clocks.adc_freq / 1000000));
  usart3_send_string(buffer);
  
  /* Информация о источнике тактирования */
  crm_sclk_type sclk_source = crm_sysclk_switch_status_get();
  usart3_send_string("System Clock Source: ");
  switch(sclk_source)
  {
    case CRM_SCLK_HICK:
      usart3_send_string("HICK (Internal)\r\n");
      break;
    case CRM_SCLK_HEXT:
      usart3_send_string("HEXT (External Crystal)\r\n");
      break;
    case CRM_SCLK_PLL:
      usart3_send_string("PLL\r\n");
      break;
    default:
      usart3_send_string("Unknown\r\n");
      break;
  }
  
  /* Информация о флагах стабильности */
  usart3_send_string("Clock Status: ");
  if(crm_flag_get(CRM_HICK_STABLE_FLAG) == SET)
    usart3_send_string("HICK Stable, ");
  if(crm_flag_get(CRM_HEXT_STABLE_FLAG) == SET)
    usart3_send_string("HEXT Stable, ");
  if(crm_flag_get(CRM_PLL_STABLE_FLAG) == SET)
    usart3_send_string("PLL Stable, ");
  usart3_send_string("\r\n");
  
  usart3_send_string("==============================\r\n");
}

/**
 * @brief Выводит информацию о температуре и напряжении
 * @param none
 * @retval none
 */
void print_adc_info(void)
{
  uint16_t adc_value;
  char buffer[128];
  
  usart3_send_string("\r\n=== ADC INFORMATION ===\r\n");
  
  /* Читаем ADC1 канал 11 (температура) */
  /* Запускаем конвертацию программным триггером */
  adc_ordinary_software_trigger_enable(ADC1, TRUE);
  
  /* Ждем завершения конвертации */
  while(adc_flag_get(ADC1, ADC_CCE_FLAG) == RESET);
  
  adc_value = adc_ordinary_conversion_data_get(ADC1);
  
  sprintf(buffer, "ADC1 Channel 11: %u (0x%04X)\r\n", adc_value, adc_value);
  usart3_send_string(buffer);
  
  /* Читаем ADC2 канал 10 */
  /* Запускаем конвертацию программным триггером */
  adc_ordinary_software_trigger_enable(ADC2, TRUE);
  
  /* Ждем завершения конвертации */
  while(adc_flag_get(ADC2, ADC_CCE_FLAG) == RESET);
  
  adc_value = adc_ordinary_conversion_data_get(ADC2);
  
  sprintf(buffer, "ADC2 Channel 10: %u (0x%04X)\r\n", adc_value, adc_value);
  usart3_send_string(buffer);
  
  usart3_send_string("=====================\r\n");
}

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config. */
  wk_timebase_init();

  /* init gpio function. */
  wk_gpio_config();

  /* usart1 already supports printf. */
  /* init usart1 function. */
  wk_usart1_init();

  /* init usart3 function. */
  wk_usart3_init();

  /* init adc1 function. */
  wk_adc1_init();

  /* init adc2 function. */
  wk_adc2_init();

  /* add user code begin 2 */
  
  /* Отправляем тестовое сообщение через USART3 */
  usart3_send_string("USART3 initialized successfully!\r\n");
  
  /* Выводим информацию о системе */
  print_system_info();
  
  /* Выводим информацию об ADC */
  print_adc_info();
  
  /* Воспроизводим тестовые beep звуки */
  speaker_short_beep();
  speaker_long_beep();
  
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    
    /* Отправляем символ через USART3 */
//    usart3_send_char('3');
    
    /* Воспроизводим beep каждые 5 секунд */
    
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */
// Примеры использования:

// Вывод системной информации
// print_system_info();

// Вывод информации об ADC
// print_adc_info();

// Короткий beep
// speaker_short_beep();

// Длинный beep
// speaker_long_beep();

// Кастомный beep
// speaker_beep(2000, 300); // 2 кГц, 300 мс

// Серия beep звуков
// for(int i = 0; i < 3; i++)
// {
//   speaker_short_beep();
//   wk_delay_ms(200);
// }

// Отправить один символ через USART3
// usart3_send_char('X');

// Отправить строку через USART3
// usart3_send_string("Hello World!\r\n");

// Отправить число как строку через USART3
// char buffer[16];
// sprintf(buffer, "Value: %d\r\n", 123);
// usart3_send_string(buffer);
  /* add user code end 4 */
