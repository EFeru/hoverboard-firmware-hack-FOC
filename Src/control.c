
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
uint8_t ppm_count = 0;
uint8_t pwm_count = 0;
uint32_t timeout = 100;
uint8_t nunchuk_data[6] = {0};

uint8_t i2cBuffer[2];

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

#ifdef CONTROL_PPM
uint16_t ppm_captured_value[PPM_NUM_CHANNELS + 1] = {500, 500};
uint16_t ppm_captured_value_buffer[PPM_NUM_CHANNELS+1] = {500, 500};
uint32_t ppm_timeout = 0;

bool ppm_valid = true;

void PPM_ISR_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_delay = TIM2->CNT;
  TIM2->CNT = 0;

  if (rc_delay > 3000) {
    if (ppm_valid && ppm_count == PPM_NUM_CHANNELS) {
      ppm_timeout = 0;
      memcpy(ppm_captured_value, ppm_captured_value_buffer, sizeof(ppm_captured_value));
    }
    ppm_valid = true;
    ppm_count = 0;
  }
  else if (ppm_count < PPM_NUM_CHANNELS && IN_RANGE(rc_delay, 900, 2100)){
    timeout = 0;
    ppm_captured_value_buffer[ppm_count++] = CLAMP(rc_delay, 1000, 2000) - 1000;
  } else {
    ppm_valid = false;
  }
}

// SysTick executes once each ms
void PPM_SysTick_Callback(void) {
  ppm_timeout++;
  // Stop after 500 ms without PPM signal
  if(ppm_timeout > 500) {
    int i;
    for(i = 0; i < PPM_NUM_CHANNELS; i++) {
      ppm_captured_value[i] = 500;
    }
    ppm_timeout = 0;
  }
}

void PPM_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period = UINT16_MAX;
  TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);
}
#endif


#ifdef CONTROL_PWM
uint16_t pwm_captured_ch1_value = 500;
uint16_t pwm_captured_ch2_value = 500;
uint32_t pwm_timeout_ch1 = 0;
uint32_t pwm_timeout_ch2 = 0;

void PWM_ISR_CH1_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_signal = TIM3->CNT;
  TIM3->CNT = 0;

  if (IN_RANGE(rc_signal, 900, 2100)){
    timeout = 0;
    pwm_timeout_ch1 = 0;
    pwm_captured_ch1_value = CLAMP(rc_signal, 1000, 2000) - 1000;
  }
}


void PWM_ISR_CH2_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_signal = TIM2->CNT;
  TIM2->CNT = 0;

  if (IN_RANGE(rc_signal, 900, 2100)){
    timeout = 0;
    pwm_timeout_ch2 = 0;
    pwm_captured_ch2_value = CLAMP(rc_signal, 1000, 2000) - 1000;
  }
}

// SysTick executes once each ms
void PWM_SysTick_Callback(void) {
  pwm_timeout_ch1++;
  pwm_timeout_ch2++;
  // Stop after 500 ms without PWM signal
  if(pwm_timeout_ch1 > 500) {
    pwm_captured_ch1_value = 500;
    pwm_timeout_ch1 = 500;          // limit the timeout to max timeout value of 500 ms
  }
  if(pwm_timeout_ch2 > 500) {
    pwm_captured_ch2_value = 500;
    pwm_timeout_ch2 = 500;          // limit the timeout to max timeout value of 500 ms
  }
}

void PWM_Init(void) {
  // Channel 1 (steering)
  GPIO_InitTypeDef GPIO_InitStruct2;
  // Configure GPIO pin : PA2
  GPIO_InitStruct2.Pin          = GPIO_PIN_2;
  GPIO_InitStruct2.Mode         = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct2.Speed        = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct2.Pull         = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);

  __HAL_RCC_TIM3_CLK_ENABLE();
  TimHandle2.Instance           = TIM3;
  TimHandle2.Init.Period        = UINT16_MAX;
  TimHandle2.Init.Prescaler     = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle2);

  // EXTI interrupt init
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_TIM_Base_Start(&TimHandle2);

  // Channel 2 (speed)
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin             = GPIO_PIN_3;
  GPIO_InitStruct.Mode            = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Speed           = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull            = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance              = TIM2;
  TimHandle.Init.Period           = UINT16_MAX;
  TimHandle.Init.Prescaler        = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision    = 0;
  TimHandle.Init.CounterMode      = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);

  #ifdef SUPPORT_BUTTONS  
  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin             = BUTTON1_RIGHT_PIN;
  GPIO_InitStruct.Mode            = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed           = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Pull            = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON1_RIGHT_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct2.Pin            = BUTTON2_RIGHT_PIN;
  GPIO_InitStruct2.Mode           = GPIO_MODE_INPUT;
  GPIO_InitStruct2.Speed          = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct2.Pull           = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON2_RIGHT_PORT, &GPIO_InitStruct2);
  #endif
}
#endif

uint8_t Nunchuk_Ping(void) {
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuk_data, 1, 10) == HAL_OK) {
    return 1;
  }
  return 0;
}

void Nunchuk_Init(void) {
    //-- START -- init WiiNunchuk
  i2cBuffer[0] = 0xF0;
  i2cBuffer[1] = 0x55;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);

  i2cBuffer[0] = 0xFB;
  i2cBuffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);
}

void Nunchuk_Read(void) {
  i2cBuffer[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 1, 10);
  HAL_Delay(3);
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuk_data, 6, 10) == HAL_OK) {
    timeout = 0;
  }

  #ifndef TRANSPOTTER
    if (timeout > 3) {
      HAL_Delay(50);
      Nunchuk_Init();
    }
  #endif

  //setScopeChannel(0, (int)nunchuk_data[0]);
  //setScopeChannel(1, (int)nunchuk_data[1]);
  //setScopeChannel(2, (int)nunchuk_data[5] & 1);
  //setScopeChannel(3, ((int)nunchuk_data[5] >> 1) & 1);
}
