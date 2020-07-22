/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
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

#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "comms.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "rtwtypes.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
#include "hd44780.h"
#endif

void SystemClock_Config(void);

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  extern LCD_PCF8574_HandleTypeDef lcd;
  extern uint8_t LCDerrorFlag;
#endif

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Matlab defines - from auto-code generation
//---------------
extern P    rtP_Left;                   /* Block parameters (auto storage) */
extern P    rtP_Right;                  /* Block parameters (auto storage) */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
//---------------

extern int16_t cmd1;                    // normalized input value. -1000 to 1000
extern int16_t cmd2;                    // normalized input value. -1000 to 1000

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern volatile uint32_t timeoutCnt;    // Timeout counter for the General timeout (PPM, PWM, Nunchuck)
extern uint8_t timeoutFlagADC;          // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlagSerial;       // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern int16_t batVoltage;              // global variable for battery voltage

#if defined(SIDEBOARD_SERIAL_USART2)
extern SerialSideboard Sideboard_L;
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
extern SerialSideboard Sideboard_R;
#endif
#if (defined(CONTROL_PPM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PPM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif
#if (defined(CONTROL_PWM_LEFT) && defined(DEBUG_SERIAL_USART3)) || (defined(CONTROL_PWM_RIGHT) && defined(DEBUG_SERIAL_USART2))
extern volatile uint16_t pwm_captured_ch1_value;
extern volatile uint16_t pwm_captured_ch2_value;
#endif


//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
uint8_t backwardDrive;
volatile uint32_t main_loop_counter;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t 	cmdLed;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

#ifdef VARIANT_TRANSPOTTER
  extern uint8_t  nunchuk_connected;
  extern float    setDistance;  

  static uint8_t  checkRemote = 0;
  static uint16_t distance;
  static float    steering;
  static int      distanceErr;  
  static int      lastDistance = 0;
  static uint16_t transpotter_counter = 0;
#endif

static int16_t    speed;                // local variable for speed. -1000 to 1000
#ifndef VARIANT_TRANSPOTTER
  static int16_t  steer;                // local variable for steering. -1000 to 1000
  static int16_t  steerRateFixdt;       // local fixed-point variable for steering rate limiter
  static int16_t  speedRateFixdt;       // local fixed-point variable for speed rate limiter
  static int32_t  steerFixdt;           // local fixed-point variable for steering low-pass filter
  static int32_t  speedFixdt;           // local fixed-point variable for speed low-pass filter
#endif

static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // define multiple tap functionality for the Brake pedal


int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();        // BLDC Controller Init
  Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);  

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int16_t speedL     = 0, speedR     = 0;
  int16_t lastSpeedL = 0, lastSpeedR = 0;

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;
  int16_t board_temp_deg_c;


  while(1) {
    HAL_Delay(DELAY_IN_MAIN_LOOP);        //delay in ms

    readCommand();                        // Read Command: cmd1, cmd2
    calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAbs

    #ifndef VARIANT_TRANSPOTTER
      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 && (!rtY_Left.z_errCode && !rtY_Right.z_errCode) && (cmd1 > -50 && cmd1 < 50) && (cmd2 > -50 && cmd2 < 50)){
        shortBeep(6);                     // make 2 beeps indicating the motor enable
        shortBeep(4); HAL_Delay(100);
        enable = 1;                       // enable motors
        consoleLog("-- Motors enabled --\r\n");
      }

      // ####### VARIANT_HOVERCAR ####### 
      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;                                        // Calculate speed Blend, a number between [0, 1] in fixdt(0,16,15)
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50); // speedBlend [0,1] is within [10 rpm, 60rpm]
      #endif

      #ifdef VARIANT_HOVERCAR        
        if (speedAvgAbs < 60) {                                     // Check if Hovercar is physically close to standstill to enable Double tap detection on Brake pedal for Reverse functionality
          multipleTapDet(cmd1, HAL_GetTick(), &MultipleTapBrake);   // Brake pedal in this case is "cmd1" variable
        }
        
        if (cmd1 > 30) {                                            // If Brake pedal (cmd1) is pressed, bring to 0 also the Throttle pedal (cmd2) to avoid "Double pedal" driving
          cmd2 = (int16_t)((cmd2 * speedBlend) >> 15);
        }
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE        
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);  // Apply Electric Brake. Only available and makes sense for TORQUE Mode
      #endif

      #ifdef VARIANT_HOVERCAR        
        if (speedAvg > 0) {                                         // Make sure the Brake pedal is opposite to the direction of motion AND it goes to 0 as we reach standstill (to avoid Reverse driving by Brake pedal) 
          cmd1 = (int16_t)((-cmd1 * speedBlend) >> 15);
        } else {
          cmd1 = (int16_t)(( cmd1 * speedBlend) >> 15);
        }
      #endif

      #ifdef VARIANT_SKATEBOARD
        if (cmd2 < 0) {                                           // When Throttle is negative, it acts as brake. This condition is to make sure it goes to 0 as we reach standstill (to avoid Reverse driving) 
          if (speedAvg > 0) {                                     // Make sure the braking is opposite to the direction of motion
            cmd2 = (int16_t)(( cmd2 * speedBlend) >> 15);
          } else {
            cmd2 = (int16_t)((-cmd2 * speedBlend) >> 15);
          }
        }
      #endif

      // ####### LOW-PASS FILTER #######
      rateLimiter16(cmd1, RATE, &steerRateFixdt);
      rateLimiter16(cmd2, RATE, &speedRateFixdt);
      filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      steer = (int16_t)(steerFixdt >> 16);  // convert fixed-point to integer
      speed = (int16_t)(speedFixdt >> 16);  // convert fixed-point to integer

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold(&speed);                 // Apply Standstill Hold functionality. Only available and makes sense for VOLTAGE or TORQUE Mode
      #endif

      // ####### VARIANT_HOVERCAR #######
      #ifdef VARIANT_HOVERCAR        
        if (!MultipleTapBrake.b_multipleTap) {  // Check driving direction
          speed = steer + speed;                // Forward driving: in this case steer = Brake, speed = Throttle
        } else {
          speed = steer - speed;                // Reverse driving: in this case steer = Brake, speed = Throttle
        }
      #endif

      // ####### MIXER #######
      // speedR = CLAMP((int)(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT), INPUT_MIN, INPUT_MA);
      // speedL = CLAMP((int)(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT), INPUT_MIN, INPUT_MA);
      mixerFcn(speed << 4, steer << 4, &speedR, &speedL);   // This function implements the equations above

      // ####### SET OUTPUTS (if the target change is less than +/- 100) #######
      if ((speedL > lastSpeedL-100 && speedL < lastSpeedL+100) && (speedR > lastSpeedR-100 && speedR < lastSpeedR+100)) {
        #ifdef INVERT_R_DIRECTION
          pwmr = speedR;
        #else
          pwmr = -speedR;
        #endif
        #ifdef INVERT_L_DIRECTION
          pwml = -speedL;
        #else
          pwml = speedL;
        #endif
      }
    #endif

    #ifdef VARIANT_TRANSPOTTER
      distance    = CLAMP(cmd1 - 180, 0, 4095);
      steering    = (cmd2 - 2048) / 2048.0;
      distanceErr = distance - (int)(setDistance * 1345);

      if (nunchuk_connected == 0) {
        speedL = speedL * 0.8f + (CLAMP(distanceErr + (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        speedR = speedR * 0.8f + (CLAMP(distanceErr - (steering*((float)MAX(ABS(distanceErr), 50)) * ROT_P), -850, 850) * -0.2f);
        if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50)) {
          if (distanceErr > 0) {
            enable = 1;
          }
          if (distanceErr > -300) {
            #ifdef INVERT_R_DIRECTION
              pwmr = speedR;
            #else
              pwmr = -speedR;
            #endif
            #ifdef INVERT_L_DIRECTION
              pwml = -speedL;
            #else
              pwml = speedL;
            #endif

            if (checkRemote) {
              if (!HAL_GPIO_ReadPin(LED_PORT, LED_PIN)) {
                //enable = 1;
              } else {
                enable = 0;
              }
            }
          } else {
            enable = 0;
          }
        }
        timeoutCnt = 0;
      }

      if (timeoutCnt > TIMEOUT) {
        pwml = 0;
        pwmr = 0;
        enable = 0;
        #ifdef SUPPORT_LCD
          LCD_SetLocation(&lcd,  0, 0); LCD_WriteString(&lcd, "Len:");
          LCD_SetLocation(&lcd,  8, 0); LCD_WriteString(&lcd, "m(");
          LCD_SetLocation(&lcd, 14, 0); LCD_WriteString(&lcd, "m)");
        #endif
        HAL_Delay(1000);
        nunchuk_connected = 0;
      }

      if ((distance / 1345.0) - setDistance > 0.5 && (lastDistance / 1345.0) - setDistance > 0.5) { // Error, robot too far away!
        enable = 0;
        longBeep(5);
        #ifdef SUPPORT_LCD
          LCD_ClearDisplay(&lcd);
          HAL_Delay(5);
          LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Emergency Off!");
          LCD_SetLocation(&lcd, 0, 1); LCD_WriteString(&lcd, "Keeper too fast.");
        #endif
        poweroff();
      }

      #ifdef SUPPORT_NUNCHUK
        if (transpotter_counter % 500 == 0) {
          if (nunchuk_connected == 0 && enable == 0) {
            if (Nunchuk_Ping()) {
              HAL_Delay(500);
              Nunchuk_Init();
              #ifdef SUPPORT_LCD
                LCD_SetLocation(&lcd, 0, 0); LCD_WriteString(&lcd, "Nunchuk Control");
              #endif
              timeoutCnt = 0;
              HAL_Delay(1000);
              nunchuk_connected = 1;
            }
          }
        }   
      #endif

      #ifdef SUPPORT_LCD
        if (transpotter_counter % 100 == 0) {
          if (LCDerrorFlag == 1 && enable == 0) {

          } else {
            if (nunchuk_connected == 0) {
              LCD_SetLocation(&lcd,  4, 0); LCD_WriteFloat(&lcd,distance/1345.0,2);
              LCD_SetLocation(&lcd, 10, 0); LCD_WriteFloat(&lcd,setDistance,2);
            }
            LCD_SetLocation(&lcd,  4, 1); LCD_WriteFloat(&lcd,batVoltage, 1);
            // LCD_SetLocation(&lcd, 11, 1); LCD_WriteFloat(&lcd,MAX(ABS(currentR), ABS(currentL)),2);
          }
        }
      #endif
      transpotter_counter++;
    #endif

    // ####### SIDEBOARDS HANDLING #######
    #if defined(SIDEBOARD_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### DEBUG SERIAL OUT #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
      if (main_loop_counter % 25 == 0) {    // Send data periodically every 125 ms
        #ifdef CONTROL_ADC
        setScopeChannel(0, (int16_t)adc_buffer.l_tx2);          // 1: ADC1
        setScopeChannel(1, (int16_t)adc_buffer.l_rx2);          // 2: ADC2
        #endif
        #if defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)
        setScopeChannel(0, ppm_captured_value[0]);              // 1: CH1
        setScopeChannel(1, ppm_captured_value[1]);              // 2: CH2
        #endif
        #if defined(CONTROL_PWM_LEFT) || defined(CONTROL_PWM_RIGHT)
        setScopeChannel(0, (pwm_captured_ch1_value - 500) * 2); // 1: CH1
        setScopeChannel(1, (pwm_captured_ch2_value - 500) * 2); // 2: CH2
        #endif
        setScopeChannel(2, (int16_t)speedR);                    // 3: output command: [-1000, 1000]
        setScopeChannel(3, (int16_t)speedL);                    // 4: output command: [-1000, 1000]
        setScopeChannel(4, (int16_t)adc_buffer.batt1);          // 5: for battery voltage calibration
        setScopeChannel(5, (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC)); // 6: for verifying battery voltage calibration
        setScopeChannel(6, (int16_t)board_temp_adcFilt);        // 7: for board temperature calibration
        setScopeChannel(7, (int16_t)board_temp_deg_c);          // 8: for verifying board temperature calibration
        consoleScope();
      }
    #endif

    // ####### FEEDBACK SERIAL OUT #######
    #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
      if (main_loop_counter % 2 == 0) {    // Send data periodically every 10 ms
        Feedback.start	        = (uint16_t)SERIAL_START_FRAME;
        Feedback.cmd1           = (int16_t)cmd1;
        Feedback.cmd2           = (int16_t)cmd2;
        Feedback.speedR_meas	  = (int16_t)rtY_Right.n_mot;
        Feedback.speedL_meas	  = (int16_t)rtY_Left.n_mot;
        Feedback.batVoltage	    = (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC);
        Feedback.boardTemp	    = (int16_t)board_temp_deg_c;

        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            Feedback.cmdLed         = (uint16_t)sideboard_leds_L;
            Feedback.checksum       = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                               ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            Feedback.cmdLed         = (uint16_t)sideboard_leds_R;
            Feedback.checksum       = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                               ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif            
      }
    #endif

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {     // disable motors and beep in case of Motor error - fast beep
      enable        = 0;
      buzzerFreq    = 8;
      buzzerPattern = 1;
    } else if (timeoutFlagADC || timeoutFlagSerial || timeoutCnt > TIMEOUT) { // beep in case of ADC timeout, Serial timeout or General timeout - fast beep      
      buzzerFreq    = 24;
      buzzerPattern = 1;
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzzerFreq    = 4;
      buzzerPattern = 1;
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {      // low bat 1: fast beep
      buzzerFreq    = 5;
      buzzerPattern = 6;
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {      // low bat 2: slow beep
      buzzerFreq    = 5;
      buzzerPattern = 42;
    } else if (BEEPS_BACKWARD && ((speed < -50 && speedAvg < 0) || MultipleTapBrake.b_multipleTap)) {  // backward beep
      buzzerFreq    = 5;
      buzzerPattern = 1;
      backwardDrive = 1;
    } else {  // do not beep
      buzzerFreq    = 0;
      buzzerPattern = 0;
      backwardDrive = 0;
    }


    // ####### INACTIVITY TIMEOUT #######
    if (abs(speedL) > 50 || abs(speedR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      poweroff();
    }

    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN
    // Update main loop states
    lastSpeedL = speedL;
    lastSpeedR = speedR;
    main_loop_counter++;
    timeoutCnt++;
  }
}


// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
