#ifndef __BOARD_1_H__
#define __BOARD_1_H__

//
// Alternate board type with different pin mapping for DCLINK, Buzzer and ON/OFF, Button and Charger
//

#include "config.h"


#define LEFT_HALL_U_PIN         GPIO_PIN_5
#define LEFT_HALL_V_PIN         GPIO_PIN_6
#define LEFT_HALL_W_PIN         GPIO_PIN_7

#define LEFT_HALL_U_PORT        GPIOB
#define LEFT_HALL_V_PORT        GPIOB
#define LEFT_HALL_W_PORT        GPIOB


#define RIGHT_HALL_U_PIN        GPIO_PIN_10
#define RIGHT_HALL_V_PIN        GPIO_PIN_11
#define RIGHT_HALL_W_PIN        GPIO_PIN_12

#define RIGHT_HALL_U_PORT       GPIOC
#define RIGHT_HALL_V_PORT       GPIOC
#define RIGHT_HALL_W_PORT       GPIOC


#define LEFT_TIM                TIM8

#define LEFT_TIM_U              CCR1
#define LEFT_TIM_UH_PIN         GPIO_PIN_6
#define LEFT_TIM_UH_PORT        GPIOC
#define LEFT_TIM_UL_PIN         GPIO_PIN_7
#define LEFT_TIM_UL_PORT        GPIOA

#define LEFT_TIM_V              CCR2
#define LEFT_TIM_VH_PIN         GPIO_PIN_7
#define LEFT_TIM_VH_PORT        GPIOC
#define LEFT_TIM_VL_PIN         GPIO_PIN_0
#define LEFT_TIM_VL_PORT        GPIOB

#define LEFT_TIM_W              CCR3
#define LEFT_TIM_WH_PIN         GPIO_PIN_8
#define LEFT_TIM_WH_PORT        GPIOC
#define LEFT_TIM_WL_PIN         GPIO_PIN_1
#define LEFT_TIM_WL_PORT        GPIOB


#define RIGHT_TIM TIM1

#define RIGHT_TIM_U             CCR1
#define RIGHT_TIM_UH_PIN        GPIO_PIN_8
#define RIGHT_TIM_UH_PORT       GPIOA
#define RIGHT_TIM_UL_PIN        GPIO_PIN_13
#define RIGHT_TIM_UL_PORT       GPIOB

#define RIGHT_TIM_V             CCR2
#define RIGHT_TIM_VH_PIN        GPIO_PIN_9
#define RIGHT_TIM_VH_PORT       GPIOA
#define RIGHT_TIM_VL_PIN        GPIO_PIN_14
#define RIGHT_TIM_VL_PORT       GPIOB

#define RIGHT_TIM_W             CCR3
#define RIGHT_TIM_WH_PIN        GPIO_PIN_10
#define RIGHT_TIM_WH_PORT       GPIOA
#define RIGHT_TIM_WL_PIN        GPIO_PIN_15
#define RIGHT_TIM_WL_PORT       GPIOB


// #define LEFT_DC_CUR_ADC ADC1
// #define LEFT_U_CUR_ADC ADC1
// #define LEFT_V_CUR_ADC ADC1

#define LEFT_DC_CUR_PIN         GPIO_PIN_0
#define LEFT_U_CUR_PIN          GPIO_PIN_0
#define LEFT_V_CUR_PIN          GPIO_PIN_3

#define LEFT_DC_CUR_PORT        GPIOC
#define LEFT_U_CUR_PORT         GPIOA
#define LEFT_V_CUR_PORT         GPIOC


// #define RIGHT_DC_CUR_ADC ADC2
// #define RIGHT_U_CUR_ADC ADC2
// #define RIGHT_V_CUR_ADC ADC2

#define RIGHT_DC_CUR_PIN        GPIO_PIN_1
#define RIGHT_U_CUR_PIN         GPIO_PIN_4
#define RIGHT_V_CUR_PIN         GPIO_PIN_5

#define RIGHT_DC_CUR_PORT       GPIOC
#define RIGHT_U_CUR_PORT        GPIOC
#define RIGHT_V_CUR_PORT        GPIOC


// #define DCLINK_ADC ADC3
// #define DCLINK_CHANNEL

#define DCLINK_PIN              GPIO_PIN_1
#define DCLINK_PORT             GPIOA

// #define DCLINK_PULLUP        30000
// #define DCLINK_PULLDOWN      1000

#define LED_PIN                 GPIO_PIN_2
#define LED_PORT                GPIOB
#define LED_ON_LEVEL            GPIO_PIN_SET
#define LED_OFF_LEVEL           GPIO_PIN_RESET

#define BUZZER_PIN              GPIO_PIN_13
#define BUZZER_PORT             GPIOC

#define OFF_PIN                 GPIO_PIN_15
#define OFF_PORT                GPIOC

#define BUTTON_PIN              GPIO_PIN_9
#define BUTTON_PORT             GPIOB

#define CHARGER_PIN             GPIO_PIN_11
#define CHARGER_PORT            GPIOA





#if defined(CONTROL_PPM_LEFT)
#define PPM_PIN             GPIO_PIN_3
#define PPM_PORT            GPIOA
#elif defined(CONTROL_PPM_RIGHT)
#define PPM_PIN             GPIO_PIN_11
#define PPM_PORT            GPIOB
#endif

#if defined(CONTROL_PWM_LEFT)
#define PWM_PIN_CH1         GPIO_PIN_2
#define PWM_PORT_CH1        GPIOA
#define PWM_PIN_CH2         GPIO_PIN_3
#define PWM_PORT_CH2        GPIOA
#elif defined(CONTROL_PWM_RIGHT)
#define PWM_PIN_CH1         GPIO_PIN_10
#define PWM_PORT_CH1        GPIOB
#define PWM_PIN_CH2         GPIO_PIN_11
#define PWM_PORT_CH2        GPIOB
#endif

#if defined(SUPPORT_BUTTONS_LEFT)
#define BUTTON1_PIN         GPIO_PIN_2
#define BUTTON1_PORT        GPIOA
#define BUTTON2_PIN         GPIO_PIN_3
#define BUTTON2_PORT        GPIOA
#elif defined(SUPPORT_BUTTONS_RIGHT)
#define BUTTON1_PIN         GPIO_PIN_10
#define BUTTON1_PORT        GPIOB
#define BUTTON2_PIN         GPIO_PIN_11
#define BUTTON2_PORT        GPIOB
#endif


#endif // __BOARD_1_H__
