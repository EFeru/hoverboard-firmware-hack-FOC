#pragma once
#include "stm32f1xx_hal.h"

// ############################### DO-NOT-TOUCH SETTINGS ###############################

#define PWM_FREQ            16000     // PWM frequency in Hz
#define DEAD_TIME              32     // PWM deadtime
#define DELAY_IN_MAIN_LOOP      5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#define TIMEOUT                 5     // number of wrong / missing input commands before emergency off

// ADC conversion time definitions
#define ADC_CONV_TIME_1C5       (14)  //Total ADC clock cycles / conversion = (  1.5+12.5)
#define ADC_CONV_TIME_7C5       (20)  //Total ADC clock cycles / conversion = (  7.5+12.5)
#define ADC_CONV_TIME_13C5      (26)  //Total ADC clock cycles / conversion = ( 13.5+12.5)
#define ADC_CONV_TIME_28C5      (41)  //Total ADC clock cycles / conversion = ( 28.5+12.5)
#define ADC_CONV_TIME_41C5      (54)  //Total ADC clock cycles / conversion = ( 41.5+12.5)
#define ADC_CONV_TIME_55C5      (68)  //Total ADC clock cycles / conversion = ( 55.5+12.5)
#define ADC_CONV_TIME_71C5      (84)  //Total ADC clock cycles / conversion = ( 71.5+12.5)
#define ADC_CONV_TIME_239C5     (252) //Total ADC clock cycles / conversion = (239.5+12.5)

// This settings influences the actual sample-time. Only use definitions above
// This parameter needs to be the same as the ADC conversion for Current Phase of the FIRST Motor in setup.c
#define ADC_CONV_CLOCK_CYCLES   (ADC_CONV_TIME_7C5)

// Set the configured ADC divider. This parameter needs to be the same ADC divider as PeriphClkInit.AdcClockSelection (see main.c)
#define ADC_CLOCK_DIV           (4)

// ADC Total conversion time: this will be used to offset TIM8 in advance of TIM1 to align the Phase current ADC measurement
// This parameter is used in setup.c
#define ADC_TOTAL_CONV_TIME     (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES) // = ((SystemCoreClock / ADC_CLOCK_HZ) * ADC_CONV_CLOCK_CYCLES), where ADC_CLOCK_HZ = SystemCoreClock/ADC_CLOCK_DIV

// ############################### GENERAL ###############################

// How to calibrate: connect GND and RX of a 3.3v uart-usb adapter to the right sensor board cable (be careful not to use the red wire of the cable. 15v will destroye verything.). if you are using nunchuck, disable it temporarily. enable DEBUG_SERIAL_USART3 and DEBUG_SERIAL_ASCII use asearial terminal.

// Battery voltage calibration: connect power source. see <How to calibrate>. write value nr 5 to BAT_CALIB_ADC. make and flash firmware. then you can verify voltage on value 6 (devide it by 100.0 to get calibrated voltage).
#define BAT_CALIB_REAL_VOLTAGE  43.0f     // input voltage measured by multimeter
#define BAT_CALIB_ADC           1704      // adc-value measured by mainboard (value nr 5 on UART debug output)

#define BAT_NUMBER_OF_CELLS     10        // normal Hoverboard battery: 10s
#define BAT_LOW_LVL1_ENABLE     0         // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL1            3.6f      // gently beeps at this voltage level. [V/cell]
#define BAT_LOW_LVL2_ENABLE     1         // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL2            3.5f      // your battery is almost empty. Charge now! [V/cell]
#define BAT_LOW_DEAD            3.37f     // undervoltage poweroff. (while not driving) [V/cell]

// Board overheat detection: the sensor is inside the STM/GD chip. it is very inaccurate without calibration (up to 45°C). so only enable this funcion after calibration! let your board cool down. see <How to calibrate>. get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C. write debug value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values. enable warning and/or poweroff and make and flash firmware.
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      35.8f     // temperature 1: measured temperature [°C]
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     48.9f     // temperature 2: measured temperature [°C]
#define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING            60        // annoying fast beeps [°C]
#define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF           65        // overheat poweroff. (while not driving) [°C]

#define INACTIVITY_TIMEOUT 8        // minutes of not driving until poweroff. it is not very precise.

// ############################### LCD DEBUG ###############################

//#define DEBUG_I2C_LCD             // standard 16x2 or larger text-lcd via i2c-converter on right sensor board cable

// ############################### SERIAL DEBUG ###############################

#define DEBUG_SERIAL_USART3         // right sensor board cable, disable if I2C (nunchuck or lcd) is used!
#define DEBUG_BAUD       115200     // UART baud rate
//#define DEBUG_SERIAL_SERVOTERM
#define DEBUG_SERIAL_ASCII          // "1:345 2:1337 3:0 4:0 5:0 6:0 7:0 8:0\r\n"

// ############################### INPUT ###############################

// ###### CONTROL VIA UART (serial) ######
//#define CONTROL_SERIAL_USART2       // left sensor board cable, disable if ADC or PPM is used!
#define CONTROL_BAUD       19200    // control via usart from eg an Arduino or raspberry
// for Arduino, use void loop(void){ Serial.write((uint8_t *) &steer, sizeof(steer)); Serial.write((uint8_t *) &speed, sizeof(speed));delay(20); }

// ###### CONTROL VIA RC REMOTE ######
// left sensor board cable. Channel 1: steering, Channel 2: speed.
//#define CONTROL_PPM                 // use PPM-Sum as input. disable CONTROL_SERIAL_USART2!
//#define PPM_NUM_CHANNELS 6          // total number of PPM channels to receive, even if they are not used.

// ###### CONTROL VIA TWO POTENTIOMETERS ######
// ADC-calibration to cover the full poti-range: connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire in the cable!). see <How to calibrate>. turn the potis to minimum position, write value 1 to ADC1_MIN and value 2 to ADC2_MIN. turn to maximum position and repeat it for ADC?_MAX. make, flash and test it.
#define CONTROL_ADC           // use ADC as input. disable CONTROL_SERIAL_USART2!
#define ADC1_MIN 0            // min ADC1-value while poti at minimum-position (0 - 4095)
#define ADC1_MAX 4095         // max ADC1-value while poti at maximum-position (0 - 4095)
#define ADC2_MIN 0            // min ADC2-value while poti at minimum-position (0 - 4095)
#define ADC2_MAX 4095         // max ADC2-value while poti at maximum-position (0 - 4095)

// ###### CONTROL VIA NINTENDO NUNCHUCK ######
// left sensor board cable. keep cable short, use shielded cable, use ferrits, stabalize voltage in nunchuck, use the right one of the 2 types of nunchucks, add i2c pullups. use original nunchuck. most clones does not work very well.
//#define CONTROL_NUNCHUCK            // use nunchuck as input. disable DEBUG_SERIAL_USART3!


// ############################### MOTOR CONTROL (overwrite) #########################
#define CTRL_TYP_SEL    1               // [-] Control type selection: 0 = Commutation , 1 = FOC Field Oriented Control (default)
#define CTRL_MOD_REQ    1               // [-] Control mode request: 0 = Open mode, 1 = Voltage mode (default), 2 = Speed mode, 3 = Torque mode
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)
#define FIELD_WEAK_ENA  0               // [-] Field Weakening enable flag: 0 = Disabled (default), 1 = Enabled
#define I_MOT_MAX       (15 * 50) << 4  // [A] Maximum motor current (change only the first number, the rest is needed for fixed-point conversion)
#define DC_CUR_LIMIT    17              // [A] DC current limit in amps per motor. (Final current protection. Above this value, current chopping is applied. To avoid this make sure that DC_CUR_LIMIT = I_MOT_MAX + 2A )
#define N_MOT_MAX       800 << 4        // [rpm] Maximum motor speed (change only the first number, the rest is needed for fixed-point conversion)


// GENERAL NOTES:
// 1. The above parameters are over-writing the default motor parameters. For all the available parameters check BLDC_controller_data.c
// 2. The parameters are represented in fixed point data type for a more efficient code execution
// 3. For calibrating the fixed-point parameters use the Fixed-Point Viewer tool (see <https://github.com/EmanuelFeru/FixedPointViewer>)
// 4. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
// 5. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code

// NOTES Field weakening:
// 1. In BLDC_controller_data.c you can find the field weakening Map as a function of speed: MAP = id_fieldWeak_M1, XAXIS = n_fieldWeak_XA
// 2. The default calibration was experimentally calibrated to my particular needs
// 3. If you re-calibrate the field weakening map please take all the safety measures! The motors can spin very fast!
// 4. During the recalibration make sure the speed values in XAXIS are equally spaced for a correct Map interpolation.


// ############################### DRIVING BEHAVIOR ###############################

// inputs:
// - cmd1 and cmd2: analog normalized input values. -1000 to 1000
// - button1 and button2: digital input values. 0 or 1
// - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
// outputs:
// - speedR and speedL: normal driving -1000 to 1000

#define FILTER              0.1f // lower value == softer filter. do not use values <0.01, you will get float precision issues.
#define SPEED_COEFFICIENT   1.0f // higher value == stronger. 0.0 to ~2.0?
#define STEER_COEFFICIENT   0.5f // higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#define INVERT_R_DIRECTION
#define INVERT_L_DIRECTION
#define BEEPS_BACKWARD 0    // 0 or 1

// ###### SIMPLE BOBBYCAR ######
// for better bobbycar code see: https://github.com/larsmm/hoverboard-firmware-hack-bbcar
// #define FILTER              0.1f
// #define SPEED_COEFFICIENT   -1f
// #define STEER_COEFFICIENT   0f

// ###### ARMCHAIR ######
// #define FILTER              0.05f
// #define SPEED_COEFFICIENT   0.5f
// #define STEER_COEFFICIENT  -0.2f

// ############################### VALIDATE SETTINGS ###############################

#if defined CONTROL_SERIAL_USART2 && defined CONTROL_ADC
  #error CONTROL_ADC and CONTROL_SERIAL_USART2 not allowed. it is on the same cable.
#endif

#if defined CONTROL_SERIAL_USART2 && defined CONTROL_PPM
  #error CONTROL_PPM and CONTROL_SERIAL_USART2 not allowed. it is on the same cable.
#endif

#if defined DEBUG_SERIAL_USART3 && defined CONTROL_NUNCHUCK
  #error CONTROL_NUNCHUCK and DEBUG_SERIAL_USART3 not allowed. it is on the same cable.
#endif

#if defined DEBUG_SERIAL_USART3 && defined DEBUG_I2C_LCD
  #error DEBUG_I2C_LCD and DEBUG_SERIAL_USART3 not allowed. it is on the same cable.
#endif

#if defined CONTROL_PPM && defined CONTROL_ADC && defined CONTROL_NUNCHUCK || defined CONTROL_PPM && defined CONTROL_ADC || defined CONTROL_ADC && defined CONTROL_NUNCHUCK || defined CONTROL_PPM && defined CONTROL_NUNCHUCK
  #error only 1 input method allowed. use CONTROL_PPM or CONTROL_ADC or CONTROL_NUNCHUCK.
#endif
