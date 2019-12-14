#pragma once
#include "stm32f1xx_hal.h"

// ############################### DEFINE FIRMWARE VARIANT ###############################
#define TRANSPOTTER                   // Uncomment this line for TRANSPORTER configuration

// ############################### DO-NOT-TOUCH SETTINGS ###############################

#define PWM_FREQ            16000     // PWM frequency in Hz
#define DEAD_TIME              32     // PWM deadtime
#ifdef TRANSPOTTER
  #define DELAY_IN_MAIN_LOOP    2
#else
  #define DELAY_IN_MAIN_LOOP    5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#endif
#define TIMEOUT                 5     // number of wrong / missing input commands before emergency off
#define A2BIT_CONV             50     // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc

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

/* How to calibrate: connect GND and RX of a 3.3v uart-usb adapter to the right sensor board cable
 * Be careful not to use the red wire of the cable. 15v will destroye verything.).
 * If you are using nunchuck, disable it temporarily. enable DEBUG_SERIAL_USART3 and DEBUG_SERIAL_ASCII use asearial terminal.
 */

/* Battery voltage calibration: connect power source. see <How to calibrate>.
 * Write value nr 5 to BAT_CALIB_ADC. make and flash firmware.
 * Then you can verify voltage on value 6 (to get calibrated voltage multiplied by 100).
 */
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define BAT_CALIB_REAL_VOLTAGE  4300      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           1704      // adc-value measured by mainboard (value nr 5 on UART debug output)

#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_LOW_LVL1_ENABLE     0         // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL2_ENABLE     1         // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL1            (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // gently beeps at this voltage level. [V*100/cell]. In this case 3.60 V/cell
#define BAT_LOW_LVL2            (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // your battery is almost empty. Charge now! [V*100/cell]. In this case 3.50 V/cell
#define BAT_LOW_DEAD            (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // undervoltage poweroff. (while not driving) [V*100/cell]. In this case 3.37 V/cell


/* Board overheat detection: the sensor is inside the STM/GD chip.
 * It is very inaccurate without calibration (up to 45°C). So only enable this funcion after calibration!
 * Let your board cool down. see <How to calibrate>.
 * Get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C.
 * Write debug value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values.
 * Enable warning and/or poweroff and make and flash firmware.
 */
#define TEMP_FILT_COEF          655       // temperature filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      358       // temperature 1: measured temperature [°C * 10]. Here 35.8 °C
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     489       // temperature 2: measured temperature [°C * 10]. Here 48.9 °C
#define TEMP_WARNING_ENABLE     0         // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING            600       // annoying fast beeps [°C * 10].  Here 60.0 °C
#define TEMP_POWEROFF_ENABLE    0         // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF           650       // overheat poweroff. (while not driving) [°C * 10]. Here 65.0 °C

#define INACTIVITY_TIMEOUT      8         // minutes of not driving until poweroff. it is not very precise.

// ############################### LCD DEBUG ###############################

//#define DEBUG_I2C_LCD             // standard 16x2 or larger text-lcd via i2c-converter on right sensor board cable


// ############################### SERIAL DEBUG ###############################

#ifndef TRANSPOTTER
  //#define DEBUG_SERIAL_SERVOTERM
  #define DEBUG_SERIAL_ASCII          // "1:345 2:1337 3:0 4:0 5:0 6:0 7:0 8:0\r\n"
#endif


// ############################### INPUT ###############################

// ###### CONTROL VIA UART (serial) ######
#define START_FRAME             0xAAAA                  // [-] Start frame definition for serial commands
#define SERIAL_TIMEOUT          160                     // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec

#define USART2_BAUD             38400                   // UART2 baud rate (long wired cable)
#define USART2_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
// #define CONTROL_SERIAL_USART2                           // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
// #define FEEDBACK_SERIAL_USART2                          // left sensor board cable, disable if ADC or PPM is used!
// #define DEBUG_SERIAL_USART2                             // left sensor board cable, disable if ADC or PPM is used!

#ifndef TRANSPOTTER
  #define USART3_BAUD             38400                   // UART3 baud rate (short wired cable)
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
  // #define CONTROL_SERIAL_USART3                           // right sensor board cable, disable if I2C (nunchuck or lcd) is used! For Arduino control check the hoverSerial.ino
  // #define FEEDBACK_SERIAL_USART3                          // right sensor board cable, disable if I2C (nunchuck or lcd) is used!
  #define DEBUG_SERIAL_USART3                             // right sensor board cable, disable if I2C (nunchuck or lcd) is used!
#endif

#if defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2)
#define UART_DMA_CHANNEL DMA1_Channel7
#endif

#if defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)
#define UART_DMA_CHANNEL DMA1_Channel2
#endif

// ###### CONTROL VIA RC REMOTE ######
// left sensor board cable. Channel 1: steering, Channel 2: speed.
//#define CONTROL_PPM                 // use PPM-Sum as input. disable CONTROL_SERIAL_USART2!
//#define PPM_NUM_CHANNELS 6          // total number of PPM channels to receive, even if they are not used.

// ###### CONTROL VIA TWO POTENTIOMETERS ######
/* ADC-calibration to cover the full poti-range:
 * Connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire in the cable!). see <How to calibrate>.
 * Turn the potis to minimum position, write value 1 to ADC1_MIN and value 2 to ADC2_MIN
 * Turn the potis to maximum position, write value 1 to ADC1_MAX and value 2 to ADC2_MAX
 * For middle resting potis: Let the potis in the middle resting position, write value 1 to ADC1_MID and value 2 to ADC2_MID
 * Make, flash and test it.
 */
#ifndef TRANSPOTTER
  #define CONTROL_ADC           // use ADC as input. disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
  // #define ADC1_MID_POT          // ADC1 middle resting poti: comment-out if NOT a middle resting poti
  #define ADC1_MIN 0            // min ADC1-value while poti at minimum-position (0 - 4095)
  #define ADC1_MID 1963         // mid ADC1-value while poti at minimum-position (ADC1_MIN - ADC1_MAX)
  #define ADC1_MAX 4095         // max ADC1-value while poti at maximum-position (0 - 4095)
  // #define ADC2_MID_POT          // ADC2 middle resting poti: comment-out if NOT a middle resting poti
  #define ADC2_MIN 0            // min ADC2-value while poti at minimum-position (0 - 4095)
  #define ADC2_MID 2006         // mid ADC2-value while poti at minimum-position (ADC2_MIN - ADC2_MAX)
  #define ADC2_MAX 4095         // max ADC2-value while poti at maximum-position (0 - 4095)
#endif

// ###### CONTROL VIA NINTENDO NUNCHUCK ######
/* left sensor board cable.
 * keep cable short, use shielded cable, use ferrits, stabalize voltage in nunchuck,
 * use the right one of the 2 types of nunchucks, add i2c pullups.
 * use original nunchuck. most clones does not work very well.
 */
// #define CONTROL_NUNCHUCK            // use nunchuck as input. disable FEEDBACK_SERIAL_USART3, DEBUG_SERIAL_USART3!


// ############################### MOTOR CONTROL #########################
// Control selections
#define CTRL_TYP_SEL    2                       // [-] Control type selection: 0 = Commutation , 1 = Sinusoidal, 2 = FOC Field Oriented Control (default)
#define CTRL_MOD_REQ    1                       // [-] Control mode request: 0 = Open mode, 1 = VOLTAGE mode (default), 2 = SPEED mode, 3 = TORQUE mode. Note: SPEED and TORQUE modes are only available for FOC!
#define DIAG_ENA        1                       // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

// Limitation settings
#define I_MOT_MAX       15                      // [A] Maximum motor current limit
#define I_DC_MAX        17                      // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000                    // [rpm] Maximum motor speed limit

// Field Weakening / Phase Advance
#define FIELD_WEAK_ENA  0                       // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5                       // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
#define PHASE_ADV_MAX   25                      // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1500                    // [-] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   1000                    // [-] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

// Data checks - Do NOT touch
#if (FIELD_WEAK_ENA == 0)
  #undef  FIELD_WEAK_HI                       
  #define FIELD_WEAK_HI 1000                    // [-] This prevents the input target going beyond 1000 when Field Weakening is not enabled
#endif
#define INPUT_MAX   MAX( 1000, FIELD_WEAK_HI)   // [-] Defines the Input target maximum limitation        
#define INPUT_MIN   MIN(-1000,-FIELD_WEAK_HI)   // [-] Defines the Input target minimum limitation 
#define INPUT_MID   INPUT_MAX / 2      

/* GENERAL NOTES:
 * 1. The above parameters are over-writing the default motor parameters. For all the available parameters check BLDC_controller_data.c
 * 2. The parameters are represented in fixed point data type for a more efficient code execution
 * 3. For calibrating the fixed-point parameters use the Fixed-Point Viewer tool (see <https://github.com/EmanuelFeru/FixedPointViewer>)
 * 4. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
 * 5. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code. The webview is an html page that can be opened with browsers like: Microsoft Internet Explorer or Microsoft Edge
 *
 * NOTES Field Weakening / Phase Advance:
 * 1. The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 * 2. The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 * 3. If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!
 */


// ############################### DRIVING BEHAVIOR ###############################

/* Inputs:
 * - cmd1 and cmd2: analog normalized input values. INPUT_MIN to INPUT_MAX
 * - button1 and button2: digital input values. 0 or 1
 * - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
 * Outputs:
 * - speedR and speedL: normal driving INPUT_MIN to INPUT_MAX 
 */

// Beep in Reverse
#define BEEPS_BACKWARD      0     // 0 or 1

// Value of RATE is in fixdt(1,16,4): VAL_fixedPoint = VAL_floatingPoint * 2^4. In this case 480 = 30 * 2^4
#define RATE                480   // 30.0f [-] lower value == slower rate [0, 32767] = [0.0, 2047.9375]. Do NOT make rate negative (>32767)

// Value of FILTER is in fixdt(0,16,16): VAL_fixedPoint = VAL_floatingPoint * 2^16. In this case 6553 = 0.1 * 2^16
#define FILTER              6553  // 0.1f [-] lower value == softer filter [0, 65535] = [0.0 - 1.0].

// ################################# DEFAULT SETTINGS ############################
#ifndef TRANSPOTTER
  // Value of COEFFICIENT is in fixdt(1,16,14)
  // If VAL_floatingPoint >= 0, VAL_fixedPoint = VAL_floatingPoint * 2^14
  // If VAL_floatingPoint < 0,  VAL_fixedPoint = 2^16 + floor(VAL_floatingPoint * 2^14).
  #define SPEED_COEFFICIENT   16384 // 1.0f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14 
  #define STEER_COEFFICIENT   8192  // 0.5f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case  8192 = 0.5 * 2^14. If you do not want any steering, set it to 0. 

  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
#endif

// ################################# TRANSPOTTER SETTINGS ############################
#ifdef TRANSPOTTER
  #define CONTROL_GAMETRAK
  #define SUPPORT_LCD
  #define SUPPORT_NUNCHUCK

  #define GAMETRAK_CONNECTION_NORMAL    // for normal wiring according to the wiki instructions
  //#define GAMETRAK_CONNECTION_ALTERNATE // use this define instead if you messed up the gametrak ADC wiring (steering is speed, and length of the wire is steering)

  #define ROT_P          1.2          // P coefficient for the direction controller. Positive / Negative values to invert gametrak steering direction.  

  //#define INVERT_R_DIRECTION        // Invert right motor
  #define INVERT_L_DIRECTION          // Invert left motor

  // during nunchuck control (only relevant when activated)
  #define SPEED_COEFFICIENT   14746  // 0.9f - higher value == stronger. 0.0 to ~2.0?
  #define STEER_COEFFICIENT   8192   // 0.5f - higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#endif

// ################################# SIMPLE BOBBYCAR #################################
// for better bobbycar code see: https://github.com/larsmm/hoverboard-firmware-hack-bbcar
// #define FILTER             6553    //  0.1f 
// #define SPEED_COEFFICIENT  49152   // -1.0f
// #define STEER_COEFFICIENT  0       //  0.0f

// ################################# ARMCHAIR #################################
// #define FILTER             3276    //  0.05f
// #define SPEED_COEFFICIENT  8192    //  0.5f
// #define STEER_COEFFICIENT  62259   // -0.2f

// ############################### VALIDATE SETTINGS ###############################

#if defined(CONTROL_SERIAL_USART2) && defined(CONTROL_SERIAL_USART3)
  #error CONTROL_SERIAL_USART2 and CONTROL_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(FEEDBACK_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART3)
  #error FEEDBACK_SERIAL_USART2 and FEEDBACK_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART2)
  #error DEBUG_SERIAL_USART2 and FEEDBACK_SERIAL_USART2 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART3) && defined(FEEDBACK_SERIAL_USART3)
  #error DEBUG_SERIAL_USART3 and FEEDBACK_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(DEBUG_SERIAL_USART2) && defined(DEBUG_SERIAL_USART3)
  #error DEBUG_SERIAL_USART2 and DEBUG_SERIAL_USART3 not allowed, choose one.
#endif

#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if (defined(DEBUG_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2)) && defined(CONTROL_PPM)
  #error CONTROL_PPM and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if (defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3)) && defined(CONTROL_NUNCHUCK)
  #error CONTROL_NUNCHUCK and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if (defined(DEBUG_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3)) && defined(DEBUG_I2C_LCD)
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM) && defined(CONTROL_ADC) && defined(CONTROL_NUNCHUCK) || defined(CONTROL_PPM) && defined(CONTROL_ADC) || defined(CONTROL_ADC) && defined(CONTROL_NUNCHUCK) || defined(CONTROL_PPM) && defined(CONTROL_NUNCHUCK)
  #error only 1 input method allowed. use CONTROL_PPM or CONTROL_ADC or CONTROL_NUNCHUCK.
#endif
