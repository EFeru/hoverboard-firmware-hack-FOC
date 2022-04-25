// Define to prevent recursive inclusion
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// ############################### VARIANT SELECTION ###############################
// PlatformIO: uncomment desired variant in platformio.ini
// Keil uVision: select desired variant from the Target drop down menu (to the right of the Load button)
// Ubuntu: define the desired build variant here if you want to use make in console
// or use VARIANT environment variable for example like "make -e VARIANT=VARIANT_NUNCHUK". Select only one at a time.
#if !defined(PLATFORMIO)
  //#define VARIANT_ADC         // Variant for control via ADC input
  //#define VARIANT_USART       // Variant for Serial control via USART3 input
  //#define VARIANT_NUNCHUK     // Variant for Nunchuk controlled vehicle build
  //#define VARIANT_PPM         // Variant for RC-Remote with PPM-Sum Signal
  //#define VARIANT_PWM         // Variant for RC-Remote with PWM Signal
  //#define VARIANT_IBUS        // Variant for RC-Remotes with FLYSKY IBUS
  //#define VARIANT_HOVERCAR    // Variant for HOVERCAR build
  //#define VARIANT_HOVERBOARD  // Variant for HOVERBOARD build
  //#define VARIANT_TRANSPOTTER // Variant for TRANSPOTTER build https://github.com/NiklasFauth/hoverboard-firmware-hack/wiki/Build-Instruction:-TranspOtter https://hackaday.io/project/161891-transpotter-ng
  //#define VARIANT_SKATEBOARD  // Variant for SKATEBOARD build
#endif
// ########################### END OF VARIANT SELECTION ############################


// ############################### DO-NOT-TOUCH SETTINGS ###############################
#define PWM_FREQ            16000     // PWM frequency in Hz / is also used for buzzer
#define DEAD_TIME              48     // PWM deadtime
#ifdef VARIANT_TRANSPOTTER
  #define DELAY_IN_MAIN_LOOP    2
#else
  #define DELAY_IN_MAIN_LOOP    5     // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.
#endif
#define TIMEOUT                20     // number of wrong / missing input commands before emergency off
#define A2BIT_CONV             50     // A to bit for current conversion on ADC. Example: 1 A = 50, 2 A = 100, etc
// #define PRINTF_FLOAT_SUPPORT          // [-] Uncomment this for printf to support float on Serial Debug. It will increase code size! Better to avoid it!

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
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################

// ############################### BOARD VARIANT ###############################
/* Board Variant
 * 0 - Default board type
 * 1 - Alternate board type with different pin mapping for DCLINK, Buzzer and ON/OFF, Button and Charger
*/
#define BOARD_VARIANT           0         // change if board with alternate pin mapping
// ######################## END OF BOARD VARIANT ###############################

// ############################### BATTERY ###############################
/* Battery voltage calibration: connect power source.
 * see How to calibrate.
 * Write debug output value nr 5 to BAT_CALIB_ADC. make and flash firmware.
 * Then you can verify voltage on debug output value 6 (to get calibrated voltage multiplied by 100).
*/
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. coef_fixedPoint = coef_floatingPoint * 2^16. In this case 655 = 0.01 * 2^16
#define BAT_CALIB_REAL_VOLTAGE  3970      // input voltage measured by multimeter (multiplied by 100). In this case 43.00 V * 100 = 4300
#define BAT_CALIB_ADC           1492      // adc-value measured by mainboard (value nr 5 on UART debug output)
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_LVL2_ENABLE         0         // to beep or not to beep, 1 or 0
#define BAT_LVL1_ENABLE         1         // to beep or not to beep, 1 or 0
#define BAT_DEAD_ENABLE         1         // to poweroff or not to poweroff, 1 or 0
#define BAT_BLINK_INTERVAL      80        // battery led blink interval (80 loops * 5ms ~= 400ms)
#define BAT_LVL5                (390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Green blink:  no beep
#define BAT_LVL4                (380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow:       no beep
#define BAT_LVL3                (370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Yellow blink: no beep 
#define BAT_LVL2                (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red:          gently beep at this voltage level. [V*100/cell]. In this case 3.60 V/cell
#define BAT_LVL1                (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // Red blink:    fast beep. Your battery is almost empty. Charge now! [V*100/cell]. In this case 3.50 V/cell
#define BAT_DEAD                (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE    // All leds off: undervoltage poweroff. (while not driving) [V*100/cell]. In this case 3.37 V/cell
// ######################## END OF BATTERY ###############################



// ############################### TEMPERATURE ###############################
/* Board overheat detection: the sensor is inside the STM/GD chip.
 * It is very inaccurate without calibration (up to 45°C). So only enable this funcion after calibration!
 * Let your board cool down.
 * see <How to calibrate.
 * Get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C.
 * Write debug output value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values.
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
// ######################## END OF TEMPERATURE ###############################



// ############################### MOTOR CONTROL #########################
/* GENERAL NOTES:
 * 1. The parameters here are over-writing the default motor parameters. For all the available parameters check BLDC_controller_data.c
 * 2. The parameters are represented in fixed point data type for a more efficient code execution
 * 3. For calibrating the fixed-point parameters use the Fixed-Point Viewer tool (see <https://github.com/EmanuelFeru/FixedPointViewer>)
 * 4. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
 * 5. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code.
 * The webview is an html page that can be opened with browsers like: Microsoft Internet Explorer or Microsoft Edge
 *
 * NOTES Field Weakening / Phase Advance:
 * 1. The Field Weakening is a linear interpolation from 0 to FIELD_WEAK_MAX or PHASE_ADV_MAX (depeding if FOC or SIN is selected, respectively)
 * 2. The Field Weakening starts engaging at FIELD_WEAK_LO and reaches the maximum value at FIELD_WEAK_HI
 * 3. If you re-calibrate the Field Weakening please take all the safety measures! The motors can spin very fast!

   Inputs:
    - input1[inIdx].cmd and input2[inIdx].cmd: normalized input values. INPUT_MIN to INPUT_MAX
    - button1 and button2: digital input values. 0 or 1
    - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
   Outputs:
    - cmdL and cmdR: normal driving INPUT_MIN to INPUT_MAX
*/
#define COM_CTRL        0               // [-] Commutation Control Type
#define SIN_CTRL        1               // [-] Sinusoidal Control Type
#define FOC_CTRL        2               // [-] Field Oriented Control (FOC) Type

#define OPEN_MODE       0               // [-] OPEN mode
#define VLT_MODE        1               // [-] VOLTAGE mode
#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

// Enable/Disable Motor
#define MOTOR_LEFT_ENA                  // [-] Enable LEFT motor.  Comment-out if this motor is not needed to be operational
#define MOTOR_RIGHT_ENA                 // [-] Enable RIGHT motor. Comment-out if this motor is not needed to be operational

// Control selections
#define CTRL_TYP_SEL    FOC_CTRL        // [-] Control type selection: COM_CTRL, SIN_CTRL, FOC_CTRL (default)
#define CTRL_MOD_REQ    VLT_MODE        // [-] Control mode request: OPEN_MODE, VLT_MODE (default), SPD_MODE, TRQ_MODE. Note: SPD_MODE and TRQ_MODE are only available for CTRL_FOC!
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum single motor current limit
#define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit

// Field Weakening / Phase Advance
#define FIELD_WEAK_ENA  0               // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5               // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed. Up to 10A has been tested using 10" wheels.
#define PHASE_ADV_MAX   25              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1000            // (1000, 1500] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   750             // ( 500, 1000] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

// Extra functionality
// #define STANDSTILL_HOLD_ENABLE          // [-] Flag to hold the position when standtill is reached. Only available and makes sense for VOLTAGE or TORQUE mode.
// #define ELECTRIC_BRAKE_ENABLE           // [-] Flag to enable electric brake and replace the motor "freewheel" with a constant braking when the input torque request is 0. Only available and makes sense for TORQUE mode.
// #define ELECTRIC_BRAKE_MAX    100       // (0, 500) Maximum electric brake to be applied when input torque request is 0 (pedal fully released).
// #define ELECTRIC_BRAKE_THRES  120       // (0, 500) Threshold below at which the electric brake starts engaging.
// ########################### END OF MOTOR CONTROL ########################



// ############################## DEFAULT SETTINGS ############################
// Default settings will be applied at the end of this config file if not set before
#define INACTIVITY_TIMEOUT        8       // Minutes of not driving until poweroff. it is not very precise.
#define BEEPS_BACKWARD            1       // 0 or 1
#define ADC_MARGIN                100     // ADC input margin applied on the raw ADC min and max to make sure the MIN and MAX values are reached even in the presence of noise
#define ADC_PROTECT_TIMEOUT       100     // ADC Protection: number of wrong / missing input commands before safety state is taken
#define ADC_PROTECT_THRESH        200     // ADC Protection threshold below/above the MIN/MAX ADC values
#define AUTO_CALIBRATION_ENA              // Enable/Disable input auto-calibration by holding power button pressed. Un-comment this if auto-calibration is not needed.

/* FILTER is in fixdt(0,16,16): VAL_fixedPoint = VAL_floatingPoint * 2^16. In this case 6553 = 0.1 * 2^16
 * Value of COEFFICIENT is in fixdt(1,16,14)
 * If VAL_floatingPoint >= 0, VAL_fixedPoint = VAL_floatingPoint * 2^14
 * If VAL_floatingPoint < 0,  VAL_fixedPoint = 2^16 + floor(VAL_floatingPoint * 2^14).
*/
// Value of RATE is in fixdt(1,16,4): VAL_fixedPoint = VAL_floatingPoint * 2^4. In this case 480 = 30 * 2^4
#define DEFAULT_RATE                480   // 30.0f [-] lower value == slower rate [0, 32767] = [0.0, 2047.9375]. Do NOT make rate negative (>32767)
#define DEFAULT_FILTER              6553  // Default for FILTER 0.1f [-] lower value == softer filter [0, 65535] = [0.0 - 1.0].
#define DEFAULT_SPEED_COEFFICIENT   16384 // Default for SPEED_COEFFICIENT 1.0f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
#define DEFAULT_STEER_COEFFICIENT   8192  // Defualt for STEER_COEFFICIENT 0.5f [-] higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case  8192 = 0.5 * 2^14. If you do not want any steering, set it to 0.
// ######################### END OF DEFAULT SETTINGS ##########################



// ############################## INPUT FORMAT ############################
/* ***_INPUT: TYPE, MIN, MID, MAX, DEADBAND
 * -----------------------------------------
 * TYPE:      0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect
 * MIN:       min ADC1-value while poti at minimum-position (0 - 4095)
 * MID:       mid ADC1-value while poti at mid-position (INPUT_MIN - INPUT_MAX)
 * MAX:       max ADC2-value while poti at maximum-position (0 - 4095)
 * DEADBAND:  how much of the center position is considered 'center' (100 = values -100 to 100 are considered 0)
 * 
 * Dual-inputs
 * PRI_INPUT: Primary   Input. These limits will be used for the input with priority 0
 * AUX_INPUT: Auxiliary Input. These limits will be used for the input with priority 1
 * -----------------------------------------
*/
 // ############################## END OF INPUT FORMAT ############################



// ############################## CRUISE CONTROL SETTINGS ############################
/* Cruise Control info:
 * enable CRUISE_CONTROL_SUPPORT and (SUPPORT_BUTTONS_LEFT or SUPPORT_BUTTONS_RIGHT depending on which cable is the button installed)
 * can be activated/deactivated by pressing button1 (Blue cable) to GND
 * when activated, it maintains the current speed by switching to SPD_MODE. Acceleration is still possible via the input request, but when released it resumes to previous set speed.
 * when deactivated, it returns to previous control MODE and follows the input request.
*/
// #define CRUISE_CONTROL_SUPPORT
// #define SUPPORT_BUTTONS_LEFT              // Use button1 (Blue Left cable)  to activate/deactivate Cruise Control
// #define SUPPORT_BUTTONS_RIGHT             // Use button1 (Blue Right cable) to activate/deactivate Cruise Control

// ######################### END OF CRUISE CONTROL SETTINGS ##########################



// ############################### DEBUG SERIAL ###############################
/* Connect GND and RX of a 3.3v uart-usb adapter to the left (USART2) or right sensor board cable (USART3)
 * Be careful not to use the red wire of the cable. 15v will destroy everything.
 * If you are using VARIANT_NUNCHUK, disable it temporarily.
 * enable DEBUG_SERIAL_USART3 or DEBUG_SERIAL_USART2
 *
 *
 * DEBUG ASCII output is:
 * // "in1:345 in2:1337 cmdL:0 cmdR:0 BatADC:0 BatV:0 TempADC:0 Temp:0\r\n"
 *
 * in1:     (int16_t)input1[inIdx].raw);                                        raw input1: ADC1, UART, PWM, PPM, iBUS
 * in2:     (int16_t)input2[inIdx].raw);                                        raw input2: ADC2, UART, PWM, PPM, iBUS
 * cmdL:    (int16_t)cmdL);                                                     output command Left: [-1000, 1000]
 * cmdR:    (int16_t)cmdR);                                                     output command Right: [-1000, 1000]
 * BatADC:  (int16_t)adc_buffer.batt1);                                         Battery adc-value measured by mainboard
 * BatV:    (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC));    Battery calibrated voltage multiplied by 100 for verifying battery voltage calibration
 * TempADC: (int16_t)board_temp_adcFilt);                                       for board temperature calibration
 * Temp:    (int16_t)board_temp_deg_c);                                         Temperature in celcius for verifying board temperature calibration
 *
*/

// #define DEBUG_SERIAL_USART2          // left sensor board cable, disable if ADC or PPM is used!
// #define DEBUG_SERIAL_USART3          // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
// #define DEBUG_SERIAL_PROTOCOL        // uncomment this to send user commands to the board, change parameters and print specific signals (see comms.c for the user commands)
// ########################### END OF DEBUG SERIAL ############################



// ############################### DEBUG LCD ###############################
// #define DEBUG_I2C_LCD                // standard 16x2 or larger text-lcd via i2c-converter on right sensor board cable
// ########################### END OF DEBUG LCD ############################



// ################################# VARIANT_ADC SETTINGS ############################
#ifdef VARIANT_ADC
/* CONTROL VIA TWO POTENTIOMETERS
 * Connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire!)
 *
 * Auto-calibration of the ADC Limit to finds the Minimum, Maximum, and Middle for the ADC input
 * Procedure:
 * - press the power button for more than 5 sec and release after the beep sound
 * - move the potentiometers freely to the min and max limits repeatedly
 * - release potentiometers to the resting postion
 * - press the power button to confirm or wait for the 20 sec timeout
 * The Values will be saved to flash. Values are persistent if you flash with platformio. To erase them, make a full chip erase.
 *
 * After calibration you can optionally write the values to the following defines
 * Procedure:
 * - connect gnd, rx and tx of a usb-uart converter in 3.3V mode to the right sensor board cable (do NOT use the red 15V wire!)
 * - readout values using a serial terminal in 115200 baud rate
 * - turn the potis to minimum position, write value in1 to PRI_INPUT1 MIN and value in2 to PRI_INPUT2 MIN
 * - turn the potis to maximum position, write value in1 to PRI_INPUT1 MAX and value in2 to PRI_INPUT2 MAX
 * - for middle resting potis: Let the potis in the middle resting position, write value in1 to PRI_INPUT1 MID and value in2 to PRI_INPUT2 MID
*/
  #define CONTROL_ADC           0         // use ADC as input. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!

  // #define DUAL_INPUTS                     //  ADC*(Primary) + UART(Auxiliary). Uncomment this to use Dual-inputs
  #define PRI_INPUT1            3, 0, 0, 4095, 0      // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2            3, 0, 0, 4095, 0      // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY     0x1101    // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    // #define SIDEBOARD_SERIAL_USART3 1
    #define CONTROL_SERIAL_USART3 1       // right sensor board cable. Number indicates priority for dual-input. Disable if I2C (nunchuk or lcd) is used! For Arduino control check the hoverSerial.ino
    #define FEEDBACK_SERIAL_USART3        // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
    #define AUX_INPUT1          3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2          3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY     0x1001    // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define DEBUG_SERIAL_USART3           // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
  #endif

  // #define TANK_STEERING                   // use for tank steering, each input controls each wheel 
  // #define ADC_ALTERNATE_CONNECT           // use to swap ADC inputs
  // #define SUPPORT_BUTTONS_LEFT            // use left sensor board cable for button inputs.  Disable DEBUG_SERIAL_USART2!
  // #define SUPPORT_BUTTONS_RIGHT           // use right sensor board cable for button inputs. Disable DEBUG_SERIAL_USART3!
#endif
// ############################# END OF VARIANT_ADC SETTINGS #########################



// ############################ VARIANT_USART SETTINGS ############################
#ifdef VARIANT_USART
  // #define SIDEBOARD_SERIAL_USART2 0
  #define CONTROL_SERIAL_USART2  0    // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
  #define FEEDBACK_SERIAL_USART2      // left sensor board cable, disable if ADC or PPM is used!

  // #define SIDEBOARD_SERIAL_USART3 0
  // #define CONTROL_SERIAL_USART3  0    // right sensor board cable. Number indicates priority for dual-input. Disable if I2C (nunchuk or lcd) is used! For Arduino control check the hoverSerial.ino
  // #define FEEDBACK_SERIAL_USART3      // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
 
  // #define DUAL_INPUTS                 //  UART*(Primary) + SIDEBOARD(Auxiliary). Uncomment this to use Dual-inputs
  #define PRI_INPUT1             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2             3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY      0x1102  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    // #define SIDEBOARD_SERIAL_USART2 1   // left sideboard
    #define SIDEBOARD_SERIAL_USART3 1   // right sideboard
    #define AUX_INPUT1           3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2           3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY      0x1002  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #endif

  // #define TANK_STEERING              // use for tank steering, each input controls each wheel 
  // #define SUPPORT_BUTTONS_LEFT       // use left sensor board cable for button inputs.  Disable DEBUG_SERIAL_USART2!
  // #define SUPPORT_BUTTONS_RIGHT      // use right sensor board cable for button inputs. Disable DEBUG_SERIAL_USART3!
#endif
// ######################## END OF VARIANT_USART SETTINGS #########################



// ################################# VARIANT_NUNCHUK SETTINGS ############################
#ifdef VARIANT_NUNCHUK
  /* on Right sensor cable
   * keep cable short, use shielded cable, use ferrits, stabalize voltage in nunchuk,
   * use the right one of the 2 types of nunchuks, add i2c pullups.
   * use original nunchuk. most clones does not work very well.
   * Recommendation: Nunchuk Breakout Board https://github.com/Jan--Henrik/hoverboard-breakout
  */
  #define CONTROL_NUNCHUK         0       // use nunchuk as input. Number indicates priority for dual-input. Disable FEEDBACK_SERIAL_USART3, DEBUG_SERIAL_USART3!

  // #define DUAL_INPUTS                     // Nunchuk*(Primary) + UART(Auxiliary). Uncomment this to use Dual-inputs
  #define PRI_INPUT1              2, -1024, 0, 1024, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2              2, -1024, 0, 1024, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1103  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    // #define SIDEBOARD_SERIAL_USART2 1
    #define CONTROL_SERIAL_USART2 1       // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
    #define FEEDBACK_SERIAL_USART2        // left sensor board cable, disable if ADC or PPM is used!
    #define AUX_INPUT1            3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2            3, -1000, 0, 1000, 0     // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY       0x1003  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define DEBUG_SERIAL_USART2           // left sensor cable debug
  #endif

  // # maybe good for ARMCHAIR #
  #define FILTER                  3276    //  0.05f
  #define SPEED_COEFFICIENT       8192    //  0.5f
  #define STEER_COEFFICIENT       62259   // -0.2f
  // #define SUPPORT_BUTTONS                 // Define for Nunchuk buttons support
#endif
// ############################# END OF VARIANT_NUNCHUK SETTINGS #########################



// ################################# VARIANT_PPM SETTINGS ##############################
#ifdef VARIANT_PPM
/* ###### CONTROL VIA RC REMOTE ######
 * Right sensor board cable. Channel 1: steering, Channel 2: speed.
 * https://gist.github.com/peterpoetzi/1b63a4a844162196613871767189bd05
*/
  // #define DUAL_INPUTS                     // ADC*(Primary) + PPM(Auxiliary). Uncomment this to use Dual-inputs
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1104  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define CONTROL_ADC           0       // use ADC as input. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
    #define CONTROL_PPM_RIGHT     1       // use PPM-Sum as input on the RIGHT cable. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART3!
    #define PRI_INPUT1            3,     0, 0, 4095,   0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3,     0, 0, 4095,   0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT1            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY       0x1004  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    // #define CONTROL_PPM_LEFT      0       // use PPM-Sum as input on the LEFT cable. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2!
    #define CONTROL_PPM_RIGHT     0       // use PPM-Sum as input on the RIGHT cable. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART3!
    #define PRI_INPUT1            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #endif
  #define PPM_NUM_CHANNELS        6       // total number of PPM channels to receive, even if they are not used.

  // #define TANK_STEERING                   // use for tank steering, each input controls each wheel 
  // #define SUPPORT_BUTTONS                 // Define for PPM buttons support
  // #define SUPPORT_BUTTONS_LEFT            // use left sensor board cable for button inputs.  Disable DEBUG_SERIAL_USART2!
  // #define SUPPORT_BUTTONS_RIGHT           // use right sensor board cable for button inputs. Disable DEBUG_SERIAL_USART3!

  #if defined(CONTROL_PPM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2           // left sensor cable debug
  #elif defined(CONTROL_PPM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3           // right sensor cable debug
  #endif
#endif
// ############################# END OF VARIANT_PPM SETTINGS ############################


// ################################# VARIANT_PWM SETTINGS ##############################
#ifdef VARIANT_PWM
/* ###### CONTROL VIA RC REMOTE ######
 * Right sensor board cable. Connect PA2 to channel 1 and PA3 to channel 2 on receiver.
 * Channel 1: steering, Channel 2: speed.
*/
  // #define DUAL_INPUTS                     // ADC*(Primary) + PWM(Auxiliary). Uncomment this to use Dual-inputs
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1105  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define CONTROL_ADC           0       // use ADC as input. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
    #define CONTROL_PWM_RIGHT     1       // use RC PWM as input on the RIGHT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART3!
    #define PRI_INPUT1            3,     0, 0, 4095,   0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3,     0, 0, 4095,   0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT1            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY       0x1005  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    // #define CONTROL_PWM_LEFT      0       // use RC PWM as input on the LEFT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART2!
    #define CONTROL_PWM_RIGHT     0       // use RC PWM as input on the RIGHT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART3!
    #define PRI_INPUT1            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3, -1000, 0, 1000, 100  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #endif

  #define FILTER                  6553    // 0.1f [-] fixdt(0,16,16) lower value == softer filter [0, 65535] = [0.0 - 1.0].
  #define SPEED_COEFFICIENT       16384   // 1.0f [-] fixdt(1,16,14) higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
  #define STEER_COEFFICIENT       16384   // 1.0f [-] fixdt(1,16,14) higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14. If you do not want any steering, set it to 0.
  // #define TANK_STEERING                   // use for tank steering, each input controls each wheel 
  // #define INVERT_R_DIRECTION
  // #define INVERT_L_DIRECTION
  // #define SUPPORT_BUTTONS_LEFT            // use left sensor board cable for button inputs.  Disable DEBUG_SERIAL_USART2!
  // #define SUPPORT_BUTTONS_RIGHT           // use right sensor board cable for button inputs. Disable DEBUG_SERIAL_USART3!

  #if defined(CONTROL_PWM_RIGHT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2           // left sensor cable debug
  #elif defined(CONTROL_PWM_LEFT) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3           // right sensor cable debug
  #endif
#endif
// ############################# END OF VARIANT_PWM SETTINGS ############################



// ################################# VARIANT_IBUS SETTINGS ##############################
#ifdef VARIANT_IBUS
/* CONTROL VIA RC REMOTE WITH FLYSKY IBUS PROTOCOL 
* Connected to Right sensor board cable. Channel 1: steering, Channel 2: speed.
*/
  #define CONTROL_IBUS                    // use IBUS as input. Number indicates priority for dual-input.
  #define IBUS_NUM_CHANNELS       14      // total number of IBUS channels to receive, even if they are not used.
  #define IBUS_LENGTH             0x20
  #define IBUS_COMMAND            0x40
  #define USART3_BAUD             115200

  // #define DUAL_INPUTS                     // ADC*(Primary) + iBUS(Auxiliary). Uncomment this to use Dual-inputs
  #ifdef DUAL_INPUTS
    #define FLASH_WRITE_KEY       0x1106  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define CONTROL_ADC           0       // use ADC as input. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
    #define CONTROL_SERIAL_USART3 1       // use RC iBUS input on the RIGHT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART3!
    #define FEEDBACK_SERIAL_USART3        // right sensor board cable, disable if ADC or PPM is used!
    #define PRI_INPUT1            3,     0, 0, 4095, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3,     0, 0, 4095, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT1            3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define AUX_INPUT2            3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #else
    #define FLASH_WRITE_KEY       0x1006  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
    #define CONTROL_SERIAL_USART3 0       // use RC iBUS input on the RIGHT cable, disable if ADC or PPM is used!
    #define FEEDBACK_SERIAL_USART3        // right sensor board cable, disable if ADC or PPM is used!
    #define PRI_INPUT1            3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
    #define PRI_INPUT2            3, -1000, 0, 1000, 0  // TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #endif

  // #define TANK_STEERING                // use for tank steering, each input controls each wheel 

  #if defined(CONTROL_SERIAL_USART3) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART2           // left sensor cable debug
  #elif defined(DEBUG_SERIAL_USART2) && !defined(DUAL_INPUTS)
    #define DEBUG_SERIAL_USART3           // right sensor cable debug
  #endif
#endif
// ############################# END OF VARIANT_IBUS SETTINGS ############################



// ############################ VARIANT_HOVERCAR SETTINGS ############################
#ifdef VARIANT_HOVERCAR
  #define FLASH_WRITE_KEY         0x1107  // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ            TRQ_MODE  // HOVERCAR works best in TORQUE Mode
  #define CONTROL_ADC             0         // use ADC as input. Number indicates priority for dual-input. Disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
  #define SIDEBOARD_SERIAL_USART3 1         // Rx from right sensor board: to use photosensors as buttons. Number indicates priority for dual-input. Comment-out if sideboard is not used!
  #define FEEDBACK_SERIAL_USART3            // Tx to   right sensor board: for LED battery indication. Comment-out if sideboard is not used!

  #define DUAL_INPUTS                       // ADC*(Primary) + Sideboard_R(Auxiliary). Uncomment this to use Dual-inputs
  #define PRI_INPUT1              1,  1000, 0, 2500, 0  // Pedal Brake        TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2              1,   500, 0, 2200, 0  // Pedal Accel        TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define AUX_INPUT1              2, -1000, 0, 1000, 0  // Sideboard Steer    TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define AUX_INPUT2              2, -1000, 0, 1000, 0  // Sideboard Speed    TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section

  #define SPEED_COEFFICIENT       16384     // 1.0f
  #define STEER_COEFFICIENT       8192      // 0.5f Only active in Sideboard input
  // #define ADC_ALTERNATE_CONNECT             // use to swap ADC inputs
  // #define INVERT_R_DIRECTION                // Invert rotation of right motor
  // #define INVERT_L_DIRECTION                // Invert rotation of left motor
  // #define DEBUG_SERIAL_USART3               // right sensor board cable, disable if I2C (nunchuk or lcd) is used!

  // Extra functionality
  // #define CRUISE_CONTROL_SUPPORT            // [-] Flag to enable Cruise Control support. Activation/Deactivation is done by sideboard button or Brake pedal press.
  // #define STANDSTILL_HOLD_ENABLE            // [-] Flag to hold the position when standtill is reached. Only available and makes sense for VOLTAGE or TORQUE mode.
  // #define ELECTRIC_BRAKE_ENABLE             // [-] Flag to enable electric brake and replace the motor "freewheel" with a constant braking when the input torque request is 0. Only available and makes sense for TORQUE mode.
  // #define ELECTRIC_BRAKE_MAX    100         // (0, 500) Maximum electric brake to be applied when input torque request is 0 (pedal fully released).
  // #define ELECTRIC_BRAKE_THRES  120         // (0, 500) Threshold below at which the electric brake starts engaging.
#endif

// Multiple tap detection: default DOUBLE Tap on Brake pedal (4 pulses)
#define MULTIPLE_TAP_NR           2 * 2       // [-] Define tap number: MULTIPLE_TAP_NR = number_of_taps * 2, number_of_taps = 1 (for single taping), 2 (for double tapping), 3 (for triple tapping), etc...
#define MULTIPLE_TAP_HI           600         // [-] Multiple tap detection High hysteresis threshold
#define MULTIPLE_TAP_LO           200         // [-] Multiple tap detection Low hysteresis threshold
#define MULTIPLE_TAP_TIMEOUT      2000        // [ms] Multiple tap detection Timeout period. The taps need to happen within this time window to be accepted.
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################



// ############################ VARIANT_HOVERBOARD SETTINGS ############################
// Communication:         [DONE]
// Balancing controller:  [TODO]
#ifdef VARIANT_HOVERBOARD
  #define FLASH_WRITE_KEY     0x1008          // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #define SIDEBOARD_SERIAL_USART2 1           // left sensor board cable. Number indicates priority for dual-input. Disable if ADC or PPM is used! 
  #define FEEDBACK_SERIAL_USART2
  #define SIDEBOARD_SERIAL_USART3 0           // right sensor board cable. Number indicates priority for dual-input. Disable if I2C (nunchuk or lcd) is used!
  #define FEEDBACK_SERIAL_USART3

  // If an iBUS RC receiver is connected to either Left Sideboard (AUX_INPUT) or Right Sideboard (PRI_INPUT)
  // PRIMARY INPUT:          TYPE, MIN, MID, MAX, DEADBAND /* TYPE: 0:Disabled, 1:Normal Pot, 2:Middle Resting Pot, 3:Auto-detect */
  #define PRI_INPUT1          3, -1000, 0, 1000, 0  // Priority Sideboard can be used to send commands via an iBUS Receiver connected to the sideboard
  #define PRI_INPUT2          3, -1000, 0, 1000, 0  // Priority Sideboard can be used to send commands via an iBUS Receiver connected to the sideboard
  #define AUX_INPUT1          3, -1000, 0, 1000, 0  // not used
  #define AUX_INPUT2          3, -1000, 0, 1000, 0  // not used
#endif
// ######################## END OF VARIANT_HOVERBOARD SETTINGS #########################



// ################################# VARIANT_TRANSPOTTER SETTINGS ############################
//TODO ADD VALIDATION
#ifdef VARIANT_TRANSPOTTER
  #define FLASH_WRITE_KEY     0x1009    // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #define CONTROL_GAMETRAK
  #define SUPPORT_LCD
  // #define SUPPORT_NUNCHUK
  #define GAMETRAK_CONNECTION_NORMAL    // for normal wiring according to the wiki instructions
  // #define GAMETRAK_CONNECTION_ALTERNATE // use this define instead if you messed up the gametrak ADC wiring (steering is speed, and length of the wire is steering)
  #define ROT_P               1.2       // P coefficient for the direction controller. Positive / Negative values to invert gametrak steering direction.
  // during nunchuk control (only relevant when activated)
  #define SPEED_COEFFICIENT   14746     // 0.9f - higher value == stronger. 0.0 to ~2.0?
  #define STEER_COEFFICIENT   8192      // 0.5f - higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
  #define INVERT_R_DIRECTION            // Invert right motor
  #define INVERT_L_DIRECTION            // Invert left motor
  #define PRI_INPUT1          2, -1000, 0, 1000, 0  // dummy input, TRANSPOTTER does not use input limitations
  #define PRI_INPUT2          2, -1000, 0, 1000, 0  // dummy input, TRANSPOTTER does not use input limitations
#endif
// ############################# END OF VARIANT_TRANSPOTTER SETTINGS ########################


// ################################# VARIANT_SKATEBOARD SETTINGS ##############################
#ifdef VARIANT_SKATEBOARD
/* ###### CONTROL VIA RC REMOTE ######
 * right sensor board cable. Connect PB10 to channel 1 and PB11 to channel 2 on receiver.
 * Channel 1: steering, Channel 2: speed.
*/
  #define FLASH_WRITE_KEY     0x1010    // Flash memory writing key. Change this key to ignore the input calibrations from the flash memory and use the ones in config.h
  #undef  CTRL_MOD_REQ
  #define CTRL_MOD_REQ        TRQ_MODE  // SKATEBOARD works best in TORQUE Mode
  // #define CONTROL_PWM_LEFT    0         // use RC PWM as input on the LEFT cable. Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART2!
  #define CONTROL_PWM_RIGHT   0         // use RC PWM as input on the RIGHT cable.  Number indicates priority for dual-input. Disable DEBUG_SERIAL_USART3!

  #define PRI_INPUT1          0, -1000, 0, 1000,   0    // Disabled. TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define PRI_INPUT2          2,  -800, 0,  700, 100    // Active.   TYPE, MIN, MID, MAX, DEADBAND. See INPUT FORMAT section
  #define INPUT_BRK           -400      // (-1000 - 0) Change this value to adjust the braking amount

  #define FILTER              6553      // 0.1f [-] fixdt(0,16,16) lower value == softer filter [0, 65535] = [0.0 - 1.0].
  #define SPEED_COEFFICIENT   16384     // 1.0f [-] fixdt(1,16,14) higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14
  #define STEER_COEFFICIENT   0         // 1.0f [-] fixdt(1,16,14) higher value == stronger. [0, 65535] = [-2.0 - 2.0]. In this case 16384 = 1.0 * 2^14. If you do not want any steering, set it to 0.
  #define INVERT_R_DIRECTION
  #define INVERT_L_DIRECTION
  // #define SUPPORT_BUTTONS_LEFT       // use left sensor board cable for button inputs.  Disable DEBUG_SERIAL_USART2!
  // #define SUPPORT_BUTTONS_RIGHT      // use right sensor board cable for button inputs. Disable DEBUG_SERIAL_USART3!
  // #define STANDSTILL_HOLD_ENABLE     // [-] Flag to hold the position when standtill is reached. Only available and makes sense for VOLTAGE or TORQUE mode.

  #ifdef CONTROL_PWM_RIGHT
    #define DEBUG_SERIAL_USART2         // left sensor cable debug
  #else
    #define DEBUG_SERIAL_USART3         // right sensor cable debug
  #endif
#endif
// ############################# END OF VARIANT_SKATEBOARD SETTINGS ############################



// ########################### UART SETIINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD                  // [-] Start frame definition for serial commands
  #define SERIAL_BUFFER_SIZE      64                      // [bytes] Size of Serial Rx buffer. Make sure it is always larger than the structure size
  #define SERIAL_TIMEOUT          160                     // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           115200                  // UART2 baud rate (long wired cable)
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
#if defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #ifndef USART3_BAUD
    #define USART3_BAUD           115200                  // UART3 baud rate (short wired cable)
  #endif
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
// ########################### UART SETIINGS ############################



// ############################### APPLY DEFAULT SETTINGS ###############################
#ifndef RATE
  #define RATE DEFAULT_RATE
#endif
#ifndef FILTER
  #define FILTER DEFAULT_FILTER
#endif
#ifndef SPEED_COEFFICIENT
  #define SPEED_COEFFICIENT DEFAULT_SPEED_COEFFICIENT
#endif
#ifndef STEER_COEFFICIENT
  #define STEER_COEFFICIENT DEFAULT_STEER_COEFFICIENT
#endif
#if defined(PRI_INPUT1) && defined(PRI_INPUT2) && defined(AUX_INPUT1) && defined(AUX_INPUT2)
  #define INPUTS_NR               2
#else
  #define INPUTS_NR               1
#endif
// ########################### END OF APPLY DEFAULT SETTING ############################



// ############################### VALIDATE SETTINGS ###############################
#if !defined(VARIANT_ADC) && !defined(VARIANT_USART) && !defined(VARIANT_NUNCHUK) && !defined(VARIANT_PPM) && !defined(VARIANT_PWM) && \
    !defined(VARIANT_IBUS) && !defined(VARIANT_HOVERCAR) && !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER) && !defined(VARIANT_SKATEBOARD)
  #error Variant not defined! Please check platformio.ini or Inc/config.h for available variants.
#endif


// General checks
#if defined(CONTROL_SERIAL_USART2) && defined(SIDEBOARD_SERIAL_USART2)
  #error CONTROL_SERIAL_USART2 and SIDEBOARD_SERIAL_USART2 not allowed, choose one.
#endif

#if defined(CONTROL_SERIAL_USART3) && defined(SIDEBOARD_SERIAL_USART3)
  #error CONTROL_SERIAL_USART3 and SIDEBOARD_SERIAL_USART3 not allowed, choose one.
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

#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PPM_RIGHT)
  #error CONTROL_PPM_LEFT and CONTROL_PPM_RIGHT not allowed, choose one.
#endif

#if defined(CONTROL_PWM_LEFT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PWM_LEFT and CONTROL_PWM_RIGHT not allowed, choose one.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && defined(SUPPORT_BUTTONS_RIGHT)
  #error SUPPORT_BUTTONS_LEFT and SUPPORT_BUTTONS_RIGHT not allowed, choose one.
#endif


// LEFT cable checks
#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PPM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PWM_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_PWM_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error SUPPORT_BUTTONS_LEFT and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_LEFT) && (defined(CONTROL_ADC) || defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error SUPPORT_BUTTONS_LEFT and (CONTROL_ADC or CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_ADC) && (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PWM_LEFT))
  #error CONTROL_ADC and (CONTROL_PPM_LEFT or CONTROL_PWM_LEFT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_LEFT) && defined(CONTROL_PWM_LEFT)
  #error CONTROL_PPM_LEFT and CONTROL_PWM_LEFT not allowed. It is on the same cable.
#endif


// RIGHT cable checks
#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_NUNCHUK and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PPM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PWM_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error CONTROL_PWM_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3))
  #error SUPPORT_BUTTONS_RIGHT and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(SUPPORT_BUTTONS_RIGHT) && (defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error SUPPORT_BUTTONS_RIGHT and (CONTROL_NUNCHUK or CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_NUNCHUK) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT) || defined(DEBUG_I2C_LCD))
  #error CONTROL_NUNCHUK and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT or DEBUG_I2C_LCD) not allowed. It is on the same cable.
#endif

#if defined(DEBUG_I2C_LCD) && (defined(CONTROL_PPM_RIGHT) || defined(CONTROL_PWM_RIGHT))
  #error DEBUG_I2C_LCD and (CONTROL_PPM_RIGHT or CONTROL_PWM_RIGHT) not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM_RIGHT) && defined(CONTROL_PWM_RIGHT)
  #error CONTROL_PPM_RIGHT and CONTROL_PWM_RIGHT not allowed. It is on the same cable.
#endif


// Functional checks
#if (defined(CONTROL_PPM_LEFT) || defined(CONTROL_PPM_RIGHT)) && !defined(PPM_NUM_CHANNELS)
  #error Total number of PPM channels needs to be set
#endif
// ############################# END OF VALIDATE SETTINGS ############################

#endif

