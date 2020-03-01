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
  //#define VARIANT_IBUS        // Variant for RC-Remotes with FLYSKY IBUS
  //#define VARIANT_HOVERCAR    // Variant for HOVERCAR build
  //#define VARIANT_HOVERBOARD  // Variant for HOVERBOARD build
  //#define VARIANT_TRANSPOTTER // Variant for TRANSPOTTER build https://github.com/NiklasFauth/hoverboard-firmware-hack/wiki/Build-Instruction:-TranspOtter https://hackaday.io/project/161891-transpotter-ng
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
// ########################### END OF  DO-NOT-TOUCH SETTINGS ############################



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
    - cmd1 and cmd2: analog normalized input values. INPUT_MIN to INPUT_MAX
    - button1 and button2: digital input values. 0 or 1
    - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
   Outputs:
    - speedR and speedL: normal driving INPUT_MIN to INPUT_MAX
*/
// Control selections
#define CTRL_TYP_SEL    2               // [-] Control type selection: 0 = Commutation , 1 = Sinusoidal, 2 = FOC Field Oriented Control (default)
#define CTRL_MOD_REQ    1               // [-] Control mode request: 0 = Open mode, 1 = VOLTAGE mode (default), 2 = SPEED mode, 3 = TORQUE mode. Note: SPEED and TORQUE modes are only available for FOC!
#define DIAG_ENA        1               // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

// Limitation settings
#define I_MOT_MAX       15              // [A] Maximum motor current limit
#define I_DC_MAX        17              // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
#define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit

// Field Weakening / Phase Advance
#define FIELD_WEAK_ENA  0               // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled (default), 1 = Enabled
#define FIELD_WEAK_MAX  5               // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
#define PHASE_ADV_MAX   25              // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
#define FIELD_WEAK_HI   1500            // [-] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
#define FIELD_WEAK_LO   1000            // [-] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.
// ########################### END OF MOTOR CONTROL ########################



// ############################## DEFAULT SETTINGS ############################
// Default settings will be applied at the end of this config file if not set before
#define INACTIVITY_TIMEOUT      	8       // Minutes of not driving until poweroff. it is not very precise.
#define BEEPS_BACKWARD          	1       // 0 or 1
#define FLASH_WRITE_KEY           0x1234  // Flash writing key, used when writing data to flash memory
// #define SUPPORT_BUTTONS							  // Define for buttons support on ADC, Nunchuck

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



// ############################### DEBUG SERIAL ###############################
/* Connect GND and RX of a 3.3v uart-usb adapter to the left (USART2) or right sensor board cable (USART3)
 * Be careful not to use the red wire of the cable. 15v will destroye evrything.
 * If you are using VARIANT_NUNCHUK, disable it temporarily.
 * enable DEBUG_SERIAL_USART3 or DEBUG_SERIAL_USART2
 * and DEBUG_SERIAL_ASCII use asearial terminal.
 *
 *
 * DEBUG_SERIAL_ASCII output is:
 * // "1:345 2:1337 3:0 4:0 5:0 6:0 7:0 8:0\r\n"
 *
 * 1:   (int16_t)adc_buffer.l_tx2);                                         ADC1
 * 2:   (int16_t)adc_buffer.l_rx2);                                         ADC2
 * 3:   (int16_t)speedR);                                                   output command: [-1000, 1000]
 * 4:   (int16_t)speedL);                                                   output command: [-1000, 1000]
 * 5:   (int16_t)adc_buffer.batt1);                                         Battery adc-value measured by mainboard
 * 6:   (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC));    Battery calibrated voltage multiplied by 100 for verifying battery voltage calibration
 * 7:   (int16_t)board_temp_adcFilt);                                       for board temperature calibration
 * 8:   (int16_t)board_temp_deg_c);                                         Temperature in celcius for verifying board temperature calibration
 *
*/

// #define DEBUG_SERIAL_USART2          // left sensor board cable, disable if ADC or PPM is used!
#if defined(VARIANT_ADC)
  #define DEBUG_SERIAL_USART3          // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
#endif

#ifndef VARIANT_TRANSPOTTER
  //#define DEBUG_SERIAL_SERVOTERM
  #define DEBUG_SERIAL_ASCII
#endif
// ########################### END OF DEBUG SERIAL ############################



// ############################### DEBUG LCD ###############################
//#define DEBUG_I2C_LCD             // standard 16x2 or larger text-lcd via i2c-converter on right sensor board cable
// ########################### END OF DEBUG LCD ############################



// ################################# VARIANT_ADC SETTINGS ############################
#ifdef VARIANT_ADC
/* CONTROL VIA TWO POTENTIOMETERS
 * ADC-calibration to cover the full poti-range:
 * Connect potis to left sensor board cable (0 to 3.3V) (do NOT use the red 15V wire in the cable!). see <How to calibrate>.
 * Turn the potis to minimum position, write value 1 to ADC1_MIN and value 2 to ADC2_MIN
 * Turn the potis to maximum position, write value 1 to ADC1_MAX and value 2 to ADC2_MAX
 * For middle resting potis: Let the potis in the middle resting position, write value 1 to ADC1_MID and value 2 to ADC2_MID
 * Make, flash and test it.
*/
  #define CONTROL_ADC                   // use ADC as input. disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
  // #define ADC_PROTECT_ENA               // ADC Protection Enable flag. Use this flag to make sure the ADC is protected when GND or Vcc wire is disconnected
  #define ADC_PROTECT_TIMEOUT 30        // ADC Protection: number of wrong / missing input commands before safety state is taken
  #define ADC_PROTECT_THRESH  400       // ADC Protection threshold below/above the MIN/MAX ADC values
  // #define ADC1_MID_POT                  // ADC1 middle resting poti: comment-out if NOT a middle resting poti
  #define ADC1_MIN            0         // min ADC1-value while poti at minimum-position (0 - 4095)
  #define ADC1_MID            2048      // mid ADC1-value while poti at minimum-position (ADC1_MIN - ADC1_MAX)
  #define ADC1_MAX            4095      // max ADC1-value while poti at maximum-position (0 - 4095)
  // #define ADC2_MID_POT                  // ADC2 middle resting poti: comment-out if NOT a middle resting poti
  #define ADC2_MIN            0         // min ADC2-value while poti at minimum-position (0 - 4095)
  #define ADC2_MID            2048      // mid ADC2-value while poti at minimum-position (ADC2_MIN - ADC2_MAX)
  #define ADC2_MAX            4095      // max ADC2-value while poti at maximum-position (0 - 4095)
#endif
// ############################# END OF VARIANT_ADC SETTINGS #########################



// ############################ VARIANT_USART SETTINGS ############################
#ifdef VARIANT_USART
  // #define SIDEBOARD_SERIAL_USART2
  // #define CONTROL_SERIAL_USART2         // left sensor board cable, disable if ADC or PPM is used! For Arduino control check the hoverSerial.ino
  // #define FEEDBACK_SERIAL_USART2        // left sensor board cable, disable if ADC or PPM is used!

  // #define SIDEBOARD_SERIAL_USART3
  #define CONTROL_SERIAL_USART3         // right sensor board cable, disable if I2C (nunchuk or lcd) is used! For Arduino control check the hoverSerial.ino
  #define FEEDBACK_SERIAL_USART3        // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
#endif
// ######################## END OF VARIANT_USART SETTINGS #########################



// ################################# VARIANT_NUNCHUK SETTINGS ############################
#ifdef VARIANT_NUNCHUK
  /* left sensor board cable. USART3
   * keep cable short, use shielded cable, use ferrits, stabalize voltage in nunchuk,
   * use the right one of the 2 types of nunchuks, add i2c pullups.
   * use original nunchuk. most clones does not work very well.
   * Recommendation: Nunchuk Breakout Board https://github.com/Jan--Henrik/hoverboard-breakout
  */
  #define CONTROL_NUNCHUK           // use nunchuk as input. disable FEEDBACK_SERIAL_USART3, DEBUG_SERIAL_USART3!
  // # maybe good for ARMCHAIR #
  #define FILTER             3276    //  0.05f
  #define SPEED_COEFFICIENT  8192    //  0.5f
  #define STEER_COEFFICIENT  62259   // -0.2f
#endif
// ############################# END OF VARIANT_NUNCHUK SETTINGS #########################



// ################################# VARIANT_PPM SETTINGS ##############################
#ifdef VARIANT_PPM
/* ###### CONTROL VIA RC REMOTE ######
 * left sensor board cable. Channel 1: steering, Channel 2: speed.
 * https://gist.github.com/peterpoetzi/1b63a4a844162196613871767189bd05
*/
  #define CONTROL_PPM                 // use PPM-Sum as input. disable CONTROL_SERIAL_USART2!
  #define PPM_NUM_CHANNELS    6       // total number of PPM channels to receive, even if they are not used.
#endif
// ############################# END OF VARIANT_PPM SETTINGS ############################



// ################################# VARIANT_IBUS SETTINGS ##############################
#ifdef VARIANT_IBUS
/* CONTROL VIA RC REMOTE WITH FLYSKY IBUS PROTOCOL 
* Connected to Left sensor board cable. Channel 1: steering, Channel 2: speed.
*/
  #define CONTROL_IBUS                                  // use IBUS as input
  #define IBUS_NUM_CHANNELS   14                        // total number of IBUS channels to receive, even if they are not used.
  #define IBUS_LENGTH         0x20
  #define IBUS_COMMAND        0x40

  #undef  USART2_BAUD
  #define USART2_BAUD         115200
  #define CONTROL_SERIAL_USART2                         // left sensor board cable, disable if ADC or PPM is used!
  #define FEEDBACK_SERIAL_USART2                        // left sensor board cable, disable if ADC or PPM is used!
#endif
// ############################# END OF VARIANT_IBUS SETTINGS ############################



// ############################ VARIANT_HOVERCAR SETTINGS ############################
#ifdef VARIANT_HOVERCAR
  #define CONTROL_ADC                   // use ADC as input. disable CONTROL_SERIAL_USART2, FEEDBACK_SERIAL_USART2, DEBUG_SERIAL_USART2!
  #define ADC_PROTECT_ENA               // ADC Protection Enable flag. Use this flag to make sure the ADC is protected when GND or Vcc wire is disconnected
  #define ADC_PROTECT_TIMEOUT 30        // ADC Protection: number of wrong / missing input commands before safety state is taken
  #define ADC_PROTECT_THRESH  300       // ADC Protection threshold below/above the MIN/MAX ADC values
  #define ADC1_MIN            1000      // min ADC1-value while poti at minimum-position (0 - 4095)
  #define ADC1_MAX            2500      // max ADC1-value while poti at maximum-position (0 - 4095)
  #define ADC2_MIN            500       // min ADC2-value while poti at minimum-position (0 - 4095)
  #define ADC2_MAX            2200      // max ADC2-value while poti at maximum-position (0 - 4095)
  #define SPEED_COEFFICIENT   16384     //  1.0f
  #define STEER_COEFFICIENT   0         //  0.0f
  // #define INVERT_R_DIRECTION           // Invert rotation of right motor
  // #define INVERT_L_DIRECTION           // Invert rotation of left motor
  #define SIDEBOARD_SERIAL_USART3
  #define FEEDBACK_SERIAL_USART3        // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
  // #define DEBUG_SERIAL_USART3          // right sensor board cable, disable if I2C (nunchuk or lcd) is used!
#endif

// Multiple tap detection: default DOUBLE Tap on Brake pedal (4 pulses)
#define MULTIPLE_TAP_NR       2 * 2      // [-] Define tap number: MULTIPLE_TAP_NR = number_of_taps * 2, number_of_taps = 1 (for single taping), 2 (for double tapping), 3 (for triple tapping), etc...
#define MULTIPLE_TAP_HI       600        // [-] Multiple tap detection High hysteresis threshold
#define MULTIPLE_TAP_LO       200        // [-] Multiple tap detection Low hysteresis threshold
#define MULTIPLE_TAP_TIMEOUT  2000       // [ms] Multiple tap detection Timeout period. The taps need to happen within this time window to be accepted.
// ######################## END OF VARIANT_HOVERCAR SETTINGS #########################



// ############################ VARIANT_HOVERBOARD SETTINGS ############################
// Communication:         [DONE]
// Balancing controller:  [TODO]
#ifdef VARIANT_HOVERBOARD
  #define SIDEBOARD_SERIAL_USART2       // left sensor board cable, disable if ADC or PPM is used! 
  #define FEEDBACK_SERIAL_USART2
  #define SIDEBOARD_SERIAL_USART3       // right sensor board cable, disable if I2C (nunchuk or lcd) is used!        
  #define FEEDBACK_SERIAL_USART3        
#endif
// ######################## END OF VARIANT_HOVERBOARD SETTINGS #########################



// ################################# VARIANT_TRANSPOTTER SETTINGS ############################
//TODO ADD VALIDATION
#ifdef VARIANT_TRANSPOTTER
  #define CONTROL_GAMETRAK
  #define SUPPORT_LCD
  // #define SUPPORT_NUNCHUK
  #define GAMETRAK_CONNECTION_NORMAL    // for normal wiring according to the wiki instructions
  //#define GAMETRAK_CONNECTION_ALTERNATE // use this define instead if you messed up the gametrak ADC wiring (steering is speed, and length of the wire is steering)
  #define ROT_P               1.2       // P coefficient for the direction controller. Positive / Negative values to invert gametrak steering direction.
  // during nunchuk control (only relevant when activated)
  #define SPEED_COEFFICIENT   14746     // 0.9f - higher value == stronger. 0.0 to ~2.0?
  #define STEER_COEFFICIENT   8192      // 0.5f - higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
  #define INVERT_R_DIRECTION            // Invert right motor
  #define INVERT_L_DIRECTION            // Invert left motor
#endif
// ############################# END OF VARIANT_TRANSPOTTER SETTINGS ########################



// ########################### UART SETIINGS ############################
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || \
    defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define SERIAL_START_FRAME      0xABCD                  // [-] Start frame definition for serial commands
  #define SERIAL_TIMEOUT          160                     // [-] Serial timeout duration for the received data. 160 ~= 0.8 sec. Calculation: 0.8 sec / 0.005 sec
#endif
#if defined(FEEDBACK_SERIAL_USART2) || defined(CONTROL_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2)
  #ifndef USART2_BAUD
    #define USART2_BAUD           38400                   // UART2 baud rate (long wired cable)
  #endif
  #define USART2_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif
#if defined(FEEDBACK_SERIAL_USART3) || defined(CONTROL_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3)
  #define USART3_BAUD             38400                   // UART3 baud rate (short wired cable)
  #define USART3_WORDLENGTH       UART_WORDLENGTH_8B      // UART_WORDLENGTH_8B or UART_WORDLENGTH_9B
#endif

#if defined(DEBUG_SERIAL_USART2)
  #define UART_DMA_CHANNEL_TX DMA1_Channel7
#elif defined(DEBUG_SERIAL_USART3)
  #define UART_DMA_CHANNEL_TX DMA1_Channel2  
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
// ########################### END OF APPLY DEFAULT SETTING ############################



// ############################### VALIDATE SETTINGS ###############################
#if !defined(VARIANT_ADC) && !defined(VARIANT_USART) && !defined(VARIANT_NUNCHUK) && !defined(VARIANT_PPM) && !defined(VARIANT_IBUS) && \
    !defined(VARIANT_HOVERCAR) && !defined(VARIANT_HOVERBOARD) && !defined(VARIANT_TRANSPOTTER)
  #error Variant not defined! Please check platformio.ini or Inc/config.h for available variants.
#endif

#if defined(CONTROL_SERIAL_USART2) && defined(CONTROL_SERIAL_USART3)
  #error CONTROL_SERIAL_USART2 and CONTROL_SERIAL_USART3 not allowed, choose one.
#endif

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

#if defined(CONTROL_ADC) && (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2))
  #error CONTROL_ADC and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if (defined(CONTROL_SERIAL_USART2) || defined(SIDEBOARD_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2)) && defined(CONTROL_PPM)
  #error CONTROL_PPM and SERIAL_USART2 not allowed. It is on the same cable.
#endif

#if (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)) && defined(CONTROL_NUNCHUK)
  #error CONTROL_NUNCHUK and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if (defined(CONTROL_SERIAL_USART3) || defined(SIDEBOARD_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3)) && defined(DEBUG_I2C_LCD)
  #error DEBUG_I2C_LCD and SERIAL_USART3 not allowed. It is on the same cable.
#endif

#if defined(CONTROL_PPM) && defined(CONTROL_ADC) && defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM) && defined(CONTROL_ADC) || defined(CONTROL_ADC) && defined(CONTROL_NUNCHUK) || defined(CONTROL_PPM) && defined(CONTROL_NUNCHUK)
  #error only 1 input method allowed. use CONTROL_PPM or CONTROL_ADC or CONTROL_NUNCHUK.
#endif

#if defined(ADC_PROTECT_ENA) && ((ADC1_MIN - ADC_PROTECT_THRESH) <= 0 || (ADC1_MAX + ADC_PROTECT_THRESH) >= 4095)
  #warning ADC1 Protection NOT possible! Adjust the ADC thresholds.
  #undef ADC_PROTECT_ENA
#endif

#if defined(ADC_PROTECT_ENA) && ((ADC2_MIN - ADC_PROTECT_THRESH) <= 0 || (ADC2_MAX + ADC_PROTECT_THRESH) >= 4095)
  #warning ADC2 Protection NOT possible! Adjust the ADC thresholds.
  #undef ADC_PROTECT_ENA
#endif

#if defined(CONTROL_PPM) && !defined(PPM_NUM_CHANNELS)
  #error Total number of PPM channels needs to be set
#endif
// ############################# END OF VALIDATE SETTINGS ############################

#endif

