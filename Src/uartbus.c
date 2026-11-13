/*
* This file is part of the hoverboard-firmware-hack project.
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

/*
 * VARIANT_UARTBUS: the bus protocols of the split-board firmwares
 *   Hoverboard-Firmware-Hack-Gen2.x-GD32 (REMOTE_UARTBUS, REMOTE_CANSIMPLE) and Hoverboard-Firmware-Hack-Gen2.x-MM32:
 *   - REMOTE_UARTBUS:   v1 frames '/' | type | slave | ... | CRC-16, answered with a 15 byte telemetry frame
 *   - REMOTE_CANSIMPLE: ODrive CANSimple over UART, 0xA5 | node | cmd (bit 7 = request) | len | payload | CRC-16
 * Each one can be built alone; with both they are read on the same USART at the same time (a frame in progress
 * keeps its bytes).
 *
 * There one board drives one motor and has one id. This board drives two, so it has two ids:
 * UARTBUS_ID_LEFT is the left motor, UARTBUS_ID_RIGHT the right one. Each of them is an axis of its own
 * (arm state, control mode, set point, watchdog, limits); the CANSimple broadcast id 63 reaches both.
 *
 * Directions: positive = the direction a positive cmdL / cmdR drives that wheel (INVERT_x_DIRECTION applied),
 * for the commands and for everything measured (speed, position, current).
 *
 * Commands are passed to the motor controller without the input filter / rate limiter / mixer of the other variants
 * (the v1 type 1 frame excepted, it goes through the mixer). Protocol and units: docs/uartbus.md
 */

#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"
#include "rtwtypes.h"
#include "uartbus.h"

#ifdef VARIANT_UARTBUS

#if defined(UARTBUS_SERIAL_USART2)
  extern UART_HandleTypeDef huart2;
  #define BUS_UART      huart2
  #define BUS_RX_CHECK  usart2_rx_check
  #define BUS_BAUD      USART2_BAUD
#else
  extern UART_HandleTypeDef huart3;
  #define BUS_UART      huart3
  #define BUS_RX_CHECK  usart3_rx_check
  #define BUS_BAUD      USART3_BAUD
#endif

extern P    rtP_Left;
extern P    rtP_Right;
extern ExtY rtY_Left;
extern ExtY rtY_Right;
extern uint8_t enable;                  // both motors (main.c: startup, motor errors)
extern volatile uint8_t enableL, enableR, ctrlModReqL, ctrlModReqR;   // bldc.c
extern volatile int32_t hallPosL, hallPosR;                           // bldc.c, motor direction
extern int16_t batVoltageCalib;         // 0.01 V
extern int16_t board_temp_deg_c;        // 0.1 deg C
extern int16_t left_dc_curr;            // 0.01 A, DC link current of the left motor
extern int16_t right_dc_curr;
extern uint8_t timeoutFlgSerial;        // main.c: 3 beeps

#define STEPS         UARTBUS_HALL_STEPS
#define BYTE_GAP_MS   20                // a partial frame older than this is dropped

// ODrive error bits (active errors / disarm reason)
#define ERR_MISSING_ESTIMATE  0x00000008UL   // hall sensors: all 0 (not connected) or all 1 (short), z_errCode 1 / 2
#define ERR_ESTOP_REQUESTED   0x02000000UL
#define ERR_SPINOUT_DETECTED  0x04000000UL   // motor blocked: driving, but it does not turn, z_errCode 4

enum { CTRL_PWM = 0, CTRL_SPEED = 1, CTRL_TORQUE = 2, CTRL_POSITION = 3 };   // v1 drive mode: commutation * 4 + control

typedef struct {
  // fixed
  uint8_t           id;                 // bus id
  int8_t            sign;               // bus direction = sign * motor direction (n_mot, hall counter, iq)
  P                *rtP;
  ExtY             *rtY;
  volatile uint8_t *ena;                // bldc.c: gates of this motor
  volatile uint8_t *ctrlMod;            // bldc.c: VLT_MODE / SPD_MODE / TRQ_MODE of this motor
  volatile int32_t *hall;               // bldc.c: hall step counter
  int16_t          *ibus;               // main.c: DC link current, 0.01 A
  // state
  uint8_t  armed;                       // CLOSED_LOOP_CONTROL
  uint8_t  timedOut;                    // watchdog expired, until the next set point
  uint8_t  control;                     // CTRL_*
  uint32_t disarm;                      // disarm reason (latched until Clear_Errors; arming refused while set)
  float    inVolt;                      // fraction of the max voltage, -1..1   (CTRL_PWM)
  float    inVel;                       // turns/s                               (CTRL_SPEED)
  float    inCur;                       // A, motor current                      (CTRL_TORQUE)
  int32_t  posTarget;                   // hall steps, bus direction              (CTRL_POSITION)
  int32_t  posOffset;                   // hall steps at position 0 turns
  float    velSp;                       // turns/s, position loop output
  uint32_t lastRx;                      // HAL_GetTick() of the last set point
  // settings (RAM)
  float    velLimit;                    // turns/s
  float    posGain;                     // (turns/s)/turn
  float    kt;                          // Nm/A
  uint32_t timeoutMs;
} Axis;

#if defined(INVERT_L_DIRECTION)
  #define SIGN_L  (-1)
#else
  #define SIGN_L  1
#endif
#if defined(INVERT_R_DIRECTION)                 // main.c drives the right motor with -cmdR
  #define SIGN_R  1
#else
  #define SIGN_R  (-1)
#endif

#if CTRL_TYP_SEL == FOC_CTRL && CTRL_MOD_REQ == SPD_MODE
  #define CTRL_INIT CTRL_SPEED
#elif CTRL_TYP_SEL == FOC_CTRL && CTRL_MOD_REQ == TRQ_MODE
  #define CTRL_INIT CTRL_TORQUE
#else
  #define CTRL_INIT CTRL_PWM
#endif

#define AXIS_INIT(i, s, p, y, e, m, h, c) { .id = (i), .sign = (s), .rtP = (p), .rtY = (y), .ena = (e), .ctrlMod = (m), \
  .hall = (h), .ibus = (c), .control = CTRL_INIT, .velLimit = UARTBUS_VEL_LIMIT, .posGain = UARTBUS_POS_GAIN, \
  .kt = UARTBUS_TORQUE_CONST, .timeoutMs = UARTBUS_TIMEOUT_MS }

static Axis axes[2] = {
  AXIS_INIT(UARTBUS_ID_LEFT,  SIGN_L, &rtP_Left,  &rtY_Left,  &enableL, &ctrlModReqL, &hallPosL, &left_dc_curr),
  AXIS_INIT(UARTBUS_ID_RIGHT, SIGN_R, &rtP_Right, &rtY_Right, &enableR, &ctrlModReqR, &hallPosR, &right_dc_curr),
};

static volatile uint8_t poweroffReq = 0;
static uint8_t txBuf[16];               // reply being sent by the TX DMA


// =========================== common ===========================
static void     wrU32(uint8_t *p, uint32_t v)  { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static float    clampF(float f, float lim)     { return f != f ? 0.0f : (f > lim ? lim : (f < -lim ? -lim : f)); }   // NaN -> 0
static int32_t  roundClamp(float f, int32_t lim) {
  if (f != f) return 0;
  return f > lim ? lim : (f < -lim ? -lim : (int32_t)(f >= 0.0f ? f + 0.5f : f - 0.5f));
}

static uint16_t crc16(const uint8_t *p, uint32_t n) {    // CRC-16/XMODEM: poly 0x1021, init 0 (v1 frames and CANSimple)
  uint16_t crc = 0;
  while (n--) {
    crc ^= (uint16_t)(*p++ << 8);
    for (uint8_t i = 0; i < 8; i++) crc = crc & 0x8000 ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static Axis *axisById(uint8_t id) {
  for (uint8_t i = 0; i < 2; i++) if (axes[i].id == id) return &axes[i];
  return NULL;
}

static int32_t axPos(const Axis *a)    { return a->sign * *a->hall; }                          // hall steps, bus direction
static float   axImax(const Axis *a)   { return (float)a->rtP->i_max / (16.0f * A2BIT_CONV); }  // A

static uint32_t axErrors(const Axis *a) {
  uint8_t e = a->rtY->z_errCode;
  return ((e & 3) ? ERR_MISSING_ESTIMATE : 0) | ((e & 4) ? ERR_SPINOUT_DETECTED : 0);
}

static void feed(Axis *a) { a->lastRx = HAL_GetTick(); a->timedOut = 0; }

static void resetInputs(Axis *a) {      // no jump: zero set points, the position target where the wheel is
  a->inVolt = a->inVel = a->inCur = a->velSp = 0.0f;
  a->posTarget = axPos(a);
}

static void disarm(Axis *a, uint32_t reason) {
  a->armed   = 0;
  a->disarm |= reason;
  resetInputs(a);
}

static uint8_t arm(Axis *a) {           // returns 1 if armed
  if (a->armed) return 1;
  if (a->disarm || axErrors(a)) return 0;   // Estop / motor error: Clear_Errors first
  resetInputs(a);
  a->armed = 1;
  feed(a);
  return 1;
}

// commutation: COM_CTRL / SIN_CTRL / FOC_CTRL (= the v1 numbering block / sine / FOC), control: CTRL_*.
// Speed, torque and position need FOC (this firmware's SPD_MODE / TRQ_MODE). The commutation changes only while the
// motor is disarmed or stands still. Returns 0 if not taken (unchanged)
static uint8_t setMode(Axis *a, uint32_t commut, uint32_t control) {
  if (commut > FOC_CTRL || control > CTRL_POSITION) return 0;
  if (commut != FOC_CTRL && control != CTRL_PWM) return 0;
  if (commut != a->rtP->z_ctrlTypSel) {
    if (a->armed && ABS(a->rtY->n_mot) > 10) return 0;
    a->rtP->z_ctrlTypSel = (uint8_T)commut;
  }
  if (control != a->control) {
    a->control = (uint8_t)control;
    resetInputs(a);                     // no stale set point in the new mode
  }
  return 1;
}

static uint8_t txReady(void) { return BUS_UART.gState == HAL_UART_STATE_READY; }   // the last reply is out (txBuf free)


// =========================== RemoteUartBus v1 frames ===========================
#ifdef REMOTE_UARTBUS

#define V1_START      '/'
#define STATE_DISABLE 64                // v1 state bits
#define STATE_SHUTOFF 128

static void v1Reply(const Axis *a) {   // 15 bytes: CD AB | slave | speed 0.1 rpm | 0.01 V | 0.01 A | odometer i32 | CRC
  if (!txReady()) return;
  int16_t  spd  = (int16_t)CLAMP(a->sign * a->rtY->n_mot * 10, -32768, 32767);
  uint16_t volt = (uint16_t)batVoltageCalib;
  int16_t  amp  = *a->ibus;
  txBuf[0]  = 0xCD; txBuf[1] = 0xAB;    // 0xABCD little endian
  txBuf[2]  = a->id;
  txBuf[3]  = (uint8_t)spd;  txBuf[4]  = (uint8_t)((uint16_t)spd >> 8);
  txBuf[5]  = (uint8_t)volt; txBuf[6]  = (uint8_t)(volt >> 8);
  txBuf[7]  = (uint8_t)amp;  txBuf[8]  = (uint8_t)((uint16_t)amp >> 8);
  wrU32(&txBuf[9], (uint32_t)axPos(a));
  uint16_t c = crc16(txBuf, 13);
  txBuf[13] = (uint8_t)c; txBuf[14] = (uint8_t)(c >> 8);
  HAL_UART_Transmit_DMA(&BUS_UART, txBuf, 15);
}

static void v1State(Axis *a, uint8_t st) {   // bit 64: disarm, clear: arm (like the MM32 firmware); bit 128: power off
  if (st & STATE_DISABLE) disarm(a, 0);
  else arm(a);
  if (st & STATE_SHUTOFF) poweroffReq = 1;
}

static void v1Input(Axis *a, int16_t v) {    // the speed field, meaning by the control mode (MM32 units)
  switch (a->control) {
    case CTRL_PWM:      a->inVolt    = clampF((float)v / 1000.0f, 1.0f); break;   // permille
    case CTRL_SPEED:    a->inVel     = (float)v / 60.0f; break;                   // rpm, limited by N_MOT_MAX
    case CTRL_TORQUE:   a->inCur     = clampF((float)v / 100.0f, axImax(a)); break; // 0.01 A
    case CTRL_POSITION: a->posTarget = v; break;                                  // hall steps, odometer
  }
  feed(a);
}

static void v1Frame(const uint8_t *f) {
  Axis *a = axisById(f[2]);
  if (!a) return;
  switch (f[1]) {
    case 0:                             // speed, state: this motor
      v1State(a, f[5]);
      v1Input(a, (int16_t)(f[3] | f[4] << 8));
      break;
    case 1: {                           // speed, steer, state, stateSlave: both motors through the mixer
      int16_t speed = (int16_t)CLAMP((int16_t)(f[3] | f[4] << 8), -1000, 1000);
      int16_t steer = (int16_t)CLAMP((int16_t)(f[5] | f[6] << 8), -1000, 1000);
      int16_t l, r;
      mixerFcn((int16_t)(speed << 4), (int16_t)(steer << 4), &r, &l);
      v1State(&axes[0], f[7]);          // state: left, stateSlave: right
      v1State(&axes[1], f[8]);
      v1Input(&axes[0], l);
      v1Input(&axes[1], r);
      break;
    }
    case 2:                             // config: drive mode = commutation * 4 + control (battery limits, new id: not applied)
      setMode(a, (uint32_t)(f[11] >> 2), (uint32_t)(f[11] & 3));
      break;
  }
  v1Reply(a);                           // every valid frame with this id is answered
}

static int8_t  v1Pos = -1;              // -1: waiting for '/'
static uint8_t v1Buf[15];
static uint8_t v1Size;

static void v1Feed(uint8_t c) {
  static const uint8_t size[3] = { 8, 11, 15 };   // by type
  if (v1Pos < 0) {
    if (c != V1_START) return;
    v1Pos = 0;
  }
  v1Buf[v1Pos++] = c;
  if (v1Pos == 2) {
    if (c > 2) { v1Pos = -1; return; }   // unknown type: look for the next '/'
    v1Size = size[c];
    return;
  }
  if (v1Pos < 3 || v1Pos < v1Size) return;
  v1Pos = -1;
  if ((uint16_t)(v1Buf[v1Size - 2] | v1Buf[v1Size - 1] << 8) != crc16(v1Buf, v1Size - 2u)) return;
  v1Frame(v1Buf);
}

#endif // REMOTE_UARTBUS


// =========================== CANSimple over UART ===========================
#ifdef REMOTE_CANSIMPLE

#define FW_MAJOR      2                 // Get_Version: the protocol level of the GD32 / MM32 implementations
#define FW_MINOR      0
#define FW_REVISION   0
#define CS_START      0xA5
#define CS_BROADCAST  0x3F

enum { AXIS_IDLE = 1, AXIS_CLOSED_LOOP = 8 };
enum { CM_VOLTAGE = 0, CM_TORQUE = 1, CM_VELOCITY = 2, CM_POSITION = 3 };    // ODrive control modes
enum {
  CS_GET_VERSION = 0x00, CS_HEARTBEAT = 0x01, CS_ESTOP = 0x02, CS_GET_ERROR = 0x03, CS_RXSDO = 0x04, CS_TXSDO = 0x05,
  CS_SET_AXIS_STATE = 0x07, CS_GET_ENCODER_ESTIMATES = 0x09, CS_SET_CONTROLLER_MODE = 0x0B, CS_SET_INPUT_POS = 0x0C,
  CS_SET_INPUT_VEL = 0x0D, CS_SET_INPUT_TORQUE = 0x0E, CS_SET_LIMITS = 0x0F, CS_GET_IQ = 0x14, CS_GET_TEMPERATURE = 0x15,
  CS_GET_BUS_VOLTAGE_CURRENT = 0x17, CS_CLEAR_ERRORS = 0x18, CS_SET_ABSOLUTE_POSITION = 0x19, CS_SET_POS_GAIN = 0x1A,
  CS_GET_TORQUES = 0x1C, CS_GET_POWERS = 0x1D, CS_SET_INPUT_VOLTAGE = 0x1E,
};
// RxSdo / TxSdo endpoints: the ids and names of the MM32 firmware's endpoints.json, the ones that exist here
enum {
  EP_VBUS = 0x100, EP_IBUS, EP_POS_ESTIMATE, EP_VEL_ESTIMATE, EP_IQ_MEASURED, EP_IQ_SETPOINT, EP_AXIS_STATE,
  EP_ACTIVE_ERRORS, EP_DISARM_REASON, EP_ODOMETER, EP_CONTROL_MODE, EP_INPUT_MODE, EP_COMMUTATION, EP_INPUT_POS,
  EP_INPUT_VEL, EP_INPUT_TORQUE, EP_INPUT_VOLTAGE, EP_TORQUE_CONSTANT_IN_USE,
  EP_VEL_LIMIT = 0x120, EP_POS_GAIN = 0x121, EP_CURRENT_SOFT_MAX = 0x124, EP_TORQUE_CONSTANT = 0x126,
  EP_POLE_PAIRS = 0x127, EP_WATCHDOG_TIMEOUT = 0x129, EP_NODE_ID = 0x12C, EP_UART_BAUDRATE = 0x12D,
  EP_POS_SETPOINT = 0x135, EP_VEL_SETPOINT = 0x136,
  EP_CLEAR_ERRORS = 0x201, EP_POWER_OFF = 0x202,
};

static float    rdF(const uint8_t *p)          { float f; memcpy(&f, p, 4); return f; }
static uint32_t rdU32(const uint8_t *p)        { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void     wrF(uint8_t *p, float f)       { memcpy(p, &f, 4); }
static float    nanF(void)                     { return __builtin_nanf(""); }

// measurements, bus direction
static float   axTurns(const Axis *a)  { return (float)(axPos(a) - a->posOffset) / (float)STEPS; }
static float   axVel(const Axis *a)    { return (float)(a->sign * a->rtY->n_mot) / 60.0f; }               // turns/s
static uint8_t axFoc(const Axis *a)    { return a->rtP->z_ctrlTypSel == FOC_CTRL; }
static float   axIq(const Axis *a)     { return axFoc(a) ? (float)(a->sign * a->rtY->iq) / (16.0f * A2BIT_CONV) : nanF(); }
static float   axIqSp(const Axis *a)   { return a->control == CTRL_TORQUE ? clampF(a->inCur, axImax(a)) : nanF(); }
static float   vbus(void)              { return (float)batVoltageCalib / 100.0f; }
static float   axIbus(const Axis *a)   { return (float)*a->ibus / 100.0f; }
static uint8_t axPosDone(const Axis *a){ int32_t e = a->posTarget - axPos(a); return a->control != CTRL_POSITION || (e >= -1 && e <= 1); }
static uint8_t axState(const Axis *a)  { return a->armed && enable && !axErrors(a) ? AXIS_CLOSED_LOOP : AXIS_IDLE; }   // the state in effect

static void setAxisState(Axis *a, uint32_t st) {
  if (st == AXIS_IDLE) disarm(a, 0);
  else if (st == AXIS_CLOSED_LOOP && arm(a)) feed(a);
}

static void setControlMode(Axis *a, uint32_t cm) {   // ODrive control mode; needs FOC unless VOLTAGE: switches to it
  static const uint8_t map[4] = { CTRL_PWM, CTRL_TORQUE, CTRL_SPEED, CTRL_POSITION };
  if (cm > CM_POSITION) return;
  setMode(a, map[cm] == CTRL_PWM ? a->rtP->z_ctrlTypSel : FOC_CTRL, map[cm]);
}
static uint8_t controlModeOdrive(const Axis *a) {
  static const uint8_t map[4] = { CM_VOLTAGE, CM_VELOCITY, CM_TORQUE, CM_POSITION };   // by CTRL_*
  return map[a->control];
}

static void setCurrentLimit(Axis *a, float amp) {   // A, also the scale of the torque mode
  if (!(amp > 0.0f)) return;
  if (amp < 1.0f)  amp = 1.0f;
  if (amp > 40.0f) amp = 40.0f;         // the range of the I_MOT_MAX parameter (comms.c)
  a->rtP->i_max = (int16_T)(amp * (16.0f * A2BIT_CONV));
}

static void setPosition(Axis *a, float turns) {     // the wheel is at `turns` now; the target follows (no jump)
  int32_t p = axPos(a);
  a->posOffset = p - roundClamp(turns * (float)STEPS, 2000000000);
  a->posTarget = p;
}

// set points (physical units), taken only in their control mode; return 1 if taken
static uint8_t setInputPos(Axis *a, float turns) {
  if (a->control != CTRL_POSITION || turns != turns) return 0;
  a->posTarget = a->posOffset + roundClamp(turns * (float)STEPS, 2000000000);
  return 1;
}
static uint8_t setInputVel(Axis *a, float tps) {
  if (a->control != CTRL_SPEED) return 0;
  a->inVel = clampF(tps, a->velLimit);
  return 1;
}
static uint8_t setInputTorque(Axis *a, float nm) {
  if (a->control != CTRL_TORQUE) return 0;
  a->inCur = clampF(nm / a->kt, axImax(a));
  return 1;
}
static uint8_t setInputVoltage(Axis *a, float frac) {
  if (a->control != CTRL_PWM) return 0;
  a->inVolt = clampF(frac, 1.0f);
  return 1;
}

static void csReply(uint8_t node, uint8_t cmd, const uint8_t *data, uint8_t len) {
  if (!txReady()) return;
  txBuf[0] = CS_START; txBuf[1] = node; txBuf[2] = cmd; txBuf[3] = len;
  memcpy(&txBuf[4], data, len);
  uint16_t c = crc16(&txBuf[1], 3u + len);   // over node id .. payload
  txBuf[4 + len] = (uint8_t)c;
  txBuf[5 + len] = (uint8_t)(c >> 8);
  HAL_UART_Transmit_DMA(&BUS_UART, txBuf, (uint16_t)(6 + len));
}

static uint8_t csGet(Axis *a, uint8_t cmd, uint8_t *d) {   // Get_ payload, its length; 0xFF: not a readable message
  switch (cmd) {
    case CS_GET_VERSION:
      d[0] = 2; d[1] = 0; d[2] = 0; d[3] = 0;               // protocol 2, hardware unknown
      d[4] = FW_MAJOR; d[5] = FW_MINOR; d[6] = FW_REVISION; d[7] = 1;
      return 8;
    case CS_HEARTBEAT:
      wrU32(d, axErrors(a) | a->disarm);
      d[4] = axState(a); d[5] = 0; d[6] = axPosDone(a); d[7] = 0;
      return 8;
    case CS_GET_ERROR:
      wrU32(d, axErrors(a)); wrU32(d + 4, a->disarm);
      return 8;
    case CS_GET_ENCODER_ESTIMATES:
      wrF(d, axTurns(a)); wrF(d + 4, axVel(a));
      return 8;
    case CS_GET_IQ:
      wrF(d, axIqSp(a)); wrF(d + 4, axIq(a));
      return 8;
    case CS_GET_TEMPERATURE:                                // fet: the board temperature (MCU sensor), motor: none
      wrF(d, (float)board_temp_deg_c / 10.0f); wrF(d + 4, nanF());
      return 8;
    case CS_GET_BUS_VOLTAGE_CURRENT:                        // the current of this motor (the two ids add up to the board's)
      wrF(d, vbus()); wrF(d + 4, axIbus(a));
      return 8;
    case CS_GET_TORQUES:
      wrF(d, axIqSp(a) * a->kt); wrF(d + 4, axIq(a) * a->kt);
      return 8;
    case CS_GET_POWERS:
      wrF(d, vbus() * axIbus(a)); wrF(d + 4, axIq(a) * a->kt * axVel(a) * 6.2831853f);
      return 8;
  }
  return 0xFF;
}

static uint8_t csSdo(Axis *a, const uint8_t *d, uint8_t len, uint8_t *out) {   // RxSdo -> TxSdo payload length, 0: no answer
  uint8_t  write  = d[0] == 1;
  uint16_t ep     = (uint16_t)(d[1] | d[2] << 8);
  uint8_t  hasVal = len >= 8;
  uint32_t v      = hasVal ? rdU32(d + 4) : 0;
  float    f      = hasVal ? rdF(d + 4) : 0.0f;
  uint32_t r      = 0;
  float    x      = 0.0f;
  uint8_t  isF    = 1;
  if (write && !hasVal && ep < EP_CLEAR_ERRORS) return 0;
  switch (ep) {
    case EP_VBUS:            x = vbus(); break;
    case EP_IBUS:            x = axIbus(a); break;
    case EP_POS_ESTIMATE:    if (write) setPosition(a, f); x = axTurns(a); break;
    case EP_VEL_ESTIMATE:    x = axVel(a); break;
    case EP_IQ_MEASURED:     x = axIq(a); break;
    case EP_IQ_SETPOINT:     x = axIqSp(a); break;
    case EP_AXIS_STATE:      if (write) setAxisState(a, v); isF = 0; r = axState(a); break;
    case EP_ACTIVE_ERRORS:   isF = 0; r = axErrors(a); break;
    case EP_DISARM_REASON:   isF = 0; r = a->disarm; break;
    case EP_ODOMETER:        isF = 0; r = (uint32_t)axPos(a); break;
    case EP_CONTROL_MODE:    if (write) setControlMode(a, v); isF = 0; r = controlModeOdrive(a); break;
    case EP_INPUT_MODE:      isF = 0; r = 1; break;         // PASSTHROUGH only
    case EP_COMMUTATION:     if (write) setMode(a, v, a->control); isF = 0; r = a->rtP->z_ctrlTypSel; break;
    case EP_INPUT_POS:       if (write && setInputPos(a, f)) feed(a);
                             x = (float)(a->posTarget - a->posOffset) / (float)STEPS; break;
    case EP_INPUT_VEL:       if (write && setInputVel(a, f)) feed(a);
                             x = a->control == CTRL_SPEED ? a->inVel : 0.0f; break;
    case EP_INPUT_TORQUE:    if (write && setInputTorque(a, f)) feed(a);
                             x = a->control == CTRL_TORQUE ? a->inCur * a->kt : 0.0f; break;
    case EP_INPUT_VOLTAGE:   if (write && setInputVoltage(a, f)) feed(a);
                             x = a->control == CTRL_PWM ? a->inVolt : 0.0f; break;
    case EP_TORQUE_CONSTANT_IN_USE: x = a->kt; break;
    case EP_VEL_LIMIT:       if (write && f > 0.0f) a->velLimit = f; x = a->velLimit; break;
    case EP_POS_GAIN:        if (write && f >= 0.0f) a->posGain = f; x = a->posGain; break;
    case EP_CURRENT_SOFT_MAX:if (write) setCurrentLimit(a, f); x = axImax(a); break;
    case EP_TORQUE_CONSTANT: if (write && f > 0.0f) a->kt = f; x = a->kt; break;
    case EP_POLE_PAIRS:      isF = 0; r = STEPS / 6; break;
    case EP_WATCHDOG_TIMEOUT:if (write && f >= 0.02f && f <= 60.0f) a->timeoutMs = (uint32_t)(f * 1000.0f + 0.5f);
                             x = (float)a->timeoutMs / 1000.0f; break;
    case EP_NODE_ID:         isF = 0; r = a->id; break;    // config.h, read only
    case EP_UART_BAUDRATE:   isF = 0; r = BUS_BAUD; break;
    case EP_POS_SETPOINT:    x = (float)(a->posTarget - a->posOffset) / (float)STEPS; break;
    case EP_VEL_SETPOINT:    x = a->control == CTRL_POSITION ? a->velSp : (a->control == CTRL_SPEED ? a->inVel : 0.0f); break;
    case EP_CLEAR_ERRORS:    if (!write) return 0; a->disarm = 0; break;
    case EP_POWER_OFF:       if (!write) return 0; poweroffReq = 1; break;
    default:                 return 0;
  }
  if (isF) memcpy(&r, &x, 4);
  out[0] = 0; out[1] = (uint8_t)ep; out[2] = (uint8_t)(ep >> 8); out[3] = 0;
  if (write) return 4;                  // a write is confirmed without a value
  wrU32(out + 4, r);
  return 8;
}

static void csHandle(Axis *a, uint8_t answer, uint8_t cmdByte, const uint8_t *d, uint8_t len) {
  uint8_t cmd = cmdByte & 0x1F;
  uint8_t out[8];
  if (cmdByte & 0x80) {                 // request (CAN RTR)
    uint8_t n = csGet(a, cmd, out);
    if (n != 0xFF && answer) csReply(a->id, cmd, out, n);
    return;
  }
  switch (cmd) {
    case CS_ESTOP:                  disarm(a, ERR_ESTOP_REQUESTED); break;
    case CS_RXSDO:                  if (len >= 4) { uint8_t n = csSdo(a, d, len, out); if (n && answer) csReply(a->id, CS_TXSDO, out, n); } break;
    case CS_SET_AXIS_STATE:         if (len >= 4) setAxisState(a, rdU32(d)); break;
    case CS_SET_CONTROLLER_MODE:    if (len >= 4) setControlMode(a, rdU32(d)); break;   // input mode: PASSTHROUGH only
    case CS_SET_INPUT_POS:          if (len >= 4 && setInputPos(a, rdF(d))) feed(a); break;   // feed forwards ignored
    case CS_SET_INPUT_VEL:          if (len >= 4 && setInputVel(a, rdF(d))) feed(a); break;
    case CS_SET_INPUT_TORQUE:       if (len >= 4 && setInputTorque(a, rdF(d))) feed(a); break;
    case CS_SET_INPUT_VOLTAGE:      if (len >= 4 && setInputVoltage(a, rdF(d))) feed(a); break;
    case CS_SET_LIMITS:             if (len >= 8) { float vl = rdF(d); if (vl > 0.0f) a->velLimit = vl; setCurrentLimit(a, rdF(d + 4)); } break;
    case CS_CLEAR_ERRORS:           a->disarm = 0; break;
    case CS_SET_ABSOLUTE_POSITION:  if (len >= 4 && rdF(d) == rdF(d)) setPosition(a, rdF(d)); break;
    case CS_SET_POS_GAIN:           if (len >= 4 && rdF(d) >= 0.0f) a->posGain = rdF(d); break;
    // Set_Vel_Gains, Set_Traj_*, Reboot: not supported, ignored
  }
}

static uint8_t csBuf[14];
static uint8_t csPos = 0;

static uint8_t csFeed(uint8_t c) {      // one byte; 0: not inside a CANSimple frame (the byte is for the v1 parser)
  if (csPos == 0) {
    if (c != CS_START) return 0;
    csBuf[csPos++] = c;
    return 1;
  }
  csBuf[csPos++] = c;
  if (csPos == 4 && csBuf[3] > 8) { csPos = 0; return 1; }   // length out of range: resync
  if (csPos < 4 || csPos < 6 + csBuf[3]) return 1;
  uint8_t len = csBuf[3];
  csPos = 0;
  if ((uint16_t)(csBuf[4 + len] | csBuf[5 + len] << 8) != crc16(&csBuf[1], 3u + len)) return 1;
  uint8_t node = csBuf[1];
  if (node == CS_BROADCAST) {           // every axis acts, none answers
    csHandle(&axes[0], 0, csBuf[2], &csBuf[4], len);
    csHandle(&axes[1], 0, csBuf[2], &csBuf[4], len);
  } else {
    Axis *a = axisById(node);
    if (a) csHandle(a, 1, csBuf[2], &csBuf[4], len);
  }
  return 1;
}

#endif // REMOTE_CANSIMPLE


// =========================== interface ===========================
void uartbusRx(const uint8_t *data, uint32_t len) {
  static uint32_t lastByte = 0;
  uint32_t now = HAL_GetTick();
  if (now - lastByte > BYTE_GAP_MS) {   // a frame does not pause: drop a stale partial one
    #ifdef REMOTE_UARTBUS
      v1Pos = -1;
    #endif
    #ifdef REMOTE_CANSIMPLE
      csPos = 0;
    #endif
  }
  lastByte = now;
  while (len--) {
    uint8_t c = *data++;
    #if defined(REMOTE_UARTBUS) && defined(REMOTE_CANSIMPLE)
      if (v1Pos >= 0 || !csFeed(c)) v1Feed(c);   // a v1 frame in progress keeps its bytes, else 0xA5 starts CANSimple
    #elif defined(REMOTE_UARTBUS)
      v1Feed(c);
    #else
      csFeed(c);
    #endif
  }
}

// The RX DMA half / full interrupts also drain the buffer (otherwise only the idle line does: 64 bytes in a row
// without a pause would overwrite the oldest ones)
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) { if (huart == &BUS_UART) BUS_RX_CHECK(); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)     { if (huart == &BUS_UART) BUS_RX_CHECK(); }

void uartbusUpdate(int16_t *cmdL, int16_t *cmdR) {
  int16_t  out[2];
  uint8_t  timeout = 0;
  uint32_t now = HAL_GetTick();
  uint32_t basepri = __get_BASEPRI();
  __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));   // hold off the USART / DMA interrupts (priority 1), not the motor control (0)

  for (uint8_t i = 0; i < 2; i++) {
    Axis *a = &axes[i];
    uint32_t err = axErrors(a);
    if (err && a->armed) disarm(a, err);   // a motor error disarms, until Clear_Errors
    if (a->armed && !a->timedOut && now - a->lastRx > a->timeoutMs) {
      a->timedOut = 1;                  // watchdog: set point 0, position mode holds where it is
      resetInputs(a);
    }
    if (a->armed && a->timedOut) timeout = 1;

    float   nmax = (float)(a->rtP->n_max >> 4);   // rpm at command 1000 (speed mode)
    float   r    = 0.0f;
    uint8_t mode = VLT_MODE;
    if (nmax < 1.0f) nmax = 1.0f;
    switch (a->control) {
      case CTRL_PWM:    r = a->inVolt * 1000.0f; break;
      case CTRL_SPEED:  mode = SPD_MODE; r = a->inVel * 60000.0f / nmax; break;
      case CTRL_TORQUE: mode = TRQ_MODE; r = a->inCur * 1000.0f / axImax(a); break;
      case CTRL_POSITION: {             // P loop on the hall position -> speed set point
        int32_t e = a->posTarget - axPos(a);
        a->velSp = (e > 1 || e < -1) ? clampF(a->posGain * (float)e / (float)STEPS, a->velLimit) : 0.0f;
        mode = SPD_MODE;
        r = a->velSp * 60000.0f / nmax;
        break;
      }
    }
    if (!a->armed) r = 0.0f;
    *a->ctrlMod = mode;
    *a->ena     = a->armed;
    out[i]      = (int16_t)roundClamp(r, 1000);
  }

  __set_BASEPRI(basepri);
  timeoutFlgSerial = timeout;           // 3 beeps while an armed motor gets no set points
  *cmdL = out[0];
  *cmdR = out[1];
  if (poweroffReq) poweroff();          // v1 state bit 128 / endpoint power_off
}

uint8_t uartbusActive(void) {
  return (axes[0].armed && !axes[0].timedOut) || (axes[1].armed && !axes[1].timedOut);
}

#endif // VARIANT_UARTBUS
