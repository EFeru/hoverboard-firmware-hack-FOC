/*
 * File: BLDC_controller.c
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 1.1284
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sun Oct 11 21:38:56 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "BLDC_controller.h"

/* Named constants for Chart: '<S4>/F03_02_Control_Mode_Manager' */
#define IN_ACTIVE                      ((uint8_T)1U)
#define IN_NO_ACTIVE_CHILD             ((uint8_T)0U)
#define IN_OPEN                        ((uint8_T)2U)
#define IN_SPEED_MODE                  ((uint8_T)1U)
#define IN_TORQUE_MODE                 ((uint8_T)2U)
#define IN_VOLTAGE_MODE                ((uint8_T)3U)
#define OPEN_MODE                      ((uint8_T)0U)
#define SPD_MODE                       ((uint8_T)2U)
#define TRQ_MODE                       ((uint8_T)3U)
#define VLT_MODE                       ((uint8_T)1U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( ULLONG_MAX != (0xFFFFFFFFFFFFFFFFULL) ) || ( LLONG_MAX != (0x7FFFFFFFFFFFFFFFLL) )
#error Code was generated for compiler with different sized ulong_long/long_long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif
#endif

uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex);
uint8_T plook_u8u16_evencka(uint16_T u, uint16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern void Counter_Init(DW_Counter *localDW, int16_T rtp_z_cntInit);
extern int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst,
  DW_Counter *localDW);
extern void PI_clamp_fixdt_Init(DW_PI_clamp_fixdt *localDW);
extern void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int32_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, int16_T *rty_out, DW_PI_clamp_fixdt *localDW);
extern void PI_clamp_fixdt_g_Init(DW_PI_clamp_fixdt_i *localDW);
extern void PI_clamp_fixdt_g_Reset(DW_PI_clamp_fixdt_i *localDW);
extern int16_T PI_clamp_fixdt_o(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, DW_PI_clamp_fixdt_i *localDW);
extern void PI_clamp_fixdt_k_Init(DW_PI_clamp_fixdt_e *localDW);
extern void PI_clamp_fixdt_b_Reset(DW_PI_clamp_fixdt_e *localDW);
extern int16_T PI_clamp_fixdt_a(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, DW_PI_clamp_fixdt_e *localDW);
extern void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW);
extern void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T
  rty_y[2], DW_Low_Pass_Filter *localDW);
extern void I_backCalc_fixdt_Init(DW_I_backCalc_fixdt *localDW, int32_T
  rtp_yInit);
extern void I_backCalc_fixdt(int16_T rtu_err, uint16_T rtu_I, uint16_T rtu_Kb,
  int16_T rtu_satMax, int16_T rtu_satMin, int16_T *rty_out, DW_I_backCalc_fixdt *
  localDW);
extern void Counter_b_Init(DW_Counter_l *localDW, uint16_T rtp_z_cntInit);
extern uint16_T Counter_i(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
  DW_Counter_l *localDW);
extern boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW);
extern void Debounce_Filter_Init(DW_Debounce_Filter *localDW);
extern void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T
  rtu_tDeacv, boolean_T *rty_y, DW_Debounce_Filter *localDW);
uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex)
{
  uint8_T bpIndex;
  uint16_T fbpIndex;

  /* Prelookup - Index only
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    fbpIndex = (uint16_T)((uint32_T)(uint16_T)(u - bp0) / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_T)fbpIndex;
    } else {
      bpIndex = (uint8_T)maxIndex;
    }
  }

  return bpIndex;
}

uint8_T plook_u8u16_evencka(uint16_T u, uint16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex)
{
  uint8_T bpIndex;
  uint16_T fbpIndex;

  /* Prelookup - Index only
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    fbpIndex = (uint16_T)((uint32_T)(uint16_T)((uint32_T)u - bp0) / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_T)fbpIndex;
    } else {
      bpIndex = (uint8_T)maxIndex;
    }
  }

  return bpIndex;
}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* System initialize for atomic system: '<S12>/Counter' */
void Counter_Init(DW_Counter *localDW, int16_T rtp_z_cntInit)
{
  /* InitializeConditions for UnitDelay: '<S17>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/* Output and update for atomic system: '<S12>/Counter' */
int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst, DW_Counter *
                localDW)
{
  int16_T rtu_rst_0;
  int16_T rty_cnt_0;

  /* Switch: '<S17>/Switch1' incorporates:
   *  Constant: '<S17>/Constant23'
   *  UnitDelay: '<S17>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S17>/Switch1' */

  /* Sum: '<S15>/Sum1' */
  rty_cnt_0 = (int16_T)(rtu_inc + rtu_rst_0);

  /* MinMax: '<S15>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S17>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S17>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S15>/MinMax' */
  return rty_cnt_0;
}

/* System initialize for atomic system: '<S57>/PI_clamp_fixdt' */
void PI_clamp_fixdt_Init(DW_PI_clamp_fixdt *localDW)
{
  /* InitializeConditions for Delay: '<S71>/Resettable Delay' */
  localDW->icLoad = 1U;
}

/* Output and update for atomic system: '<S57>/PI_clamp_fixdt' */
void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I, int32_T
                    rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
                    rtu_ext_limProt, int16_T *rty_out, DW_PI_clamp_fixdt
                    *localDW)
{
  boolean_T rtb_LowerRelop1_c;
  boolean_T rtb_UpperRelop_e;
  int32_T rtb_Sum1_n;
  int32_T q0;
  int32_T tmp;
  int16_T tmp_0;

  /* Sum: '<S68>/Sum2' incorporates:
   *  Product: '<S68>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  if ((q0 < 0) && (rtu_ext_limProt < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (rtu_ext_limProt > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += rtu_ext_limProt;
  }

  /* Delay: '<S71>/Resettable Delay' */
  if (localDW->icLoad != 0) {
    localDW->ResettableDelay_DSTATE = rtu_init;
  }

  /* Switch: '<S68>/Switch1' incorporates:
   *  Constant: '<S68>/Constant'
   *  Sum: '<S68>/Sum2'
   *  UnitDelay: '<S68>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  /* End of Switch: '<S68>/Switch1' */

  /* Sum: '<S71>/Sum1' incorporates:
   *  Delay: '<S71>/Resettable Delay'
   */
  rtb_Sum1_n = tmp + localDW->ResettableDelay_DSTATE;

  /* Product: '<S68>/Divide5' */
  tmp = (rtu_err * rtu_P) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* Sum: '<S68>/Sum1' incorporates:
   *  DataTypeConversion: '<S71>/Data Type Conversion1'
   *  Product: '<S68>/Divide5'
   */
  tmp = (((rtb_Sum1_n >> 16) << 1) + tmp) >> 1;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* RelationalOperator: '<S72>/LowerRelop1' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  rtb_LowerRelop1_c = ((int16_T)tmp > rtu_satMax);

  /* RelationalOperator: '<S72>/UpperRelop' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  rtb_UpperRelop_e = ((int16_T)tmp < rtu_satMin);

  /* Switch: '<S72>/Switch1' incorporates:
   *  Sum: '<S68>/Sum1'
   *  Switch: '<S72>/Switch3'
   */
  if (rtb_LowerRelop1_c) {
    *rty_out = rtu_satMax;
  } else if (rtb_UpperRelop_e) {
    /* Switch: '<S72>/Switch3' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = (int16_T)tmp;
  }

  /* End of Switch: '<S72>/Switch1' */

  /* Signum: '<S70>/SignDeltaU2' incorporates:
   *  Sum: '<S68>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S70>/SignDeltaU2' */

  /* Signum: '<S70>/SignDeltaU3' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  if ((int16_T)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_T)((int16_T)tmp > 0);
  }

  /* End of Signum: '<S70>/SignDeltaU3' */

  /* Update for UnitDelay: '<S68>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S70>/DataTypeConv4'
   *  Logic: '<S68>/AND1'
   *  Logic: '<S70>/AND1'
   *  RelationalOperator: '<S70>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp_0) && (rtb_LowerRelop1_c ||
    rtb_UpperRelop_e));

  /* Update for Delay: '<S71>/Resettable Delay' */
  localDW->icLoad = 0U;
  localDW->ResettableDelay_DSTATE = rtb_Sum1_n;
}

/* System initialize for atomic system: '<S55>/PI_clamp_fixdt' */
void PI_clamp_fixdt_g_Init(DW_PI_clamp_fixdt_i *localDW)
{
  /* InitializeConditions for Delay: '<S61>/Resettable Delay' */
  localDW->icLoad = 1U;
}

/* System reset for atomic system: '<S55>/PI_clamp_fixdt' */
void PI_clamp_fixdt_g_Reset(DW_PI_clamp_fixdt_i *localDW)
{
  /* InitializeConditions for UnitDelay: '<S59>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE = false;

  /* InitializeConditions for Delay: '<S61>/Resettable Delay' */
  localDW->icLoad = 1U;
}

/* Output and update for atomic system: '<S55>/PI_clamp_fixdt' */
int16_T PI_clamp_fixdt_o(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, DW_PI_clamp_fixdt_i *localDW)
{
  boolean_T rtb_LowerRelop1_l;
  boolean_T rtb_UpperRelop_f2;
  int32_T rtb_Sum1_o;
  int32_T q0;
  int32_T tmp;
  int16_T tmp_0;
  int16_T rty_out_0;

  /* Sum: '<S59>/Sum2' incorporates:
   *  Product: '<S59>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  if ((q0 < 0) && (rtu_ext_limProt < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (rtu_ext_limProt > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += rtu_ext_limProt;
  }

  /* Delay: '<S61>/Resettable Delay' */
  if (localDW->icLoad != 0) {
    localDW->ResettableDelay_DSTATE = rtu_init << 16;
  }

  /* Switch: '<S59>/Switch1' incorporates:
   *  Constant: '<S59>/Constant'
   *  Sum: '<S59>/Sum2'
   *  UnitDelay: '<S59>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  /* End of Switch: '<S59>/Switch1' */

  /* Sum: '<S61>/Sum1' incorporates:
   *  Delay: '<S61>/Resettable Delay'
   */
  rtb_Sum1_o = tmp + localDW->ResettableDelay_DSTATE;

  /* Product: '<S59>/Divide5' */
  tmp = (rtu_err * rtu_P) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* Sum: '<S59>/Sum1' incorporates:
   *  DataTypeConversion: '<S61>/Data Type Conversion1'
   *  Product: '<S59>/Divide5'
   */
  tmp = (((rtb_Sum1_o >> 16) << 1) + tmp) >> 1;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* RelationalOperator: '<S62>/LowerRelop1' incorporates:
   *  Sum: '<S59>/Sum1'
   */
  rtb_LowerRelop1_l = ((int16_T)tmp > rtu_satMax);

  /* RelationalOperator: '<S62>/UpperRelop' incorporates:
   *  Sum: '<S59>/Sum1'
   */
  rtb_UpperRelop_f2 = ((int16_T)tmp < rtu_satMin);

  /* Switch: '<S62>/Switch1' incorporates:
   *  Sum: '<S59>/Sum1'
   *  Switch: '<S62>/Switch3'
   */
  if (rtb_LowerRelop1_l) {
    rty_out_0 = rtu_satMax;
  } else if (rtb_UpperRelop_f2) {
    /* Switch: '<S62>/Switch3' */
    rty_out_0 = rtu_satMin;
  } else {
    rty_out_0 = (int16_T)tmp;
  }

  /* End of Switch: '<S62>/Switch1' */

  /* Signum: '<S60>/SignDeltaU2' incorporates:
   *  Sum: '<S59>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S60>/SignDeltaU2' */

  /* Signum: '<S60>/SignDeltaU3' incorporates:
   *  Sum: '<S59>/Sum1'
   */
  if ((int16_T)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_T)((int16_T)tmp > 0);
  }

  /* End of Signum: '<S60>/SignDeltaU3' */

  /* Update for UnitDelay: '<S59>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S60>/DataTypeConv4'
   *  Logic: '<S59>/AND1'
   *  Logic: '<S60>/AND1'
   *  RelationalOperator: '<S60>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp_0) && (rtb_LowerRelop1_l ||
    rtb_UpperRelop_f2));

  /* Update for Delay: '<S61>/Resettable Delay' */
  localDW->icLoad = 0U;
  localDW->ResettableDelay_DSTATE = rtb_Sum1_o;
  return rty_out_0;
}

/* System initialize for atomic system: '<S56>/PI_clamp_fixdt' */
void PI_clamp_fixdt_k_Init(DW_PI_clamp_fixdt_e *localDW)
{
  /* InitializeConditions for Delay: '<S66>/Resettable Delay' */
  localDW->icLoad = 1U;
}

/* System reset for atomic system: '<S56>/PI_clamp_fixdt' */
void PI_clamp_fixdt_b_Reset(DW_PI_clamp_fixdt_e *localDW)
{
  /* InitializeConditions for UnitDelay: '<S63>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE = false;

  /* InitializeConditions for Delay: '<S66>/Resettable Delay' */
  localDW->icLoad = 1U;
}

/* Output and update for atomic system: '<S56>/PI_clamp_fixdt' */
int16_T PI_clamp_fixdt_a(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, DW_PI_clamp_fixdt_e *localDW)
{
  boolean_T rtb_LowerRelop1_lt;
  boolean_T rtb_UpperRelop_i;
  int16_T rtb_Sum1_n;
  int16_T tmp;
  int32_T tmp_0;
  int32_T q0;
  int16_T rty_out_0;

  /* Sum: '<S63>/Sum2' incorporates:
   *  Product: '<S63>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  if ((q0 < 0) && (rtu_ext_limProt < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (rtu_ext_limProt > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += rtu_ext_limProt;
  }

  /* Delay: '<S66>/Resettable Delay' */
  if (localDW->icLoad != 0) {
    localDW->ResettableDelay_DSTATE = rtu_init;
  }

  /* Switch: '<S63>/Switch1' incorporates:
   *  Constant: '<S63>/Constant'
   *  Sum: '<S63>/Sum2'
   *  UnitDelay: '<S63>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp = 0;
  } else {
    tmp = (int16_T)(((q0 < 0 ? 65535 : 0) + q0) >> 16);
  }

  /* End of Switch: '<S63>/Switch1' */

  /* Sum: '<S66>/Sum1' incorporates:
   *  Delay: '<S66>/Resettable Delay'
   */
  rtb_Sum1_n = (int16_T)(tmp + localDW->ResettableDelay_DSTATE);

  /* Product: '<S63>/Divide5' */
  tmp_0 = (rtu_err * rtu_P) >> 11;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else {
    if (tmp_0 < -32768) {
      tmp_0 = -32768;
    }
  }

  /* Sum: '<S63>/Sum1' incorporates:
   *  Product: '<S63>/Divide5'
   */
  tmp_0 = ((rtb_Sum1_n << 1) + tmp_0) >> 1;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else {
    if (tmp_0 < -32768) {
      tmp_0 = -32768;
    }
  }

  /* RelationalOperator: '<S67>/LowerRelop1' incorporates:
   *  Sum: '<S63>/Sum1'
   */
  rtb_LowerRelop1_lt = ((int16_T)tmp_0 > rtu_satMax);

  /* RelationalOperator: '<S67>/UpperRelop' incorporates:
   *  Sum: '<S63>/Sum1'
   */
  rtb_UpperRelop_i = ((int16_T)tmp_0 < rtu_satMin);

  /* Switch: '<S67>/Switch1' incorporates:
   *  Sum: '<S63>/Sum1'
   *  Switch: '<S67>/Switch3'
   */
  if (rtb_LowerRelop1_lt) {
    rty_out_0 = rtu_satMax;
  } else if (rtb_UpperRelop_i) {
    /* Switch: '<S67>/Switch3' */
    rty_out_0 = rtu_satMin;
  } else {
    rty_out_0 = (int16_T)tmp_0;
  }

  /* End of Switch: '<S67>/Switch1' */

  /* Signum: '<S65>/SignDeltaU2' incorporates:
   *  Sum: '<S63>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S65>/SignDeltaU2' */

  /* Signum: '<S65>/SignDeltaU3' incorporates:
   *  Sum: '<S63>/Sum1'
   */
  if ((int16_T)tmp_0 < 0) {
    tmp = -1;
  } else {
    tmp = (int16_T)((int16_T)tmp_0 > 0);
  }

  /* End of Signum: '<S65>/SignDeltaU3' */

  /* Update for UnitDelay: '<S63>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S65>/DataTypeConv4'
   *  Logic: '<S63>/AND1'
   *  Logic: '<S65>/AND1'
   *  RelationalOperator: '<S65>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp) && (rtb_LowerRelop1_lt ||
    rtb_UpperRelop_i));

  /* Update for Delay: '<S66>/Resettable Delay' */
  localDW->icLoad = 0U;
  localDW->ResettableDelay_DSTATE = rtb_Sum1_n;
  return rty_out_0;
}

/* System reset for atomic system: '<S43>/Low_Pass_Filter' */
void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S53>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[0] = 0;
  localDW->UnitDelay1_DSTATE[1] = 0;
}

/* Output and update for atomic system: '<S43>/Low_Pass_Filter' */
void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T rty_y[2],
                     DW_Low_Pass_Filter *localDW)
{
  int32_T rtb_Sum3_g;

  /* Sum: '<S53>/Sum2' incorporates:
   *  UnitDelay: '<S53>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_u[0] - (localDW->UnitDelay1_DSTATE[0] >> 16);
  if (rtb_Sum3_g > 32767) {
    rtb_Sum3_g = 32767;
  } else {
    if (rtb_Sum3_g < -32768) {
      rtb_Sum3_g = -32768;
    }
  }

  /* Sum: '<S53>/Sum3' incorporates:
   *  Product: '<S53>/Divide3'
   *  Sum: '<S53>/Sum2'
   *  UnitDelay: '<S53>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_coef * rtb_Sum3_g + localDW->UnitDelay1_DSTATE[0];

  /* DataTypeConversion: '<S53>/Data Type Conversion' */
  rty_y[0] = (int16_T)(rtb_Sum3_g >> 16);

  /* Update for UnitDelay: '<S53>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[0] = rtb_Sum3_g;

  /* Sum: '<S53>/Sum2' incorporates:
   *  UnitDelay: '<S53>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_u[1] - (localDW->UnitDelay1_DSTATE[1] >> 16);
  if (rtb_Sum3_g > 32767) {
    rtb_Sum3_g = 32767;
  } else {
    if (rtb_Sum3_g < -32768) {
      rtb_Sum3_g = -32768;
    }
  }

  /* Sum: '<S53>/Sum3' incorporates:
   *  Product: '<S53>/Divide3'
   *  Sum: '<S53>/Sum2'
   *  UnitDelay: '<S53>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_coef * rtb_Sum3_g + localDW->UnitDelay1_DSTATE[1];

  /* DataTypeConversion: '<S53>/Data Type Conversion' */
  rty_y[1] = (int16_T)(rtb_Sum3_g >> 16);

  /* Update for UnitDelay: '<S53>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[1] = rtb_Sum3_g;
}

/*
 * System initialize for atomic system:
 *    '<S76>/I_backCalc_fixdt'
 *    '<S76>/I_backCalc_fixdt1'
 *    '<S75>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt_Init(DW_I_backCalc_fixdt *localDW, int32_T rtp_yInit)
{
  /* InitializeConditions for UnitDelay: '<S83>/UnitDelay' */
  localDW->UnitDelay_DSTATE_h = rtp_yInit;
}

/*
 * Output and update for atomic system:
 *    '<S76>/I_backCalc_fixdt'
 *    '<S76>/I_backCalc_fixdt1'
 *    '<S75>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt(int16_T rtu_err, uint16_T rtu_I, uint16_T rtu_Kb, int16_T
                      rtu_satMax, int16_T rtu_satMin, int16_T *rty_out,
                      DW_I_backCalc_fixdt *localDW)
{
  int32_T rtb_Sum1_m;
  int16_T rtb_DataTypeConversion1_no;

  /* Sum: '<S81>/Sum2' incorporates:
   *  Product: '<S81>/Divide2'
   *  UnitDelay: '<S81>/UnitDelay'
   */
  rtb_Sum1_m = (rtu_err * rtu_I) >> 4;
  if ((rtb_Sum1_m < 0) && (localDW->UnitDelay_DSTATE < MIN_int32_T - rtb_Sum1_m))
  {
    rtb_Sum1_m = MIN_int32_T;
  } else if ((rtb_Sum1_m > 0) && (localDW->UnitDelay_DSTATE > MAX_int32_T
              - rtb_Sum1_m)) {
    rtb_Sum1_m = MAX_int32_T;
  } else {
    rtb_Sum1_m += localDW->UnitDelay_DSTATE;
  }

  /* End of Sum: '<S81>/Sum2' */

  /* Sum: '<S83>/Sum1' incorporates:
   *  UnitDelay: '<S83>/UnitDelay'
   */
  rtb_Sum1_m += localDW->UnitDelay_DSTATE_h;

  /* DataTypeConversion: '<S83>/Data Type Conversion1' */
  rtb_DataTypeConversion1_no = (int16_T)(rtb_Sum1_m >> 12);

  /* Switch: '<S84>/Switch2' incorporates:
   *  RelationalOperator: '<S84>/LowerRelop1'
   *  RelationalOperator: '<S84>/UpperRelop'
   *  Switch: '<S84>/Switch'
   */
  if (rtb_DataTypeConversion1_no > rtu_satMax) {
    *rty_out = rtu_satMax;
  } else if (rtb_DataTypeConversion1_no < rtu_satMin) {
    /* Switch: '<S84>/Switch' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = rtb_DataTypeConversion1_no;
  }

  /* End of Switch: '<S84>/Switch2' */

  /* Update for UnitDelay: '<S81>/UnitDelay' incorporates:
   *  Product: '<S81>/Divide1'
   *  Sum: '<S81>/Sum3'
   */
  localDW->UnitDelay_DSTATE = (int16_T)(*rty_out - rtb_DataTypeConversion1_no) *
    rtu_Kb;

  /* Update for UnitDelay: '<S83>/UnitDelay' */
  localDW->UnitDelay_DSTATE_h = rtb_Sum1_m;
}

/*
 * System initialize for atomic system:
 *    '<S23>/Counter'
 *    '<S22>/Counter'
 */
void Counter_b_Init(DW_Counter_l *localDW, uint16_T rtp_z_cntInit)
{
  /* InitializeConditions for UnitDelay: '<S28>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/*
 * Output and update for atomic system:
 *    '<S23>/Counter'
 *    '<S22>/Counter'
 */
uint16_T Counter_i(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
                   DW_Counter_l *localDW)
{
  uint16_T rtu_rst_0;
  uint16_T rty_cnt_0;

  /* Switch: '<S28>/Switch1' incorporates:
   *  Constant: '<S28>/Constant23'
   *  UnitDelay: '<S28>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0U;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S28>/Switch1' */

  /* Sum: '<S27>/Sum1' */
  rty_cnt_0 = (uint16_T)((uint32_T)rtu_inc + rtu_rst_0);

  /* MinMax: '<S27>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S28>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S28>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S27>/MinMax' */
  return rty_cnt_0;
}

/*
 * Output and update for atomic system:
 *    '<S19>/either_edge'
 *    '<S3>/either_edge'
 */
boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW)
{
  boolean_T rty_y_0;

  /* RelationalOperator: '<S24>/Relational Operator' incorporates:
   *  UnitDelay: '<S24>/UnitDelay'
   */
  rty_y_0 = (rtu_u != localDW->UnitDelay_DSTATE);

  /* Update for UnitDelay: '<S24>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtu_u;
  return rty_y_0;
}

/* System initialize for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter_Init(DW_Debounce_Filter *localDW)
{
  /* SystemInitialize for IfAction SubSystem: '<S19>/Qualification' */

  /* SystemInitialize for Atomic SubSystem: '<S23>/Counter' */
  Counter_b_Init(&localDW->Counter_i0, 0U);

  /* End of SystemInitialize for SubSystem: '<S23>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S19>/Qualification' */

  /* SystemInitialize for IfAction SubSystem: '<S19>/Dequalification' */

  /* SystemInitialize for Atomic SubSystem: '<S22>/Counter' */
  Counter_b_Init(&localDW->Counter_h, 0U);

  /* End of SystemInitialize for SubSystem: '<S22>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S19>/Dequalification' */
}

/* Output and update for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T rtu_tDeacv,
                     boolean_T *rty_y, DW_Debounce_Filter *localDW)
{
  boolean_T rtb_UnitDelay_o;
  uint16_T rtb_Sum1_l;
  boolean_T rtb_RelationalOperator_f;

  /* UnitDelay: '<S19>/UnitDelay' */
  rtb_UnitDelay_o = localDW->UnitDelay_DSTATE;

  /* Outputs for Atomic SubSystem: '<S19>/either_edge' */
  rtb_RelationalOperator_f = either_edge(rtu_u, &localDW->either_edge_k);

  /* End of Outputs for SubSystem: '<S19>/either_edge' */

  /* If: '<S19>/If2' incorporates:
   *  Constant: '<S22>/Constant6'
   *  Constant: '<S23>/Constant6'
   *  Inport: '<S21>/yPrev'
   *  Logic: '<S19>/Logical Operator1'
   *  Logic: '<S19>/Logical Operator2'
   *  Logic: '<S19>/Logical Operator3'
   *  Logic: '<S19>/Logical Operator4'
   */
  if (rtu_u && (!rtb_UnitDelay_o)) {
    /* Outputs for IfAction SubSystem: '<S19>/Qualification' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S23>/Counter' */
    rtb_Sum1_l = (uint16_T) Counter_i(1U, rtu_tAcv, rtb_RelationalOperator_f,
      &localDW->Counter_i0);

    /* End of Outputs for SubSystem: '<S23>/Counter' */

    /* Switch: '<S23>/Switch2' incorporates:
     *  Constant: '<S23>/Constant6'
     *  RelationalOperator: '<S23>/Relational Operator2'
     */
    *rty_y = (rtb_Sum1_l > rtu_tAcv);

    /* End of Outputs for SubSystem: '<S19>/Qualification' */
  } else if ((!rtu_u) && rtb_UnitDelay_o) {
    /* Outputs for IfAction SubSystem: '<S19>/Dequalification' incorporates:
     *  ActionPort: '<S22>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S22>/Counter' */
    rtb_Sum1_l = (uint16_T) Counter_i(1U, rtu_tDeacv, rtb_RelationalOperator_f,
      &localDW->Counter_h);

    /* End of Outputs for SubSystem: '<S22>/Counter' */

    /* Switch: '<S22>/Switch2' incorporates:
     *  Constant: '<S22>/Constant6'
     *  RelationalOperator: '<S22>/Relational Operator2'
     */
    *rty_y = !(rtb_Sum1_l > rtu_tDeacv);

    /* End of Outputs for SubSystem: '<S19>/Dequalification' */
  } else {
    /* Outputs for IfAction SubSystem: '<S19>/Default' incorporates:
     *  ActionPort: '<S21>/Action Port'
     */
    *rty_y = rtb_UnitDelay_o;

    /* End of Outputs for SubSystem: '<S19>/Default' */
  }

  /* End of If: '<S19>/If2' */

  /* Update for UnitDelay: '<S19>/UnitDelay' */
  localDW->UnitDelay_DSTATE = *rty_y;
}

/* Model step function */
void BLDC_controller_step(RT_MODEL *const rtM)
{
  P *rtP = ((P *) rtM->defaultParam);
  DW *rtDW = ((DW *) rtM->dwork);
  ExtU *rtU = (ExtU *) rtM->inputs;
  ExtY *rtY = (ExtY *) rtM->outputs;
  uint8_T rtb_Sum;
  boolean_T rtb_LogicalOperator;
  int8_T rtb_Sum2_h;
  boolean_T rtb_RelationalOperator4_d;
  boolean_T rtb_RelationalOperator1_m;
  uint8_T rtb_Sum_l;
  boolean_T rtb_LogicalOperator2_p;
  boolean_T rtb_LogicalOperator4;
  int16_T rtb_Switch2_k;
  int16_T rtb_Abs5;
  int16_T rtb_DataTypeConversion2;
  int16_T rtb_Switch1_l;
  int16_T rtb_Saturation;
  int16_T rtb_Saturation1;
  int16_T rtb_Merge;
  int16_T rtb_Switch2_l;
  int16_T rtb_toNegative;
  int32_T rtb_DataTypeConversion;
  int32_T rtb_Switch1;
  int32_T rtb_Sum1;
  int32_T rtb_Gain3;
  int16_T rtb_TmpSignalConversionAtLow_Pa[2];
  int16_T tmp[4];
  int8_T UnitDelay3;
  int16_T rtb_Merge_f_idx_2;

  /* Outputs for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Sum: '<S10>/Sum' incorporates:
   *  Gain: '<S10>/g_Ha'
   *  Gain: '<S10>/g_Hb'
   *  Inport: '<Root>/b_hallA '
   *  Inport: '<Root>/b_hallB'
   *  Inport: '<Root>/b_hallC'
   */
  rtb_Sum = (uint8_T)((uint32_T)(uint8_T)((uint32_T)(uint8_T)(rtU->b_hallA << 2)
    + (uint8_T)(rtU->b_hallB << 1)) + rtU->b_hallC);

  /* Logic: '<S9>/Logical Operator' incorporates:
   *  Inport: '<Root>/b_hallA '
   *  Inport: '<Root>/b_hallB'
   *  Inport: '<Root>/b_hallC'
   *  UnitDelay: '<S9>/UnitDelay1'
   *  UnitDelay: '<S9>/UnitDelay2'
   *  UnitDelay: '<S9>/UnitDelay3'
   */
  rtb_LogicalOperator = (boolean_T)((rtU->b_hallA != 0) ^ (rtU->b_hallB != 0) ^
    (rtU->b_hallC != 0) ^ (rtDW->UnitDelay3_DSTATE_fy != 0) ^
    (rtDW->UnitDelay1_DSTATE != 0)) ^ (rtDW->UnitDelay2_DSTATE_f != 0);

  /* If: '<S12>/If2' incorporates:
   *  If: '<S2>/If2'
   *  Inport: '<S16>/z_counterRawPrev'
   *  UnitDelay: '<S12>/UnitDelay3'
   */
  if (rtb_LogicalOperator) {
    /* Outputs for IfAction SubSystem: '<S2>/F01_03_Direction_Detection' incorporates:
     *  ActionPort: '<S11>/Action Port'
     */
    /* UnitDelay: '<S11>/UnitDelay3' */
    UnitDelay3 = rtDW->Switch2_e;

    /* Sum: '<S11>/Sum2' incorporates:
     *  Constant: '<S10>/vec_hallToPos'
     *  Selector: '<S10>/Selector'
     *  UnitDelay: '<S11>/UnitDelay2'
     */
    rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] -
                          rtDW->UnitDelay2_DSTATE_b);

    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/Constant20'
     *  Constant: '<S11>/Constant23'
     *  Constant: '<S11>/Constant24'
     *  Constant: '<S11>/Constant8'
     *  Logic: '<S11>/Logical Operator3'
     *  RelationalOperator: '<S11>/Relational Operator1'
     *  RelationalOperator: '<S11>/Relational Operator6'
     */
    if ((rtb_Sum2_h == 1) || (rtb_Sum2_h == -5)) {
      rtDW->Switch2_e = 1;
    } else {
      rtDW->Switch2_e = -1;
    }

    /* End of Switch: '<S11>/Switch2' */

    /* Update for UnitDelay: '<S11>/UnitDelay2' incorporates:
     *  Constant: '<S10>/vec_hallToPos'
     *  Selector: '<S10>/Selector'
     */
    rtDW->UnitDelay2_DSTATE_b = rtConstP.vec_hallToPos_Value[rtb_Sum];

    /* End of Outputs for SubSystem: '<S2>/F01_03_Direction_Detection' */

    /* Outputs for IfAction SubSystem: '<S12>/Raw_Motor_Speed_Estimation' incorporates:
     *  ActionPort: '<S16>/Action Port'
     */
    rtDW->z_counterRawPrev = rtDW->UnitDelay3_DSTATE;

    /* Sum: '<S16>/Sum7' incorporates:
     *  Inport: '<S16>/z_counterRawPrev'
     *  UnitDelay: '<S12>/UnitDelay3'
     *  UnitDelay: '<S16>/UnitDelay4'
     */
    rtb_Switch2_k = (int16_T)(rtDW->z_counterRawPrev - rtDW->UnitDelay4_DSTATE);

    /* Abs: '<S16>/Abs2' */
    if (rtb_Switch2_k < 0) {
      rtb_Switch1_l = (int16_T)-rtb_Switch2_k;
    } else {
      rtb_Switch1_l = rtb_Switch2_k;
    }

    /* End of Abs: '<S16>/Abs2' */

    /* Relay: '<S16>/dz_cntTrnsDet' */
    if (rtb_Switch1_l >= rtP->dz_cntTrnsDetHi) {
      rtDW->dz_cntTrnsDet_Mode = true;
    } else {
      if (rtb_Switch1_l <= rtP->dz_cntTrnsDetLo) {
        rtDW->dz_cntTrnsDet_Mode = false;
      }
    }

    rtDW->dz_cntTrnsDet = rtDW->dz_cntTrnsDet_Mode;

    /* End of Relay: '<S16>/dz_cntTrnsDet' */

    /* RelationalOperator: '<S16>/Relational Operator4' */
    rtb_RelationalOperator4_d = (rtDW->Switch2_e != UnitDelay3);

    /* Switch: '<S16>/Switch3' incorporates:
     *  Constant: '<S16>/Constant4'
     *  Logic: '<S16>/Logical Operator1'
     *  Switch: '<S16>/Switch1'
     *  Switch: '<S16>/Switch2'
     *  UnitDelay: '<S16>/UnitDelay1'
     */
    if (rtb_RelationalOperator4_d && rtDW->UnitDelay1_DSTATE_n) {
      rtb_Switch1_l = 0;
    } else if (rtb_RelationalOperator4_d) {
      /* Switch: '<S16>/Switch2' incorporates:
       *  UnitDelay: '<S12>/UnitDelay4'
       */
      rtb_Switch1_l = rtDW->UnitDelay4_DSTATE_e;
    } else if (rtDW->dz_cntTrnsDet) {
      /* Switch: '<S16>/Switch1' incorporates:
       *  Constant: '<S16>/cf_speedCoef'
       *  Product: '<S16>/Divide14'
       *  Switch: '<S16>/Switch2'
       */
      rtb_Switch1_l = (int16_T)((rtP->cf_speedCoef << 4) /
        rtDW->z_counterRawPrev);
    } else {
      /* Switch: '<S16>/Switch1' incorporates:
       *  Constant: '<S16>/cf_speedCoef'
       *  Gain: '<S16>/g_Ha'
       *  Product: '<S16>/Divide13'
       *  Sum: '<S16>/Sum13'
       *  Switch: '<S16>/Switch2'
       *  UnitDelay: '<S16>/UnitDelay2'
       *  UnitDelay: '<S16>/UnitDelay3'
       *  UnitDelay: '<S16>/UnitDelay5'
       */
      rtb_Switch1_l = (int16_T)(((uint16_T)(rtP->cf_speedCoef << 2) << 4) /
        (int16_T)(((rtDW->UnitDelay2_DSTATE + rtDW->UnitDelay3_DSTATE_o) +
                   rtDW->UnitDelay5_DSTATE) + rtDW->z_counterRawPrev));
    }

    /* End of Switch: '<S16>/Switch3' */

    /* Product: '<S16>/Divide11' */
    rtDW->Divide11 = (int16_T)(rtb_Switch1_l * rtDW->Switch2_e);

    /* Update for UnitDelay: '<S16>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S16>/UnitDelay2' incorporates:
     *  UnitDelay: '<S16>/UnitDelay3'
     */
    rtDW->UnitDelay2_DSTATE = rtDW->UnitDelay3_DSTATE_o;

    /* Update for UnitDelay: '<S16>/UnitDelay3' incorporates:
     *  UnitDelay: '<S16>/UnitDelay5'
     */
    rtDW->UnitDelay3_DSTATE_o = rtDW->UnitDelay5_DSTATE;

    /* Update for UnitDelay: '<S16>/UnitDelay5' */
    rtDW->UnitDelay5_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S16>/UnitDelay1' */
    rtDW->UnitDelay1_DSTATE_n = rtb_RelationalOperator4_d;

    /* End of Outputs for SubSystem: '<S12>/Raw_Motor_Speed_Estimation' */
  }

  /* End of If: '<S12>/If2' */

  /* Outputs for Atomic SubSystem: '<S12>/Counter' */

  /* Constant: '<S12>/Constant6' incorporates:
   *  Constant: '<S12>/z_maxCntRst2'
   */
  rtb_Switch1_l = (int16_T) Counter(1, rtP->z_maxCntRst, rtb_LogicalOperator,
    &rtDW->Counter_e);

  /* End of Outputs for SubSystem: '<S12>/Counter' */

  /* Switch: '<S12>/Switch2' incorporates:
   *  Constant: '<S12>/Constant4'
   *  Constant: '<S12>/z_maxCntRst'
   *  RelationalOperator: '<S12>/Relational Operator2'
   */
  if (rtb_Switch1_l > rtP->z_maxCntRst) {
    rtb_Switch2_k = 0;
  } else {
    rtb_Switch2_k = rtDW->Divide11;
  }

  /* End of Switch: '<S12>/Switch2' */

  /* Abs: '<S12>/Abs5' */
  if (rtb_Switch2_k < 0) {
    rtb_Abs5 = (int16_T)-rtb_Switch2_k;
  } else {
    rtb_Abs5 = rtb_Switch2_k;
  }

  /* End of Abs: '<S12>/Abs5' */

  /* Relay: '<S12>/n_commDeacv' */
  if (rtb_Abs5 >= rtP->n_commDeacvHi) {
    rtDW->n_commDeacv_Mode = true;
  } else {
    if (rtb_Abs5 <= rtP->n_commAcvLo) {
      rtDW->n_commDeacv_Mode = false;
    }
  }

  /* Logic: '<S12>/Logical Operator3' incorporates:
   *  Constant: '<S12>/b_angleMeasEna'
   *  Logic: '<S12>/Logical Operator1'
   *  Logic: '<S12>/Logical Operator2'
   *  Relay: '<S12>/n_commDeacv'
   */
  rtb_LogicalOperator = (rtP->b_angleMeasEna || (rtDW->n_commDeacv_Mode &&
    (!rtDW->dz_cntTrnsDet)));

  /* DataTypeConversion: '<S1>/Data Type Conversion2' incorporates:
   *  Inport: '<Root>/r_inpTgt'
   */
  rtb_DataTypeConversion2 = (int16_T)(rtU->r_inpTgt << 4);

  /* UnitDelay: '<S8>/UnitDelay2' */
  rtb_RelationalOperator4_d = rtDW->UnitDelay2_DSTATE_g;

  /* If: '<S1>/If2' incorporates:
   *  Constant: '<S1>/b_diagEna'
   *  Constant: '<S3>/CTRL_COMM2'
   *  Constant: '<S3>/t_errDequal'
   *  Constant: '<S3>/t_errQual'
   *  Logic: '<S1>/Logical Operator2'
   *  RelationalOperator: '<S3>/Relational Operator2'
   *  UnitDelay: '<S8>/UnitDelay2'
   */
  if (rtP->b_diagEna && rtDW->UnitDelay2_DSTATE_g) {
    /* Outputs for IfAction SubSystem: '<S1>/F02_Diagnostics' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Switch: '<S3>/Switch3' incorporates:
     *  Abs: '<S3>/Abs4'
     *  Constant: '<S12>/n_stdStillDet'
     *  Constant: '<S3>/CTRL_COMM4'
     *  Constant: '<S3>/r_errInpTgtThres'
     *  Inport: '<Root>/b_motEna'
     *  Logic: '<S3>/Logical Operator1'
     *  RelationalOperator: '<S12>/Relational Operator9'
     *  RelationalOperator: '<S3>/Relational Operator7'
     *  S-Function (sfix_bitop): '<S3>/Bitwise Operator1'
     *  UnitDelay: '<S3>/UnitDelay'
     *  UnitDelay: '<S7>/UnitDelay4'
     */
    if ((rtY->z_errCode & 4) != 0) {
      rtb_RelationalOperator1_m = true;
    } else {
      if (rtDW->UnitDelay4_DSTATE_eu < 0) {
        /* Abs: '<S3>/Abs4' incorporates:
         *  UnitDelay: '<S7>/UnitDelay4'
         */
        rtb_Merge_f_idx_2 = (int16_T)-rtDW->UnitDelay4_DSTATE_eu;
      } else {
        /* Abs: '<S3>/Abs4' incorporates:
         *  UnitDelay: '<S7>/UnitDelay4'
         */
        rtb_Merge_f_idx_2 = rtDW->UnitDelay4_DSTATE_eu;
      }

      rtb_RelationalOperator1_m = (rtU->b_motEna && (rtb_Abs5 <
        rtP->n_stdStillDet) && (rtb_Merge_f_idx_2 > rtP->r_errInpTgtThres));
    }

    /* End of Switch: '<S3>/Switch3' */

    /* Sum: '<S3>/Sum' incorporates:
     *  Constant: '<S3>/CTRL_COMM'
     *  Constant: '<S3>/CTRL_COMM1'
     *  DataTypeConversion: '<S3>/Data Type Conversion3'
     *  Gain: '<S3>/g_Hb'
     *  Gain: '<S3>/g_Hb1'
     *  RelationalOperator: '<S3>/Relational Operator1'
     *  RelationalOperator: '<S3>/Relational Operator3'
     */
    rtb_Sum_l = (uint8_T)(((uint32_T)((rtb_Sum == 7) << 1) + (rtb_Sum == 0)) +
                          (rtb_RelationalOperator1_m << 2));

    /* Outputs for Atomic SubSystem: '<S3>/Debounce_Filter' */
    Debounce_Filter(rtb_Sum_l != 0, rtP->t_errQual, rtP->t_errDequal,
                    &rtDW->Merge_n, &rtDW->Debounce_Filter_f);

    /* End of Outputs for SubSystem: '<S3>/Debounce_Filter' */

    /* Outputs for Atomic SubSystem: '<S3>/either_edge' */
    rtb_RelationalOperator1_m = either_edge(rtDW->Merge_n, &rtDW->either_edge_a);

    /* End of Outputs for SubSystem: '<S3>/either_edge' */

    /* Switch: '<S3>/Switch1' incorporates:
     *  Constant: '<S3>/CTRL_COMM2'
     *  Constant: '<S3>/t_errDequal'
     *  Constant: '<S3>/t_errQual'
     *  RelationalOperator: '<S3>/Relational Operator2'
     */
    if (rtb_RelationalOperator1_m) {
      /* Outport: '<Root>/z_errCode' */
      rtY->z_errCode = rtb_Sum_l;
    }

    /* End of Switch: '<S3>/Switch1' */
    /* End of Outputs for SubSystem: '<S1>/F02_Diagnostics' */
  }

  /* End of If: '<S1>/If2' */

  /* If: '<S1>/If4' incorporates:
   *  UnitDelay: '<S8>/UnitDelay2'
   */
  rtb_Sum2_h = rtDW->If4_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtDW->UnitDelay2_DSTATE_g) {
    UnitDelay3 = 0;
  }

  rtDW->If4_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
    /* Disable for If: '<S31>/If2' */
    rtDW->If2_ActiveSubsystem = -1;
  }

  if (UnitDelay3 == 0) {
    /* Outputs for IfAction SubSystem: '<S1>/F03_Control_Mode_Manager' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Logic: '<S29>/Logical Operator1' incorporates:
     *  Constant: '<S1>/b_cruiseCtrlEna'
     *  Constant: '<S29>/constant1'
     *  Inport: '<Root>/z_ctrlModReq'
     *  RelationalOperator: '<S29>/Relational Operator1'
     */
    rtb_RelationalOperator1_m = ((rtU->z_ctrlModReq == 2) ||
      rtP->b_cruiseCtrlEna);

    /* Logic: '<S29>/Logical Operator2' incorporates:
     *  Constant: '<S1>/b_cruiseCtrlEna'
     *  Constant: '<S29>/constant'
     *  Inport: '<Root>/z_ctrlModReq'
     *  Logic: '<S29>/Logical Operator5'
     *  RelationalOperator: '<S29>/Relational Operator4'
     */
    rtb_LogicalOperator2_p = ((rtU->z_ctrlModReq == 3) && (!rtP->b_cruiseCtrlEna));

    /* Logic: '<S29>/Logical Operator4' incorporates:
     *  Constant: '<S29>/constant8'
     *  Inport: '<Root>/b_motEna'
     *  Inport: '<Root>/z_ctrlModReq'
     *  Logic: '<S29>/Logical Operator7'
     *  RelationalOperator: '<S29>/Relational Operator10'
     */
    rtb_LogicalOperator4 = (rtDW->Merge_n || (!rtU->b_motEna) ||
      (rtU->z_ctrlModReq == 0));

    /* Chart: '<S4>/F03_02_Control_Mode_Manager' incorporates:
     *  Constant: '<S29>/constant5'
     *  Inport: '<Root>/z_ctrlModReq'
     *  Logic: '<S29>/Logical Operator3'
     *  Logic: '<S29>/Logical Operator6'
     *  Logic: '<S29>/Logical Operator9'
     *  RelationalOperator: '<S29>/Relational Operator5'
     */
    if (rtDW->is_active_c1_BLDC_controller == 0U) {
      rtDW->is_active_c1_BLDC_controller = 1U;
      rtDW->is_c1_BLDC_controller = IN_OPEN;
      rtDW->z_ctrlMod = OPEN_MODE;
    } else if (rtDW->is_c1_BLDC_controller == IN_ACTIVE) {
      if (rtb_LogicalOperator4) {
        rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
        rtDW->is_c1_BLDC_controller = IN_OPEN;
        rtDW->z_ctrlMod = OPEN_MODE;
      } else {
        switch (rtDW->is_ACTIVE) {
         case IN_SPEED_MODE:
          rtDW->z_ctrlMod = SPD_MODE;
          if (!rtb_RelationalOperator1_m) {
            rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
            if (rtb_LogicalOperator2_p) {
              rtDW->is_ACTIVE = IN_TORQUE_MODE;
              rtDW->z_ctrlMod = TRQ_MODE;
            } else {
              rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
              rtDW->z_ctrlMod = VLT_MODE;
            }
          }
          break;

         case IN_TORQUE_MODE:
          rtDW->z_ctrlMod = TRQ_MODE;
          if (!rtb_LogicalOperator2_p) {
            rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
            if (rtb_RelationalOperator1_m) {
              rtDW->is_ACTIVE = IN_SPEED_MODE;
              rtDW->z_ctrlMod = SPD_MODE;
            } else {
              rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
              rtDW->z_ctrlMod = VLT_MODE;
            }
          }
          break;

         default:
          rtDW->z_ctrlMod = VLT_MODE;
          if (rtb_LogicalOperator2_p || rtb_RelationalOperator1_m) {
            rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
            if (rtb_LogicalOperator2_p) {
              rtDW->is_ACTIVE = IN_TORQUE_MODE;
              rtDW->z_ctrlMod = TRQ_MODE;
            } else if (rtb_RelationalOperator1_m) {
              rtDW->is_ACTIVE = IN_SPEED_MODE;
              rtDW->z_ctrlMod = SPD_MODE;
            } else {
              rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
              rtDW->z_ctrlMod = VLT_MODE;
            }
          }
          break;
        }
      }
    } else {
      rtDW->z_ctrlMod = OPEN_MODE;
      if ((!rtb_LogicalOperator4) && ((rtU->z_ctrlModReq == 1) ||
           rtb_RelationalOperator1_m || rtb_LogicalOperator2_p)) {
        rtDW->is_c1_BLDC_controller = IN_ACTIVE;
        if (rtb_LogicalOperator2_p) {
          rtDW->is_ACTIVE = IN_TORQUE_MODE;
          rtDW->z_ctrlMod = TRQ_MODE;
        } else if (rtb_RelationalOperator1_m) {
          rtDW->is_ACTIVE = IN_SPEED_MODE;
          rtDW->z_ctrlMod = SPD_MODE;
        } else {
          rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
          rtDW->z_ctrlMod = VLT_MODE;
        }
      }
    }

    /* End of Chart: '<S4>/F03_02_Control_Mode_Manager' */

    /* If: '<S31>/If1' incorporates:
     *  Constant: '<S1>/z_ctrlTypSel'
     *  Inport: '<S32>/r_inpTgt'
     *  Saturate: '<S31>/Saturation'
     */
    if (rtP->z_ctrlTypSel == 2) {
      /* Outputs for IfAction SubSystem: '<S31>/FOC_Control_Type' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* SignalConversion: '<S34>/TmpSignal ConversionAtSelectorInport1' incorporates:
       *  Constant: '<S34>/Vd_max'
       *  Constant: '<S34>/constant1'
       *  Constant: '<S34>/i_max'
       *  Constant: '<S34>/n_max'
       */
      tmp[0] = 0;
      tmp[1] = rtP->Vd_max;
      tmp[2] = rtP->n_max;
      tmp[3] = rtP->i_max;

      /* End of Outputs for SubSystem: '<S31>/FOC_Control_Type' */

      /* Saturate: '<S31>/Saturation' */
      if (rtb_DataTypeConversion2 > 16000) {
        rtb_Merge = 16000;
      } else if (rtb_DataTypeConversion2 < -16000) {
        rtb_Merge = -16000;
      } else {
        rtb_Merge = rtb_DataTypeConversion2;
      }

      /* Outputs for IfAction SubSystem: '<S31>/FOC_Control_Type' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* Product: '<S34>/Divide1' incorporates:
       *  Inport: '<Root>/z_ctrlModReq'
       *  Product: '<S34>/Divide4'
       *  Selector: '<S34>/Selector'
       */
      rtb_Merge = (int16_T)(((uint16_T)((tmp[rtU->z_ctrlModReq] << 5) / 125) *
        rtb_Merge) >> 12);

      /* End of Outputs for SubSystem: '<S31>/FOC_Control_Type' */
    } else if (rtb_DataTypeConversion2 > 16000) {
      /* Outputs for IfAction SubSystem: '<S31>/Default_Control_Type' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* Saturate: '<S31>/Saturation' incorporates:
       *  Inport: '<S32>/r_inpTgt'
       */
      rtb_Merge = 16000;

      /* End of Outputs for SubSystem: '<S31>/Default_Control_Type' */
    } else if (rtb_DataTypeConversion2 < -16000) {
      /* Outputs for IfAction SubSystem: '<S31>/Default_Control_Type' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* Saturate: '<S31>/Saturation' incorporates:
       *  Inport: '<S32>/r_inpTgt'
       */
      rtb_Merge = -16000;

      /* End of Outputs for SubSystem: '<S31>/Default_Control_Type' */
    } else {
      /* Outputs for IfAction SubSystem: '<S31>/Default_Control_Type' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      rtb_Merge = rtb_DataTypeConversion2;

      /* End of Outputs for SubSystem: '<S31>/Default_Control_Type' */
    }

    /* End of If: '<S31>/If1' */

    /* If: '<S31>/If2' incorporates:
     *  Inport: '<S33>/r_inpTgtScaRaw'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem;
    UnitDelay3 = (int8_T)!(rtDW->z_ctrlMod == 0);
    rtDW->If2_ActiveSubsystem = UnitDelay3;
    switch (UnitDelay3) {
     case 0:
      if (UnitDelay3 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S31>/Open_Mode' incorporates:
         *  ActionPort: '<S35>/Action Port'
         */
        /* SystemReset for Atomic SubSystem: '<S35>/rising_edge_init' */
        /* SystemReset for If: '<S31>/If2' incorporates:
         *  UnitDelay: '<S37>/UnitDelay'
         *  UnitDelay: '<S38>/UnitDelay'
         */
        rtDW->UnitDelay_DSTATE_b = true;

        /* End of SystemReset for SubSystem: '<S35>/rising_edge_init' */

        /* SystemReset for Atomic SubSystem: '<S35>/Rate_Limiter' */
        rtDW->UnitDelay_DSTATE = 0;

        /* End of SystemReset for SubSystem: '<S35>/Rate_Limiter' */
        /* End of SystemReset for SubSystem: '<S31>/Open_Mode' */
      }

      /* Outputs for IfAction SubSystem: '<S31>/Open_Mode' incorporates:
       *  ActionPort: '<S35>/Action Port'
       */
      /* DataTypeConversion: '<S35>/Data Type Conversion' incorporates:
       *  UnitDelay: '<S7>/UnitDelay4'
       */
      rtb_Gain3 = rtDW->UnitDelay4_DSTATE_eu << 12;
      rtb_DataTypeConversion = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 |
        -134217728 : rtb_Gain3 & 134217727;

      /* Outputs for Atomic SubSystem: '<S35>/rising_edge_init' */
      /* UnitDelay: '<S37>/UnitDelay' */
      rtb_RelationalOperator1_m = rtDW->UnitDelay_DSTATE_b;

      /* Update for UnitDelay: '<S37>/UnitDelay' incorporates:
       *  Constant: '<S37>/Constant'
       */
      rtDW->UnitDelay_DSTATE_b = false;

      /* End of Outputs for SubSystem: '<S35>/rising_edge_init' */

      /* Outputs for Atomic SubSystem: '<S35>/Rate_Limiter' */
      /* Switch: '<S38>/Switch1' incorporates:
       *  UnitDelay: '<S38>/UnitDelay'
       */
      if (rtb_RelationalOperator1_m) {
        rtb_Switch1 = rtb_DataTypeConversion;
      } else {
        rtb_Switch1 = rtDW->UnitDelay_DSTATE;
      }

      /* End of Switch: '<S38>/Switch1' */

      /* Sum: '<S36>/Sum1' */
      rtb_Gain3 = -rtb_Switch1;
      rtb_Sum1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
        rtb_Gain3 & 134217727;

      /* Switch: '<S39>/Switch2' incorporates:
       *  Constant: '<S35>/dV_openRate'
       *  RelationalOperator: '<S39>/LowerRelop1'
       */
      if (rtb_Sum1 > rtP->dV_openRate) {
        rtb_Sum1 = rtP->dV_openRate;
      } else {
        /* Gain: '<S35>/Gain3' */
        rtb_Gain3 = -rtP->dV_openRate;
        rtb_Gain3 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
          rtb_Gain3 & 134217727;

        /* Switch: '<S39>/Switch' incorporates:
         *  RelationalOperator: '<S39>/UpperRelop'
         */
        if (rtb_Sum1 < rtb_Gain3) {
          rtb_Sum1 = rtb_Gain3;
        }

        /* End of Switch: '<S39>/Switch' */
      }

      /* End of Switch: '<S39>/Switch2' */

      /* Sum: '<S36>/Sum2' */
      rtb_Gain3 = rtb_Sum1 + rtb_Switch1;
      rtb_Switch1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
        rtb_Gain3 & 134217727;

      /* Switch: '<S38>/Switch2' */
      if (rtb_RelationalOperator1_m) {
        /* Update for UnitDelay: '<S38>/UnitDelay' */
        rtDW->UnitDelay_DSTATE = rtb_DataTypeConversion;
      } else {
        /* Update for UnitDelay: '<S38>/UnitDelay' */
        rtDW->UnitDelay_DSTATE = rtb_Switch1;
      }

      /* End of Switch: '<S38>/Switch2' */
      /* End of Outputs for SubSystem: '<S35>/Rate_Limiter' */

      /* DataTypeConversion: '<S35>/Data Type Conversion1' */
      rtDW->Merge1 = (int16_T)(rtb_Switch1 >> 12);

      /* End of Outputs for SubSystem: '<S31>/Open_Mode' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S31>/Default_Mode' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
      rtDW->Merge1 = rtb_Merge;

      /* End of Outputs for SubSystem: '<S31>/Default_Mode' */
      break;
    }

    /* End of If: '<S31>/If2' */
    /* End of Outputs for SubSystem: '<S1>/F03_Control_Mode_Manager' */
  }

  /* End of If: '<S1>/If4' */

  /* UnitDelay: '<S8>/UnitDelay5' */
  rtb_RelationalOperator1_m = rtDW->UnitDelay5_DSTATE_l;

  /* Saturate: '<S1>/Saturation' incorporates:
   *  Inport: '<Root>/i_phaAB'
   */
  rtb_Gain3 = rtU->i_phaAB << 4;
  if (rtb_Gain3 >= 27200) {
    rtb_Saturation = 27200;
  } else if (rtb_Gain3 <= -27200) {
    rtb_Saturation = -27200;
  } else {
    rtb_Saturation = (int16_T)(rtU->i_phaAB << 4);
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Saturate: '<S1>/Saturation1' incorporates:
   *  Inport: '<Root>/i_phaBC'
   */
  rtb_Gain3 = rtU->i_phaBC << 4;
  if (rtb_Gain3 >= 27200) {
    rtb_Saturation1 = 27200;
  } else if (rtb_Gain3 <= -27200) {
    rtb_Saturation1 = -27200;
  } else {
    rtb_Saturation1 = (int16_T)(rtU->i_phaBC << 4);
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* If: '<S2>/If1' incorporates:
   *  Constant: '<S2>/b_angleMeasEna'
   */
  if (!rtP->b_angleMeasEna) {
    /* Outputs for IfAction SubSystem: '<S2>/F01_05_Electrical_Angle_Estimation' incorporates:
     *  ActionPort: '<S13>/Action Port'
     */
    /* Switch: '<S13>/Switch2' incorporates:
     *  Constant: '<S13>/Constant16'
     *  Product: '<S13>/Divide1'
     *  Product: '<S13>/Divide3'
     *  RelationalOperator: '<S13>/Relational Operator7'
     *  Sum: '<S13>/Sum3'
     *  Switch: '<S13>/Switch3'
     */
    if (rtb_LogicalOperator) {
      /* MinMax: '<S13>/MinMax' */
      rtb_Merge = rtb_Switch1_l;
      if (!(rtb_Merge < rtDW->z_counterRawPrev)) {
        rtb_Merge = rtDW->z_counterRawPrev;
      }

      /* End of MinMax: '<S13>/MinMax' */

      /* Switch: '<S13>/Switch3' incorporates:
       *  Constant: '<S10>/vec_hallToPos'
       *  Constant: '<S13>/Constant16'
       *  RelationalOperator: '<S13>/Relational Operator7'
       *  Selector: '<S10>/Selector'
       *  Sum: '<S13>/Sum1'
       */
      if (rtDW->Switch2_e == 1) {
        rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
      } else {
        rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] + 1);
      }

      rtb_Merge = (int16_T)(((int16_T)((int16_T)((rtb_Merge << 14) /
        rtDW->z_counterRawPrev) * rtDW->Switch2_e) + (rtb_Sum2_h << 14)) >> 2);
    } else {
      if (rtDW->Switch2_e == 1) {
        /* Switch: '<S13>/Switch3' incorporates:
         *  Constant: '<S10>/vec_hallToPos'
         *  Selector: '<S10>/Selector'
         */
        rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
      } else {
        /* Switch: '<S13>/Switch3' incorporates:
         *  Constant: '<S10>/vec_hallToPos'
         *  Selector: '<S10>/Selector'
         *  Sum: '<S13>/Sum1'
         */
        rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] + 1);
      }

      rtb_Merge = (int16_T)(rtb_Sum2_h << 12);
    }

    /* End of Switch: '<S13>/Switch2' */

    /* MinMax: '<S13>/MinMax1' incorporates:
     *  Constant: '<S13>/Constant1'
     */
    if (!(rtb_Merge > 0)) {
      rtb_Merge = 0;
    }

    /* End of MinMax: '<S13>/MinMax1' */

    /* SignalConversion: '<S13>/Signal Conversion2' incorporates:
     *  Product: '<S13>/Divide2'
     */
    rtb_Merge = (int16_T)((15 * rtb_Merge) >> 4);

    /* End of Outputs for SubSystem: '<S2>/F01_05_Electrical_Angle_Estimation' */
  } else {
    /* Outputs for IfAction SubSystem: '<S2>/F01_06_Electrical_Angle_Measurement' incorporates:
     *  ActionPort: '<S14>/Action Port'
     */
    /* Sum: '<S14>/Sum1' incorporates:
     *  Constant: '<S14>/Constant2'
     *  Constant: '<S14>/n_polePairs'
     *  Inport: '<Root>/a_mechAngle'
     *  Product: '<S14>/Divide'
     */
    rtb_DataTypeConversion = rtU->a_mechAngle * rtP->n_polePairs - 480;

    /* DataTypeConversion: '<S14>/Data Type Conversion20' incorporates:
     *  Constant: '<S14>/a_elecPeriod'
     *  Product: '<S18>/Divide2'
     *  Product: '<S18>/Divide3'
     *  Sum: '<S18>/Sum3'
     */
    rtb_Merge = (int16_T)((int16_T)(rtb_DataTypeConversion - ((int16_T)((int16_T)
      div_nde_s32_floor(rtb_DataTypeConversion, 5760) * 360) << 4)) << 2);

    /* End of Outputs for SubSystem: '<S2>/F01_06_Electrical_Angle_Measurement' */
  }

  /* End of If: '<S2>/If1' */

  /* If: '<S1>/If3' incorporates:
   *  Constant: '<S1>/CTRL_COMM2'
   *  Constant: '<S1>/b_fieldWeakEna'
   *  Constant: '<S1>/z_ctrlTypSel'
   *  Logic: '<S1>/Logical Operator1'
   *  RelationalOperator: '<S1>/Relational Operator1'
   *  UnitDelay: '<S8>/UnitDelay5'
   */
  if (rtP->b_fieldWeakEna && rtDW->UnitDelay5_DSTATE_l && (rtP->z_ctrlTypSel !=
       0)) {
    /* Outputs for IfAction SubSystem: '<S1>/F04_Field_Weakening' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Abs: '<S5>/Abs5' */
    if (rtb_DataTypeConversion2 < 0) {
      rtb_DataTypeConversion2 = (int16_T)-rtb_DataTypeConversion2;
    }

    /* End of Abs: '<S5>/Abs5' */

    /* Switch: '<S41>/Switch2' incorporates:
     *  Constant: '<S5>/r_fieldWeakHi'
     *  Constant: '<S5>/r_fieldWeakLo'
     *  RelationalOperator: '<S41>/LowerRelop1'
     *  RelationalOperator: '<S41>/UpperRelop'
     *  Switch: '<S41>/Switch'
     */
    if (rtb_DataTypeConversion2 > rtP->r_fieldWeakHi) {
      rtb_DataTypeConversion2 = rtP->r_fieldWeakHi;
    } else {
      if (rtb_DataTypeConversion2 < rtP->r_fieldWeakLo) {
        /* Switch: '<S41>/Switch' incorporates:
         *  Constant: '<S5>/r_fieldWeakLo'
         */
        rtb_DataTypeConversion2 = rtP->r_fieldWeakLo;
      }
    }

    /* End of Switch: '<S41>/Switch2' */

    /* Switch: '<S5>/Switch2' incorporates:
     *  Constant: '<S5>/CTRL_COMM2'
     *  Constant: '<S5>/a_phaAdvMax'
     *  Constant: '<S5>/id_fieldWeakMax'
     *  RelationalOperator: '<S5>/Relational Operator1'
     */
    if (rtP->z_ctrlTypSel == 2) {
      rtb_Merge_f_idx_2 = rtP->id_fieldWeakMax;
    } else {
      rtb_Merge_f_idx_2 = rtP->a_phaAdvMax;
    }

    /* End of Switch: '<S5>/Switch2' */

    /* Switch: '<S40>/Switch2' incorporates:
     *  Constant: '<S5>/n_fieldWeakAuthHi'
     *  Constant: '<S5>/n_fieldWeakAuthLo'
     *  RelationalOperator: '<S40>/LowerRelop1'
     *  RelationalOperator: '<S40>/UpperRelop'
     *  Switch: '<S40>/Switch'
     */
    if (rtb_Abs5 > rtP->n_fieldWeakAuthHi) {
      rtb_Switch2_l = rtP->n_fieldWeakAuthHi;
    } else if (rtb_Abs5 < rtP->n_fieldWeakAuthLo) {
      /* Switch: '<S40>/Switch' incorporates:
       *  Constant: '<S5>/n_fieldWeakAuthLo'
       */
      rtb_Switch2_l = rtP->n_fieldWeakAuthLo;
    } else {
      rtb_Switch2_l = rtb_Abs5;
    }

    /* End of Switch: '<S40>/Switch2' */

    /* Product: '<S5>/Divide3' incorporates:
     *  Constant: '<S5>/n_fieldWeakAuthHi'
     *  Constant: '<S5>/n_fieldWeakAuthLo'
     *  Constant: '<S5>/r_fieldWeakHi'
     *  Constant: '<S5>/r_fieldWeakLo'
     *  Product: '<S5>/Divide1'
     *  Product: '<S5>/Divide14'
     *  Product: '<S5>/Divide2'
     *  Sum: '<S5>/Sum1'
     *  Sum: '<S5>/Sum2'
     *  Sum: '<S5>/Sum3'
     *  Sum: '<S5>/Sum4'
     */
    rtDW->Divide3 = (int16_T)(((uint16_T)(((uint32_T)(uint16_T)(((int16_T)
      (rtb_DataTypeConversion2 - rtP->r_fieldWeakLo) << 15) / (int16_T)
      (rtP->r_fieldWeakHi - rtP->r_fieldWeakLo)) * (uint16_T)(((int16_T)
      (rtb_Switch2_l - rtP->n_fieldWeakAuthLo) << 15) / (int16_T)
      (rtP->n_fieldWeakAuthHi - rtP->n_fieldWeakAuthLo))) >> 15) *
      rtb_Merge_f_idx_2) >> 15);

    /* End of Outputs for SubSystem: '<S1>/F04_Field_Weakening' */
  }

  /* End of If: '<S1>/If3' */

  /* If: '<S1>/If1' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel'
   */
  rtb_Sum2_h = rtDW->If1_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtP->z_ctrlTypSel == 2) {
    UnitDelay3 = 0;
  }

  rtDW->If1_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
    /* Disable for If: '<S6>/If2' */
    if (rtDW->If2_ActiveSubsystem_a == 0) {
      /* Disable for Outport: '<S43>/iq' */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Outport: '<S43>/id' */
      rtDW->DataTypeConversion[1] = 0;
    }

    rtDW->If2_ActiveSubsystem_a = -1;

    /* End of Disable for If: '<S6>/If2' */

    /* Disable for If: '<S6>/If1' */
    if (rtDW->If1_ActiveSubsystem_e == 0) {
      /* Disable for SwitchCase: '<S44>/Switch Case' */
      rtDW->SwitchCase_ActiveSubsystem = -1;
    }

    rtDW->If1_ActiveSubsystem_e = -1;

    /* End of Disable for If: '<S6>/If1' */

    /* Disable for Outport: '<S6>/V_phaABC_FOC' */
    rtDW->Gain4[0] = 0;
    rtDW->Gain4[1] = 0;
    rtDW->Gain4[2] = 0;

    /* Disable for Outport: '<S6>/iq' */
    rtDW->DataTypeConversion[0] = 0;

    /* Disable for Outport: '<S6>/id' */
    rtDW->DataTypeConversion[1] = 0;
  }

  if (UnitDelay3 == 0) {
    if (0 != rtb_Sum2_h) {
      /* InitializeConditions for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' incorporates:
       *  ActionPort: '<S6>/Action Port'
       */
      /* InitializeConditions for If: '<S1>/If1' incorporates:
       *  UnitDelay: '<S6>/UnitDelay4'
       */
      rtDW->UnitDelay4_DSTATE_h = 0;

      /* End of InitializeConditions for SubSystem: '<S1>/F05_Field_Oriented_Control' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Abs: '<S6>/Abs1' */
    if (rtDW->Merge1 < 0) {
      rtb_Switch2_l = (int16_T)-rtDW->Merge1;
    } else {
      rtb_Switch2_l = rtDW->Merge1;
    }

    /* End of Abs: '<S6>/Abs1' */

    /* Gain: '<S6>/toNegative' */
    rtb_toNegative = (int16_T)-rtDW->Divide3;

    /* If: '<S42>/If1' incorporates:
     *  Constant: '<S42>/z_selPhaCurMeasABC'
     */
    if (rtP->z_selPhaCurMeasABC == 0) {
      /* Outputs for IfAction SubSystem: '<S42>/Clarke_PhasesAB' incorporates:
       *  ActionPort: '<S50>/Action Port'
       */
      /* Gain: '<S50>/Gain4' */
      rtb_Gain3 = 18919 * rtb_Saturation;

      /* Gain: '<S50>/Gain2' */
      rtb_DataTypeConversion = 18919 * rtb_Saturation1;

      /* Sum: '<S50>/Sum1' incorporates:
       *  Gain: '<S50>/Gain2'
       *  Gain: '<S50>/Gain4'
       */
      rtb_Gain3 = (((rtb_Gain3 < 0 ? 32767 : 0) + rtb_Gain3) >> 15) + (int16_T)
        (((rtb_DataTypeConversion < 0 ? 16383 : 0) + rtb_DataTypeConversion) >>
         14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtb_DataTypeConversion2 = (int16_T)rtb_Gain3;

      /* End of Sum: '<S50>/Sum1' */
      /* End of Outputs for SubSystem: '<S42>/Clarke_PhasesAB' */
    } else if (rtP->z_selPhaCurMeasABC == 1) {
      /* Outputs for IfAction SubSystem: '<S42>/Clarke_PhasesBC' incorporates:
       *  ActionPort: '<S52>/Action Port'
       */
      /* Sum: '<S52>/Sum3' */
      rtb_Gain3 = rtb_Saturation - rtb_Saturation1;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Gain: '<S52>/Gain2' incorporates:
       *  Sum: '<S52>/Sum3'
       */
      rtb_Gain3 *= 18919;
      rtb_DataTypeConversion2 = (int16_T)(((rtb_Gain3 < 0 ? 32767 : 0) +
        rtb_Gain3) >> 15);

      /* Sum: '<S52>/Sum1' */
      rtb_Gain3 = -rtb_Saturation - rtb_Saturation1;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtb_Saturation = (int16_T)rtb_Gain3;

      /* End of Sum: '<S52>/Sum1' */
      /* End of Outputs for SubSystem: '<S42>/Clarke_PhasesBC' */
    } else {
      /* Outputs for IfAction SubSystem: '<S42>/Clarke_PhasesAC' incorporates:
       *  ActionPort: '<S51>/Action Port'
       */
      /* Gain: '<S51>/Gain4' */
      rtb_Gain3 = 18919 * rtb_Saturation;

      /* Gain: '<S51>/Gain2' */
      rtb_DataTypeConversion = 18919 * rtb_Saturation1;

      /* Sum: '<S51>/Sum1' incorporates:
       *  Gain: '<S51>/Gain2'
       *  Gain: '<S51>/Gain4'
       */
      rtb_Gain3 = -(((rtb_Gain3 < 0 ? 32767 : 0) + rtb_Gain3) >> 15) - (int16_T)
        (((rtb_DataTypeConversion < 0 ? 16383 : 0) + rtb_DataTypeConversion) >>
         14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtb_DataTypeConversion2 = (int16_T)rtb_Gain3;

      /* End of Sum: '<S51>/Sum1' */
      /* End of Outputs for SubSystem: '<S42>/Clarke_PhasesAC' */
    }

    /* End of If: '<S42>/If1' */

    /* PreLookup: '<S49>/a_elecAngle_XA' */
    rtb_Sum_l = plook_u8s16_evencka(rtb_Merge, 0, 128U, 180U);

    /* If: '<S6>/If2' incorporates:
     *  Constant: '<S43>/cf_currFilt'
     *  Inport: '<Root>/b_motEna'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem_a;
    UnitDelay3 = -1;
    if (rtU->b_motEna) {
      UnitDelay3 = 0;
    }

    rtDW->If2_ActiveSubsystem_a = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S43>/iq' */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Outport: '<S43>/id' */
      rtDW->DataTypeConversion[1] = 0;
    }

    if (UnitDelay3 == 0) {
      if (0 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
         *  ActionPort: '<S43>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S43>/Low_Pass_Filter' */

        /* SystemReset for If: '<S6>/If2' */
        Low_Pass_Filter_Reset(&rtDW->Low_Pass_Filter_m);

        /* End of SystemReset for SubSystem: '<S43>/Low_Pass_Filter' */

        /* End of SystemReset for SubSystem: '<S6>/Current_Filtering' */
      }

      /* Sum: '<S48>/Sum6' incorporates:
       *  Interpolation_n-D: '<S49>/r_cos_M1'
       *  Interpolation_n-D: '<S49>/r_sin_M1'
       *  Product: '<S48>/Divide1'
       *  Product: '<S48>/Divide4'
       */
      rtb_Gain3 = (int16_T)((rtb_DataTypeConversion2 *
        rtConstP.r_cos_M1_Table[rtb_Sum_l]) >> 14) - (int16_T)((rtb_Saturation *
        rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
       *  ActionPort: '<S43>/Action Port'
       */
      /* SignalConversion: '<S43>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S48>/Sum6'
       */
      rtb_TmpSignalConversionAtLow_Pa[0] = (int16_T)rtb_Gain3;

      /* End of Outputs for SubSystem: '<S6>/Current_Filtering' */

      /* Sum: '<S48>/Sum1' incorporates:
       *  Interpolation_n-D: '<S49>/r_cos_M1'
       *  Interpolation_n-D: '<S49>/r_sin_M1'
       *  Product: '<S48>/Divide2'
       *  Product: '<S48>/Divide3'
       */
      rtb_Gain3 = (int16_T)((rtb_Saturation * rtConstP.r_cos_M1_Table[rtb_Sum_l])
                            >> 14) + (int16_T)((rtb_DataTypeConversion2 *
        rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
       *  ActionPort: '<S43>/Action Port'
       */
      /* SignalConversion: '<S43>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S48>/Sum1'
       */
      rtb_TmpSignalConversionAtLow_Pa[1] = (int16_T)rtb_Gain3;

      /* Outputs for Atomic SubSystem: '<S43>/Low_Pass_Filter' */
      Low_Pass_Filter(rtb_TmpSignalConversionAtLow_Pa, rtP->cf_currFilt,
                      rtDW->DataTypeConversion, &rtDW->Low_Pass_Filter_m);

      /* End of Outputs for SubSystem: '<S43>/Low_Pass_Filter' */

      /* End of Outputs for SubSystem: '<S6>/Current_Filtering' */
    }

    /* End of If: '<S6>/If2' */

    /* If: '<S6>/If3' incorporates:
     *  Constant: '<S47>/Vd_max1'
     *  Constant: '<S47>/i_max'
     *  UnitDelay: '<S8>/UnitDelay5'
     */
    if (rtDW->UnitDelay5_DSTATE_l) {
      /* Outputs for IfAction SubSystem: '<S6>/Motor_Limitations' incorporates:
       *  ActionPort: '<S47>/Action Port'
       */
      rtDW->Vd_max1 = rtP->Vd_max;

      /* Gain: '<S47>/Gain3' incorporates:
       *  Constant: '<S47>/Vd_max1'
       */
      rtDW->Gain3 = (int16_T)-rtDW->Vd_max1;

      /* Interpolation_n-D: '<S47>/Vq_max_M1' incorporates:
       *  Abs: '<S47>/Abs5'
       *  PreLookup: '<S47>/Vq_max_XA'
       *  UnitDelay: '<S6>/UnitDelay4'
       */
      if (rtDW->UnitDelay4_DSTATE_h < 0) {
        rtb_Merge_f_idx_2 = (int16_T)-rtDW->UnitDelay4_DSTATE_h;
      } else {
        rtb_Merge_f_idx_2 = rtDW->UnitDelay4_DSTATE_h;
      }

      rtDW->Vq_max_M1 = rtP->Vq_max_M1[plook_u8s16_evencka(rtb_Merge_f_idx_2,
        rtP->Vq_max_XA[0], (uint16_T)(rtP->Vq_max_XA[1] - rtP->Vq_max_XA[0]),
        45U)];

      /* End of Interpolation_n-D: '<S47>/Vq_max_M1' */

      /* Gain: '<S47>/Gain5' */
      rtDW->Gain5 = (int16_T)-rtDW->Vq_max_M1;
      rtDW->i_max = rtP->i_max;

      /* Interpolation_n-D: '<S47>/iq_maxSca_M1' incorporates:
       *  Constant: '<S47>/i_max'
       *  Product: '<S47>/Divide4'
       */
      rtb_Gain3 = rtDW->Divide3 << 16;
      rtb_Gain3 = (rtb_Gain3 == MIN_int32_T) && (rtDW->i_max == -1) ?
        MAX_int32_T : rtb_Gain3 / rtDW->i_max;
      if (rtb_Gain3 < 0) {
        rtb_Gain3 = 0;
      } else {
        if (rtb_Gain3 > 65535) {
          rtb_Gain3 = 65535;
        }
      }

      /* Product: '<S47>/Divide1' incorporates:
       *  Interpolation_n-D: '<S47>/iq_maxSca_M1'
       *  PreLookup: '<S47>/iq_maxSca_XA'
       *  Product: '<S47>/Divide4'
       */
      rtDW->Divide1_a = (int16_T)
        ((rtConstP.iq_maxSca_M1_Table[plook_u8u16_evencka((uint16_T)rtb_Gain3,
           0U, 1311U, 49U)] * rtDW->i_max) >> 16);

      /* Gain: '<S47>/Gain1' */
      rtDW->Gain1 = (int16_T)-rtDW->Divide1_a;

      /* SwitchCase: '<S47>/Switch Case' incorporates:
       *  Constant: '<S47>/n_max1'
       *  Constant: '<S75>/Constant1'
       *  Constant: '<S75>/cf_KbLimProt'
       *  Constant: '<S75>/cf_nKiLimProt'
       *  Constant: '<S76>/Constant'
       *  Constant: '<S76>/Constant1'
       *  Constant: '<S76>/cf_KbLimProt'
       *  Constant: '<S76>/cf_iqKiLimProt'
       *  Constant: '<S76>/cf_nKiLimProt'
       *  Sum: '<S75>/Sum1'
       *  Sum: '<S76>/Sum1'
       *  Sum: '<S76>/Sum2'
       */
      switch (rtDW->z_ctrlMod) {
       case 1:
        /* Abs: '<S6>/Abs5' */
        if (rtDW->DataTypeConversion[0] < 0) {
          rtb_Merge_f_idx_2 = (int16_T)-rtDW->DataTypeConversion[0];
        } else {
          rtb_Merge_f_idx_2 = rtDW->DataTypeConversion[0];
        }

        /* End of Abs: '<S6>/Abs5' */

        /* Outputs for IfAction SubSystem: '<S47>/Voltage_Mode_Protection' incorporates:
         *  ActionPort: '<S76>/Action Port'
         */

        /* Outputs for Atomic SubSystem: '<S76>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtDW->Divide1_a - rtb_Merge_f_idx_2),
                         rtP->cf_iqKiLimProt, rtP->cf_KbLimProt, rtb_Switch2_l,
                         0, &rtDW->Switch2_c, &rtDW->I_backCalc_fixdt_i);

        /* End of Outputs for SubSystem: '<S76>/I_backCalc_fixdt' */

        /* Outputs for Atomic SubSystem: '<S76>/I_backCalc_fixdt1' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - rtb_Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtb_Switch2_l, 0, &rtDW->Switch2_l,
                         &rtDW->I_backCalc_fixdt1);

        /* End of Outputs for SubSystem: '<S76>/I_backCalc_fixdt1' */

        /* End of Outputs for SubSystem: '<S47>/Voltage_Mode_Protection' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S47>/Speed_Mode_Protection' incorporates:
         *  ActionPort: '<S74>/Action Port'
         */
        /* Switch: '<S77>/Switch2' incorporates:
         *  RelationalOperator: '<S77>/LowerRelop1'
         *  RelationalOperator: '<S77>/UpperRelop'
         *  Switch: '<S77>/Switch'
         */
        if (rtDW->DataTypeConversion[0] > rtDW->Divide1_a) {
          rtb_Merge_f_idx_2 = rtDW->Divide1_a;
        } else if (rtDW->DataTypeConversion[0] < rtDW->Gain1) {
          /* Switch: '<S77>/Switch' */
          rtb_Merge_f_idx_2 = rtDW->Gain1;
        } else {
          rtb_Merge_f_idx_2 = rtDW->DataTypeConversion[0];
        }

        /* End of Switch: '<S77>/Switch2' */

        /* Product: '<S74>/Divide1' incorporates:
         *  Constant: '<S74>/cf_iqKiLimProt'
         *  Sum: '<S74>/Sum3'
         */
        rtDW->Divide1 = (int16_T)(rtb_Merge_f_idx_2 - rtDW->DataTypeConversion[0])
          * rtP->cf_iqKiLimProt;

        /* End of Outputs for SubSystem: '<S47>/Speed_Mode_Protection' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S47>/Torque_Mode_Protection' incorporates:
         *  ActionPort: '<S75>/Action Port'
         */

        /* Outputs for Atomic SubSystem: '<S75>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - rtb_Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtDW->Vq_max_M1, 0, &rtDW->Switch2,
                         &rtDW->I_backCalc_fixdt_g);

        /* End of Outputs for SubSystem: '<S75>/I_backCalc_fixdt' */

        /* End of Outputs for SubSystem: '<S47>/Torque_Mode_Protection' */
        break;
      }

      /* End of SwitchCase: '<S47>/Switch Case' */

      /* Gain: '<S47>/Gain4' */
      rtDW->Gain4_c = (int16_T)-rtDW->i_max;

      /* End of Outputs for SubSystem: '<S6>/Motor_Limitations' */
    }

    /* End of If: '<S6>/If3' */

    /* If: '<S6>/If1' incorporates:
     *  UnitDelay: '<S8>/UnitDelay6'
     */
    rtb_Sum2_h = rtDW->If1_ActiveSubsystem_e;
    UnitDelay3 = -1;
    if (rtDW->UnitDelay6_DSTATE) {
      UnitDelay3 = 0;
    }

    rtDW->If1_ActiveSubsystem_e = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for SwitchCase: '<S44>/Switch Case' */
      rtDW->SwitchCase_ActiveSubsystem = -1;
    }

    if (UnitDelay3 == 0) {
      /* Outputs for IfAction SubSystem: '<S6>/FOC' incorporates:
       *  ActionPort: '<S44>/Action Port'
       */
      /* If: '<S44>/If1' incorporates:
       *  Constant: '<S57>/cf_idKi1'
       *  Constant: '<S57>/cf_idKp1'
       *  Constant: '<S57>/constant1'
       *  Constant: '<S57>/constant2'
       *  Sum: '<S57>/Sum3'
       */
      if (rtb_LogicalOperator) {
        /* Outputs for IfAction SubSystem: '<S44>/Vd_Calculation' incorporates:
         *  ActionPort: '<S57>/Action Port'
         */
        /* Switch: '<S69>/Switch2' incorporates:
         *  RelationalOperator: '<S69>/LowerRelop1'
         *  RelationalOperator: '<S69>/UpperRelop'
         *  Switch: '<S69>/Switch'
         */
        if (rtb_toNegative > rtDW->i_max) {
          rtb_toNegative = rtDW->i_max;
        } else {
          if (rtb_toNegative < rtDW->Gain4_c) {
            /* Switch: '<S69>/Switch' */
            rtb_toNegative = rtDW->Gain4_c;
          }
        }

        /* End of Switch: '<S69>/Switch2' */

        /* Sum: '<S57>/Sum3' */
        rtb_Gain3 = rtb_toNegative - rtDW->DataTypeConversion[1];
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* Outputs for Atomic SubSystem: '<S57>/PI_clamp_fixdt' */
        PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_idKp, rtP->cf_idKi, 0,
                       rtDW->Vd_max1, rtDW->Gain3, 0, &rtDW->Switch1,
                       &rtDW->PI_clamp_fixdt_k);

        /* End of Outputs for SubSystem: '<S57>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S44>/Vd_Calculation' */
      }

      /* End of If: '<S44>/If1' */

      /* SwitchCase: '<S44>/Switch Case' incorporates:
       *  Constant: '<S55>/cf_nKi'
       *  Constant: '<S55>/cf_nKp'
       *  Constant: '<S56>/cf_iqKi'
       *  Constant: '<S56>/cf_iqKp'
       *  Constant: '<S56>/constant2'
       *  Inport: '<S54>/r_inpTgtSca'
       *  Sum: '<S55>/Sum3'
       *  Sum: '<S56>/Sum2'
       *  UnitDelay: '<S7>/UnitDelay4'
       */
      rtb_Sum2_h = rtDW->SwitchCase_ActiveSubsystem;
      switch (rtDW->z_ctrlMod) {
       case 1:
        break;

       case 2:
        UnitDelay3 = 1;
        break;

       case 3:
        UnitDelay3 = 2;
        break;

       default:
        UnitDelay3 = 3;
        break;
      }

      rtDW->SwitchCase_ActiveSubsystem = UnitDelay3;
      switch (UnitDelay3) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S44>/Voltage_Mode' incorporates:
         *  ActionPort: '<S58>/Action Port'
         */
        /* MinMax: '<S58>/MinMax' */
        if (!(rtb_Switch2_l < rtDW->Switch2_c)) {
          rtb_Switch2_l = rtDW->Switch2_c;
        }

        if (!(rtb_Switch2_l < rtDW->Switch2_l)) {
          rtb_Switch2_l = rtDW->Switch2_l;
        }

        /* End of MinMax: '<S58>/MinMax' */

        /* Signum: '<S58>/SignDeltaU2' */
        if (rtDW->Merge1 < 0) {
          rtb_Merge_f_idx_2 = -1;
        } else {
          rtb_Merge_f_idx_2 = (int16_T)(rtDW->Merge1 > 0);
        }

        /* End of Signum: '<S58>/SignDeltaU2' */

        /* Product: '<S58>/Divide1' */
        rtb_Saturation = (int16_T)(rtb_Switch2_l * rtb_Merge_f_idx_2);

        /* Switch: '<S73>/Switch2' incorporates:
         *  RelationalOperator: '<S73>/LowerRelop1'
         *  RelationalOperator: '<S73>/UpperRelop'
         *  Switch: '<S73>/Switch'
         */
        if (rtb_Saturation > rtDW->Vq_max_M1) {
          /* SignalConversion: '<S58>/Signal Conversion2' */
          rtDW->Merge = rtDW->Vq_max_M1;
        } else if (rtb_Saturation < rtDW->Gain5) {
          /* Switch: '<S73>/Switch' incorporates:
           *  SignalConversion: '<S58>/Signal Conversion2'
           */
          rtDW->Merge = rtDW->Gain5;
        } else {
          /* SignalConversion: '<S58>/Signal Conversion2' incorporates:
           *  Switch: '<S73>/Switch'
           */
          rtDW->Merge = rtb_Saturation;
        }

        /* End of Switch: '<S73>/Switch2' */
        /* End of Outputs for SubSystem: '<S44>/Voltage_Mode' */
        break;

       case 1:
        if (UnitDelay3 != rtb_Sum2_h) {
          /* SystemReset for IfAction SubSystem: '<S44>/Speed_Mode' incorporates:
           *  ActionPort: '<S55>/Action Port'
           */

          /* SystemReset for Atomic SubSystem: '<S55>/PI_clamp_fixdt' */

          /* SystemReset for SwitchCase: '<S44>/Switch Case' */
          PI_clamp_fixdt_g_Reset(&rtDW->PI_clamp_fixdt_oc);

          /* End of SystemReset for SubSystem: '<S55>/PI_clamp_fixdt' */

          /* End of SystemReset for SubSystem: '<S44>/Speed_Mode' */
        }

        /* Outputs for IfAction SubSystem: '<S44>/Speed_Mode' incorporates:
         *  ActionPort: '<S55>/Action Port'
         */
        /* DataTypeConversion: '<S55>/Data Type Conversion2' incorporates:
         *  Constant: '<S55>/n_cruiseMotTgt'
         */
        rtb_Saturation = (int16_T)(rtP->n_cruiseMotTgt << 4);

        /* Switch: '<S55>/Switch4' incorporates:
         *  Constant: '<S1>/b_cruiseCtrlEna'
         *  Logic: '<S55>/Logical Operator1'
         *  RelationalOperator: '<S55>/Relational Operator3'
         */
        if (rtP->b_cruiseCtrlEna && (rtb_Saturation != 0)) {
          /* Switch: '<S55>/Switch3' incorporates:
           *  MinMax: '<S55>/MinMax4'
           */
          if (rtb_Saturation > 0) {
            rtb_TmpSignalConversionAtLow_Pa[0] = rtDW->Vq_max_M1;

            /* MinMax: '<S55>/MinMax3' */
            if (rtDW->Merge1 > rtDW->Gain5) {
              rtb_TmpSignalConversionAtLow_Pa[1] = rtDW->Merge1;
            } else {
              rtb_TmpSignalConversionAtLow_Pa[1] = rtDW->Gain5;
            }

            /* End of MinMax: '<S55>/MinMax3' */
          } else {
            if (rtDW->Vq_max_M1 < rtDW->Merge1) {
              /* MinMax: '<S55>/MinMax4' */
              rtb_TmpSignalConversionAtLow_Pa[0] = rtDW->Vq_max_M1;
            } else {
              rtb_TmpSignalConversionAtLow_Pa[0] = rtDW->Merge1;
            }

            rtb_TmpSignalConversionAtLow_Pa[1] = rtDW->Gain5;
          }

          /* End of Switch: '<S55>/Switch3' */
        } else {
          rtb_TmpSignalConversionAtLow_Pa[0] = rtDW->Vq_max_M1;
          rtb_TmpSignalConversionAtLow_Pa[1] = rtDW->Gain5;
        }

        /* End of Switch: '<S55>/Switch4' */

        /* Switch: '<S55>/Switch2' incorporates:
         *  Constant: '<S1>/b_cruiseCtrlEna'
         */
        if (!rtP->b_cruiseCtrlEna) {
          rtb_Saturation = rtDW->Merge1;
        }

        /* End of Switch: '<S55>/Switch2' */

        /* Sum: '<S55>/Sum3' */
        rtb_Gain3 = rtb_Saturation - rtb_Switch2_k;
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* Outputs for Atomic SubSystem: '<S55>/PI_clamp_fixdt' */
        rtDW->Merge = (int16_T) PI_clamp_fixdt_o((int16_T)rtb_Gain3, rtP->cf_nKp,
          rtP->cf_nKi, rtDW->UnitDelay4_DSTATE_eu,
          rtb_TmpSignalConversionAtLow_Pa[0], rtb_TmpSignalConversionAtLow_Pa[1],
          rtDW->Divide1, &rtDW->PI_clamp_fixdt_oc);

        /* End of Outputs for SubSystem: '<S55>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S44>/Speed_Mode' */
        break;

       case 2:
        if (UnitDelay3 != rtb_Sum2_h) {
          /* SystemReset for IfAction SubSystem: '<S44>/Torque_Mode' incorporates:
           *  ActionPort: '<S56>/Action Port'
           */

          /* SystemReset for Atomic SubSystem: '<S56>/PI_clamp_fixdt' */

          /* SystemReset for SwitchCase: '<S44>/Switch Case' */
          PI_clamp_fixdt_b_Reset(&rtDW->PI_clamp_fixdt_at);

          /* End of SystemReset for SubSystem: '<S56>/PI_clamp_fixdt' */

          /* End of SystemReset for SubSystem: '<S44>/Torque_Mode' */
        }

        /* Outputs for IfAction SubSystem: '<S44>/Torque_Mode' incorporates:
         *  ActionPort: '<S56>/Action Port'
         */
        /* Gain: '<S56>/Gain4' */
        rtb_Saturation = (int16_T)-rtDW->Switch2;

        /* Switch: '<S64>/Switch2' incorporates:
         *  RelationalOperator: '<S64>/LowerRelop1'
         *  RelationalOperator: '<S64>/UpperRelop'
         *  Switch: '<S64>/Switch'
         */
        if (rtDW->Merge1 > rtDW->Divide1_a) {
          rtb_Merge_f_idx_2 = rtDW->Divide1_a;
        } else if (rtDW->Merge1 < rtDW->Gain1) {
          /* Switch: '<S64>/Switch' */
          rtb_Merge_f_idx_2 = rtDW->Gain1;
        } else {
          rtb_Merge_f_idx_2 = rtDW->Merge1;
        }

        /* End of Switch: '<S64>/Switch2' */

        /* Sum: '<S56>/Sum2' */
        rtb_Gain3 = rtb_Merge_f_idx_2 - rtDW->DataTypeConversion[0];
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* MinMax: '<S56>/MinMax1' */
        if (rtDW->Vq_max_M1 < rtDW->Switch2) {
          rtb_Merge_f_idx_2 = rtDW->Vq_max_M1;
        } else {
          rtb_Merge_f_idx_2 = rtDW->Switch2;
        }

        /* End of MinMax: '<S56>/MinMax1' */

        /* MinMax: '<S56>/MinMax2' */
        if (!(rtb_Saturation > rtDW->Gain5)) {
          rtb_Saturation = rtDW->Gain5;
        }

        /* End of MinMax: '<S56>/MinMax2' */

        /* Outputs for Atomic SubSystem: '<S56>/PI_clamp_fixdt' */
        rtDW->Merge = (int16_T) PI_clamp_fixdt_a((int16_T)rtb_Gain3,
          rtP->cf_iqKp, rtP->cf_iqKi, rtDW->UnitDelay4_DSTATE_eu,
          rtb_Merge_f_idx_2, rtb_Saturation, 0, &rtDW->PI_clamp_fixdt_at);

        /* End of Outputs for SubSystem: '<S56>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S44>/Torque_Mode' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S44>/Open_Mode' incorporates:
         *  ActionPort: '<S54>/Action Port'
         */
        rtDW->Merge = rtDW->Merge1;

        /* End of Outputs for SubSystem: '<S44>/Open_Mode' */
        break;
      }

      /* End of SwitchCase: '<S44>/Switch Case' */
      /* End of Outputs for SubSystem: '<S6>/FOC' */
    }

    /* End of If: '<S6>/If1' */

    /* Sum: '<S46>/Sum6' incorporates:
     *  Interpolation_n-D: '<S49>/r_cos_M1'
     *  Interpolation_n-D: '<S49>/r_sin_M1'
     *  Product: '<S46>/Divide1'
     *  Product: '<S46>/Divide4'
     */
    rtb_Gain3 = (int16_T)((rtDW->Switch1 * rtConstP.r_cos_M1_Table[rtb_Sum_l]) >>
                          14) - (int16_T)((rtDW->Merge *
      rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14);
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Sum: '<S46>/Sum1' incorporates:
     *  Interpolation_n-D: '<S49>/r_cos_M1'
     *  Interpolation_n-D: '<S49>/r_sin_M1'
     *  Product: '<S46>/Divide2'
     *  Product: '<S46>/Divide3'
     */
    rtb_DataTypeConversion = (int16_T)((rtDW->Switch1 *
      rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14) + (int16_T)((rtDW->Merge *
      rtConstP.r_cos_M1_Table[rtb_Sum_l]) >> 14);
    if (rtb_DataTypeConversion > 32767) {
      rtb_DataTypeConversion = 32767;
    } else {
      if (rtb_DataTypeConversion < -32768) {
        rtb_DataTypeConversion = -32768;
      }
    }

    /* Gain: '<S45>/Gain1' incorporates:
     *  Sum: '<S46>/Sum1'
     */
    rtb_DataTypeConversion *= 14189;

    /* Sum: '<S45>/Sum6' incorporates:
     *  Gain: '<S45>/Gain1'
     *  Gain: '<S45>/Gain3'
     *  Sum: '<S46>/Sum6'
     */
    rtb_DataTypeConversion = (((rtb_DataTypeConversion < 0 ? 16383 : 0) +
      rtb_DataTypeConversion) >> 14) - ((int16_T)(((int16_T)rtb_Gain3 < 0) +
      (int16_T)rtb_Gain3) >> 1);
    if (rtb_DataTypeConversion > 32767) {
      rtb_DataTypeConversion = 32767;
    } else {
      if (rtb_DataTypeConversion < -32768) {
        rtb_DataTypeConversion = -32768;
      }
    }

    /* Sum: '<S45>/Sum2' incorporates:
     *  Sum: '<S45>/Sum6'
     *  Sum: '<S46>/Sum6'
     */
    rtb_Switch1 = -(int16_T)rtb_Gain3 - (int16_T)rtb_DataTypeConversion;
    if (rtb_Switch1 > 32767) {
      rtb_Switch1 = 32767;
    } else {
      if (rtb_Switch1 < -32768) {
        rtb_Switch1 = -32768;
      }
    }

    /* MinMax: '<S45>/MinMax1' incorporates:
     *  Sum: '<S45>/Sum2'
     *  Sum: '<S45>/Sum6'
     *  Sum: '<S46>/Sum6'
     */
    rtb_Switch2_l = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 < (int16_T)rtb_DataTypeConversion)) {
      rtb_Switch2_l = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Switch2_l < (int16_T)rtb_Switch1)) {
      rtb_Switch2_l = (int16_T)rtb_Switch1;
    }

    /* MinMax: '<S45>/MinMax2' incorporates:
     *  Sum: '<S45>/Sum2'
     *  Sum: '<S45>/Sum6'
     *  Sum: '<S46>/Sum6'
     */
    rtb_Saturation = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 > (int16_T)rtb_DataTypeConversion)) {
      rtb_Saturation = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Saturation > (int16_T)rtb_Switch1)) {
      rtb_Saturation = (int16_T)rtb_Switch1;
    }

    /* Sum: '<S45>/Add' incorporates:
     *  MinMax: '<S45>/MinMax1'
     *  MinMax: '<S45>/MinMax2'
     */
    rtb_Sum1 = rtb_Switch2_l + rtb_Saturation;
    if (rtb_Sum1 > 32767) {
      rtb_Sum1 = 32767;
    } else {
      if (rtb_Sum1 < -32768) {
        rtb_Sum1 = -32768;
      }
    }

    /* Gain: '<S45>/Gain2' incorporates:
     *  Sum: '<S45>/Add'
     */
    rtb_DataTypeConversion2 = (int16_T)(rtb_Sum1 >> 1);

    /* Sum: '<S45>/Add1' incorporates:
     *  Sum: '<S46>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_Gain3 - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S45>/Gain4' incorporates:
     *  Sum: '<S45>/Add1'
     */
    rtDW->Gain4[0] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S45>/Add1' incorporates:
     *  Sum: '<S45>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_DataTypeConversion - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S45>/Gain4' incorporates:
     *  Sum: '<S45>/Add1'
     */
    rtDW->Gain4[1] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S45>/Add1' incorporates:
     *  Sum: '<S45>/Sum2'
     */
    rtb_Gain3 = (int16_T)rtb_Switch1 - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S45>/Gain4' incorporates:
     *  Sum: '<S45>/Add1'
     */
    rtDW->Gain4[2] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Update for UnitDelay: '<S6>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE_h = rtDW->Switch1;

    /* End of Outputs for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  }

  /* End of If: '<S1>/If1' */

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel'
   *  Constant: '<S7>/CTRL_COMM1'
   *  RelationalOperator: '<S7>/Relational Operator6'
   */
  if (rtP->z_ctrlTypSel == 2) {
    rtb_Saturation = rtDW->Merge;
  } else {
    rtb_Saturation = rtDW->Merge1;
  }

  /* End of Switch: '<S7>/Switch2' */

  /* If: '<S7>/If' incorporates:
   *  Constant: '<S10>/vec_hallToPos'
   *  Constant: '<S1>/z_ctrlTypSel'
   *  Constant: '<S7>/CTRL_COMM2'
   *  Constant: '<S7>/CTRL_COMM3'
   *  Inport: '<S88>/V_phaABC_FOC_in'
   *  Logic: '<S7>/Logical Operator1'
   *  Logic: '<S7>/Logical Operator2'
   *  LookupNDDirect: '<S87>/z_commutMap_M1'
   *  RelationalOperator: '<S7>/Relational Operator1'
   *  RelationalOperator: '<S7>/Relational Operator2'
   *  Selector: '<S10>/Selector'
   *
   * About '<S87>/z_commutMap_M1':
   *  2-dimensional Direct Look-Up returning a Column
   */
  if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 2)) {
    /* Outputs for IfAction SubSystem: '<S7>/FOC_Method' incorporates:
     *  ActionPort: '<S88>/Action Port'
     */
    rtb_DataTypeConversion2 = rtDW->Gain4[0];
    rtb_Saturation1 = rtDW->Gain4[1];
    rtb_Merge_f_idx_2 = rtDW->Gain4[2];

    /* End of Outputs for SubSystem: '<S7>/FOC_Method' */
  } else if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 1)) {
    /* Outputs for IfAction SubSystem: '<S7>/SIN_Method' incorporates:
     *  ActionPort: '<S89>/Action Port'
     */
    /* Switch: '<S90>/Switch_PhaAdv' incorporates:
     *  Constant: '<S90>/b_fieldWeakEna'
     *  Product: '<S91>/Divide2'
     *  Product: '<S91>/Divide3'
     *  Sum: '<S91>/Sum3'
     */
    if (rtP->b_fieldWeakEna) {
      /* Sum: '<S90>/Sum3' incorporates:
       *  Product: '<S90>/Product2'
       */
      rtb_Saturation1 = (int16_T)((int16_T)((int16_T)(rtDW->Divide3 *
        rtDW->Switch2_e) << 2) + rtb_Merge);
      rtb_Saturation1 -= (int16_T)((int16_T)((int16_T)div_nde_s32_floor
        (rtb_Saturation1, 23040) * 360) << 6);
    } else {
      rtb_Saturation1 = rtb_Merge;
    }

    /* End of Switch: '<S90>/Switch_PhaAdv' */

    /* PreLookup: '<S89>/a_elecAngle_XA' */
    rtb_Sum = plook_u8s16_evencka(rtb_Saturation1, 0, 128U, 180U);

    /* Product: '<S89>/Divide2' incorporates:
     *  Interpolation_n-D: '<S89>/r_sin3PhaA_M1'
     *  Interpolation_n-D: '<S89>/r_sin3PhaB_M1'
     *  Interpolation_n-D: '<S89>/r_sin3PhaC_M1'
     */
    rtb_DataTypeConversion2 = (int16_T)((rtb_Saturation *
      rtConstP.r_sin3PhaA_M1_Table[rtb_Sum]) >> 14);
    rtb_Saturation1 = (int16_T)((rtb_Saturation *
      rtConstP.r_sin3PhaB_M1_Table[rtb_Sum]) >> 14);
    rtb_Merge_f_idx_2 = (int16_T)((rtb_Saturation *
      rtConstP.r_sin3PhaC_M1_Table[rtb_Sum]) >> 14);

    /* End of Outputs for SubSystem: '<S7>/SIN_Method' */
  } else {
    /* Outputs for IfAction SubSystem: '<S7>/COM_Method' incorporates:
     *  ActionPort: '<S87>/Action Port'
     */
    if (rtConstP.vec_hallToPos_Value[rtb_Sum] > 5) {
      /* LookupNDDirect: '<S87>/z_commutMap_M1'
       *
       * About '<S87>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 5;
    } else if (rtConstP.vec_hallToPos_Value[rtb_Sum] < 0) {
      /* LookupNDDirect: '<S87>/z_commutMap_M1'
       *
       * About '<S87>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 0;
    } else {
      /* LookupNDDirect: '<S87>/z_commutMap_M1' incorporates:
       *  Constant: '<S10>/vec_hallToPos'
       *  Selector: '<S10>/Selector'
       *
       * About '<S87>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
    }

    /* LookupNDDirect: '<S87>/z_commutMap_M1' incorporates:
     *  Constant: '<S10>/vec_hallToPos'
     *  Selector: '<S10>/Selector'
     *
     * About '<S87>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_DataTypeConversion = rtb_Sum2_h * 3;

    /* Product: '<S87>/Divide2' incorporates:
     *  LookupNDDirect: '<S87>/z_commutMap_M1'
     *
     * About '<S87>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_DataTypeConversion2 = (int16_T)(rtb_Saturation *
      rtConstP.z_commutMap_M1_table[rtb_DataTypeConversion]);
    rtb_Saturation1 = (int16_T)(rtConstP.z_commutMap_M1_table[1 +
      rtb_DataTypeConversion] * rtb_Saturation);
    rtb_Merge_f_idx_2 = (int16_T)(rtConstP.z_commutMap_M1_table[2 +
      rtb_DataTypeConversion] * rtb_Saturation);

    /* End of Outputs for SubSystem: '<S7>/COM_Method' */
  }

  /* End of If: '<S7>/If' */

  /* Outport: '<Root>/DC_phaA' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion6'
   */
  rtY->DC_phaA = (int16_T)(rtb_DataTypeConversion2 >> 4);

  /* Outport: '<Root>/DC_phaB' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion6'
   */
  rtY->DC_phaB = (int16_T)(rtb_Saturation1 >> 4);

  /* Update for UnitDelay: '<S9>/UnitDelay3' incorporates:
   *  Inport: '<Root>/b_hallA '
   */
  rtDW->UnitDelay3_DSTATE_fy = rtU->b_hallA;

  /* Update for UnitDelay: '<S9>/UnitDelay1' incorporates:
   *  Inport: '<Root>/b_hallB'
   */
  rtDW->UnitDelay1_DSTATE = rtU->b_hallB;

  /* Update for UnitDelay: '<S9>/UnitDelay2' incorporates:
   *  Inport: '<Root>/b_hallC'
   */
  rtDW->UnitDelay2_DSTATE_f = rtU->b_hallC;

  /* Update for UnitDelay: '<S12>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtb_Switch1_l;

  /* Update for UnitDelay: '<S12>/UnitDelay4' */
  rtDW->UnitDelay4_DSTATE_e = rtb_Abs5;

  /* Update for UnitDelay: '<S8>/UnitDelay2' incorporates:
   *  UnitDelay: '<S8>/UnitDelay6'
   */
  rtDW->UnitDelay2_DSTATE_g = rtDW->UnitDelay6_DSTATE;

  /* Update for UnitDelay: '<S7>/UnitDelay4' */
  rtDW->UnitDelay4_DSTATE_eu = rtb_Saturation;

  /* Update for UnitDelay: '<S8>/UnitDelay5' */
  rtDW->UnitDelay5_DSTATE_l = rtb_RelationalOperator4_d;

  /* Update for UnitDelay: '<S8>/UnitDelay6' */
  rtDW->UnitDelay6_DSTATE = rtb_RelationalOperator1_m;

  /* Outport: '<Root>/DC_phaC' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion6'
   */
  rtY->DC_phaC = (int16_T)(rtb_Merge_f_idx_2 >> 4);

  /* Outport: '<Root>/n_mot' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion1'
   */
  rtY->n_mot = (int16_T)(rtb_Switch2_k >> 4);

  /* Outport: '<Root>/a_elecAngle' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion3'
   */
  rtY->a_elecAngle = (int16_T)(rtb_Merge >> 6);

  /* End of Outputs for SubSystem: '<Root>/BLDC_controller' */

  /* Outport: '<Root>/iq' */
  rtY->iq = rtDW->DataTypeConversion[0];

  /* Outport: '<Root>/id' */
  rtY->id = rtDW->DataTypeConversion[1];
}

/* Model initialize function */
void BLDC_controller_initialize(RT_MODEL *const rtM)
{
  P *rtP = ((P *) rtM->defaultParam);
  DW *rtDW = ((DW *) rtM->dwork);

  /* Start for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Start for If: '<S1>/If4' */
  rtDW->If4_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S1>/F03_Control_Mode_Manager' */
  /* Start for If: '<S31>/If2' */
  rtDW->If2_ActiveSubsystem = -1;

  /* End of Start for SubSystem: '<S1>/F03_Control_Mode_Manager' */

  /* Start for If: '<S1>/If1' */
  rtDW->If1_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* Start for If: '<S6>/If2' */
  rtDW->If2_ActiveSubsystem_a = -1;

  /* Start for If: '<S6>/If1' */
  rtDW->If1_ActiveSubsystem_e = -1;

  /* Start for IfAction SubSystem: '<S6>/FOC' */
  /* Start for SwitchCase: '<S44>/Switch Case' */
  rtDW->SwitchCase_ActiveSubsystem = -1;

  /* End of Start for SubSystem: '<S6>/FOC' */
  /* End of Start for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* End of Start for SubSystem: '<Root>/BLDC_controller' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* InitializeConditions for UnitDelay: '<S12>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtP->z_maxCntRst;

  /* InitializeConditions for UnitDelay: '<S8>/UnitDelay2' */
  rtDW->UnitDelay2_DSTATE_g = true;

  /* SystemInitialize for IfAction SubSystem: '<S12>/Raw_Motor_Speed_Estimation' */
  /* SystemInitialize for Outport: '<S16>/z_counter' */
  rtDW->z_counterRawPrev = rtP->z_maxCntRst;

  /* End of SystemInitialize for SubSystem: '<S12>/Raw_Motor_Speed_Estimation' */

  /* SystemInitialize for Atomic SubSystem: '<S12>/Counter' */
  Counter_Init(&rtDW->Counter_e, rtP->z_maxCntRst);

  /* End of SystemInitialize for SubSystem: '<S12>/Counter' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/F02_Diagnostics' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Debounce_Filter' */
  Debounce_Filter_Init(&rtDW->Debounce_Filter_f);

  /* End of SystemInitialize for SubSystem: '<S3>/Debounce_Filter' */

  /* End of SystemInitialize for SubSystem: '<S1>/F02_Diagnostics' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/F03_Control_Mode_Manager' */
  /* SystemInitialize for IfAction SubSystem: '<S31>/Open_Mode' */
  /* SystemInitialize for Atomic SubSystem: '<S35>/rising_edge_init' */
  /* InitializeConditions for UnitDelay: '<S37>/UnitDelay' */
  rtDW->UnitDelay_DSTATE_b = true;

  /* End of SystemInitialize for SubSystem: '<S35>/rising_edge_init' */
  /* End of SystemInitialize for SubSystem: '<S31>/Open_Mode' */
  /* End of SystemInitialize for SubSystem: '<S1>/F03_Control_Mode_Manager' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* SystemInitialize for IfAction SubSystem: '<S6>/Motor_Limitations' */

  /* SystemInitialize for IfAction SubSystem: '<S47>/Voltage_Mode_Protection' */

  /* SystemInitialize for Atomic SubSystem: '<S76>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt_i, 65536000);

  /* End of SystemInitialize for SubSystem: '<S76>/I_backCalc_fixdt' */

  /* SystemInitialize for Atomic SubSystem: '<S76>/I_backCalc_fixdt1' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt1, 65536000);

  /* End of SystemInitialize for SubSystem: '<S76>/I_backCalc_fixdt1' */

  /* End of SystemInitialize for SubSystem: '<S47>/Voltage_Mode_Protection' */

  /* SystemInitialize for IfAction SubSystem: '<S47>/Torque_Mode_Protection' */

  /* SystemInitialize for Atomic SubSystem: '<S75>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt_g, 58982400);

  /* End of SystemInitialize for SubSystem: '<S75>/I_backCalc_fixdt' */

  /* End of SystemInitialize for SubSystem: '<S47>/Torque_Mode_Protection' */

  /* SystemInitialize for Outport: '<S47>/Vd_max' */
  rtDW->Vd_max1 = 14400;

  /* SystemInitialize for Outport: '<S47>/Vd_min' */
  rtDW->Gain3 = -14400;

  /* SystemInitialize for Outport: '<S47>/Vq_max' */
  rtDW->Vq_max_M1 = 14400;

  /* SystemInitialize for Outport: '<S47>/Vq_min' */
  rtDW->Gain5 = -14400;

  /* SystemInitialize for Outport: '<S47>/id_max' */
  rtDW->i_max = 12000;

  /* SystemInitialize for Outport: '<S47>/id_min' */
  rtDW->Gain4_c = -12000;

  /* SystemInitialize for Outport: '<S47>/iq_max' */
  rtDW->Divide1_a = 12000;

  /* SystemInitialize for Outport: '<S47>/iq_min' */
  rtDW->Gain1 = -12000;

  /* End of SystemInitialize for SubSystem: '<S6>/Motor_Limitations' */

  /* SystemInitialize for IfAction SubSystem: '<S6>/FOC' */

  /* SystemInitialize for IfAction SubSystem: '<S44>/Vd_Calculation' */

  /* SystemInitialize for Atomic SubSystem: '<S57>/PI_clamp_fixdt' */
  PI_clamp_fixdt_Init(&rtDW->PI_clamp_fixdt_k);

  /* End of SystemInitialize for SubSystem: '<S57>/PI_clamp_fixdt' */

  /* End of SystemInitialize for SubSystem: '<S44>/Vd_Calculation' */

  /* SystemInitialize for IfAction SubSystem: '<S44>/Speed_Mode' */

  /* SystemInitialize for Atomic SubSystem: '<S55>/PI_clamp_fixdt' */
  PI_clamp_fixdt_g_Init(&rtDW->PI_clamp_fixdt_oc);

  /* End of SystemInitialize for SubSystem: '<S55>/PI_clamp_fixdt' */

  /* End of SystemInitialize for SubSystem: '<S44>/Speed_Mode' */

  /* SystemInitialize for IfAction SubSystem: '<S44>/Torque_Mode' */

  /* SystemInitialize for Atomic SubSystem: '<S56>/PI_clamp_fixdt' */
  PI_clamp_fixdt_k_Init(&rtDW->PI_clamp_fixdt_at);

  /* End of SystemInitialize for SubSystem: '<S56>/PI_clamp_fixdt' */

  /* End of SystemInitialize for SubSystem: '<S44>/Torque_Mode' */

  /* End of SystemInitialize for SubSystem: '<S6>/FOC' */

  /* End of SystemInitialize for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* End of SystemInitialize for SubSystem: '<Root>/BLDC_controller' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
