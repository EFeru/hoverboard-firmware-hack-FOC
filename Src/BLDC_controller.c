/*
 * File: BLDC_controller.c
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 1.1212
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sat Nov 30 08:54:28 2019
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
uint8_T plook_u8s16u8n7_evenc_s(int16_T u, int16_T bp0, uint16_T bpSpace,
  uint32_T maxIndex, uint8_T *fraction);
int16_T intrp1d_s16s32s32u8u8n7l_s(uint8_T bpIndex, uint8_T frac, const int16_T
  table[]);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern void Counter_Init(DW_Counter *localDW, int16_T rtp_z_cntInit);
extern int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst,
  DW_Counter *localDW);
extern void PI_clamp_fixdt_Reset(DW_PI_clamp_fixdt *localDW);
extern void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_satMax, int16_T rtu_satMin, int32_T rtu_ext_limProt, int16_T
  *rty_out, DW_PI_clamp_fixdt *localDW);
extern void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW);
extern void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T
  rty_y[2], DW_Low_Pass_Filter *localDW);
extern void PI_clamp_fixdt_n_Reset(DW_PI_clamp_fixdt_c *localDW);
extern int16_T PI_clamp_fixdt_n(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_satMax, int16_T rtu_satMin, int16_T rtu_ext_limProt,
  DW_PI_clamp_fixdt_c *localDW);
extern void Counter_b_Init(DW_Counter_l *localDW, uint16_T rtp_z_cntInit);
extern uint16_T Counter_i(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
  DW_Counter_l *localDW);
extern void either_edge_Reset(DW_either_edge *localDW);
extern boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW);
extern void Debounce_Filter_Init(DW_Debounce_Filter *localDW);
extern void Debounce_Filter_Reset(DW_Debounce_Filter *localDW);
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

uint8_T plook_u8s16u8n7_evenc_s(int16_T u, int16_T bp0, uint16_T bpSpace,
  uint32_T maxIndex, uint8_T *fraction)
{
  uint8_T bpIndex;
  uint16_T uAdjust;
  uint16_T fbpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
     Rounding mode: 'simplest'
   */
  if (u <= bp0) {
    bpIndex = 0U;
    *fraction = 0U;
  } else {
    uAdjust = (uint16_T)(u - bp0);
    fbpIndex = (uint16_T)((uint32_T)uAdjust / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_T)fbpIndex;
      *fraction = (uint8_T)(((uint32_T)(uint16_T)((uint32_T)uAdjust - (uint16_T)
        ((uint32_T)bpIndex * bpSpace)) << 7) / bpSpace);
    } else {
      bpIndex = (uint8_T)(maxIndex - 1U);
      *fraction = 128U;
    }
  }

  return bpIndex;
}

int16_T intrp1d_s16s32s32u8u8n7l_s(uint8_T bpIndex, uint8_T frac, const int16_T
  table[])
{
  uint32_T offset_0d;

  /* Interpolation 1-D
     Interpolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  offset_0d = bpIndex;
  return (int16_T)((int16_T)(((table[offset_0d + 1U] - table[offset_0d]) * frac)
    >> 7) + table[offset_0d]);
}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* System initialize for atomic system: '<S10>/Counter' */
void Counter_Init(DW_Counter *localDW, int16_T rtp_z_cntInit)
{
  /* InitializeConditions for UnitDelay: '<S14>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/* Output and update for atomic system: '<S10>/Counter' */
int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst, DW_Counter *
                localDW)
{
  int16_T rtu_rst_0;
  int16_T rty_cnt_0;

  /* Switch: '<S14>/Switch1' incorporates:
   *  Constant: '<S14>/Constant23'
   *  UnitDelay: '<S14>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S14>/Switch1' */

  /* Sum: '<S12>/Sum1' */
  rty_cnt_0 = (int16_T)(rtu_inc + rtu_rst_0);

  /* MinMax: '<S12>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S14>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S14>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S12>/MinMax' */
  return rty_cnt_0;
}

/*
 * System reset for atomic system:
 *    '<S46>/PI_clamp_fixdt_id'
 *    '<S45>/PI_clamp_fixdt_iq'
 */
void PI_clamp_fixdt_Reset(DW_PI_clamp_fixdt *localDW)
{
  /* InitializeConditions for UnitDelay: '<S64>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE = false;

  /* InitializeConditions for UnitDelay: '<S67>/UnitDelay' */
  localDW->UnitDelay_DSTATE = 0;
}

/*
 * Output and update for atomic system:
 *    '<S46>/PI_clamp_fixdt_id'
 *    '<S45>/PI_clamp_fixdt_iq'
 */
void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I, int16_T
                    rtu_satMax, int16_T rtu_satMin, int32_T rtu_ext_limProt,
                    int16_T *rty_out, DW_PI_clamp_fixdt *localDW)
{
  boolean_T rtb_LowerRelop1_e;
  boolean_T rtb_UpperRelop_f;
  int32_T rtb_Sum1_b4;
  int32_T q0;
  int32_T tmp;
  int16_T tmp_0;

  /* Sum: '<S64>/Sum2' incorporates:
   *  Product: '<S64>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  if ((q0 < 0) && (rtu_ext_limProt < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (rtu_ext_limProt > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += rtu_ext_limProt;
  }

  /* Switch: '<S64>/Switch1' incorporates:
   *  Constant: '<S64>/a_elecPeriod1'
   *  Sum: '<S64>/Sum2'
   *  UnitDelay: '<S64>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  /* End of Switch: '<S64>/Switch1' */

  /* Sum: '<S67>/Sum1' incorporates:
   *  UnitDelay: '<S67>/UnitDelay'
   */
  rtb_Sum1_b4 = tmp + localDW->UnitDelay_DSTATE;

  /* Product: '<S64>/Divide5' */
  tmp = (rtu_err * rtu_P) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* Sum: '<S64>/Sum1' incorporates:
   *  DataTypeConversion: '<S67>/Data Type Conversion1'
   *  Product: '<S64>/Divide5'
   */
  tmp = (((rtb_Sum1_b4 >> 16) << 1) + tmp) >> 1;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* RelationalOperator: '<S68>/LowerRelop1' incorporates:
   *  Sum: '<S64>/Sum1'
   */
  rtb_LowerRelop1_e = ((int16_T)tmp > rtu_satMax);

  /* RelationalOperator: '<S68>/UpperRelop' incorporates:
   *  Sum: '<S64>/Sum1'
   */
  rtb_UpperRelop_f = ((int16_T)tmp < rtu_satMin);

  /* Switch: '<S68>/Switch1' incorporates:
   *  Sum: '<S64>/Sum1'
   *  Switch: '<S68>/Switch3'
   */
  if (rtb_LowerRelop1_e) {
    *rty_out = rtu_satMax;
  } else if (rtb_UpperRelop_f) {
    /* Switch: '<S68>/Switch3' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = (int16_T)tmp;
  }

  /* End of Switch: '<S68>/Switch1' */

  /* Signum: '<S66>/SignDeltaU2' incorporates:
   *  Sum: '<S64>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S66>/SignDeltaU2' */

  /* Signum: '<S66>/SignDeltaU3' incorporates:
   *  Sum: '<S64>/Sum1'
   */
  if ((int16_T)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_T)((int16_T)tmp > 0);
  }

  /* End of Signum: '<S66>/SignDeltaU3' */

  /* Update for UnitDelay: '<S64>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S66>/DataTypeConv4'
   *  Logic: '<S64>/AND1'
   *  Logic: '<S66>/AND1'
   *  RelationalOperator: '<S66>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp_0) && (rtb_LowerRelop1_e ||
    rtb_UpperRelop_f));

  /* Update for UnitDelay: '<S67>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtb_Sum1_b4;
}

/* System reset for atomic system: '<S37>/Low_Pass_Filter' */
void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S50>/UnitDelay3' */
  localDW->UnitDelay3_DSTATE[0] = 0;
  localDW->UnitDelay3_DSTATE[1] = 0;
}

/* Output and update for atomic system: '<S37>/Low_Pass_Filter' */
void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T rty_y[2],
                     DW_Low_Pass_Filter *localDW)
{
  uint16_T rtb_Sum5;

  /* Sum: '<S50>/Sum5' */
  rtb_Sum5 = (uint16_T)(65535U - rtu_coef);

  /* Sum: '<S50>/Sum1' incorporates:
   *  Product: '<S50>/Divide1'
   *  Product: '<S50>/Divide2'
   *  UnitDelay: '<S50>/UnitDelay3'
   */
  rty_y[0] = (int16_T)(((rtu_u[0] * rtu_coef) >> 16) +
                       ((localDW->UnitDelay3_DSTATE[0] * rtb_Sum5) >> 16));

  /* Update for UnitDelay: '<S50>/UnitDelay3' */
  localDW->UnitDelay3_DSTATE[0] = rty_y[0];

  /* Sum: '<S50>/Sum1' incorporates:
   *  Product: '<S50>/Divide1'
   *  Product: '<S50>/Divide2'
   *  UnitDelay: '<S50>/UnitDelay3'
   */
  rty_y[1] = (int16_T)(((rtu_u[1] * rtu_coef) >> 16) +
                       ((localDW->UnitDelay3_DSTATE[1] * rtb_Sum5) >> 16));

  /* Update for UnitDelay: '<S50>/UnitDelay3' */
  localDW->UnitDelay3_DSTATE[1] = rty_y[1];
}

/* System reset for atomic system: '<S44>/PI_clamp_fixdt_n' */
void PI_clamp_fixdt_n_Reset(DW_PI_clamp_fixdt_c *localDW)
{
  /* InitializeConditions for UnitDelay: '<S55>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE = false;

  /* InitializeConditions for UnitDelay: '<S57>/UnitDelay' */
  localDW->UnitDelay_DSTATE = 0;
}

/* Output and update for atomic system: '<S44>/PI_clamp_fixdt_n' */
int16_T PI_clamp_fixdt_n(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_satMax, int16_T rtu_satMin, int16_T rtu_ext_limProt,
  DW_PI_clamp_fixdt_c *localDW)
{
  boolean_T rtb_LowerRelop1_ge;
  boolean_T rtb_UpperRelop_o;
  int32_T rtb_Sum1_mz;
  int32_T q0;
  int32_T q1;
  int16_T tmp;
  int16_T rty_out_0;

  /* Sum: '<S55>/Sum2' incorporates:
   *  Product: '<S55>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  q1 = rtu_ext_limProt << 10;
  if ((q0 < 0) && (q1 < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (q1 > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += q1;
  }

  /* Switch: '<S55>/Switch1' incorporates:
   *  Constant: '<S55>/a_elecPeriod1'
   *  Sum: '<S55>/Sum2'
   *  UnitDelay: '<S55>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    q1 = 0;
  } else {
    q1 = q0;
  }

  /* End of Switch: '<S55>/Switch1' */

  /* Sum: '<S57>/Sum1' incorporates:
   *  UnitDelay: '<S57>/UnitDelay'
   */
  rtb_Sum1_mz = q1 + localDW->UnitDelay_DSTATE;

  /* Product: '<S55>/Divide5' */
  q1 = (rtu_err * rtu_P) >> 11;
  if (q1 > 32767) {
    q1 = 32767;
  } else {
    if (q1 < -32768) {
      q1 = -32768;
    }
  }

  /* Sum: '<S55>/Sum1' incorporates:
   *  DataTypeConversion: '<S57>/Data Type Conversion1'
   *  Product: '<S55>/Divide5'
   */
  q1 = (((rtb_Sum1_mz >> 16) << 1) + q1) >> 1;
  if (q1 > 32767) {
    q1 = 32767;
  } else {
    if (q1 < -32768) {
      q1 = -32768;
    }
  }

  /* RelationalOperator: '<S58>/LowerRelop1' incorporates:
   *  Sum: '<S55>/Sum1'
   */
  rtb_LowerRelop1_ge = ((int16_T)q1 > rtu_satMax);

  /* RelationalOperator: '<S58>/UpperRelop' incorporates:
   *  Sum: '<S55>/Sum1'
   */
  rtb_UpperRelop_o = ((int16_T)q1 < rtu_satMin);

  /* Switch: '<S58>/Switch1' incorporates:
   *  Sum: '<S55>/Sum1'
   *  Switch: '<S58>/Switch3'
   */
  if (rtb_LowerRelop1_ge) {
    rty_out_0 = rtu_satMax;
  } else if (rtb_UpperRelop_o) {
    /* Switch: '<S58>/Switch3' */
    rty_out_0 = rtu_satMin;
  } else {
    rty_out_0 = (int16_T)q1;
  }

  /* End of Switch: '<S58>/Switch1' */

  /* Signum: '<S56>/SignDeltaU2' incorporates:
   *  Sum: '<S55>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S56>/SignDeltaU2' */

  /* Signum: '<S56>/SignDeltaU3' incorporates:
   *  Sum: '<S55>/Sum1'
   */
  if ((int16_T)q1 < 0) {
    tmp = -1;
  } else {
    tmp = (int16_T)((int16_T)q1 > 0);
  }

  /* End of Signum: '<S56>/SignDeltaU3' */

  /* Update for UnitDelay: '<S55>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S56>/DataTypeConv4'
   *  Logic: '<S55>/AND1'
   *  Logic: '<S56>/AND1'
   *  RelationalOperator: '<S56>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp) && (rtb_LowerRelop1_ge ||
    rtb_UpperRelop_o));

  /* Update for UnitDelay: '<S57>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtb_Sum1_mz;
  return rty_out_0;
}

/*
 * System initialize for atomic system:
 *    '<S19>/Counter'
 *    '<S18>/Counter'
 */
void Counter_b_Init(DW_Counter_l *localDW, uint16_T rtp_z_cntInit)
{
  /* InitializeConditions for UnitDelay: '<S24>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/*
 * Output and update for atomic system:
 *    '<S19>/Counter'
 *    '<S18>/Counter'
 */
uint16_T Counter_i(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
                   DW_Counter_l *localDW)
{
  uint16_T rtu_rst_0;
  uint16_T rty_cnt_0;

  /* Switch: '<S24>/Switch1' incorporates:
   *  Constant: '<S24>/Constant23'
   *  UnitDelay: '<S24>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0U;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S24>/Switch1' */

  /* Sum: '<S23>/Sum1' */
  rty_cnt_0 = (uint16_T)((uint32_T)rtu_inc + rtu_rst_0);

  /* MinMax: '<S23>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S24>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S24>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S23>/MinMax' */
  return rty_cnt_0;
}

/*
 * System reset for atomic system:
 *    '<S15>/either_edge'
 *    '<S3>/either_edge'
 */
void either_edge_Reset(DW_either_edge *localDW)
{
  /* InitializeConditions for UnitDelay: '<S20>/UnitDelay' */
  localDW->UnitDelay_DSTATE = false;
}

/*
 * Output and update for atomic system:
 *    '<S15>/either_edge'
 *    '<S3>/either_edge'
 */
boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW)
{
  boolean_T rty_y_0;

  /* RelationalOperator: '<S20>/Relational Operator' incorporates:
   *  UnitDelay: '<S20>/UnitDelay'
   */
  rty_y_0 = (rtu_u != localDW->UnitDelay_DSTATE);

  /* Update for UnitDelay: '<S20>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtu_u;
  return rty_y_0;
}

/* System initialize for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter_Init(DW_Debounce_Filter *localDW)
{
  /* SystemInitialize for IfAction SubSystem: '<S15>/Qualification' */

  /* SystemInitialize for Atomic SubSystem: '<S19>/Counter' */
  Counter_b_Init(&localDW->Counter_i0, 0U);

  /* End of SystemInitialize for SubSystem: '<S19>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S15>/Qualification' */

  /* SystemInitialize for IfAction SubSystem: '<S15>/Dequalification' */

  /* SystemInitialize for Atomic SubSystem: '<S18>/Counter' */
  Counter_b_Init(&localDW->Counter_h, 0U);

  /* End of SystemInitialize for SubSystem: '<S18>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S15>/Dequalification' */
}

/* System reset for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter_Reset(DW_Debounce_Filter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S15>/UnitDelay' */
  localDW->UnitDelay_DSTATE = false;

  /* SystemReset for Atomic SubSystem: '<S15>/either_edge' */
  either_edge_Reset(&localDW->either_edge_k);

  /* End of SystemReset for SubSystem: '<S15>/either_edge' */
}

/* Output and update for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T rtu_tDeacv,
                     boolean_T *rty_y, DW_Debounce_Filter *localDW)
{
  uint16_T rtb_Sum1_l;
  boolean_T rtb_RelationalOperator_f;

  /* Outputs for Atomic SubSystem: '<S15>/either_edge' */
  rtb_RelationalOperator_f = either_edge(rtu_u, &localDW->either_edge_k);

  /* End of Outputs for SubSystem: '<S15>/either_edge' */

  /* If: '<S15>/If2' incorporates:
   *  Constant: '<S18>/Constant6'
   *  Constant: '<S19>/Constant6'
   *  Inport: '<S17>/yPrev'
   *  Logic: '<S15>/Logical Operator1'
   *  Logic: '<S15>/Logical Operator2'
   *  Logic: '<S15>/Logical Operator3'
   *  Logic: '<S15>/Logical Operator4'
   *  UnitDelay: '<S15>/UnitDelay'
   */
  if (rtu_u && (!localDW->UnitDelay_DSTATE)) {
    /* Outputs for IfAction SubSystem: '<S15>/Qualification' incorporates:
     *  ActionPort: '<S19>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S19>/Counter' */
    rtb_Sum1_l = (uint16_T) Counter_i(1U, rtu_tAcv, rtb_RelationalOperator_f,
      &localDW->Counter_i0);

    /* End of Outputs for SubSystem: '<S19>/Counter' */

    /* Switch: '<S19>/Switch2' incorporates:
     *  Constant: '<S19>/Constant6'
     *  RelationalOperator: '<S19>/Relational Operator2'
     */
    *rty_y = ((rtb_Sum1_l > rtu_tAcv) || localDW->UnitDelay_DSTATE);

    /* End of Outputs for SubSystem: '<S15>/Qualification' */
  } else if ((!rtu_u) && localDW->UnitDelay_DSTATE) {
    /* Outputs for IfAction SubSystem: '<S15>/Dequalification' incorporates:
     *  ActionPort: '<S18>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S18>/Counter' */
    rtb_Sum1_l = (uint16_T) Counter_i(1U, rtu_tDeacv, rtb_RelationalOperator_f,
      &localDW->Counter_h);

    /* End of Outputs for SubSystem: '<S18>/Counter' */

    /* Switch: '<S18>/Switch2' incorporates:
     *  Constant: '<S18>/Constant6'
     *  RelationalOperator: '<S18>/Relational Operator2'
     */
    *rty_y = ((!(rtb_Sum1_l > rtu_tDeacv)) && localDW->UnitDelay_DSTATE);

    /* End of Outputs for SubSystem: '<S15>/Dequalification' */
  } else {
    /* Outputs for IfAction SubSystem: '<S15>/Default' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    *rty_y = localDW->UnitDelay_DSTATE;

    /* End of Outputs for SubSystem: '<S15>/Default' */
  }

  /* End of If: '<S15>/If2' */

  /* Update for UnitDelay: '<S15>/UnitDelay' */
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
  uint8_T rtb_r_fieldWeak_XA_o1;
  int16_T rtb_Switch2_k;
  int16_T rtb_Abs5;
  int16_T rtb_Switch2_fl;
  int16_T rtb_Switch1_l;
  int16_T rtb_Merge;
  int16_T rtb_Merge1;
  int32_T rtb_DataTypeConversion;
  int16_T rtb_Saturation;
  int32_T rtb_Switch1;
  int32_T rtb_Sum1;
  int32_T rtb_Gain3;
  int16_T rtb_toNegative;
  int16_T rtb_Gain4;
  uint8_T rtb_r_fieldWeak_XA_o2;
  int16_T rtb_Gain2_f;
  int16_T rtb_id_fieldWeak_M1;
  int16_T rtb_MinMax2;
  int16_T rtb_TmpSignalConversionAtLow_Pa[2];
  int16_T tmp[4];
  int8_T UnitDelay3;

  /* Outputs for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Sum: '<S8>/Sum' incorporates:
   *  Gain: '<S8>/g_Ha'
   *  Gain: '<S8>/g_Hb'
   *  Inport: '<Root>/b_hallA '
   *  Inport: '<Root>/b_hallB'
   *  Inport: '<Root>/b_hallC'
   */
  rtb_Sum = (uint8_T)((uint32_T)(uint8_T)((uint32_T)(uint8_T)(rtU->b_hallA << 2)
    + (uint8_T)(rtU->b_hallB << 1)) + rtU->b_hallC);

  /* Logic: '<S7>/Logical Operator' incorporates:
   *  Inport: '<Root>/b_hallA '
   *  Inport: '<Root>/b_hallB'
   *  Inport: '<Root>/b_hallC'
   *  UnitDelay: '<S7>/UnitDelay1'
   *  UnitDelay: '<S7>/UnitDelay2'
   *  UnitDelay: '<S7>/UnitDelay3'
   */
  rtb_LogicalOperator = (boolean_T)((rtU->b_hallA != 0) ^ (rtU->b_hallB != 0) ^
    (rtU->b_hallC != 0) ^ (rtDW->UnitDelay3_DSTATE_fy != 0) ^
    (rtDW->UnitDelay1_DSTATE != 0)) ^ (rtDW->UnitDelay2_DSTATE_f != 0);

  /* If: '<S10>/If2' incorporates:
   *  If: '<S2>/If2'
   *  Inport: '<S13>/z_counterRawPrev'
   *  UnitDelay: '<S10>/UnitDelay3'
   */
  if (rtb_LogicalOperator) {
    /* Outputs for IfAction SubSystem: '<S2>/F01_03_Direction_Detection' incorporates:
     *  ActionPort: '<S9>/Action Port'
     */
    /* UnitDelay: '<S9>/UnitDelay3' */
    UnitDelay3 = rtDW->Switch2;

    /* Sum: '<S9>/Sum2' incorporates:
     *  Constant: '<S8>/vec_hallToPos'
     *  Selector: '<S8>/Selector'
     *  UnitDelay: '<S9>/UnitDelay2'
     */
    rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] -
                          rtDW->UnitDelay2_DSTATE_b);

    /* Switch: '<S9>/Switch2' incorporates:
     *  Constant: '<S9>/Constant20'
     *  Constant: '<S9>/Constant23'
     *  Constant: '<S9>/Constant24'
     *  Constant: '<S9>/Constant8'
     *  Logic: '<S9>/Logical Operator3'
     *  RelationalOperator: '<S9>/Relational Operator1'
     *  RelationalOperator: '<S9>/Relational Operator6'
     */
    if ((rtb_Sum2_h == 1) || (rtb_Sum2_h == -5)) {
      rtDW->Switch2 = 1;
    } else {
      rtDW->Switch2 = -1;
    }

    /* End of Switch: '<S9>/Switch2' */

    /* Update for UnitDelay: '<S9>/UnitDelay2' incorporates:
     *  Constant: '<S8>/vec_hallToPos'
     *  Selector: '<S8>/Selector'
     */
    rtDW->UnitDelay2_DSTATE_b = rtConstP.vec_hallToPos_Value[rtb_Sum];

    /* End of Outputs for SubSystem: '<S2>/F01_03_Direction_Detection' */

    /* Outputs for IfAction SubSystem: '<S10>/Raw_Motor_Speed_Estimation' incorporates:
     *  ActionPort: '<S13>/Action Port'
     */
    rtDW->z_counterRawPrev = rtDW->UnitDelay3_DSTATE;

    /* Sum: '<S13>/Sum7' incorporates:
     *  Inport: '<S13>/z_counterRawPrev'
     *  UnitDelay: '<S10>/UnitDelay3'
     *  UnitDelay: '<S13>/UnitDelay4'
     */
    rtb_Switch2_k = (int16_T)(rtDW->z_counterRawPrev - rtDW->UnitDelay4_DSTATE);

    /* Abs: '<S13>/Abs2' */
    if (rtb_Switch2_k < 0) {
      rtb_Switch1_l = (int16_T)-rtb_Switch2_k;
    } else {
      rtb_Switch1_l = rtb_Switch2_k;
    }

    /* End of Abs: '<S13>/Abs2' */

    /* Relay: '<S13>/dz_cntTrnsDet' */
    if (rtb_Switch1_l >= rtP->dz_cntTrnsDetHi) {
      rtDW->dz_cntTrnsDet_Mode = true;
    } else {
      if (rtb_Switch1_l <= rtP->dz_cntTrnsDetLo) {
        rtDW->dz_cntTrnsDet_Mode = false;
      }
    }

    rtDW->dz_cntTrnsDet = rtDW->dz_cntTrnsDet_Mode;

    /* End of Relay: '<S13>/dz_cntTrnsDet' */

    /* RelationalOperator: '<S13>/Relational Operator4' */
    rtb_RelationalOperator4_d = (rtDW->Switch2 != UnitDelay3);

    /* Switch: '<S13>/Switch3' incorporates:
     *  Constant: '<S13>/Constant4'
     *  Logic: '<S13>/Logical Operator1'
     *  Switch: '<S13>/Switch1'
     *  Switch: '<S13>/Switch2'
     *  UnitDelay: '<S13>/UnitDelay1'
     */
    if (rtb_RelationalOperator4_d && rtDW->UnitDelay1_DSTATE_n) {
      rtb_Switch1_l = 0;
    } else if (rtb_RelationalOperator4_d) {
      /* Switch: '<S13>/Switch2' incorporates:
       *  UnitDelay: '<S10>/UnitDelay4'
       */
      rtb_Switch1_l = rtDW->UnitDelay4_DSTATE_e;
    } else if (rtDW->dz_cntTrnsDet) {
      /* Switch: '<S13>/Switch1' incorporates:
       *  Constant: '<S13>/cf_speedCoef'
       *  Product: '<S13>/Divide14'
       *  Switch: '<S13>/Switch2'
       */
      rtb_Switch1_l = (int16_T)((rtP->cf_speedCoef << 4) /
        rtDW->z_counterRawPrev);
    } else {
      /* Switch: '<S13>/Switch1' incorporates:
       *  Constant: '<S13>/cf_speedCoef'
       *  Gain: '<S13>/g_Ha'
       *  Product: '<S13>/Divide13'
       *  Sum: '<S13>/Sum13'
       *  Switch: '<S13>/Switch2'
       *  UnitDelay: '<S13>/UnitDelay2'
       *  UnitDelay: '<S13>/UnitDelay3'
       *  UnitDelay: '<S13>/UnitDelay5'
       */
      rtb_Switch1_l = (int16_T)(((uint16_T)(rtP->cf_speedCoef << 2) << 4) /
        (int16_T)(((rtDW->UnitDelay2_DSTATE + rtDW->UnitDelay3_DSTATE_o) +
                   rtDW->UnitDelay5_DSTATE) + rtDW->z_counterRawPrev));
    }

    /* End of Switch: '<S13>/Switch3' */

    /* Product: '<S13>/Divide11' */
    rtDW->Divide11 = (int16_T)(rtb_Switch1_l * rtDW->Switch2);

    /* Update for UnitDelay: '<S13>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S13>/UnitDelay2' incorporates:
     *  UnitDelay: '<S13>/UnitDelay3'
     */
    rtDW->UnitDelay2_DSTATE = rtDW->UnitDelay3_DSTATE_o;

    /* Update for UnitDelay: '<S13>/UnitDelay3' incorporates:
     *  UnitDelay: '<S13>/UnitDelay5'
     */
    rtDW->UnitDelay3_DSTATE_o = rtDW->UnitDelay5_DSTATE;

    /* Update for UnitDelay: '<S13>/UnitDelay5' */
    rtDW->UnitDelay5_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S13>/UnitDelay1' */
    rtDW->UnitDelay1_DSTATE_n = rtb_RelationalOperator4_d;

    /* End of Outputs for SubSystem: '<S10>/Raw_Motor_Speed_Estimation' */
  }

  /* End of If: '<S10>/If2' */

  /* Outputs for Atomic SubSystem: '<S10>/Counter' */

  /* Constant: '<S10>/Constant6' incorporates:
   *  Constant: '<S10>/z_maxCntRst2'
   */
  rtb_Switch1_l = (int16_T) Counter(1, rtP->z_maxCntRst, rtb_LogicalOperator,
    &rtDW->Counter_e);

  /* End of Outputs for SubSystem: '<S10>/Counter' */

  /* Switch: '<S10>/Switch2' incorporates:
   *  Constant: '<S10>/Constant4'
   *  Constant: '<S10>/z_maxCntRst'
   *  RelationalOperator: '<S10>/Relational Operator2'
   */
  if (rtb_Switch1_l > rtP->z_maxCntRst) {
    rtb_Switch2_k = 0;
  } else {
    rtb_Switch2_k = rtDW->Divide11;
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Abs: '<S10>/Abs5' */
  if (rtb_Switch2_k < 0) {
    rtb_Abs5 = (int16_T)-rtb_Switch2_k;
  } else {
    rtb_Abs5 = rtb_Switch2_k;
  }

  /* End of Abs: '<S10>/Abs5' */

  /* Relay: '<S10>/n_commDeacv' */
  if (rtb_Abs5 >= rtP->n_commDeacvHi) {
    rtDW->n_commDeacv_Mode = true;
  } else {
    if (rtb_Abs5 <= rtP->n_commAcvLo) {
      rtDW->n_commDeacv_Mode = false;
    }
  }

  /* Logic: '<S10>/Logical Operator2' incorporates:
   *  Logic: '<S10>/Logical Operator1'
   *  Relay: '<S10>/n_commDeacv'
   */
  rtb_LogicalOperator = (rtDW->n_commDeacv_Mode && (!rtDW->dz_cntTrnsDet));

  /* Switch: '<S11>/Switch2' incorporates:
   *  Constant: '<S11>/Constant16'
   *  Product: '<S11>/Divide1'
   *  Product: '<S11>/Divide3'
   *  RelationalOperator: '<S11>/Relational Operator7'
   *  Sum: '<S11>/Sum3'
   *  Switch: '<S11>/Switch3'
   */
  if (rtb_LogicalOperator) {
    /* MinMax: '<S11>/MinMax' */
    rtb_Switch2_fl = rtb_Switch1_l;
    if (!(rtb_Switch2_fl < rtDW->z_counterRawPrev)) {
      rtb_Switch2_fl = rtDW->z_counterRawPrev;
    }

    /* End of MinMax: '<S11>/MinMax' */

    /* Switch: '<S11>/Switch3' incorporates:
     *  Constant: '<S11>/Constant16'
     *  Constant: '<S8>/vec_hallToPos'
     *  RelationalOperator: '<S11>/Relational Operator7'
     *  Selector: '<S8>/Selector'
     *  Sum: '<S11>/Sum1'
     */
    if (rtDW->Switch2 == 1) {
      rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
    } else {
      rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] + 1);
    }

    rtb_Switch2_fl = (int16_T)(((int16_T)((int16_T)((rtb_Switch2_fl << 14) /
      rtDW->z_counterRawPrev) * rtDW->Switch2) + (rtb_Sum2_h << 14)) >> 2);
  } else {
    if (rtDW->Switch2 == 1) {
      /* Switch: '<S11>/Switch3' incorporates:
       *  Constant: '<S8>/vec_hallToPos'
       *  Selector: '<S8>/Selector'
       */
      rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
    } else {
      /* Switch: '<S11>/Switch3' incorporates:
       *  Constant: '<S8>/vec_hallToPos'
       *  Selector: '<S8>/Selector'
       *  Sum: '<S11>/Sum1'
       */
      rtb_Sum2_h = (int8_T)(rtConstP.vec_hallToPos_Value[rtb_Sum] + 1);
    }

    rtb_Switch2_fl = (int16_T)(rtb_Sum2_h << 12);
  }

  /* End of Switch: '<S11>/Switch2' */

  /* MinMax: '<S11>/MinMax1' incorporates:
   *  Constant: '<S11>/Constant1'
   */
  if (!(rtb_Switch2_fl > 0)) {
    rtb_Switch2_fl = 0;
  }

  /* End of MinMax: '<S11>/MinMax1' */

  /* Product: '<S11>/Divide2' */
  rtb_Switch2_fl = (int16_T)((15 * rtb_Switch2_fl) >> 4);

  /* RelationalOperator: '<S10>/Relational Operator9' incorporates:
   *  Constant: '<S10>/n_stdStillDet'
   */
  rtb_RelationalOperator4_d = (rtb_Abs5 < rtP->n_stdStillDet);

  /* If: '<S1>/If2' incorporates:
   *  Constant: '<S1>/b_diagEna'
   *  Constant: '<S3>/CTRL_COMM2'
   *  Constant: '<S3>/t_errDequal'
   *  Constant: '<S3>/t_errQual'
   *  RelationalOperator: '<S3>/Relational Operator2'
   */
  rtb_Sum2_h = rtDW->If2_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtP->b_diagEna) {
    UnitDelay3 = 0;
  }

  rtDW->If2_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
    /* Disable for Outport: '<Root>/z_errCode' incorporates:
     *  Outport: '<S3>/z_errCode '
     */
    rtY->z_errCode = 0U;

    /* Disable for Outport: '<S3>/b_errFlag' */
    rtDW->Merge_n = false;
  }

  if (UnitDelay3 == 0) {
    if (0 != rtb_Sum2_h) {
      /* InitializeConditions for IfAction SubSystem: '<S1>/F02_Diagnostics' incorporates:
       *  ActionPort: '<S3>/Action Port'
       */
      /* InitializeConditions for If: '<S1>/If2' incorporates:
       *  UnitDelay: '<S3>/UnitDelay'
       */
      rtDW->UnitDelay_DSTATE_c = 0U;

      /* End of InitializeConditions for SubSystem: '<S1>/F02_Diagnostics' */

      /* SystemReset for IfAction SubSystem: '<S1>/F02_Diagnostics' incorporates:
       *  ActionPort: '<S3>/Action Port'
       */

      /* SystemReset for Atomic SubSystem: '<S3>/Debounce_Filter' */

      /* SystemReset for If: '<S1>/If2' */
      Debounce_Filter_Reset(&rtDW->Debounce_Filter_f);

      /* End of SystemReset for SubSystem: '<S3>/Debounce_Filter' */

      /* SystemReset for Atomic SubSystem: '<S3>/either_edge' */
      either_edge_Reset(&rtDW->either_edge_a);

      /* End of SystemReset for SubSystem: '<S3>/either_edge' */

      /* End of SystemReset for SubSystem: '<S1>/F02_Diagnostics' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/F02_Diagnostics' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Switch: '<S3>/Switch3' incorporates:
     *  Abs: '<S3>/Abs4'
     *  Constant: '<S3>/CTRL_COMM4'
     *  Constant: '<S3>/r_errInpTgtThres'
     *  Logic: '<S3>/Logical Operator1'
     *  RelationalOperator: '<S3>/Relational Operator7'
     *  S-Function (sfix_bitop): '<S3>/Bitwise Operator1'
     *  UnitDelay: '<S3>/UnitDelay'
     *  UnitDelay: '<S6>/UnitDelay4'
     */
    if ((rtDW->UnitDelay_DSTATE_c & 4) != 0) {
      rtb_RelationalOperator1_m = true;
    } else {
      if (rtDW->UnitDelay4_DSTATE_eu < 0) {
        /* Abs: '<S3>/Abs4' incorporates:
         *  UnitDelay: '<S6>/UnitDelay4'
         */
        rtb_toNegative = (int16_T)-rtDW->UnitDelay4_DSTATE_eu;
      } else {
        /* Abs: '<S3>/Abs4' incorporates:
         *  UnitDelay: '<S6>/UnitDelay4'
         */
        rtb_toNegative = rtDW->UnitDelay4_DSTATE_eu;
      }

      rtb_RelationalOperator1_m = ((rtb_toNegative > rtP->r_errInpTgtThres) &&
        rtb_RelationalOperator4_d);
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
    } else {
      /* Outport: '<Root>/z_errCode' incorporates:
       *  UnitDelay: '<S3>/UnitDelay'
       */
      rtY->z_errCode = rtDW->UnitDelay_DSTATE_c;
    }

    /* End of Switch: '<S3>/Switch1' */

    /* Update for UnitDelay: '<S3>/UnitDelay' incorporates:
     *  Outport: '<Root>/z_errCode'
     */
    rtDW->UnitDelay_DSTATE_c = rtY->z_errCode;

    /* End of Outputs for SubSystem: '<S1>/F02_Diagnostics' */
  }

  /* End of If: '<S1>/If2' */

  /* Logic: '<S25>/Logical Operator4' incorporates:
   *  Constant: '<S25>/constant2'
   *  Constant: '<S25>/constant8'
   *  Inport: '<Root>/b_motEna'
   *  Inport: '<Root>/z_ctrlModReq'
   *  Logic: '<S25>/Logical Operator1'
   *  Logic: '<S25>/Logical Operator7'
   *  RelationalOperator: '<S25>/Relational Operator10'
   *  RelationalOperator: '<S25>/Relational Operator11'
   *  RelationalOperator: '<S25>/Relational Operator2'
   *  UnitDelay: '<S4>/UnitDelay1'
   */
  rtb_RelationalOperator1_m = ((!rtU->b_motEna) || rtDW->Merge_n ||
    (rtU->z_ctrlModReq == 0) || ((rtU->z_ctrlModReq != rtDW->UnitDelay1_DSTATE_p)
    && (rtDW->UnitDelay1_DSTATE_p != 0)));

  /* Chart: '<S4>/F03_02_Control_Mode_Manager' incorporates:
   *  Constant: '<S25>/constant'
   *  Constant: '<S25>/constant1'
   *  Constant: '<S25>/constant5'
   *  Constant: '<S25>/constant6'
   *  Constant: '<S25>/constant7'
   *  Inport: '<Root>/z_ctrlModReq'
   *  Logic: '<S25>/Logical Operator3'
   *  Logic: '<S25>/Logical Operator6'
   *  Logic: '<S25>/Logical Operator9'
   *  RelationalOperator: '<S25>/Relational Operator1'
   *  RelationalOperator: '<S25>/Relational Operator3'
   *  RelationalOperator: '<S25>/Relational Operator4'
   *  RelationalOperator: '<S25>/Relational Operator5'
   *  RelationalOperator: '<S25>/Relational Operator6'
   */
  if (rtDW->is_active_c1_BLDC_controller == 0U) {
    rtDW->is_active_c1_BLDC_controller = 1U;
    rtDW->is_c1_BLDC_controller = IN_OPEN;
    rtb_Sum_l = OPEN_MODE;
  } else if (rtDW->is_c1_BLDC_controller == IN_ACTIVE) {
    if (rtb_RelationalOperator1_m) {
      rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
      rtDW->is_c1_BLDC_controller = IN_OPEN;
      rtb_Sum_l = OPEN_MODE;
    } else {
      switch (rtDW->is_ACTIVE) {
       case IN_SPEED_MODE:
        rtb_Sum_l = SPD_MODE;
        break;

       case IN_TORQUE_MODE:
        rtb_Sum_l = TRQ_MODE;
        break;

       default:
        rtb_Sum_l = VLT_MODE;
        break;
      }
    }
  } else {
    rtb_Sum_l = OPEN_MODE;
    if ((!rtb_RelationalOperator1_m) && ((rtU->z_ctrlModReq == 1) ||
         (rtU->z_ctrlModReq == 2) || (rtU->z_ctrlModReq == 3)) &&
        rtb_RelationalOperator4_d) {
      rtDW->is_c1_BLDC_controller = IN_ACTIVE;
      if (rtU->z_ctrlModReq == 3) {
        rtDW->is_ACTIVE = IN_TORQUE_MODE;
        rtb_Sum_l = TRQ_MODE;
      } else if (rtU->z_ctrlModReq == 2) {
        rtDW->is_ACTIVE = IN_SPEED_MODE;
        rtb_Sum_l = SPD_MODE;
      } else {
        rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
        rtb_Sum_l = VLT_MODE;
      }
    }
  }

  /* End of Chart: '<S4>/F03_02_Control_Mode_Manager' */

  /* Saturate: '<S1>/Saturation2' incorporates:
   *  Inport: '<Root>/r_inpTgt'
   */
  rtb_Gain3 = rtU->r_inpTgt << 4;

  /* If: '<S27>/If1' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Inport: '<Root>/r_inpTgt'
   *  Inport: '<S28>/r_inpTgt'
   *  Saturate: '<S1>/Saturation2'
   */
  if (rtP->z_ctrlTypSel == 2) {
    /* Outputs for IfAction SubSystem: '<S27>/FOC_Control_Type' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    /* SignalConversion: '<S30>/TmpSignal ConversionAtSelectorInport1' incorporates:
     *  Constant: '<S30>/Vd_max'
     *  Constant: '<S30>/constant1'
     *  Constant: '<S30>/i_max'
     *  Constant: '<S30>/n_max'
     */
    tmp[0] = 0;
    tmp[1] = rtP->Vd_max;
    tmp[2] = rtP->n_max;
    tmp[3] = rtP->i_max;

    /* End of Outputs for SubSystem: '<S27>/FOC_Control_Type' */

    /* Saturate: '<S1>/Saturation2' incorporates:
     *  Inport: '<Root>/r_inpTgt'
     */
    if (rtb_Gain3 >= 16000) {
      rtb_toNegative = 16000;
    } else if (rtb_Gain3 <= -16000) {
      rtb_toNegative = -16000;
    } else {
      rtb_toNegative = (int16_T)(rtU->r_inpTgt << 4);
    }

    /* Outputs for IfAction SubSystem: '<S27>/FOC_Control_Type' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    /* Product: '<S30>/Divide1' incorporates:
     *  Inport: '<Root>/z_ctrlModReq'
     *  Product: '<S30>/Divide4'
     *  Selector: '<S30>/Selector'
     */
    rtb_Merge = (int16_T)(((uint16_T)((tmp[rtU->z_ctrlModReq] << 5) / 125) *
      rtb_toNegative) >> 12);

    /* End of Outputs for SubSystem: '<S27>/FOC_Control_Type' */
  } else if (rtb_Gain3 >= 16000) {
    /* Outputs for IfAction SubSystem: '<S27>/Default_Control_Type' incorporates:
     *  ActionPort: '<S28>/Action Port'
     */
    /* Saturate: '<S1>/Saturation2' incorporates:
     *  Inport: '<S28>/r_inpTgt'
     */
    rtb_Merge = 16000;

    /* End of Outputs for SubSystem: '<S27>/Default_Control_Type' */
  } else if (rtb_Gain3 <= -16000) {
    /* Outputs for IfAction SubSystem: '<S27>/Default_Control_Type' incorporates:
     *  ActionPort: '<S28>/Action Port'
     */
    /* Saturate: '<S1>/Saturation2' incorporates:
     *  Inport: '<S28>/r_inpTgt'
     */
    rtb_Merge = -16000;

    /* End of Outputs for SubSystem: '<S27>/Default_Control_Type' */
  } else {
    /* Outputs for IfAction SubSystem: '<S27>/Default_Control_Type' incorporates:
     *  ActionPort: '<S28>/Action Port'
     */
    rtb_Merge = (int16_T)(rtU->r_inpTgt << 4);

    /* End of Outputs for SubSystem: '<S27>/Default_Control_Type' */
  }

  /* End of If: '<S27>/If1' */

  /* If: '<S27>/If2' incorporates:
   *  Inport: '<S29>/r_inpTgtScaRaw'
   */
  rtb_Sum2_h = rtDW->If2_ActiveSubsystem_j;
  UnitDelay3 = (int8_T)!(rtb_Sum_l == 0);
  rtDW->If2_ActiveSubsystem_j = UnitDelay3;
  switch (UnitDelay3) {
   case 0:
    if (UnitDelay3 != rtb_Sum2_h) {
      /* SystemReset for IfAction SubSystem: '<S27>/Open_Mode' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      /* SystemReset for Atomic SubSystem: '<S31>/rising_edge_init' */
      /* SystemReset for If: '<S27>/If2' incorporates:
       *  UnitDelay: '<S33>/UnitDelay'
       *  UnitDelay: '<S34>/UnitDelay'
       */
      rtDW->UnitDelay_DSTATE_e = true;

      /* End of SystemReset for SubSystem: '<S31>/rising_edge_init' */

      /* SystemReset for Atomic SubSystem: '<S31>/Rate_Limiter' */
      rtDW->UnitDelay_DSTATE = 0;

      /* End of SystemReset for SubSystem: '<S31>/Rate_Limiter' */
      /* End of SystemReset for SubSystem: '<S27>/Open_Mode' */
    }

    /* Outputs for IfAction SubSystem: '<S27>/Open_Mode' incorporates:
     *  ActionPort: '<S31>/Action Port'
     */
    /* DataTypeConversion: '<S31>/Data Type Conversion' incorporates:
     *  UnitDelay: '<S6>/UnitDelay4'
     */
    rtb_Gain3 = rtDW->UnitDelay4_DSTATE_eu << 12;
    rtb_DataTypeConversion = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 |
      -134217728 : rtb_Gain3 & 134217727;

    /* Outputs for Atomic SubSystem: '<S31>/rising_edge_init' */
    /* UnitDelay: '<S33>/UnitDelay' */
    rtb_RelationalOperator4_d = rtDW->UnitDelay_DSTATE_e;

    /* Update for UnitDelay: '<S33>/UnitDelay' incorporates:
     *  Constant: '<S33>/Constant'
     */
    rtDW->UnitDelay_DSTATE_e = false;

    /* End of Outputs for SubSystem: '<S31>/rising_edge_init' */

    /* Outputs for Atomic SubSystem: '<S31>/Rate_Limiter' */
    /* Switch: '<S34>/Switch1' incorporates:
     *  UnitDelay: '<S34>/UnitDelay'
     */
    if (rtb_RelationalOperator4_d) {
      rtb_Switch1 = rtb_DataTypeConversion;
    } else {
      rtb_Switch1 = rtDW->UnitDelay_DSTATE;
    }

    /* End of Switch: '<S34>/Switch1' */

    /* Sum: '<S32>/Sum1' */
    rtb_Gain3 = -rtb_Switch1;
    rtb_Sum1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 : rtb_Gain3
      & 134217727;

    /* Switch: '<S35>/Switch2' incorporates:
     *  Constant: '<S31>/dV_openRate'
     *  RelationalOperator: '<S35>/LowerRelop1'
     */
    if (rtb_Sum1 > rtP->dV_openRate) {
      rtb_Sum1 = rtP->dV_openRate;
    } else {
      /* Gain: '<S31>/Gain3' */
      rtb_Gain3 = -rtP->dV_openRate;
      rtb_Gain3 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
        rtb_Gain3 & 134217727;

      /* Switch: '<S35>/Switch' incorporates:
       *  RelationalOperator: '<S35>/UpperRelop'
       */
      if (rtb_Sum1 < rtb_Gain3) {
        rtb_Sum1 = rtb_Gain3;
      }

      /* End of Switch: '<S35>/Switch' */
    }

    /* End of Switch: '<S35>/Switch2' */

    /* Sum: '<S32>/Sum2' */
    rtb_Gain3 = rtb_Sum1 + rtb_Switch1;
    rtb_Switch1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
      rtb_Gain3 & 134217727;

    /* Switch: '<S34>/Switch2' */
    if (rtb_RelationalOperator4_d) {
      /* Update for UnitDelay: '<S34>/UnitDelay' */
      rtDW->UnitDelay_DSTATE = rtb_DataTypeConversion;
    } else {
      /* Update for UnitDelay: '<S34>/UnitDelay' */
      rtDW->UnitDelay_DSTATE = rtb_Switch1;
    }

    /* End of Switch: '<S34>/Switch2' */
    /* End of Outputs for SubSystem: '<S31>/Rate_Limiter' */

    /* DataTypeConversion: '<S31>/Data Type Conversion1' */
    rtb_Merge1 = (int16_T)(rtb_Switch1 >> 12);

    /* End of Outputs for SubSystem: '<S27>/Open_Mode' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S27>/Default_Mode' incorporates:
     *  ActionPort: '<S29>/Action Port'
     */
    rtb_Merge1 = rtb_Merge;

    /* End of Outputs for SubSystem: '<S27>/Default_Mode' */
    break;
  }

  /* End of If: '<S27>/If2' */

  /* Saturate: '<S1>/Saturation' incorporates:
   *  Inport: '<Root>/i_phaAB'
   */
  rtb_Gain3 = rtU->i_phaAB << 4;
  if (rtb_Gain3 >= 32000) {
    rtb_Saturation = 32000;
  } else if (rtb_Gain3 <= -32000) {
    rtb_Saturation = -32000;
  } else {
    rtb_Saturation = (int16_T)(rtU->i_phaAB << 4);
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Saturate: '<S1>/Saturation1' incorporates:
   *  Inport: '<Root>/i_phaBC'
   */
  rtb_Gain3 = rtU->i_phaBC << 4;
  if (rtb_Gain3 >= 32000) {
    rtb_Merge = 32000;
  } else if (rtb_Gain3 <= -32000) {
    rtb_Merge = -32000;
  } else {
    rtb_Merge = (int16_T)(rtU->i_phaBC << 4);
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* If: '<S1>/If1' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel1'
   */
  rtb_Sum2_h = rtDW->If1_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtP->z_ctrlTypSel == 2) {
    UnitDelay3 = 0;
  }

  rtDW->If1_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
    /* Disable for If: '<S5>/If2' */
    if (rtDW->If2_ActiveSubsystem_a == 0) {
      /* Disable for Outport: '<S37>/iq' */
      rtDW->Sum1[0] = 0;

      /* Disable for Outport: '<S37>/id' */
      rtDW->Sum1[1] = 0;
    }

    rtDW->If2_ActiveSubsystem_a = -1;

    /* End of Disable for If: '<S5>/If2' */

    /* Disable for If: '<S5>/If1' */
    if (rtDW->If1_ActiveSubsystem_e == 0) {
      /* Disable for Outport: '<S46>/Vd' */
      rtDW->Switch1 = 0;
    }

    rtDW->If1_ActiveSubsystem_e = -1;

    /* End of Disable for If: '<S5>/If1' */

    /* Disable for If: '<S41>/If1' */
    if (rtDW->If1_ActiveSubsystem_f == 0) {
      /* Disable for Outport: '<S51>/iq_limProt' */
      rtDW->Divide4 = 0;
    }

    rtDW->If1_ActiveSubsystem_f = -1;

    /* End of Disable for If: '<S41>/If1' */

    /* Disable for If: '<S41>/If2' */
    if (rtDW->If2_ActiveSubsystem_c == 0) {
      /* Disable for Outport: '<S52>/n_limProt' */
      rtDW->Divide1 = 0;
    }

    rtDW->If2_ActiveSubsystem_c = -1;

    /* End of Disable for If: '<S41>/If2' */

    /* Disable for SwitchCase: '<S5>/Switch Case' */
    rtDW->SwitchCase_ActiveSubsystem = -1;

    /* Disable for Outport: '<S5>/V_phaABC_FOC' */
    rtDW->Gain4[0] = 0;
    rtDW->Gain4[1] = 0;
    rtDW->Gain4[2] = 0;

    /* Disable for Outport: '<S5>/Vq' */
    rtDW->Merge = 0;

    /* Disable for Outport: '<S5>/r_devSignal1' */
    rtDW->Sum1[0] = 0;

    /* Disable for Outport: '<S5>/r_devSignal2' */
    rtDW->Sum1[1] = 0;
  }

  if (UnitDelay3 == 0) {
    /* Outputs for IfAction SubSystem: '<S1>/F04_Field_Oriented_Control' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Relay: '<S38>/n_fieldWeakAuth' */
    if (rtb_Abs5 >= rtP->n_fieldWeakAuthHi) {
      rtDW->n_fieldWeakAuth_Mode = true;
    } else {
      if (rtb_Abs5 <= rtP->n_fieldWeakAuthLo) {
        rtDW->n_fieldWeakAuth_Mode = false;
      }
    }

    /* Switch: '<S38>/Switch1' incorporates:
     *  Constant: '<S38>/a_elecPeriod1'
     *  Constant: '<S38>/b_fieldWeakEna'
     *  Logic: '<S38>/Logical Operator2'
     *  Relay: '<S38>/n_fieldWeakAuth'
     */
    if (rtP->b_fieldWeakEna && rtDW->n_fieldWeakAuth_Mode) {
      /* Abs: '<S38>/Abs5' */
      if (rtb_Merge1 < 0) {
        rtb_id_fieldWeak_M1 = (int16_T)-rtb_Merge1;
      } else {
        rtb_id_fieldWeak_M1 = rtb_Merge1;
      }

      /* End of Abs: '<S38>/Abs5' */

      /* PreLookup: '<S38>/r_fieldWeak_XA' */
      rtb_r_fieldWeak_XA_o1 = plook_u8s16u8n7_evenc_s(rtb_id_fieldWeak_M1,
        rtP->r_fieldWeak_XA[0], (uint16_T)(rtP->r_fieldWeak_XA[1] -
        rtP->r_fieldWeak_XA[0]), 11U, &rtb_r_fieldWeak_XA_o2);

      /* Interpolation_n-D: '<S38>/id_fieldWeak_M1' */
      rtb_id_fieldWeak_M1 = intrp1d_s16s32s32u8u8n7l_s(rtb_r_fieldWeak_XA_o1,
        rtb_r_fieldWeak_XA_o2, rtP->id_fieldWeak_M1);
    } else {
      rtb_id_fieldWeak_M1 = 0;
    }

    /* End of Switch: '<S38>/Switch1' */

    /* Gain: '<S38>/toNegative' */
    rtb_toNegative = (int16_T)-rtb_id_fieldWeak_M1;

    /* Gain: '<S41>/Gain4' incorporates:
     *  Constant: '<S41>/i_max'
     */
    rtb_Gain4 = (int16_T)-rtP->i_max;

    /* If: '<S36>/If1' incorporates:
     *  Constant: '<S36>/b_selPhaABCurrMeas'
     */
    if (rtP->b_selPhaABCurrMeas) {
      /* Outputs for IfAction SubSystem: '<S36>/Clarke_PhasesAB' incorporates:
       *  ActionPort: '<S48>/Action Port'
       */
      /* Gain: '<S48>/Gain4' */
      rtb_Gain3 = 18919 * rtb_Saturation;

      /* Gain: '<S48>/Gain2' */
      rtb_DataTypeConversion = 18919 * rtb_Merge;

      /* Sum: '<S48>/Sum1' incorporates:
       *  Gain: '<S48>/Gain2'
       *  Gain: '<S48>/Gain4'
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

      rtb_Gain2_f = (int16_T)rtb_Gain3;

      /* End of Sum: '<S48>/Sum1' */
      /* End of Outputs for SubSystem: '<S36>/Clarke_PhasesAB' */
    } else {
      /* Outputs for IfAction SubSystem: '<S36>/Clarke_PhasesBC' incorporates:
       *  ActionPort: '<S49>/Action Port'
       */
      /* Sum: '<S49>/Sum3' */
      rtb_Gain3 = rtb_Saturation - rtb_Merge;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Gain: '<S49>/Gain2' incorporates:
       *  Sum: '<S49>/Sum3'
       */
      rtb_Gain3 *= 18919;
      rtb_Gain2_f = (int16_T)(((rtb_Gain3 < 0 ? 32767 : 0) + rtb_Gain3) >> 15);

      /* Sum: '<S49>/Sum1' */
      rtb_Gain3 = -rtb_Saturation - rtb_Merge;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtb_Saturation = (int16_T)rtb_Gain3;

      /* End of Sum: '<S49>/Sum1' */
      /* End of Outputs for SubSystem: '<S36>/Clarke_PhasesBC' */
    }

    /* End of If: '<S36>/If1' */

    /* PreLookup: '<S38>/a_elecAngle_XA' */
    rtb_r_fieldWeak_XA_o1 = plook_u8s16_evencka(rtb_Switch2_fl, 0, 128U, 180U);

    /* Interpolation_n-D: '<S38>/r_sin_M1' */
    rtb_MinMax2 = rtConstP.r_sin_M1_Table[rtb_r_fieldWeak_XA_o1];

    /* Interpolation_n-D: '<S38>/r_cos_M1' */
    rtb_Merge = rtConstP.r_cos_M1_Table[rtb_r_fieldWeak_XA_o1];

    /* If: '<S5>/If2' incorporates:
     *  Constant: '<S37>/cf_currFilt'
     *  Inport: '<Root>/b_motEna'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem_a;
    UnitDelay3 = -1;
    if (rtU->b_motEna) {
      UnitDelay3 = 0;
    }

    rtDW->If2_ActiveSubsystem_a = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S37>/iq' */
      rtDW->Sum1[0] = 0;

      /* Disable for Outport: '<S37>/id' */
      rtDW->Sum1[1] = 0;
    }

    if (UnitDelay3 == 0) {
      if (0 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S5>/Current_Filtering' incorporates:
         *  ActionPort: '<S37>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S37>/Low_Pass_Filter' */

        /* SystemReset for If: '<S5>/If2' */
        Low_Pass_Filter_Reset(&rtDW->Low_Pass_Filter_m);

        /* End of SystemReset for SubSystem: '<S37>/Low_Pass_Filter' */

        /* End of SystemReset for SubSystem: '<S5>/Current_Filtering' */
      }

      /* Sum: '<S43>/Sum6' incorporates:
       *  Interpolation_n-D: '<S38>/r_cos_M1'
       *  Interpolation_n-D: '<S38>/r_sin_M1'
       *  Product: '<S43>/Divide1'
       *  Product: '<S43>/Divide4'
       */
      rtb_Gain3 = (int16_T)((rtb_Gain2_f *
        rtConstP.r_cos_M1_Table[rtb_r_fieldWeak_XA_o1]) >> 14) - (int16_T)
        ((rtb_Saturation * rtConstP.r_sin_M1_Table[rtb_r_fieldWeak_XA_o1]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S5>/Current_Filtering' incorporates:
       *  ActionPort: '<S37>/Action Port'
       */
      /* SignalConversion: '<S37>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S43>/Sum6'
       */
      rtb_TmpSignalConversionAtLow_Pa[0] = (int16_T)rtb_Gain3;

      /* End of Outputs for SubSystem: '<S5>/Current_Filtering' */

      /* Sum: '<S43>/Sum1' incorporates:
       *  Interpolation_n-D: '<S38>/r_cos_M1'
       *  Interpolation_n-D: '<S38>/r_sin_M1'
       *  Product: '<S43>/Divide2'
       *  Product: '<S43>/Divide3'
       */
      rtb_Gain3 = (int16_T)((rtb_Saturation *
        rtConstP.r_cos_M1_Table[rtb_r_fieldWeak_XA_o1]) >> 14) + (int16_T)
        ((rtb_Gain2_f * rtConstP.r_sin_M1_Table[rtb_r_fieldWeak_XA_o1]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S5>/Current_Filtering' incorporates:
       *  ActionPort: '<S37>/Action Port'
       */
      /* SignalConversion: '<S37>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S43>/Sum1'
       */
      rtb_TmpSignalConversionAtLow_Pa[1] = (int16_T)rtb_Gain3;

      /* Outputs for Atomic SubSystem: '<S37>/Low_Pass_Filter' */
      Low_Pass_Filter(rtb_TmpSignalConversionAtLow_Pa, rtP->cf_currFilt,
                      rtDW->Sum1, &rtDW->Low_Pass_Filter_m);

      /* End of Outputs for SubSystem: '<S37>/Low_Pass_Filter' */

      /* End of Outputs for SubSystem: '<S5>/Current_Filtering' */
    }

    /* End of If: '<S5>/If2' */

    /* If: '<S5>/If1' incorporates:
     *  Constant: '<S41>/Vd_max1'
     *  Constant: '<S46>/cf_idKi1'
     *  Constant: '<S46>/cf_idKp1'
     *  Constant: '<S46>/constant1'
     *  Gain: '<S41>/Gain3'
     *  Sum: '<S46>/Sum3'
     */
    rtb_Sum2_h = rtDW->If1_ActiveSubsystem_e;
    UnitDelay3 = -1;
    if (rtb_LogicalOperator) {
      UnitDelay3 = 0;
    }

    rtDW->If1_ActiveSubsystem_e = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S46>/Vd' */
      rtDW->Switch1 = 0;
    }

    if (UnitDelay3 == 0) {
      if (0 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S5>/Vd_Calculation' incorporates:
         *  ActionPort: '<S46>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S46>/PI_clamp_fixdt_id' */

        /* SystemReset for If: '<S5>/If1' */
        PI_clamp_fixdt_Reset(&rtDW->PI_clamp_fixdt_id);

        /* End of SystemReset for SubSystem: '<S46>/PI_clamp_fixdt_id' */

        /* End of SystemReset for SubSystem: '<S5>/Vd_Calculation' */
      }

      /* Outputs for IfAction SubSystem: '<S5>/Vd_Calculation' incorporates:
       *  ActionPort: '<S46>/Action Port'
       */
      /* Switch: '<S65>/Switch2' incorporates:
       *  Constant: '<S41>/i_max'
       *  RelationalOperator: '<S65>/LowerRelop1'
       *  RelationalOperator: '<S65>/UpperRelop'
       *  Switch: '<S65>/Switch'
       */
      if (rtb_toNegative > rtP->i_max) {
        rtb_toNegative = rtP->i_max;
      } else {
        if (rtb_toNegative < rtb_Gain4) {
          /* Switch: '<S65>/Switch' */
          rtb_toNegative = rtb_Gain4;
        }
      }

      /* End of Switch: '<S65>/Switch2' */

      /* Sum: '<S46>/Sum3' */
      rtb_Gain3 = rtb_toNegative - rtDW->Sum1[1];
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for Atomic SubSystem: '<S46>/PI_clamp_fixdt_id' */
      PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_idKp, rtP->cf_idKi, rtP->Vd_max,
                     (int16_T)-rtP->Vd_max, 0, &rtDW->Switch1,
                     &rtDW->PI_clamp_fixdt_id);

      /* End of Outputs for SubSystem: '<S46>/PI_clamp_fixdt_id' */

      /* End of Outputs for SubSystem: '<S5>/Vd_Calculation' */
    }

    /* End of If: '<S5>/If1' */

    /* Abs: '<S41>/Abs5' */
    if (rtDW->Switch1 < 0) {
      rtb_toNegative = (int16_T)-rtDW->Switch1;
    } else {
      rtb_toNegative = rtDW->Switch1;
    }

    /* End of Abs: '<S41>/Abs5' */

    /* PreLookup: '<S41>/Vq_max_XA' */
    rtb_r_fieldWeak_XA_o1 = plook_u8s16_evencka(rtb_toNegative, rtP->Vq_max_XA[0],
      (uint16_T)(rtP->Vq_max_XA[1] - rtP->Vq_max_XA[0]), 45U);

    /* Gain: '<S41>/Gain5' incorporates:
     *  Interpolation_n-D: '<S41>/Vq_max_M1'
     */
    rtb_Gain2_f = (int16_T)-rtP->Vq_max_M1[rtb_r_fieldWeak_XA_o1];

    /* Interpolation_n-D: '<S41>/iq_maxSca_M1' incorporates:
     *  Constant: '<S41>/i_max'
     *  Product: '<S41>/Divide4'
     */
    rtb_Gain3 = rtb_id_fieldWeak_M1 << 16;
    rtb_Gain3 = (rtb_Gain3 == MIN_int32_T) && (rtP->i_max == -1) ? MAX_int32_T :
      rtb_Gain3 / rtP->i_max;
    if (rtb_Gain3 < 0) {
      rtb_Gain3 = 0;
    } else {
      if (rtb_Gain3 > 65535) {
        rtb_Gain3 = 65535;
      }
    }

    /* Product: '<S41>/Divide1' incorporates:
     *  Constant: '<S41>/i_max'
     *  Interpolation_n-D: '<S41>/iq_maxSca_M1'
     *  PreLookup: '<S41>/iq_maxSca_XA'
     *  Product: '<S41>/Divide4'
     */
    rtb_id_fieldWeak_M1 = (int16_T)
      ((rtConstP.iq_maxSca_M1_Table[plook_u8u16_evencka((uint16_T)rtb_Gain3, 0U,
         1311U, 49U)] * rtP->i_max) >> 16);

    /* Gain: '<S41>/Gain1' */
    rtb_Saturation = (int16_T)-rtb_id_fieldWeak_M1;

    /* If: '<S41>/If1' incorporates:
     *  Constant: '<S41>/CTRL_COMM'
     *  Constant: '<S41>/CTRL_COMM1'
     *  Logic: '<S41>/Logical Operator2'
     *  RelationalOperator: '<S41>/Relational Operator1'
     *  RelationalOperator: '<S41>/Relational Operator2'
     */
    rtb_Sum2_h = rtDW->If1_ActiveSubsystem_f;
    UnitDelay3 = -1;
    if ((rtb_Sum_l == 1) || (rtb_Sum_l == 2)) {
      UnitDelay3 = 0;
    }

    rtDW->If1_ActiveSubsystem_f = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S51>/iq_limProt' */
      rtDW->Divide4 = 0;
    }

    if (UnitDelay3 == 0) {
      /* Outputs for IfAction SubSystem: '<S41>/Current_Limit_Protection' incorporates:
       *  ActionPort: '<S51>/Action Port'
       */
      /* Switch: '<S53>/Switch2' incorporates:
       *  RelationalOperator: '<S53>/LowerRelop1'
       *  RelationalOperator: '<S53>/UpperRelop'
       *  Switch: '<S53>/Switch'
       */
      if (rtDW->Sum1[0] > rtb_id_fieldWeak_M1) {
        rtb_toNegative = rtb_id_fieldWeak_M1;
      } else if (rtDW->Sum1[0] < rtb_Saturation) {
        /* Switch: '<S53>/Switch' */
        rtb_toNegative = rtb_Saturation;
      } else {
        rtb_toNegative = rtDW->Sum1[0];
      }

      /* End of Switch: '<S53>/Switch2' */

      /* Product: '<S51>/Divide4' incorporates:
       *  Constant: '<S51>/cf_iqKpLimProt'
       *  Sum: '<S51>/Sum3'
       */
      rtb_Gain3 = ((int16_T)(rtb_toNegative - rtDW->Sum1[0]) *
                   rtP->cf_iqKpLimProt) >> 8;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtDW->Divide4 = (int16_T)rtb_Gain3;

      /* End of Product: '<S51>/Divide4' */
      /* End of Outputs for SubSystem: '<S41>/Current_Limit_Protection' */
    }

    /* End of If: '<S41>/If1' */

    /* Gain: '<S41>/Gain6' incorporates:
     *  Constant: '<S41>/n_max1'
     */
    rtb_toNegative = (int16_T)-rtP->n_max;

    /* If: '<S41>/If2' incorporates:
     *  Constant: '<S41>/CTRL_COMM2'
     *  Constant: '<S41>/CTRL_COMM3'
     *  Logic: '<S41>/Logical Operator1'
     *  RelationalOperator: '<S41>/Relational Operator3'
     *  RelationalOperator: '<S41>/Relational Operator4'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem_c;
    UnitDelay3 = -1;
    if ((rtb_Sum_l == 1) || (rtb_Sum_l == 3)) {
      UnitDelay3 = 0;
    }

    rtDW->If2_ActiveSubsystem_c = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S52>/n_limProt' */
      rtDW->Divide1 = 0;
    }

    if (UnitDelay3 == 0) {
      /* Outputs for IfAction SubSystem: '<S41>/Speed_Limit_Protection' incorporates:
       *  ActionPort: '<S52>/Action Port'
       */
      /* Switch: '<S54>/Switch2' incorporates:
       *  Constant: '<S41>/n_max1'
       *  RelationalOperator: '<S54>/LowerRelop1'
       *  RelationalOperator: '<S54>/UpperRelop'
       *  Switch: '<S54>/Switch'
       */
      if (rtb_Switch2_k > rtP->n_max) {
        rtb_toNegative = rtP->n_max;
      } else {
        if (!(rtb_Switch2_k < rtb_toNegative)) {
          rtb_toNegative = rtb_Switch2_k;
        }
      }

      /* End of Switch: '<S54>/Switch2' */

      /* Product: '<S52>/Divide1' incorporates:
       *  Constant: '<S52>/cf_nKpLimProt'
       *  Sum: '<S52>/Sum1'
       */
      rtb_Gain3 = ((int16_T)(rtb_toNegative - rtb_Switch2_k) *
                   rtP->cf_nKpLimProt) >> 8;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtDW->Divide1 = (int16_T)rtb_Gain3;

      /* End of Product: '<S52>/Divide1' */
      /* End of Outputs for SubSystem: '<S41>/Speed_Limit_Protection' */
    }

    /* End of If: '<S41>/If2' */

    /* SwitchCase: '<S5>/Switch Case' incorporates:
     *  Constant: '<S44>/cf_iqKiLimProt'
     *  Constant: '<S44>/cf_nKi'
     *  Constant: '<S44>/cf_nKp'
     *  Inport: '<S42>/r_inpTgtSca'
     *  Interpolation_n-D: '<S41>/Vq_max_M1'
     *  Product: '<S44>/Divide1'
     *  SignalConversion: '<S44>/Signal Conversion2'
     *  Sum: '<S44>/Sum3'
     */
    rtb_Sum2_h = rtDW->SwitchCase_ActiveSubsystem;
    switch (rtb_Sum_l) {
     case 1:
      UnitDelay3 = 0;
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
      /* Outputs for IfAction SubSystem: '<S5>/Voltage_Mode' incorporates:
       *  ActionPort: '<S47>/Action Port'
       */
      /* Sum: '<S47>/Sum3' */
      rtb_Gain3 = (rtb_Merge1 + rtDW->Divide4) + rtDW->Divide1;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Switch: '<S69>/Switch2' incorporates:
       *  Interpolation_n-D: '<S41>/Vq_max_M1'
       *  RelationalOperator: '<S69>/LowerRelop1'
       *  RelationalOperator: '<S69>/UpperRelop'
       *  Sum: '<S47>/Sum3'
       *  Switch: '<S69>/Switch'
       */
      if ((int16_T)rtb_Gain3 > rtP->Vq_max_M1[rtb_r_fieldWeak_XA_o1]) {
        /* SignalConversion: '<S47>/Signal Conversion2' */
        rtDW->Merge = rtP->Vq_max_M1[rtb_r_fieldWeak_XA_o1];
      } else if ((int16_T)rtb_Gain3 < rtb_Gain2_f) {
        /* Switch: '<S69>/Switch' incorporates:
         *  SignalConversion: '<S47>/Signal Conversion2'
         */
        rtDW->Merge = rtb_Gain2_f;
      } else {
        /* SignalConversion: '<S47>/Signal Conversion2' */
        rtDW->Merge = (int16_T)rtb_Gain3;
      }

      /* End of Switch: '<S69>/Switch2' */
      /* End of Outputs for SubSystem: '<S5>/Voltage_Mode' */
      break;

     case 1:
      if (UnitDelay3 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S5>/Speed_Mode' incorporates:
         *  ActionPort: '<S44>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S44>/PI_clamp_fixdt_n' */

        /* SystemReset for SwitchCase: '<S5>/Switch Case' */
        PI_clamp_fixdt_n_Reset(&rtDW->PI_clamp_fixdt_n_o);

        /* End of SystemReset for SubSystem: '<S44>/PI_clamp_fixdt_n' */

        /* End of SystemReset for SubSystem: '<S5>/Speed_Mode' */
      }

      /* Outputs for IfAction SubSystem: '<S5>/Speed_Mode' incorporates:
       *  ActionPort: '<S44>/Action Port'
       */
      /* Sum: '<S44>/Sum3' */
      rtb_Gain3 = rtb_Merge1 - rtb_Switch2_k;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for Atomic SubSystem: '<S44>/PI_clamp_fixdt_n' */
      rtDW->Merge = (int16_T) PI_clamp_fixdt_n((int16_T)rtb_Gain3, rtP->cf_nKp,
        rtP->cf_nKi, rtP->Vq_max_M1[rtb_r_fieldWeak_XA_o1], rtb_Gain2_f,
        (int16_T)((rtDW->Divide4 * rtP->cf_iqKiLimProt) >> 10),
        &rtDW->PI_clamp_fixdt_n_o);

      /* End of Outputs for SubSystem: '<S44>/PI_clamp_fixdt_n' */

      /* End of Outputs for SubSystem: '<S5>/Speed_Mode' */
      break;

     case 2:
      if (UnitDelay3 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S5>/Torque_Mode' incorporates:
         *  ActionPort: '<S45>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S45>/PI_clamp_fixdt_iq' */

        /* SystemReset for SwitchCase: '<S5>/Switch Case' */
        PI_clamp_fixdt_Reset(&rtDW->PI_clamp_fixdt_iq);

        /* End of SystemReset for SubSystem: '<S45>/PI_clamp_fixdt_iq' */

        /* End of SystemReset for SubSystem: '<S5>/Torque_Mode' */
      }

      /* Outputs for IfAction SubSystem: '<S5>/Torque_Mode' incorporates:
       *  ActionPort: '<S45>/Action Port'
       */
      /* Sum: '<S45>/Sum2' */
      rtb_Gain3 = rtb_Merge1 + rtDW->Divide1;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Switch: '<S60>/Switch2' incorporates:
       *  RelationalOperator: '<S60>/LowerRelop1'
       *  Sum: '<S45>/Sum2'
       */
      if (!((int16_T)rtb_Gain3 > rtb_id_fieldWeak_M1)) {
        /* Switch: '<S60>/Switch' incorporates:
         *  RelationalOperator: '<S60>/UpperRelop'
         */
        if ((int16_T)rtb_Gain3 < rtb_Saturation) {
          rtb_id_fieldWeak_M1 = rtb_Saturation;
        } else {
          rtb_id_fieldWeak_M1 = (int16_T)rtb_Gain3;
        }

        /* End of Switch: '<S60>/Switch' */
      }

      /* End of Switch: '<S60>/Switch2' */

      /* Sum: '<S45>/Sum1' */
      rtb_Gain3 = rtb_id_fieldWeak_M1 - rtDW->Sum1[0];
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for Atomic SubSystem: '<S45>/PI_clamp_fixdt_iq' */

      /* SignalConversion: '<S45>/Signal Conversion2' incorporates:
       *  Constant: '<S45>/cf_iqKi'
       *  Constant: '<S45>/cf_iqKp'
       *  Constant: '<S45>/constant'
       *  Interpolation_n-D: '<S41>/Vq_max_M1'
       *  Sum: '<S45>/Sum1'
       */
      PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_iqKp, rtP->cf_iqKi,
                     rtP->Vq_max_M1[rtb_r_fieldWeak_XA_o1], rtb_Gain2_f, 0,
                     &rtDW->Merge, &rtDW->PI_clamp_fixdt_iq);

      /* End of Outputs for SubSystem: '<S45>/PI_clamp_fixdt_iq' */

      /* End of Outputs for SubSystem: '<S5>/Torque_Mode' */
      break;

     case 3:
      /* Outputs for IfAction SubSystem: '<S5>/Open_Mode' incorporates:
       *  ActionPort: '<S42>/Action Port'
       */
      rtDW->Merge = rtb_Merge1;

      /* End of Outputs for SubSystem: '<S5>/Open_Mode' */
      break;
    }

    /* End of SwitchCase: '<S5>/Switch Case' */

    /* Sum: '<S40>/Sum6' incorporates:
     *  Product: '<S40>/Divide1'
     *  Product: '<S40>/Divide4'
     */
    rtb_Gain3 = (int16_T)((rtDW->Switch1 * rtb_Merge) >> 14) - (int16_T)
      ((rtDW->Merge * rtb_MinMax2) >> 14);
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Sum: '<S40>/Sum1' incorporates:
     *  Product: '<S40>/Divide2'
     *  Product: '<S40>/Divide3'
     */
    rtb_DataTypeConversion = (int16_T)((rtDW->Switch1 * rtb_MinMax2) >> 14) +
      (int16_T)((rtDW->Merge * rtb_Merge) >> 14);
    if (rtb_DataTypeConversion > 32767) {
      rtb_DataTypeConversion = 32767;
    } else {
      if (rtb_DataTypeConversion < -32768) {
        rtb_DataTypeConversion = -32768;
      }
    }

    /* Gain: '<S39>/Gain1' incorporates:
     *  Sum: '<S40>/Sum1'
     */
    rtb_DataTypeConversion = 14189 * (int16_T)rtb_DataTypeConversion;

    /* Sum: '<S39>/Sum6' incorporates:
     *  Gain: '<S39>/Gain1'
     *  Gain: '<S39>/Gain3'
     *  Sum: '<S40>/Sum6'
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

    /* Sum: '<S39>/Sum2' incorporates:
     *  Sum: '<S39>/Sum6'
     *  Sum: '<S40>/Sum6'
     */
    rtb_Switch1 = -(int16_T)rtb_Gain3 - (int16_T)rtb_DataTypeConversion;
    if (rtb_Switch1 > 32767) {
      rtb_Switch1 = 32767;
    } else {
      if (rtb_Switch1 < -32768) {
        rtb_Switch1 = -32768;
      }
    }

    /* MinMax: '<S39>/MinMax1' incorporates:
     *  Sum: '<S39>/Sum2'
     *  Sum: '<S39>/Sum6'
     *  Sum: '<S40>/Sum6'
     */
    rtb_Merge = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 < (int16_T)rtb_DataTypeConversion)) {
      rtb_Merge = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Merge < (int16_T)rtb_Switch1)) {
      rtb_Merge = (int16_T)rtb_Switch1;
    }

    /* MinMax: '<S39>/MinMax2' incorporates:
     *  Sum: '<S39>/Sum2'
     *  Sum: '<S39>/Sum6'
     *  Sum: '<S40>/Sum6'
     */
    rtb_Saturation = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 > (int16_T)rtb_DataTypeConversion)) {
      rtb_Saturation = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Saturation > (int16_T)rtb_Switch1)) {
      rtb_Saturation = (int16_T)rtb_Switch1;
    }

    /* Sum: '<S39>/Add' incorporates:
     *  MinMax: '<S39>/MinMax1'
     *  MinMax: '<S39>/MinMax2'
     */
    rtb_Sum1 = rtb_Merge + rtb_Saturation;
    if (rtb_Sum1 > 32767) {
      rtb_Sum1 = 32767;
    } else {
      if (rtb_Sum1 < -32768) {
        rtb_Sum1 = -32768;
      }
    }

    /* Gain: '<S39>/Gain2' incorporates:
     *  Sum: '<S39>/Add'
     */
    rtb_Gain2_f = (int16_T)(rtb_Sum1 >> 1);

    /* Sum: '<S39>/Add1' incorporates:
     *  Sum: '<S40>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_Gain3 - rtb_Gain2_f;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S39>/Gain4' incorporates:
     *  Sum: '<S39>/Add1'
     */
    rtDW->Gain4[0] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S39>/Add1' incorporates:
     *  Sum: '<S39>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_DataTypeConversion - rtb_Gain2_f;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S39>/Gain4' incorporates:
     *  Sum: '<S39>/Add1'
     */
    rtDW->Gain4[1] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S39>/Add1' incorporates:
     *  Sum: '<S39>/Sum2'
     */
    rtb_Gain3 = (int16_T)rtb_Switch1 - rtb_Gain2_f;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S39>/Gain4' incorporates:
     *  Sum: '<S39>/Add1'
     */
    rtDW->Gain4[2] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* End of Outputs for SubSystem: '<S1>/F04_Field_Oriented_Control' */
  }

  /* End of If: '<S1>/If1' */

  /* Switch: '<S6>/Switch2' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Constant: '<S6>/CTRL_COMM1'
   *  RelationalOperator: '<S6>/Relational Operator6'
   */
  if (rtP->z_ctrlTypSel == 2) {
    rtb_Merge = rtDW->Merge;
  } else {
    rtb_Merge = rtb_Merge1;
  }

  /* End of Switch: '<S6>/Switch2' */

  /* If: '<S6>/If' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Constant: '<S6>/CTRL_COMM2'
   *  Constant: '<S6>/CTRL_COMM3'
   *  Constant: '<S8>/vec_hallToPos'
   *  Inport: '<S72>/V_phaABC_FOC_in'
   *  Logic: '<S6>/Logical Operator1'
   *  Logic: '<S6>/Logical Operator2'
   *  LookupNDDirect: '<S70>/z_commutMap_M1'
   *  RelationalOperator: '<S6>/Relational Operator1'
   *  RelationalOperator: '<S6>/Relational Operator2'
   *  Selector: '<S8>/Selector'
   *
   * About '<S70>/z_commutMap_M1':
   *  2-dimensional Direct Look-Up returning a Column
   */
  if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 2)) {
    /* Outputs for IfAction SubSystem: '<S6>/F05_02_FOC_Method' incorporates:
     *  ActionPort: '<S72>/Action Port'
     */
    rtb_Saturation = rtDW->Gain4[0];
    rtb_id_fieldWeak_M1 = rtDW->Gain4[1];
    rtb_Merge1 = rtDW->Gain4[2];

    /* End of Outputs for SubSystem: '<S6>/F05_02_FOC_Method' */
  } else if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 1)) {
    /* Outputs for IfAction SubSystem: '<S6>/F05_01_SIN_Method' incorporates:
     *  ActionPort: '<S71>/Action Port'
     */
    /* Relay: '<S73>/n_fieldWeakAuth' */
    if (rtb_Abs5 >= rtP->n_fieldWeakAuthHi) {
      rtDW->n_fieldWeakAuth_Mode_m = true;
    } else {
      if (rtb_Abs5 <= rtP->n_fieldWeakAuthLo) {
        rtDW->n_fieldWeakAuth_Mode_m = false;
      }
    }

    /* Switch: '<S73>/Switch_PhaAdv' incorporates:
     *  Constant: '<S73>/b_fieldWeakEna'
     *  Logic: '<S73>/Logical Operator1'
     *  Product: '<S74>/Divide2'
     *  Product: '<S74>/Divide3'
     *  Relay: '<S73>/n_fieldWeakAuth'
     *  Sum: '<S74>/Sum3'
     */
    if (rtP->b_fieldWeakEna && rtDW->n_fieldWeakAuth_Mode_m) {
      /* Abs: '<S73>/Abs5' */
      if (rtb_Merge1 < 0) {
        rtb_id_fieldWeak_M1 = (int16_T)-rtb_Merge1;
      } else {
        rtb_id_fieldWeak_M1 = rtb_Merge1;
      }

      /* End of Abs: '<S73>/Abs5' */

      /* PreLookup: '<S73>/r_phaAdv_XA' */
      rtb_Sum = plook_u8s16u8n7_evenc_s(rtb_id_fieldWeak_M1, rtP->r_phaAdv_XA[0],
        (uint16_T)(rtP->r_phaAdv_XA[1] - rtP->r_phaAdv_XA[0]), 10U,
        &rtb_r_fieldWeak_XA_o1);

      /* Interpolation_n-D: '<S73>/a_phaAdv_M1' */
      rtb_MinMax2 = intrp1d_s16s32s32u8u8n7l_s(rtb_Sum, rtb_r_fieldWeak_XA_o1,
        rtP->a_phaAdv_M1);

      /* Sum: '<S73>/Sum3' incorporates:
       *  Product: '<S73>/Product2'
       */
      rtb_MinMax2 = (int16_T)(((int16_T)(rtb_MinMax2 * rtDW->Switch2) >> 2) +
        rtb_Switch2_fl);
      rtb_MinMax2 -= (int16_T)(23040 * (int16_T)div_nde_s32_floor(rtb_MinMax2,
        23040));
    } else {
      rtb_MinMax2 = rtb_Switch2_fl;
    }

    /* End of Switch: '<S73>/Switch_PhaAdv' */

    /* PreLookup: '<S71>/a_elecAngle_XA' */
    rtb_Sum = plook_u8s16_evencka(rtb_MinMax2, 0, 128U, 180U);

    /* Product: '<S71>/Divide2' incorporates:
     *  Interpolation_n-D: '<S71>/r_sin3PhaA_M1'
     *  Interpolation_n-D: '<S71>/r_sin3PhaB_M1'
     *  Interpolation_n-D: '<S71>/r_sin3PhaC_M1'
     */
    rtb_Saturation = (int16_T)((rtb_Merge1 *
      rtConstP.r_sin3PhaA_M1_Table[rtb_Sum]) >> 14);
    rtb_id_fieldWeak_M1 = (int16_T)((rtb_Merge1 *
      rtConstP.r_sin3PhaB_M1_Table[rtb_Sum]) >> 14);
    rtb_Merge1 = (int16_T)((rtb_Merge1 * rtConstP.r_sin3PhaC_M1_Table[rtb_Sum]) >>
      14);

    /* End of Outputs for SubSystem: '<S6>/F05_01_SIN_Method' */
  } else {
    /* Outputs for IfAction SubSystem: '<S6>/F05_00_COM_Method' incorporates:
     *  ActionPort: '<S70>/Action Port'
     */
    if (rtConstP.vec_hallToPos_Value[rtb_Sum] > 5) {
      /* LookupNDDirect: '<S70>/z_commutMap_M1'
       *
       * About '<S70>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 5;
    } else if (rtConstP.vec_hallToPos_Value[rtb_Sum] < 0) {
      /* LookupNDDirect: '<S70>/z_commutMap_M1'
       *
       * About '<S70>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 0;
    } else {
      /* LookupNDDirect: '<S70>/z_commutMap_M1' incorporates:
       *  Constant: '<S8>/vec_hallToPos'
       *  Selector: '<S8>/Selector'
       *
       * About '<S70>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
    }

    /* LookupNDDirect: '<S70>/z_commutMap_M1' incorporates:
     *  Constant: '<S8>/vec_hallToPos'
     *  Selector: '<S8>/Selector'
     *
     * About '<S70>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_DataTypeConversion = rtb_Sum2_h * 3;

    /* Product: '<S70>/Divide2' incorporates:
     *  LookupNDDirect: '<S70>/z_commutMap_M1'
     *
     * About '<S70>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_Saturation = (int16_T)(rtb_Merge *
      rtConstP.z_commutMap_M1_table[rtb_DataTypeConversion]);
    rtb_id_fieldWeak_M1 = (int16_T)(rtConstP.z_commutMap_M1_table[1 +
      rtb_DataTypeConversion] * rtb_Merge);
    rtb_Merge1 = (int16_T)(rtConstP.z_commutMap_M1_table[2 +
      rtb_DataTypeConversion] * rtb_Merge);

    /* End of Outputs for SubSystem: '<S6>/F05_00_COM_Method' */
  }

  /* End of If: '<S6>/If' */

  /* Outport: '<Root>/DC_phaA' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion6'
   */
  rtY->DC_phaA = (int16_T)(rtb_Saturation >> 4);

  /* Outport: '<Root>/DC_phaB' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion6'
   */
  rtY->DC_phaB = (int16_T)(rtb_id_fieldWeak_M1 >> 4);

  /* Update for UnitDelay: '<S7>/UnitDelay3' incorporates:
   *  Inport: '<Root>/b_hallA '
   */
  rtDW->UnitDelay3_DSTATE_fy = rtU->b_hallA;

  /* Update for UnitDelay: '<S7>/UnitDelay1' incorporates:
   *  Inport: '<Root>/b_hallB'
   */
  rtDW->UnitDelay1_DSTATE = rtU->b_hallB;

  /* Update for UnitDelay: '<S7>/UnitDelay2' incorporates:
   *  Inport: '<Root>/b_hallC'
   */
  rtDW->UnitDelay2_DSTATE_f = rtU->b_hallC;

  /* Update for UnitDelay: '<S10>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtb_Switch1_l;

  /* Update for UnitDelay: '<S10>/UnitDelay4' */
  rtDW->UnitDelay4_DSTATE_e = rtb_Abs5;

  /* Update for UnitDelay: '<S6>/UnitDelay4' */
  rtDW->UnitDelay4_DSTATE_eu = rtb_Merge;

  /* Update for UnitDelay: '<S4>/UnitDelay1' */
  rtDW->UnitDelay1_DSTATE_p = rtb_Sum_l;

  /* Outport: '<Root>/DC_phaC' incorporates:
   *  DataTypeConversion: '<S6>/Data Type Conversion6'
   */
  rtY->DC_phaC = (int16_T)(rtb_Merge1 >> 4);

  /* Outport: '<Root>/n_mot' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion1'
   */
  rtY->n_mot = (int16_T)(rtb_Switch2_k >> 4);

  /* Outport: '<Root>/a_elecAngle' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion7'
   */
  rtY->a_elecAngle = (int16_T)(rtb_Switch2_fl >> 6);

  /* Outport: '<Root>/r_devSignal1' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion4'
   */
  rtY->r_devSignal1 = (int16_T)(rtDW->Sum1[0] >> 4);

  /* Outport: '<Root>/r_devSignal2' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion5'
   */
  rtY->r_devSignal2 = (int16_T)(rtDW->Sum1[1] >> 4);

  /* End of Outputs for SubSystem: '<Root>/BLDC_controller' */
}

/* Model initialize function */
void BLDC_controller_initialize(RT_MODEL *const rtM)
{
  P *rtP = ((P *) rtM->defaultParam);
  DW *rtDW = ((DW *) rtM->dwork);

  /* Start for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Start for If: '<S1>/If2' */
  rtDW->If2_ActiveSubsystem = -1;

  /* Start for If: '<S27>/If2' */
  rtDW->If2_ActiveSubsystem_j = -1;

  /* Start for If: '<S1>/If1' */
  rtDW->If1_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S1>/F04_Field_Oriented_Control' */
  /* Start for If: '<S5>/If2' */
  rtDW->If2_ActiveSubsystem_a = -1;

  /* Start for If: '<S5>/If1' */
  rtDW->If1_ActiveSubsystem_e = -1;

  /* Start for If: '<S41>/If1' */
  rtDW->If1_ActiveSubsystem_f = -1;

  /* Start for If: '<S41>/If2' */
  rtDW->If2_ActiveSubsystem_c = -1;

  /* Start for SwitchCase: '<S5>/Switch Case' */
  rtDW->SwitchCase_ActiveSubsystem = -1;

  /* End of Start for SubSystem: '<S1>/F04_Field_Oriented_Control' */
  /* End of Start for SubSystem: '<Root>/BLDC_controller' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* InitializeConditions for UnitDelay: '<S10>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtP->z_maxCntRst;

  /* SystemInitialize for IfAction SubSystem: '<S10>/Raw_Motor_Speed_Estimation' */
  /* SystemInitialize for Outport: '<S13>/z_counter' */
  rtDW->z_counterRawPrev = rtP->z_maxCntRst;

  /* End of SystemInitialize for SubSystem: '<S10>/Raw_Motor_Speed_Estimation' */

  /* SystemInitialize for Atomic SubSystem: '<S10>/Counter' */
  Counter_Init(&rtDW->Counter_e, rtP->z_maxCntRst);

  /* End of SystemInitialize for SubSystem: '<S10>/Counter' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/F02_Diagnostics' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Debounce_Filter' */
  Debounce_Filter_Init(&rtDW->Debounce_Filter_f);

  /* End of SystemInitialize for SubSystem: '<S3>/Debounce_Filter' */

  /* End of SystemInitialize for SubSystem: '<S1>/F02_Diagnostics' */

  /* SystemInitialize for IfAction SubSystem: '<S27>/Open_Mode' */
  /* SystemInitialize for Atomic SubSystem: '<S31>/rising_edge_init' */
  /* InitializeConditions for UnitDelay: '<S33>/UnitDelay' */
  rtDW->UnitDelay_DSTATE_e = true;

  /* End of SystemInitialize for SubSystem: '<S31>/rising_edge_init' */
  /* End of SystemInitialize for SubSystem: '<S27>/Open_Mode' */
  /* End of SystemInitialize for SubSystem: '<Root>/BLDC_controller' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
