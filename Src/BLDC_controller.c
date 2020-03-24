/*
 * File: BLDC_controller.c
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 1.1260
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Tue Mar 24 11:01:08 2020
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
extern void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_satMax, int16_T rtu_satMin, int32_T rtu_ext_limProt, int16_T
  *rty_out, DW_PI_clamp_fixdt *localDW);
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
  /* InitializeConditions for UnitDelay: '<S16>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/* Output and update for atomic system: '<S12>/Counter' */
int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst, DW_Counter *
                localDW)
{
  int16_T rtu_rst_0;
  int16_T rty_cnt_0;

  /* Switch: '<S16>/Switch1' incorporates:
   *  Constant: '<S16>/Constant23'
   *  UnitDelay: '<S16>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S16>/Switch1' */

  /* Sum: '<S14>/Sum1' */
  rty_cnt_0 = (int16_T)(rtu_inc + rtu_rst_0);

  /* MinMax: '<S14>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S16>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S16>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S14>/MinMax' */
  return rty_cnt_0;
}

/*
 * Output and update for atomic system:
 *    '<S54>/PI_clamp_fixdt'
 *    '<S52>/PI_clamp_fixdt'
 *    '<S53>/PI_clamp_fixdt'
 */
void PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I, int16_T
                    rtu_satMax, int16_T rtu_satMin, int32_T rtu_ext_limProt,
                    int16_T *rty_out, DW_PI_clamp_fixdt *localDW)
{
  boolean_T rtb_LowerRelop1_c;
  boolean_T rtb_UpperRelop_e;
  int32_T rtb_Sum1_n;
  int32_T q0;
  int32_T tmp;
  int16_T tmp_0;

  /* Sum: '<S65>/Sum2' incorporates:
   *  Product: '<S65>/Divide2'
   */
  q0 = rtu_err * rtu_I;
  if ((q0 < 0) && (rtu_ext_limProt < MIN_int32_T - q0)) {
    q0 = MIN_int32_T;
  } else if ((q0 > 0) && (rtu_ext_limProt > MAX_int32_T - q0)) {
    q0 = MAX_int32_T;
  } else {
    q0 += rtu_ext_limProt;
  }

  /* Switch: '<S65>/Switch1' incorporates:
   *  Constant: '<S65>/Constant'
   *  Sum: '<S65>/Sum2'
   *  UnitDelay: '<S65>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  /* End of Switch: '<S65>/Switch1' */

  /* Sum: '<S68>/Sum1' incorporates:
   *  UnitDelay: '<S68>/UnitDelay'
   */
  rtb_Sum1_n = tmp + localDW->UnitDelay_DSTATE;

  /* Product: '<S65>/Divide5' */
  tmp = (rtu_err * rtu_P) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* Sum: '<S65>/Sum1' incorporates:
   *  DataTypeConversion: '<S68>/Data Type Conversion1'
   *  Product: '<S65>/Divide5'
   */
  tmp = (((rtb_Sum1_n >> 16) << 1) + tmp) >> 1;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* RelationalOperator: '<S69>/LowerRelop1' incorporates:
   *  Sum: '<S65>/Sum1'
   */
  rtb_LowerRelop1_c = ((int16_T)tmp > rtu_satMax);

  /* RelationalOperator: '<S69>/UpperRelop' incorporates:
   *  Sum: '<S65>/Sum1'
   */
  rtb_UpperRelop_e = ((int16_T)tmp < rtu_satMin);

  /* Switch: '<S69>/Switch1' incorporates:
   *  Sum: '<S65>/Sum1'
   *  Switch: '<S69>/Switch3'
   */
  if (rtb_LowerRelop1_c) {
    *rty_out = rtu_satMax;
  } else if (rtb_UpperRelop_e) {
    /* Switch: '<S69>/Switch3' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = (int16_T)tmp;
  }

  /* End of Switch: '<S69>/Switch1' */

  /* Signum: '<S67>/SignDeltaU2' incorporates:
   *  Sum: '<S65>/Sum2'
   */
  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  /* End of Signum: '<S67>/SignDeltaU2' */

  /* Signum: '<S67>/SignDeltaU3' incorporates:
   *  Sum: '<S65>/Sum1'
   */
  if ((int16_T)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_T)((int16_T)tmp > 0);
  }

  /* End of Signum: '<S67>/SignDeltaU3' */

  /* Update for UnitDelay: '<S65>/UnitDelay1' incorporates:
   *  DataTypeConversion: '<S67>/DataTypeConv4'
   *  Logic: '<S65>/AND1'
   *  Logic: '<S67>/AND1'
   *  RelationalOperator: '<S67>/Equal1'
   */
  localDW->UnitDelay1_DSTATE = ((q0 == tmp_0) && (rtb_LowerRelop1_c ||
    rtb_UpperRelop_e));

  /* Update for UnitDelay: '<S68>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtb_Sum1_n;
}

/* System reset for atomic system: '<S41>/Low_Pass_Filter' */
void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S50>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[0] = 0;
  localDW->UnitDelay1_DSTATE[1] = 0;
}

/* Output and update for atomic system: '<S41>/Low_Pass_Filter' */
void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T rty_y[2],
                     DW_Low_Pass_Filter *localDW)
{
  int32_T rtb_Sum3_g;

  /* Sum: '<S50>/Sum2' incorporates:
   *  UnitDelay: '<S50>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_u[0] - (localDW->UnitDelay1_DSTATE[0] >> 16);
  if (rtb_Sum3_g > 32767) {
    rtb_Sum3_g = 32767;
  } else {
    if (rtb_Sum3_g < -32768) {
      rtb_Sum3_g = -32768;
    }
  }

  /* Sum: '<S50>/Sum3' incorporates:
   *  Product: '<S50>/Divide3'
   *  Sum: '<S50>/Sum2'
   *  UnitDelay: '<S50>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_coef * rtb_Sum3_g + localDW->UnitDelay1_DSTATE[0];

  /* DataTypeConversion: '<S50>/Data Type Conversion' */
  rty_y[0] = (int16_T)(rtb_Sum3_g >> 16);

  /* Update for UnitDelay: '<S50>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[0] = rtb_Sum3_g;

  /* Sum: '<S50>/Sum2' incorporates:
   *  UnitDelay: '<S50>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_u[1] - (localDW->UnitDelay1_DSTATE[1] >> 16);
  if (rtb_Sum3_g > 32767) {
    rtb_Sum3_g = 32767;
  } else {
    if (rtb_Sum3_g < -32768) {
      rtb_Sum3_g = -32768;
    }
  }

  /* Sum: '<S50>/Sum3' incorporates:
   *  Product: '<S50>/Divide3'
   *  Sum: '<S50>/Sum2'
   *  UnitDelay: '<S50>/UnitDelay1'
   */
  rtb_Sum3_g = rtu_coef * rtb_Sum3_g + localDW->UnitDelay1_DSTATE[1];

  /* DataTypeConversion: '<S50>/Data Type Conversion' */
  rty_y[1] = (int16_T)(rtb_Sum3_g >> 16);

  /* Update for UnitDelay: '<S50>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[1] = rtb_Sum3_g;
}

/*
 * System initialize for atomic system:
 *    '<S73>/I_backCalc_fixdt'
 *    '<S73>/I_backCalc_fixdt1'
 *    '<S72>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt_Init(DW_I_backCalc_fixdt *localDW, int32_T rtp_yInit)
{
  /* InitializeConditions for UnitDelay: '<S80>/UnitDelay' */
  localDW->UnitDelay_DSTATE_h = rtp_yInit;
}

/*
 * Output and update for atomic system:
 *    '<S73>/I_backCalc_fixdt'
 *    '<S73>/I_backCalc_fixdt1'
 *    '<S72>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt(int16_T rtu_err, uint16_T rtu_I, uint16_T rtu_Kb, int16_T
                      rtu_satMax, int16_T rtu_satMin, int16_T *rty_out,
                      DW_I_backCalc_fixdt *localDW)
{
  int32_T rtb_Sum1_e0;
  int16_T rtb_DataTypeConversion1_no;

  /* Sum: '<S78>/Sum2' incorporates:
   *  Product: '<S78>/Divide2'
   *  UnitDelay: '<S78>/UnitDelay'
   */
  rtb_Sum1_e0 = (rtu_err * rtu_I) >> 4;
  if ((rtb_Sum1_e0 < 0) && (localDW->UnitDelay_DSTATE < MIN_int32_T
       - rtb_Sum1_e0)) {
    rtb_Sum1_e0 = MIN_int32_T;
  } else if ((rtb_Sum1_e0 > 0) && (localDW->UnitDelay_DSTATE > MAX_int32_T
              - rtb_Sum1_e0)) {
    rtb_Sum1_e0 = MAX_int32_T;
  } else {
    rtb_Sum1_e0 += localDW->UnitDelay_DSTATE;
  }

  /* End of Sum: '<S78>/Sum2' */

  /* Sum: '<S80>/Sum1' incorporates:
   *  UnitDelay: '<S80>/UnitDelay'
   */
  rtb_Sum1_e0 += localDW->UnitDelay_DSTATE_h;

  /* DataTypeConversion: '<S80>/Data Type Conversion1' */
  rtb_DataTypeConversion1_no = (int16_T)(rtb_Sum1_e0 >> 12);

  /* Switch: '<S81>/Switch2' incorporates:
   *  RelationalOperator: '<S81>/LowerRelop1'
   *  RelationalOperator: '<S81>/UpperRelop'
   *  Switch: '<S81>/Switch'
   */
  if (rtb_DataTypeConversion1_no > rtu_satMax) {
    *rty_out = rtu_satMax;
  } else if (rtb_DataTypeConversion1_no < rtu_satMin) {
    /* Switch: '<S81>/Switch' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = rtb_DataTypeConversion1_no;
  }

  /* End of Switch: '<S81>/Switch2' */

  /* Update for UnitDelay: '<S78>/UnitDelay' incorporates:
   *  Product: '<S78>/Divide1'
   *  Sum: '<S78>/Sum3'
   */
  localDW->UnitDelay_DSTATE = (int16_T)(*rty_out - rtb_DataTypeConversion1_no) *
    rtu_Kb;

  /* Update for UnitDelay: '<S80>/UnitDelay' */
  localDW->UnitDelay_DSTATE_h = rtb_Sum1_e0;
}

/*
 * System initialize for atomic system:
 *    '<S21>/Counter'
 *    '<S20>/Counter'
 */
void Counter_b_Init(DW_Counter_l *localDW, uint16_T rtp_z_cntInit)
{
  /* InitializeConditions for UnitDelay: '<S26>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/*
 * Output and update for atomic system:
 *    '<S21>/Counter'
 *    '<S20>/Counter'
 */
uint16_T Counter_i(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
                   DW_Counter_l *localDW)
{
  uint16_T rtu_rst_0;
  uint16_T rty_cnt_0;

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Constant23'
   *  UnitDelay: '<S26>/UnitDelay'
   */
  if (rtu_rst) {
    rtu_rst_0 = 0U;
  } else {
    rtu_rst_0 = localDW->UnitDelay_DSTATE;
  }

  /* End of Switch: '<S26>/Switch1' */

  /* Sum: '<S25>/Sum1' */
  rty_cnt_0 = (uint16_T)((uint32_T)rtu_inc + rtu_rst_0);

  /* MinMax: '<S25>/MinMax' */
  if (rty_cnt_0 < rtu_max) {
    /* Update for UnitDelay: '<S26>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S26>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S25>/MinMax' */
  return rty_cnt_0;
}

/*
 * Output and update for atomic system:
 *    '<S17>/either_edge'
 *    '<S3>/either_edge'
 */
boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW)
{
  boolean_T rty_y_0;

  /* RelationalOperator: '<S22>/Relational Operator' incorporates:
   *  UnitDelay: '<S22>/UnitDelay'
   */
  rty_y_0 = (rtu_u != localDW->UnitDelay_DSTATE);

  /* Update for UnitDelay: '<S22>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtu_u;
  return rty_y_0;
}

/* System initialize for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter_Init(DW_Debounce_Filter *localDW)
{
  /* SystemInitialize for IfAction SubSystem: '<S17>/Qualification' */

  /* SystemInitialize for Atomic SubSystem: '<S21>/Counter' */
  Counter_b_Init(&localDW->Counter_i0, 0U);

  /* End of SystemInitialize for SubSystem: '<S21>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S17>/Qualification' */

  /* SystemInitialize for IfAction SubSystem: '<S17>/Dequalification' */

  /* SystemInitialize for Atomic SubSystem: '<S20>/Counter' */
  Counter_b_Init(&localDW->Counter_h, 0U);

  /* End of SystemInitialize for SubSystem: '<S20>/Counter' */

  /* End of SystemInitialize for SubSystem: '<S17>/Dequalification' */
}

/* Output and update for atomic system: '<S3>/Debounce_Filter' */
void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T rtu_tDeacv,
                     boolean_T *rty_y, DW_Debounce_Filter *localDW)
{
  boolean_T rtb_UnitDelay_o;
  uint16_T rtb_Sum1_g3;
  boolean_T rtb_RelationalOperator_f;

  /* UnitDelay: '<S17>/UnitDelay' */
  rtb_UnitDelay_o = localDW->UnitDelay_DSTATE;

  /* Outputs for Atomic SubSystem: '<S17>/either_edge' */
  rtb_RelationalOperator_f = either_edge(rtu_u, &localDW->either_edge_k);

  /* End of Outputs for SubSystem: '<S17>/either_edge' */

  /* If: '<S17>/If2' incorporates:
   *  Constant: '<S20>/Constant6'
   *  Constant: '<S21>/Constant6'
   *  Inport: '<S19>/yPrev'
   *  Logic: '<S17>/Logical Operator1'
   *  Logic: '<S17>/Logical Operator2'
   *  Logic: '<S17>/Logical Operator3'
   *  Logic: '<S17>/Logical Operator4'
   */
  if (rtu_u && (!rtb_UnitDelay_o)) {
    /* Outputs for IfAction SubSystem: '<S17>/Qualification' incorporates:
     *  ActionPort: '<S21>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S21>/Counter' */
    rtb_Sum1_g3 = (uint16_T) Counter_i(1U, rtu_tAcv, rtb_RelationalOperator_f,
      &localDW->Counter_i0);

    /* End of Outputs for SubSystem: '<S21>/Counter' */

    /* Switch: '<S21>/Switch2' incorporates:
     *  Constant: '<S21>/Constant6'
     *  RelationalOperator: '<S21>/Relational Operator2'
     */
    *rty_y = (rtb_Sum1_g3 > rtu_tAcv);

    /* End of Outputs for SubSystem: '<S17>/Qualification' */
  } else if ((!rtu_u) && rtb_UnitDelay_o) {
    /* Outputs for IfAction SubSystem: '<S17>/Dequalification' incorporates:
     *  ActionPort: '<S20>/Action Port'
     */

    /* Outputs for Atomic SubSystem: '<S20>/Counter' */
    rtb_Sum1_g3 = (uint16_T) Counter_i(1U, rtu_tDeacv, rtb_RelationalOperator_f,
      &localDW->Counter_h);

    /* End of Outputs for SubSystem: '<S20>/Counter' */

    /* Switch: '<S20>/Switch2' incorporates:
     *  Constant: '<S20>/Constant6'
     *  RelationalOperator: '<S20>/Relational Operator2'
     */
    *rty_y = !(rtb_Sum1_g3 > rtu_tDeacv);

    /* End of Outputs for SubSystem: '<S17>/Dequalification' */
  } else {
    /* Outputs for IfAction SubSystem: '<S17>/Default' incorporates:
     *  ActionPort: '<S19>/Action Port'
     */
    *rty_y = rtb_UnitDelay_o;

    /* End of Outputs for SubSystem: '<S17>/Default' */
  }

  /* End of If: '<S17>/If2' */

  /* Update for UnitDelay: '<S17>/UnitDelay' */
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
  boolean_T rtb_RelationalOperator9;
  int8_T rtb_Sum2_h;
  boolean_T rtb_RelationalOperator4_d;
  boolean_T rtb_RelationalOperator1_m;
  uint8_T rtb_Sum_l;
  int16_T rtb_Switch2_k;
  int16_T rtb_Abs5;
  int16_T rtb_Switch2_fl;
  int16_T rtb_Switch1_l;
  int16_T rtb_DataTypeConversion2;
  int16_T rtb_Saturation1;
  int16_T rtb_Switch2_l;
  int16_T rtb_Merge;
  int16_T rtb_toNegative;
  int32_T rtb_DataTypeConversion;
  int32_T rtb_Switch1;
  int32_T rtb_Sum1;
  int32_T rtb_Gain3;
  int16_T rtb_TmpSignalConversionAtLow_Pa[2];
  int16_T tmp[4];
  int8_T UnitDelay3;
  int16_T rtb_Merge_f_idx_1;

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
   *  Inport: '<S15>/z_counterRawPrev'
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
     *  ActionPort: '<S15>/Action Port'
     */
    rtDW->z_counterRawPrev = rtDW->UnitDelay3_DSTATE;

    /* Sum: '<S15>/Sum7' incorporates:
     *  Inport: '<S15>/z_counterRawPrev'
     *  UnitDelay: '<S12>/UnitDelay3'
     *  UnitDelay: '<S15>/UnitDelay4'
     */
    rtb_Switch2_k = (int16_T)(rtDW->z_counterRawPrev - rtDW->UnitDelay4_DSTATE);

    /* Abs: '<S15>/Abs2' */
    if (rtb_Switch2_k < 0) {
      rtb_Switch1_l = (int16_T)-rtb_Switch2_k;
    } else {
      rtb_Switch1_l = rtb_Switch2_k;
    }

    /* End of Abs: '<S15>/Abs2' */

    /* Relay: '<S15>/dz_cntTrnsDet' */
    if (rtb_Switch1_l >= rtP->dz_cntTrnsDetHi) {
      rtDW->dz_cntTrnsDet_Mode = true;
    } else {
      if (rtb_Switch1_l <= rtP->dz_cntTrnsDetLo) {
        rtDW->dz_cntTrnsDet_Mode = false;
      }
    }

    rtDW->dz_cntTrnsDet = rtDW->dz_cntTrnsDet_Mode;

    /* End of Relay: '<S15>/dz_cntTrnsDet' */

    /* RelationalOperator: '<S15>/Relational Operator4' */
    rtb_RelationalOperator4_d = (rtDW->Switch2_e != UnitDelay3);

    /* Switch: '<S15>/Switch3' incorporates:
     *  Constant: '<S15>/Constant4'
     *  Logic: '<S15>/Logical Operator1'
     *  Switch: '<S15>/Switch1'
     *  Switch: '<S15>/Switch2'
     *  UnitDelay: '<S15>/UnitDelay1'
     */
    if (rtb_RelationalOperator4_d && rtDW->UnitDelay1_DSTATE_n) {
      rtb_Switch1_l = 0;
    } else if (rtb_RelationalOperator4_d) {
      /* Switch: '<S15>/Switch2' incorporates:
       *  UnitDelay: '<S12>/UnitDelay4'
       */
      rtb_Switch1_l = rtDW->UnitDelay4_DSTATE_e;
    } else if (rtDW->dz_cntTrnsDet) {
      /* Switch: '<S15>/Switch1' incorporates:
       *  Constant: '<S15>/cf_speedCoef'
       *  Product: '<S15>/Divide14'
       *  Switch: '<S15>/Switch2'
       */
      rtb_Switch1_l = (int16_T)((rtP->cf_speedCoef << 4) /
        rtDW->z_counterRawPrev);
    } else {
      /* Switch: '<S15>/Switch1' incorporates:
       *  Constant: '<S15>/cf_speedCoef'
       *  Gain: '<S15>/g_Ha'
       *  Product: '<S15>/Divide13'
       *  Sum: '<S15>/Sum13'
       *  Switch: '<S15>/Switch2'
       *  UnitDelay: '<S15>/UnitDelay2'
       *  UnitDelay: '<S15>/UnitDelay3'
       *  UnitDelay: '<S15>/UnitDelay5'
       */
      rtb_Switch1_l = (int16_T)(((uint16_T)(rtP->cf_speedCoef << 2) << 4) /
        (int16_T)(((rtDW->UnitDelay2_DSTATE + rtDW->UnitDelay3_DSTATE_o) +
                   rtDW->UnitDelay5_DSTATE) + rtDW->z_counterRawPrev));
    }

    /* End of Switch: '<S15>/Switch3' */

    /* Product: '<S15>/Divide11' */
    rtDW->Divide11 = (int16_T)(rtb_Switch1_l * rtDW->Switch2_e);

    /* Update for UnitDelay: '<S15>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S15>/UnitDelay2' incorporates:
     *  UnitDelay: '<S15>/UnitDelay3'
     */
    rtDW->UnitDelay2_DSTATE = rtDW->UnitDelay3_DSTATE_o;

    /* Update for UnitDelay: '<S15>/UnitDelay3' incorporates:
     *  UnitDelay: '<S15>/UnitDelay5'
     */
    rtDW->UnitDelay3_DSTATE_o = rtDW->UnitDelay5_DSTATE;

    /* Update for UnitDelay: '<S15>/UnitDelay5' */
    rtDW->UnitDelay5_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S15>/UnitDelay1' */
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

  /* Logic: '<S12>/Logical Operator2' incorporates:
   *  Logic: '<S12>/Logical Operator1'
   *  Relay: '<S12>/n_commDeacv'
   */
  rtb_LogicalOperator = (rtDW->n_commDeacv_Mode && (!rtDW->dz_cntTrnsDet));

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
    rtb_Switch2_fl = rtb_Switch1_l;
    if (!(rtb_Switch2_fl < rtDW->z_counterRawPrev)) {
      rtb_Switch2_fl = rtDW->z_counterRawPrev;
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

    rtb_Switch2_fl = (int16_T)(((int16_T)((int16_T)((rtb_Switch2_fl << 14) /
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

    rtb_Switch2_fl = (int16_T)(rtb_Sum2_h << 12);
  }

  /* End of Switch: '<S13>/Switch2' */

  /* MinMax: '<S13>/MinMax1' incorporates:
   *  Constant: '<S13>/Constant1'
   */
  if (!(rtb_Switch2_fl > 0)) {
    rtb_Switch2_fl = 0;
  }

  /* End of MinMax: '<S13>/MinMax1' */

  /* Product: '<S13>/Divide2' */
  rtb_Switch2_fl = (int16_T)((15 * rtb_Switch2_fl) >> 4);

  /* DataTypeConversion: '<S1>/Data Type Conversion2' incorporates:
   *  Inport: '<Root>/r_inpTgt'
   */
  if (rtU->r_inpTgt > 2047) {
    rtb_DataTypeConversion2 = MAX_int16_T;
  } else if (rtU->r_inpTgt <= -2048) {
    rtb_DataTypeConversion2 = MIN_int16_T;
  } else {
    rtb_DataTypeConversion2 = (int16_T)(rtU->r_inpTgt << 4);
  }

  /* UnitDelay: '<S8>/UnitDelay2' */
  rtb_RelationalOperator4_d = rtDW->UnitDelay2_DSTATE_g;

  /* RelationalOperator: '<S12>/Relational Operator9' incorporates:
   *  Constant: '<S12>/n_stdStillDet'
   */
  rtb_RelationalOperator9 = (rtb_Abs5 < rtP->n_stdStillDet);

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
     *  Constant: '<S3>/CTRL_COMM4'
     *  Constant: '<S3>/r_errInpTgtThres'
     *  Inport: '<Root>/b_motEna'
     *  Logic: '<S3>/Logical Operator1'
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
        rtb_Merge_f_idx_1 = (int16_T)-rtDW->UnitDelay4_DSTATE_eu;
      } else {
        /* Abs: '<S3>/Abs4' incorporates:
         *  UnitDelay: '<S7>/UnitDelay4'
         */
        rtb_Merge_f_idx_1 = rtDW->UnitDelay4_DSTATE_eu;
      }

      rtb_RelationalOperator1_m = (rtU->b_motEna && rtb_RelationalOperator9 &&
        (rtb_Merge_f_idx_1 > rtP->r_errInpTgtThres));
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
    /* Disable for If: '<S29>/If2' */
    rtDW->If2_ActiveSubsystem = -1;
  }

  if (UnitDelay3 == 0) {
    /* Outputs for IfAction SubSystem: '<S1>/F03_Control_Mode_Manager' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Logic: '<S27>/Logical Operator4' incorporates:
     *  Constant: '<S27>/constant2'
     *  Constant: '<S27>/constant8'
     *  Inport: '<Root>/b_motEna'
     *  Inport: '<Root>/z_ctrlModReq'
     *  Logic: '<S27>/Logical Operator1'
     *  Logic: '<S27>/Logical Operator7'
     *  RelationalOperator: '<S27>/Relational Operator10'
     *  RelationalOperator: '<S27>/Relational Operator11'
     *  RelationalOperator: '<S27>/Relational Operator2'
     *  UnitDelay: '<S4>/UnitDelay1'
     */
    rtb_RelationalOperator1_m = ((!rtU->b_motEna) || rtDW->Merge_n ||
      (rtU->z_ctrlModReq == 0) || ((rtU->z_ctrlModReq != rtDW->z_ctrlMod) &&
      (rtDW->z_ctrlMod != 0)));

    /* Chart: '<S4>/F03_02_Control_Mode_Manager' incorporates:
     *  Constant: '<S27>/constant'
     *  Constant: '<S27>/constant1'
     *  Constant: '<S27>/constant5'
     *  Constant: '<S27>/constant6'
     *  Constant: '<S27>/constant7'
     *  Inport: '<Root>/z_ctrlModReq'
     *  Logic: '<S27>/Logical Operator3'
     *  Logic: '<S27>/Logical Operator6'
     *  Logic: '<S27>/Logical Operator9'
     *  RelationalOperator: '<S27>/Relational Operator1'
     *  RelationalOperator: '<S27>/Relational Operator3'
     *  RelationalOperator: '<S27>/Relational Operator4'
     *  RelationalOperator: '<S27>/Relational Operator5'
     *  RelationalOperator: '<S27>/Relational Operator6'
     */
    if (rtDW->is_active_c1_BLDC_controller == 0U) {
      rtDW->is_active_c1_BLDC_controller = 1U;
      rtDW->is_c1_BLDC_controller = IN_OPEN;
      rtDW->z_ctrlMod = OPEN_MODE;
    } else if (rtDW->is_c1_BLDC_controller == IN_ACTIVE) {
      if (rtb_RelationalOperator1_m) {
        rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
        rtDW->is_c1_BLDC_controller = IN_OPEN;
        rtDW->z_ctrlMod = OPEN_MODE;
      } else {
        switch (rtDW->is_ACTIVE) {
         case IN_SPEED_MODE:
          rtDW->z_ctrlMod = SPD_MODE;
          break;

         case IN_TORQUE_MODE:
          rtDW->z_ctrlMod = TRQ_MODE;
          break;

         default:
          rtDW->z_ctrlMod = VLT_MODE;
          break;
        }
      }
    } else {
      rtDW->z_ctrlMod = OPEN_MODE;
      if ((!rtb_RelationalOperator1_m) && ((rtU->z_ctrlModReq == 1) ||
           (rtU->z_ctrlModReq == 2) || (rtU->z_ctrlModReq == 3)) &&
          rtb_RelationalOperator9) {
        rtDW->is_c1_BLDC_controller = IN_ACTIVE;
        if (rtU->z_ctrlModReq == 3) {
          rtDW->is_ACTIVE = IN_TORQUE_MODE;
          rtDW->z_ctrlMod = TRQ_MODE;
        } else if (rtU->z_ctrlModReq == 2) {
          rtDW->is_ACTIVE = IN_SPEED_MODE;
          rtDW->z_ctrlMod = SPD_MODE;
        } else {
          rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
          rtDW->z_ctrlMod = VLT_MODE;
        }
      }
    }

    /* End of Chart: '<S4>/F03_02_Control_Mode_Manager' */

    /* If: '<S29>/If1' incorporates:
     *  Constant: '<S1>/z_ctrlTypSel1'
     *  DataTypeConversion: '<S1>/Data Type Conversion2'
     *  Inport: '<S30>/r_inpTgt'
     *  Saturate: '<S29>/Saturation'
     */
    if (rtP->z_ctrlTypSel == 2) {
      /* Outputs for IfAction SubSystem: '<S29>/FOC_Control_Type' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* SignalConversion: '<S32>/TmpSignal ConversionAtSelectorInport1' incorporates:
       *  Constant: '<S32>/Vd_max'
       *  Constant: '<S32>/constant1'
       *  Constant: '<S32>/i_max'
       *  Constant: '<S32>/n_max'
       */
      tmp[0] = 0;
      tmp[1] = rtP->Vd_max;
      tmp[2] = rtP->n_max;
      tmp[3] = rtP->i_max;

      /* End of Outputs for SubSystem: '<S29>/FOC_Control_Type' */

      /* Saturate: '<S29>/Saturation' incorporates:
       *  DataTypeConversion: '<S1>/Data Type Conversion2'
       */
      if (rtb_DataTypeConversion2 > 16000) {
        rtb_Merge = 16000;
      } else if (rtb_DataTypeConversion2 < -16000) {
        rtb_Merge = -16000;
      } else {
        rtb_Merge = rtb_DataTypeConversion2;
      }

      /* Outputs for IfAction SubSystem: '<S29>/FOC_Control_Type' incorporates:
       *  ActionPort: '<S32>/Action Port'
       */
      /* Product: '<S32>/Divide1' incorporates:
       *  Inport: '<Root>/z_ctrlModReq'
       *  Product: '<S32>/Divide4'
       *  Selector: '<S32>/Selector'
       */
      rtb_Merge = (int16_T)(((uint16_T)((tmp[rtU->z_ctrlModReq] << 5) / 125) *
        rtb_Merge) >> 12);

      /* End of Outputs for SubSystem: '<S29>/FOC_Control_Type' */
    } else if (rtb_DataTypeConversion2 > 16000) {
      /* Outputs for IfAction SubSystem: '<S29>/Default_Control_Type' incorporates:
       *  ActionPort: '<S30>/Action Port'
       */
      /* Saturate: '<S29>/Saturation' incorporates:
       *  Inport: '<S30>/r_inpTgt'
       */
      rtb_Merge = 16000;

      /* End of Outputs for SubSystem: '<S29>/Default_Control_Type' */
    } else if (rtb_DataTypeConversion2 < -16000) {
      /* Outputs for IfAction SubSystem: '<S29>/Default_Control_Type' incorporates:
       *  ActionPort: '<S30>/Action Port'
       */
      /* Saturate: '<S29>/Saturation' incorporates:
       *  Inport: '<S30>/r_inpTgt'
       */
      rtb_Merge = -16000;

      /* End of Outputs for SubSystem: '<S29>/Default_Control_Type' */
    } else {
      /* Outputs for IfAction SubSystem: '<S29>/Default_Control_Type' incorporates:
       *  ActionPort: '<S30>/Action Port'
       */
      rtb_Merge = rtb_DataTypeConversion2;

      /* End of Outputs for SubSystem: '<S29>/Default_Control_Type' */
    }

    /* End of If: '<S29>/If1' */

    /* If: '<S29>/If2' incorporates:
     *  Inport: '<S31>/r_inpTgtScaRaw'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem;
    UnitDelay3 = (int8_T)!(rtDW->z_ctrlMod == 0);
    rtDW->If2_ActiveSubsystem = UnitDelay3;
    switch (UnitDelay3) {
     case 0:
      if (UnitDelay3 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S29>/Open_Mode' incorporates:
         *  ActionPort: '<S33>/Action Port'
         */
        /* SystemReset for Atomic SubSystem: '<S33>/rising_edge_init' */
        /* SystemReset for If: '<S29>/If2' incorporates:
         *  UnitDelay: '<S35>/UnitDelay'
         *  UnitDelay: '<S36>/UnitDelay'
         */
        rtDW->UnitDelay_DSTATE_e = true;

        /* End of SystemReset for SubSystem: '<S33>/rising_edge_init' */

        /* SystemReset for Atomic SubSystem: '<S33>/Rate_Limiter' */
        rtDW->UnitDelay_DSTATE = 0;

        /* End of SystemReset for SubSystem: '<S33>/Rate_Limiter' */
        /* End of SystemReset for SubSystem: '<S29>/Open_Mode' */
      }

      /* Outputs for IfAction SubSystem: '<S29>/Open_Mode' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
      /* DataTypeConversion: '<S33>/Data Type Conversion' incorporates:
       *  UnitDelay: '<S7>/UnitDelay4'
       */
      rtb_Gain3 = rtDW->UnitDelay4_DSTATE_eu << 12;
      rtb_DataTypeConversion = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 |
        -134217728 : rtb_Gain3 & 134217727;

      /* Outputs for Atomic SubSystem: '<S33>/rising_edge_init' */
      /* UnitDelay: '<S35>/UnitDelay' */
      rtb_RelationalOperator9 = rtDW->UnitDelay_DSTATE_e;

      /* Update for UnitDelay: '<S35>/UnitDelay' incorporates:
       *  Constant: '<S35>/Constant'
       */
      rtDW->UnitDelay_DSTATE_e = false;

      /* End of Outputs for SubSystem: '<S33>/rising_edge_init' */

      /* Outputs for Atomic SubSystem: '<S33>/Rate_Limiter' */
      /* Switch: '<S36>/Switch1' incorporates:
       *  UnitDelay: '<S36>/UnitDelay'
       */
      if (rtb_RelationalOperator9) {
        rtb_Switch1 = rtb_DataTypeConversion;
      } else {
        rtb_Switch1 = rtDW->UnitDelay_DSTATE;
      }

      /* End of Switch: '<S36>/Switch1' */

      /* Sum: '<S34>/Sum1' */
      rtb_Gain3 = -rtb_Switch1;
      rtb_Sum1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
        rtb_Gain3 & 134217727;

      /* Switch: '<S37>/Switch2' incorporates:
       *  Constant: '<S33>/dV_openRate'
       *  RelationalOperator: '<S37>/LowerRelop1'
       */
      if (rtb_Sum1 > rtP->dV_openRate) {
        rtb_Sum1 = rtP->dV_openRate;
      } else {
        /* Gain: '<S33>/Gain3' */
        rtb_Gain3 = -rtP->dV_openRate;
        rtb_Gain3 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
          rtb_Gain3 & 134217727;

        /* Switch: '<S37>/Switch' incorporates:
         *  RelationalOperator: '<S37>/UpperRelop'
         */
        if (rtb_Sum1 < rtb_Gain3) {
          rtb_Sum1 = rtb_Gain3;
        }

        /* End of Switch: '<S37>/Switch' */
      }

      /* End of Switch: '<S37>/Switch2' */

      /* Sum: '<S34>/Sum2' */
      rtb_Gain3 = rtb_Sum1 + rtb_Switch1;
      rtb_Switch1 = (rtb_Gain3 & 134217728) != 0 ? rtb_Gain3 | -134217728 :
        rtb_Gain3 & 134217727;

      /* Switch: '<S36>/Switch2' */
      if (rtb_RelationalOperator9) {
        /* Update for UnitDelay: '<S36>/UnitDelay' */
        rtDW->UnitDelay_DSTATE = rtb_DataTypeConversion;
      } else {
        /* Update for UnitDelay: '<S36>/UnitDelay' */
        rtDW->UnitDelay_DSTATE = rtb_Switch1;
      }

      /* End of Switch: '<S36>/Switch2' */
      /* End of Outputs for SubSystem: '<S33>/Rate_Limiter' */

      /* DataTypeConversion: '<S33>/Data Type Conversion1' */
      rtDW->Merge1 = (int16_T)(rtb_Switch1 >> 12);

      /* End of Outputs for SubSystem: '<S29>/Open_Mode' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S29>/Default_Mode' incorporates:
       *  ActionPort: '<S31>/Action Port'
       */
      rtDW->Merge1 = rtb_Merge;

      /* End of Outputs for SubSystem: '<S29>/Default_Mode' */
      break;
    }

    /* End of If: '<S29>/If2' */
    /* End of Outputs for SubSystem: '<S1>/F03_Control_Mode_Manager' */
  }

  /* End of If: '<S1>/If4' */

  /* UnitDelay: '<S8>/UnitDelay5' */
  rtb_RelationalOperator9 = rtDW->UnitDelay5_DSTATE_l;

  /* Saturate: '<S1>/Saturation' incorporates:
   *  Inport: '<Root>/i_phaAB'
   */
  rtb_Gain3 = rtU->i_phaAB << 4;
  if (rtb_Gain3 >= 27200) {
    rtb_Merge = 27200;
  } else if (rtb_Gain3 <= -27200) {
    rtb_Merge = -27200;
  } else {
    rtb_Merge = (int16_T)(rtU->i_phaAB << 4);
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

  /* If: '<S1>/If3' incorporates:
   *  Constant: '<S1>/CTRL_COMM2'
   *  Constant: '<S1>/b_fieldWeakEna'
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Logic: '<S1>/Logical Operator1'
   *  RelationalOperator: '<S1>/Relational Operator1'
   *  UnitDelay: '<S8>/UnitDelay5'
   */
  if (rtP->b_fieldWeakEna && rtDW->UnitDelay5_DSTATE_l && (rtP->z_ctrlTypSel !=
       0)) {
    /* Outputs for IfAction SubSystem: '<S1>/F04_Field_Weakening' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Abs: '<S5>/Abs5' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion2'
     */
    if (rtb_DataTypeConversion2 < 0) {
      rtb_DataTypeConversion2 = (int16_T)-rtb_DataTypeConversion2;
    }

    /* End of Abs: '<S5>/Abs5' */

    /* Switch: '<S39>/Switch2' incorporates:
     *  Constant: '<S5>/r_fieldWeakHi'
     *  Constant: '<S5>/r_fieldWeakLo'
     *  RelationalOperator: '<S39>/LowerRelop1'
     *  RelationalOperator: '<S39>/UpperRelop'
     *  Switch: '<S39>/Switch'
     */
    if (rtb_DataTypeConversion2 > rtP->r_fieldWeakHi) {
      rtb_DataTypeConversion2 = rtP->r_fieldWeakHi;
    } else {
      if (rtb_DataTypeConversion2 < rtP->r_fieldWeakLo) {
        /* Switch: '<S39>/Switch' incorporates:
         *  Constant: '<S5>/r_fieldWeakLo'
         */
        rtb_DataTypeConversion2 = rtP->r_fieldWeakLo;
      }
    }

    /* End of Switch: '<S39>/Switch2' */

    /* Switch: '<S5>/Switch2' incorporates:
     *  Constant: '<S5>/CTRL_COMM2'
     *  Constant: '<S5>/a_phaAdvMax'
     *  Constant: '<S5>/id_fieldWeakMax'
     *  RelationalOperator: '<S5>/Relational Operator1'
     */
    if (rtP->z_ctrlTypSel == 2) {
      rtb_Merge_f_idx_1 = rtP->id_fieldWeakMax;
    } else {
      rtb_Merge_f_idx_1 = rtP->a_phaAdvMax;
    }

    /* End of Switch: '<S5>/Switch2' */

    /* Switch: '<S38>/Switch2' incorporates:
     *  Constant: '<S5>/n_fieldWeakAuthHi'
     *  Constant: '<S5>/n_fieldWeakAuthLo'
     *  RelationalOperator: '<S38>/LowerRelop1'
     *  RelationalOperator: '<S38>/UpperRelop'
     *  Switch: '<S38>/Switch'
     */
    if (rtb_Abs5 > rtP->n_fieldWeakAuthHi) {
      rtb_Switch2_l = rtP->n_fieldWeakAuthHi;
    } else if (rtb_Abs5 < rtP->n_fieldWeakAuthLo) {
      /* Switch: '<S38>/Switch' incorporates:
       *  Constant: '<S5>/n_fieldWeakAuthLo'
       */
      rtb_Switch2_l = rtP->n_fieldWeakAuthLo;
    } else {
      rtb_Switch2_l = rtb_Abs5;
    }

    /* End of Switch: '<S38>/Switch2' */

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
      rtb_Merge_f_idx_1) >> 15);

    /* End of Outputs for SubSystem: '<S1>/F04_Field_Weakening' */
  }

  /* End of If: '<S1>/If3' */

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
    /* Disable for If: '<S6>/If2' */
    if (rtDW->If2_ActiveSubsystem_a == 0) {
      /* Disable for Outport: '<S41>/iq' */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Outport: '<S41>/id' */
      rtDW->DataTypeConversion[1] = 0;
    }

    rtDW->If2_ActiveSubsystem_a = -1;

    /* End of Disable for If: '<S6>/If2' */

    /* Disable for Outport: '<S6>/V_phaABC_FOC' */
    rtDW->Gain4[0] = 0;
    rtDW->Gain4[1] = 0;
    rtDW->Gain4[2] = 0;

    /* Disable for Outport: '<S6>/r_devSignal1' */
    rtDW->DataTypeConversion[0] = 0;

    /* Disable for Outport: '<S6>/r_devSignal2' */
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

    /* If: '<S40>/If1' incorporates:
     *  Constant: '<S40>/b_selPhaABCurrMeas'
     */
    if (rtP->b_selPhaABCurrMeas) {
      /* Outputs for IfAction SubSystem: '<S40>/Clarke_PhasesAB' incorporates:
       *  ActionPort: '<S48>/Action Port'
       */
      /* Gain: '<S48>/Gain4' */
      rtb_Gain3 = 18919 * rtb_Merge;

      /* Gain: '<S48>/Gain2' */
      rtb_DataTypeConversion = 18919 * rtb_Saturation1;

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

      rtb_DataTypeConversion2 = (int16_T)rtb_Gain3;

      /* End of Sum: '<S48>/Sum1' */
      /* End of Outputs for SubSystem: '<S40>/Clarke_PhasesAB' */
    } else {
      /* Outputs for IfAction SubSystem: '<S40>/Clarke_PhasesBC' incorporates:
       *  ActionPort: '<S49>/Action Port'
       */
      /* Sum: '<S49>/Sum3' */
      rtb_Gain3 = rtb_Merge - rtb_Saturation1;
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
      rtb_DataTypeConversion2 = (int16_T)(((rtb_Gain3 < 0 ? 32767 : 0) +
        rtb_Gain3) >> 15);

      /* Sum: '<S49>/Sum1' */
      rtb_Gain3 = -rtb_Merge - rtb_Saturation1;
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      rtb_Merge = (int16_T)rtb_Gain3;

      /* End of Sum: '<S49>/Sum1' */
      /* End of Outputs for SubSystem: '<S40>/Clarke_PhasesBC' */
    }

    /* End of If: '<S40>/If1' */

    /* PreLookup: '<S47>/a_elecAngle_XA' */
    rtb_Sum_l = plook_u8s16_evencka(rtb_Switch2_fl, 0, 128U, 180U);

    /* If: '<S6>/If2' incorporates:
     *  Constant: '<S41>/cf_currFilt'
     *  Inport: '<Root>/b_motEna'
     */
    rtb_Sum2_h = rtDW->If2_ActiveSubsystem_a;
    UnitDelay3 = -1;
    if (rtU->b_motEna) {
      UnitDelay3 = 0;
    }

    rtDW->If2_ActiveSubsystem_a = UnitDelay3;
    if ((rtb_Sum2_h != UnitDelay3) && (rtb_Sum2_h == 0)) {
      /* Disable for Outport: '<S41>/iq' */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Outport: '<S41>/id' */
      rtDW->DataTypeConversion[1] = 0;
    }

    if (UnitDelay3 == 0) {
      if (0 != rtb_Sum2_h) {
        /* SystemReset for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
         *  ActionPort: '<S41>/Action Port'
         */

        /* SystemReset for Atomic SubSystem: '<S41>/Low_Pass_Filter' */

        /* SystemReset for If: '<S6>/If2' */
        Low_Pass_Filter_Reset(&rtDW->Low_Pass_Filter_m);

        /* End of SystemReset for SubSystem: '<S41>/Low_Pass_Filter' */

        /* End of SystemReset for SubSystem: '<S6>/Current_Filtering' */
      }

      /* Sum: '<S46>/Sum6' incorporates:
       *  Interpolation_n-D: '<S47>/r_cos_M1'
       *  Interpolation_n-D: '<S47>/r_sin_M1'
       *  Product: '<S46>/Divide1'
       *  Product: '<S46>/Divide4'
       */
      rtb_Gain3 = (int16_T)((rtb_DataTypeConversion2 *
        rtConstP.r_cos_M1_Table[rtb_Sum_l]) >> 14) - (int16_T)((rtb_Merge *
        rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
       *  ActionPort: '<S41>/Action Port'
       */
      /* SignalConversion: '<S41>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S46>/Sum6'
       */
      rtb_TmpSignalConversionAtLow_Pa[0] = (int16_T)rtb_Gain3;

      /* End of Outputs for SubSystem: '<S6>/Current_Filtering' */

      /* Sum: '<S46>/Sum1' incorporates:
       *  Interpolation_n-D: '<S47>/r_cos_M1'
       *  Interpolation_n-D: '<S47>/r_sin_M1'
       *  Product: '<S46>/Divide2'
       *  Product: '<S46>/Divide3'
       */
      rtb_Gain3 = (int16_T)((rtb_Merge * rtConstP.r_cos_M1_Table[rtb_Sum_l]) >>
                            14) + (int16_T)((rtb_DataTypeConversion2 *
        rtConstP.r_sin_M1_Table[rtb_Sum_l]) >> 14);
      if (rtb_Gain3 > 32767) {
        rtb_Gain3 = 32767;
      } else {
        if (rtb_Gain3 < -32768) {
          rtb_Gain3 = -32768;
        }
      }

      /* Outputs for IfAction SubSystem: '<S6>/Current_Filtering' incorporates:
       *  ActionPort: '<S41>/Action Port'
       */
      /* SignalConversion: '<S41>/TmpSignal ConversionAtLow_Pass_FilterInport1' incorporates:
       *  Sum: '<S46>/Sum1'
       */
      rtb_TmpSignalConversionAtLow_Pa[1] = (int16_T)rtb_Gain3;

      /* Outputs for Atomic SubSystem: '<S41>/Low_Pass_Filter' */
      Low_Pass_Filter(rtb_TmpSignalConversionAtLow_Pa, rtP->cf_currFilt,
                      rtDW->DataTypeConversion, &rtDW->Low_Pass_Filter_m);

      /* End of Outputs for SubSystem: '<S41>/Low_Pass_Filter' */

      /* End of Outputs for SubSystem: '<S6>/Current_Filtering' */
    }

    /* End of If: '<S6>/If2' */

    /* If: '<S6>/If3' incorporates:
     *  Constant: '<S45>/Vd_max1'
     *  Constant: '<S45>/i_max'
     *  UnitDelay: '<S8>/UnitDelay5'
     */
    if (rtDW->UnitDelay5_DSTATE_l) {
      /* Outputs for IfAction SubSystem: '<S6>/Motor_Limitations' incorporates:
       *  ActionPort: '<S45>/Action Port'
       */
      rtDW->Vd_max1 = rtP->Vd_max;

      /* Gain: '<S45>/Gain3' incorporates:
       *  Constant: '<S45>/Vd_max1'
       */
      rtDW->Gain3 = (int16_T)-rtDW->Vd_max1;

      /* Interpolation_n-D: '<S45>/Vq_max_M1' incorporates:
       *  Abs: '<S45>/Abs5'
       *  PreLookup: '<S45>/Vq_max_XA'
       *  UnitDelay: '<S6>/UnitDelay4'
       */
      if (rtDW->UnitDelay4_DSTATE_h < 0) {
        rtb_Merge_f_idx_1 = (int16_T)-rtDW->UnitDelay4_DSTATE_h;
      } else {
        rtb_Merge_f_idx_1 = rtDW->UnitDelay4_DSTATE_h;
      }

      rtDW->Vq_max_M1 = rtP->Vq_max_M1[plook_u8s16_evencka(rtb_Merge_f_idx_1,
        rtP->Vq_max_XA[0], (uint16_T)(rtP->Vq_max_XA[1] - rtP->Vq_max_XA[0]),
        45U)];

      /* End of Interpolation_n-D: '<S45>/Vq_max_M1' */

      /* Gain: '<S45>/Gain5' */
      rtDW->Gain5 = (int16_T)-rtDW->Vq_max_M1;
      rtDW->i_max = rtP->i_max;

      /* Interpolation_n-D: '<S45>/iq_maxSca_M1' incorporates:
       *  Constant: '<S45>/i_max'
       *  Product: '<S45>/Divide4'
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

      /* Product: '<S45>/Divide1' incorporates:
       *  Interpolation_n-D: '<S45>/iq_maxSca_M1'
       *  PreLookup: '<S45>/iq_maxSca_XA'
       *  Product: '<S45>/Divide4'
       */
      rtDW->Divide1_a = (int16_T)
        ((rtConstP.iq_maxSca_M1_Table[plook_u8u16_evencka((uint16_T)rtb_Gain3,
           0U, 1311U, 49U)] * rtDW->i_max) >> 16);

      /* Gain: '<S45>/Gain1' */
      rtDW->Gain1 = (int16_T)-rtDW->Divide1_a;

      /* SwitchCase: '<S45>/Switch Case' incorporates:
       *  Constant: '<S45>/n_max1'
       *  Constant: '<S72>/Constant1'
       *  Constant: '<S72>/cf_KbLimProt'
       *  Constant: '<S72>/cf_nKiLimProt'
       *  Constant: '<S73>/Constant'
       *  Constant: '<S73>/Constant1'
       *  Constant: '<S73>/cf_KbLimProt'
       *  Constant: '<S73>/cf_iqKiLimProt'
       *  Constant: '<S73>/cf_nKiLimProt'
       *  Sum: '<S72>/Sum1'
       *  Sum: '<S73>/Sum1'
       *  Sum: '<S73>/Sum2'
       */
      switch (rtDW->z_ctrlMod) {
       case 1:
        /* Abs: '<S6>/Abs5' */
        if (rtDW->DataTypeConversion[0] < 0) {
          rtb_Merge_f_idx_1 = (int16_T)-rtDW->DataTypeConversion[0];
        } else {
          rtb_Merge_f_idx_1 = rtDW->DataTypeConversion[0];
        }

        /* End of Abs: '<S6>/Abs5' */

        /* Outputs for IfAction SubSystem: '<S45>/Voltage_Mode_Protection' incorporates:
         *  ActionPort: '<S73>/Action Port'
         */

        /* Outputs for Atomic SubSystem: '<S73>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtDW->Divide1_a - rtb_Merge_f_idx_1),
                         rtP->cf_iqKiLimProt, rtP->cf_KbLimProt, rtb_Switch2_l,
                         0, &rtDW->Switch2_c, &rtDW->I_backCalc_fixdt_i);

        /* End of Outputs for SubSystem: '<S73>/I_backCalc_fixdt' */

        /* Outputs for Atomic SubSystem: '<S73>/I_backCalc_fixdt1' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - rtb_Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtb_Switch2_l, 0, &rtDW->Switch2_l,
                         &rtDW->I_backCalc_fixdt1);

        /* End of Outputs for SubSystem: '<S73>/I_backCalc_fixdt1' */

        /* End of Outputs for SubSystem: '<S45>/Voltage_Mode_Protection' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S45>/Speed_Mode_Protection' incorporates:
         *  ActionPort: '<S71>/Action Port'
         */
        /* Switch: '<S74>/Switch2' incorporates:
         *  RelationalOperator: '<S74>/LowerRelop1'
         *  RelationalOperator: '<S74>/UpperRelop'
         *  Switch: '<S74>/Switch'
         */
        if (rtDW->DataTypeConversion[0] > rtDW->Divide1_a) {
          rtb_Merge_f_idx_1 = rtDW->Divide1_a;
        } else if (rtDW->DataTypeConversion[0] < rtDW->Gain1) {
          /* Switch: '<S74>/Switch' */
          rtb_Merge_f_idx_1 = rtDW->Gain1;
        } else {
          rtb_Merge_f_idx_1 = rtDW->DataTypeConversion[0];
        }

        /* End of Switch: '<S74>/Switch2' */

        /* Product: '<S71>/Divide1' incorporates:
         *  Constant: '<S71>/cf_iqKiLimProt'
         *  Sum: '<S71>/Sum3'
         */
        rtDW->Divide1 = (int16_T)(rtb_Merge_f_idx_1 - rtDW->DataTypeConversion[0])
          * rtP->cf_iqKiLimProt;

        /* End of Outputs for SubSystem: '<S45>/Speed_Mode_Protection' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S45>/Torque_Mode_Protection' incorporates:
         *  ActionPort: '<S72>/Action Port'
         */

        /* Outputs for Atomic SubSystem: '<S72>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - rtb_Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtDW->Vq_max_M1, 0, &rtDW->Switch2,
                         &rtDW->I_backCalc_fixdt_g);

        /* End of Outputs for SubSystem: '<S72>/I_backCalc_fixdt' */

        /* End of Outputs for SubSystem: '<S45>/Torque_Mode_Protection' */
        break;
      }

      /* End of SwitchCase: '<S45>/Switch Case' */

      /* Gain: '<S45>/Gain4' */
      rtDW->Gain4_c = (int16_T)-rtDW->i_max;

      /* End of Outputs for SubSystem: '<S6>/Motor_Limitations' */
    }

    /* End of If: '<S6>/If3' */

    /* If: '<S6>/If1' incorporates:
     *  UnitDelay: '<S8>/UnitDelay6'
     */
    if (rtDW->UnitDelay6_DSTATE) {
      /* Outputs for IfAction SubSystem: '<S6>/FOC' incorporates:
       *  ActionPort: '<S42>/Action Port'
       */
      /* If: '<S42>/If1' incorporates:
       *  Constant: '<S54>/cf_idKi1'
       *  Constant: '<S54>/cf_idKp1'
       *  Constant: '<S54>/constant1'
       *  Sum: '<S54>/Sum3'
       */
      if (rtb_LogicalOperator) {
        /* Outputs for IfAction SubSystem: '<S42>/Vd_Calculation' incorporates:
         *  ActionPort: '<S54>/Action Port'
         */
        /* Switch: '<S66>/Switch2' incorporates:
         *  RelationalOperator: '<S66>/LowerRelop1'
         *  RelationalOperator: '<S66>/UpperRelop'
         *  Switch: '<S66>/Switch'
         */
        if (rtb_toNegative > rtDW->i_max) {
          rtb_toNegative = rtDW->i_max;
        } else {
          if (rtb_toNegative < rtDW->Gain4_c) {
            /* Switch: '<S66>/Switch' */
            rtb_toNegative = rtDW->Gain4_c;
          }
        }

        /* End of Switch: '<S66>/Switch2' */

        /* Sum: '<S54>/Sum3' */
        rtb_Gain3 = rtb_toNegative - rtDW->DataTypeConversion[1];
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* Outputs for Atomic SubSystem: '<S54>/PI_clamp_fixdt' */
        PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_idKp, rtP->cf_idKi,
                       rtDW->Vd_max1, rtDW->Gain3, 0, &rtDW->Switch1,
                       &rtDW->PI_clamp_fixdt_k);

        /* End of Outputs for SubSystem: '<S54>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S42>/Vd_Calculation' */
      }

      /* End of If: '<S42>/If1' */

      /* SwitchCase: '<S42>/Switch Case' incorporates:
       *  Constant: '<S52>/cf_nKi'
       *  Constant: '<S52>/cf_nKp'
       *  Constant: '<S53>/cf_iqKi'
       *  Constant: '<S53>/cf_iqKp'
       *  Constant: '<S53>/constant2'
       *  Inport: '<S51>/r_inpTgtSca'
       *  Sum: '<S52>/Sum3'
       *  Sum: '<S53>/Sum2'
       */
      switch (rtDW->z_ctrlMod) {
       case 1:
        /* Outputs for IfAction SubSystem: '<S42>/Voltage_Mode' incorporates:
         *  ActionPort: '<S55>/Action Port'
         */
        /* MinMax: '<S55>/MinMax' */
        if (!(rtb_Switch2_l < rtDW->Switch2_c)) {
          rtb_Switch2_l = rtDW->Switch2_c;
        }

        if (!(rtb_Switch2_l < rtDW->Switch2_l)) {
          rtb_Switch2_l = rtDW->Switch2_l;
        }

        /* End of MinMax: '<S55>/MinMax' */

        /* Signum: '<S55>/SignDeltaU2' */
        if (rtDW->Merge1 < 0) {
          rtb_Merge_f_idx_1 = -1;
        } else {
          rtb_Merge_f_idx_1 = (int16_T)(rtDW->Merge1 > 0);
        }

        /* End of Signum: '<S55>/SignDeltaU2' */

        /* Product: '<S55>/Divide1' */
        rtb_Merge = (int16_T)(rtb_Switch2_l * rtb_Merge_f_idx_1);

        /* Switch: '<S70>/Switch2' incorporates:
         *  RelationalOperator: '<S70>/LowerRelop1'
         *  RelationalOperator: '<S70>/UpperRelop'
         *  Switch: '<S70>/Switch'
         */
        if (rtb_Merge > rtDW->Vq_max_M1) {
          /* SignalConversion: '<S55>/Signal Conversion2' */
          rtDW->Merge = rtDW->Vq_max_M1;
        } else if (rtb_Merge < rtDW->Gain5) {
          /* Switch: '<S70>/Switch' incorporates:
           *  SignalConversion: '<S55>/Signal Conversion2'
           */
          rtDW->Merge = rtDW->Gain5;
        } else {
          /* SignalConversion: '<S55>/Signal Conversion2' incorporates:
           *  Switch: '<S70>/Switch'
           */
          rtDW->Merge = rtb_Merge;
        }

        /* End of Switch: '<S70>/Switch2' */
        /* End of Outputs for SubSystem: '<S42>/Voltage_Mode' */
        break;

       case 2:
        /* Outputs for IfAction SubSystem: '<S42>/Speed_Mode' incorporates:
         *  ActionPort: '<S52>/Action Port'
         */
        /* Sum: '<S52>/Sum3' */
        rtb_Gain3 = rtDW->Merge1 - rtb_Switch2_k;
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* Outputs for Atomic SubSystem: '<S52>/PI_clamp_fixdt' */
        PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_nKp, rtP->cf_nKi,
                       rtDW->Vq_max_M1, rtDW->Gain5, rtDW->Divide1, &rtDW->Merge,
                       &rtDW->PI_clamp_fixdt_o);

        /* End of Outputs for SubSystem: '<S52>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S42>/Speed_Mode' */
        break;

       case 3:
        /* Outputs for IfAction SubSystem: '<S42>/Torque_Mode' incorporates:
         *  ActionPort: '<S53>/Action Port'
         */
        /* Gain: '<S53>/Gain4' */
        rtb_Merge = (int16_T)-rtDW->Switch2;

        /* Switch: '<S61>/Switch2' incorporates:
         *  RelationalOperator: '<S61>/LowerRelop1'
         *  RelationalOperator: '<S61>/UpperRelop'
         *  Switch: '<S61>/Switch'
         */
        if (rtDW->Merge1 > rtDW->Divide1_a) {
          rtb_Merge_f_idx_1 = rtDW->Divide1_a;
        } else if (rtDW->Merge1 < rtDW->Gain1) {
          /* Switch: '<S61>/Switch' */
          rtb_Merge_f_idx_1 = rtDW->Gain1;
        } else {
          rtb_Merge_f_idx_1 = rtDW->Merge1;
        }

        /* End of Switch: '<S61>/Switch2' */

        /* Sum: '<S53>/Sum2' */
        rtb_Gain3 = rtb_Merge_f_idx_1 - rtDW->DataTypeConversion[0];
        if (rtb_Gain3 > 32767) {
          rtb_Gain3 = 32767;
        } else {
          if (rtb_Gain3 < -32768) {
            rtb_Gain3 = -32768;
          }
        }

        /* MinMax: '<S53>/MinMax1' */
        if (rtDW->Vq_max_M1 < rtDW->Switch2) {
          rtb_Merge_f_idx_1 = rtDW->Vq_max_M1;
        } else {
          rtb_Merge_f_idx_1 = rtDW->Switch2;
        }

        /* End of MinMax: '<S53>/MinMax1' */

        /* MinMax: '<S53>/MinMax2' */
        if (!(rtb_Merge > rtDW->Gain5)) {
          rtb_Merge = rtDW->Gain5;
        }

        /* End of MinMax: '<S53>/MinMax2' */

        /* Outputs for Atomic SubSystem: '<S53>/PI_clamp_fixdt' */
        PI_clamp_fixdt((int16_T)rtb_Gain3, rtP->cf_iqKp, rtP->cf_iqKi,
                       rtb_Merge_f_idx_1, rtb_Merge, 0, &rtDW->Merge,
                       &rtDW->PI_clamp_fixdt_a);

        /* End of Outputs for SubSystem: '<S53>/PI_clamp_fixdt' */

        /* End of Outputs for SubSystem: '<S42>/Torque_Mode' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S42>/Open_Mode' incorporates:
         *  ActionPort: '<S51>/Action Port'
         */
        rtDW->Merge = rtDW->Merge1;

        /* End of Outputs for SubSystem: '<S42>/Open_Mode' */
        break;
      }

      /* End of SwitchCase: '<S42>/Switch Case' */
      /* End of Outputs for SubSystem: '<S6>/FOC' */
    }

    /* End of If: '<S6>/If1' */

    /* Sum: '<S44>/Sum6' incorporates:
     *  Interpolation_n-D: '<S47>/r_cos_M1'
     *  Interpolation_n-D: '<S47>/r_sin_M1'
     *  Product: '<S44>/Divide1'
     *  Product: '<S44>/Divide4'
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

    /* Sum: '<S44>/Sum1' incorporates:
     *  Interpolation_n-D: '<S47>/r_cos_M1'
     *  Interpolation_n-D: '<S47>/r_sin_M1'
     *  Product: '<S44>/Divide2'
     *  Product: '<S44>/Divide3'
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

    /* Gain: '<S43>/Gain1' incorporates:
     *  Sum: '<S44>/Sum1'
     */
    rtb_DataTypeConversion *= 14189;

    /* Sum: '<S43>/Sum6' incorporates:
     *  Gain: '<S43>/Gain1'
     *  Gain: '<S43>/Gain3'
     *  Sum: '<S44>/Sum6'
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

    /* Sum: '<S43>/Sum2' incorporates:
     *  Sum: '<S43>/Sum6'
     *  Sum: '<S44>/Sum6'
     */
    rtb_Switch1 = -(int16_T)rtb_Gain3 - (int16_T)rtb_DataTypeConversion;
    if (rtb_Switch1 > 32767) {
      rtb_Switch1 = 32767;
    } else {
      if (rtb_Switch1 < -32768) {
        rtb_Switch1 = -32768;
      }
    }

    /* MinMax: '<S43>/MinMax1' incorporates:
     *  Sum: '<S43>/Sum2'
     *  Sum: '<S43>/Sum6'
     *  Sum: '<S44>/Sum6'
     */
    rtb_Switch2_l = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 < (int16_T)rtb_DataTypeConversion)) {
      rtb_Switch2_l = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Switch2_l < (int16_T)rtb_Switch1)) {
      rtb_Switch2_l = (int16_T)rtb_Switch1;
    }

    /* MinMax: '<S43>/MinMax2' incorporates:
     *  Sum: '<S43>/Sum2'
     *  Sum: '<S43>/Sum6'
     *  Sum: '<S44>/Sum6'
     */
    rtb_Merge = (int16_T)rtb_Gain3;
    if (!((int16_T)rtb_Gain3 > (int16_T)rtb_DataTypeConversion)) {
      rtb_Merge = (int16_T)rtb_DataTypeConversion;
    }

    if (!(rtb_Merge > (int16_T)rtb_Switch1)) {
      rtb_Merge = (int16_T)rtb_Switch1;
    }

    /* Sum: '<S43>/Add' incorporates:
     *  MinMax: '<S43>/MinMax1'
     *  MinMax: '<S43>/MinMax2'
     */
    rtb_Sum1 = rtb_Switch2_l + rtb_Merge;
    if (rtb_Sum1 > 32767) {
      rtb_Sum1 = 32767;
    } else {
      if (rtb_Sum1 < -32768) {
        rtb_Sum1 = -32768;
      }
    }

    /* Gain: '<S43>/Gain2' incorporates:
     *  Sum: '<S43>/Add'
     */
    rtb_DataTypeConversion2 = (int16_T)(rtb_Sum1 >> 1);

    /* Sum: '<S43>/Add1' incorporates:
     *  Sum: '<S44>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_Gain3 - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S43>/Gain4' incorporates:
     *  Sum: '<S43>/Add1'
     */
    rtDW->Gain4[0] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S43>/Add1' incorporates:
     *  Sum: '<S43>/Sum6'
     */
    rtb_Gain3 = (int16_T)rtb_DataTypeConversion - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S43>/Gain4' incorporates:
     *  Sum: '<S43>/Add1'
     */
    rtDW->Gain4[1] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Sum: '<S43>/Add1' incorporates:
     *  Sum: '<S43>/Sum2'
     */
    rtb_Gain3 = (int16_T)rtb_Switch1 - rtb_DataTypeConversion2;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else {
      if (rtb_Gain3 < -32768) {
        rtb_Gain3 = -32768;
      }
    }

    /* Gain: '<S43>/Gain4' incorporates:
     *  Sum: '<S43>/Add1'
     */
    rtDW->Gain4[2] = (int16_T)((18919 * rtb_Gain3) >> 14);

    /* Update for UnitDelay: '<S6>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE_h = rtDW->Switch1;

    /* End of Outputs for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  }

  /* End of If: '<S1>/If1' */

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Constant: '<S7>/CTRL_COMM1'
   *  RelationalOperator: '<S7>/Relational Operator6'
   */
  if (rtP->z_ctrlTypSel == 2) {
    rtb_Merge = rtDW->Merge;
  } else {
    rtb_Merge = rtDW->Merge1;
  }

  /* End of Switch: '<S7>/Switch2' */

  /* If: '<S7>/If' incorporates:
   *  Constant: '<S10>/vec_hallToPos'
   *  Constant: '<S1>/z_ctrlTypSel1'
   *  Constant: '<S7>/CTRL_COMM2'
   *  Constant: '<S7>/CTRL_COMM3'
   *  Inport: '<S85>/V_phaABC_FOC_in'
   *  Logic: '<S7>/Logical Operator1'
   *  Logic: '<S7>/Logical Operator2'
   *  LookupNDDirect: '<S84>/z_commutMap_M1'
   *  RelationalOperator: '<S7>/Relational Operator1'
   *  RelationalOperator: '<S7>/Relational Operator2'
   *  Selector: '<S10>/Selector'
   *
   * About '<S84>/z_commutMap_M1':
   *  2-dimensional Direct Look-Up returning a Column
   */
  if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 2)) {
    /* Outputs for IfAction SubSystem: '<S7>/FOC_Method' incorporates:
     *  ActionPort: '<S85>/Action Port'
     */
    rtb_DataTypeConversion2 = rtDW->Gain4[0];
    rtb_Merge_f_idx_1 = rtDW->Gain4[1];
    rtb_Saturation1 = rtDW->Gain4[2];

    /* End of Outputs for SubSystem: '<S7>/FOC_Method' */
  } else if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 1)) {
    /* Outputs for IfAction SubSystem: '<S7>/SIN_Method' incorporates:
     *  ActionPort: '<S86>/Action Port'
     */
    /* Switch: '<S87>/Switch_PhaAdv' incorporates:
     *  Constant: '<S87>/b_fieldWeakEna'
     *  Product: '<S88>/Divide2'
     *  Product: '<S88>/Divide3'
     *  Sum: '<S88>/Sum3'
     */
    if (rtP->b_fieldWeakEna) {
      /* Sum: '<S87>/Sum3' incorporates:
       *  Product: '<S87>/Product2'
       */
      rtb_Saturation1 = (int16_T)((int16_T)((int16_T)(rtDW->Divide3 *
        rtDW->Switch2_e) << 2) + rtb_Switch2_fl);
      rtb_Saturation1 -= (int16_T)(23040 * (int16_T)div_nde_s32_floor
        (rtb_Saturation1, 23040));
    } else {
      rtb_Saturation1 = rtb_Switch2_fl;
    }

    /* End of Switch: '<S87>/Switch_PhaAdv' */

    /* PreLookup: '<S86>/a_elecAngle_XA' */
    rtb_Sum = plook_u8s16_evencka(rtb_Saturation1, 0, 128U, 180U);

    /* Product: '<S86>/Divide2' incorporates:
     *  Interpolation_n-D: '<S86>/r_sin3PhaA_M1'
     *  Interpolation_n-D: '<S86>/r_sin3PhaB_M1'
     *  Interpolation_n-D: '<S86>/r_sin3PhaC_M1'
     */
    rtb_DataTypeConversion2 = (int16_T)((rtDW->Merge1 *
      rtConstP.r_sin3PhaA_M1_Table[rtb_Sum]) >> 14);
    rtb_Merge_f_idx_1 = (int16_T)((rtDW->Merge1 *
      rtConstP.r_sin3PhaB_M1_Table[rtb_Sum]) >> 14);
    rtb_Saturation1 = (int16_T)((rtDW->Merge1 *
      rtConstP.r_sin3PhaC_M1_Table[rtb_Sum]) >> 14);

    /* End of Outputs for SubSystem: '<S7>/SIN_Method' */
  } else {
    /* Outputs for IfAction SubSystem: '<S7>/COM_Method' incorporates:
     *  ActionPort: '<S84>/Action Port'
     */
    if (rtConstP.vec_hallToPos_Value[rtb_Sum] > 5) {
      /* LookupNDDirect: '<S84>/z_commutMap_M1'
       *
       * About '<S84>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 5;
    } else if (rtConstP.vec_hallToPos_Value[rtb_Sum] < 0) {
      /* LookupNDDirect: '<S84>/z_commutMap_M1'
       *
       * About '<S84>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = 0;
    } else {
      /* LookupNDDirect: '<S84>/z_commutMap_M1' incorporates:
       *  Constant: '<S10>/vec_hallToPos'
       *  Selector: '<S10>/Selector'
       *
       * About '<S84>/z_commutMap_M1':
       *  2-dimensional Direct Look-Up returning a Column
       */
      rtb_Sum2_h = rtConstP.vec_hallToPos_Value[rtb_Sum];
    }

    /* LookupNDDirect: '<S84>/z_commutMap_M1' incorporates:
     *  Constant: '<S10>/vec_hallToPos'
     *  Selector: '<S10>/Selector'
     *
     * About '<S84>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_DataTypeConversion = rtb_Sum2_h * 3;

    /* Product: '<S84>/Divide2' incorporates:
     *  LookupNDDirect: '<S84>/z_commutMap_M1'
     *
     * About '<S84>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column
     */
    rtb_DataTypeConversion2 = (int16_T)(rtb_Merge *
      rtConstP.z_commutMap_M1_table[rtb_DataTypeConversion]);
    rtb_Merge_f_idx_1 = (int16_T)(rtConstP.z_commutMap_M1_table[1 +
      rtb_DataTypeConversion] * rtb_Merge);
    rtb_Saturation1 = (int16_T)(rtConstP.z_commutMap_M1_table[2 +
      rtb_DataTypeConversion] * rtb_Merge);

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
  rtY->DC_phaB = (int16_T)(rtb_Merge_f_idx_1 >> 4);

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
  rtDW->UnitDelay4_DSTATE_eu = rtb_Merge;

  /* Update for UnitDelay: '<S8>/UnitDelay5' */
  rtDW->UnitDelay5_DSTATE_l = rtb_RelationalOperator4_d;

  /* Update for UnitDelay: '<S8>/UnitDelay6' */
  rtDW->UnitDelay6_DSTATE = rtb_RelationalOperator9;

  /* Outport: '<Root>/DC_phaC' incorporates:
   *  DataTypeConversion: '<S7>/Data Type Conversion6'
   */
  rtY->DC_phaC = (int16_T)(rtb_Saturation1 >> 4);

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
  rtY->r_devSignal1 = (int16_T)(rtDW->DataTypeConversion[0] >> 4);

  /* Outport: '<Root>/r_devSignal2' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion5'
   */
  rtY->r_devSignal2 = (int16_T)(rtDW->DataTypeConversion[1] >> 4);

  /* End of Outputs for SubSystem: '<Root>/BLDC_controller' */
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
  /* Start for If: '<S29>/If2' */
  rtDW->If2_ActiveSubsystem = -1;

  /* End of Start for SubSystem: '<S1>/F03_Control_Mode_Manager' */

  /* Start for If: '<S1>/If1' */
  rtDW->If1_ActiveSubsystem = -1;

  /* Start for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* Start for If: '<S6>/If2' */
  rtDW->If2_ActiveSubsystem_a = -1;

  /* End of Start for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* End of Start for SubSystem: '<Root>/BLDC_controller' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* InitializeConditions for UnitDelay: '<S12>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtP->z_maxCntRst;

  /* InitializeConditions for UnitDelay: '<S8>/UnitDelay2' */
  rtDW->UnitDelay2_DSTATE_g = true;

  /* SystemInitialize for IfAction SubSystem: '<S12>/Raw_Motor_Speed_Estimation' */
  /* SystemInitialize for Outport: '<S15>/z_counter' */
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
  /* SystemInitialize for IfAction SubSystem: '<S29>/Open_Mode' */
  /* SystemInitialize for Atomic SubSystem: '<S33>/rising_edge_init' */
  /* InitializeConditions for UnitDelay: '<S35>/UnitDelay' */
  rtDW->UnitDelay_DSTATE_e = true;

  /* End of SystemInitialize for SubSystem: '<S33>/rising_edge_init' */
  /* End of SystemInitialize for SubSystem: '<S29>/Open_Mode' */
  /* End of SystemInitialize for SubSystem: '<S1>/F03_Control_Mode_Manager' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* SystemInitialize for IfAction SubSystem: '<S6>/Motor_Limitations' */

  /* SystemInitialize for IfAction SubSystem: '<S45>/Voltage_Mode_Protection' */

  /* SystemInitialize for Atomic SubSystem: '<S73>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt_i, 0);

  /* End of SystemInitialize for SubSystem: '<S73>/I_backCalc_fixdt' */

  /* SystemInitialize for Atomic SubSystem: '<S73>/I_backCalc_fixdt1' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt1, 0);

  /* End of SystemInitialize for SubSystem: '<S73>/I_backCalc_fixdt1' */

  /* End of SystemInitialize for SubSystem: '<S45>/Voltage_Mode_Protection' */

  /* SystemInitialize for IfAction SubSystem: '<S45>/Torque_Mode_Protection' */

  /* SystemInitialize for Atomic SubSystem: '<S72>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(&rtDW->I_backCalc_fixdt_g, 0);

  /* End of SystemInitialize for SubSystem: '<S72>/I_backCalc_fixdt' */

  /* End of SystemInitialize for SubSystem: '<S45>/Torque_Mode_Protection' */

  /* SystemInitialize for Outport: '<S45>/Vd_max' */
  rtDW->Vd_max1 = 14400;

  /* SystemInitialize for Outport: '<S45>/Vd_min' */
  rtDW->Gain3 = -14400;

  /* SystemInitialize for Outport: '<S45>/Vq_max' */
  rtDW->Vq_max_M1 = 14400;

  /* SystemInitialize for Outport: '<S45>/Vq_min' */
  rtDW->Gain5 = -14400;

  /* SystemInitialize for Outport: '<S45>/id_max' */
  rtDW->i_max = 12000;

  /* SystemInitialize for Outport: '<S45>/id_min' */
  rtDW->Gain4_c = -12000;

  /* SystemInitialize for Outport: '<S45>/iq_max' */
  rtDW->Divide1_a = 12000;

  /* SystemInitialize for Outport: '<S45>/iq_min' */
  rtDW->Gain1 = -12000;

  /* End of SystemInitialize for SubSystem: '<S6>/Motor_Limitations' */
  /* End of SystemInitialize for SubSystem: '<S1>/F05_Field_Oriented_Control' */
  /* End of SystemInitialize for SubSystem: '<Root>/BLDC_controller' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
