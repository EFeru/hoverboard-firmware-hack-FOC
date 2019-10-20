/*
 * File: mixer.c
 *
 * Code generated for Simulink model 'mixer'.
 *
 * Model version                  : 1.1173
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Wed Oct 16 19:40:02 2019
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

#include "mixer.h"
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

extern void mixer_k(int16_T rtu_speed, int16_T rtu_steer, int16_T rtu_speed_coef,
                    int16_T rtu_steer_coef, int16_T *rty_speedR, int16_T
                    *rty_speedL);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
# if defined(__LCC__)
#   define UNUSED_PARAMETER(x)                                   /* do nothing */
# else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#   define UNUSED_PARAMETER(x)         (void) (x)
# endif
#endif

/* Output and update for atomic system: '<Root>/mixer' */
void mixer_k(int16_T rtu_speed, int16_T rtu_steer, int16_T rtu_speed_coef,
             int16_T rtu_steer_coef, int16_T *rty_speedR, int16_T *rty_speedL)
{
  int16_T rtb_Divide1;
  int16_T rtb_Divide2;
  int32_T tmp;

  /* Product: '<S1>/Divide1' */
  rtb_Divide1 = (int16_T)((rtu_speed * rtu_speed_coef) >> 14);

  /* Product: '<S1>/Divide2' */
  rtb_Divide2 = (int16_T)((rtu_steer * rtu_steer_coef) >> 14);

  /* Sum: '<S1>/Sum1' */
  tmp = rtb_Divide1 - rtb_Divide2;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* DataTypeConversion: '<S1>/Data Type Conversion2' incorporates:
   *  Sum: '<S1>/Sum1'
   */
  *rty_speedR = (int16_T)(tmp >> 4);

  /* Sum: '<S1>/Sum2' */
  tmp = rtb_Divide1 + rtb_Divide2;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  /* DataTypeConversion: '<S1>/Data Type Conversion3' incorporates:
   *  Sum: '<S1>/Sum2'
   */
  *rty_speedL = (int16_T)(tmp >> 4);
}

/* Model step function */
void mixer_step(RT_MODEL *const rtM)
{
  ExtU *rtU = (ExtU *) rtM->inputs;
  ExtY *rtY = (ExtY *) rtM->outputs;

  /* Outputs for Atomic SubSystem: '<Root>/mixer' */

  /* Inport: '<Root>/speed' incorporates:
   *  Inport: '<Root>/speed_coef'
   *  Inport: '<Root>/steer'
   *  Inport: '<Root>/steer_coef'
   *  Outport: '<Root>/speedL'
   *  Outport: '<Root>/speedR'
   */
  mixer_k(rtU->speed, rtU->steer, rtU->speed_coef, rtU->steer_coef, &rtY->speedR,
          &rtY->speedL);

  /* End of Outputs for SubSystem: '<Root>/mixer' */
}

/* Model initialize function */
void mixer_initialize(RT_MODEL *const rtM)
{
  /* (no initialization code required) */
  UNUSED_PARAMETER(rtM);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
