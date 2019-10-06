/*
 * File: filtLowPass.c
 *
 * Code generated for Simulink model 'filtLowPass'.
 *
 * Model version                  : 1.1167
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sun Oct  6 22:11:53 2019
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

#include "filtLowPass.h"
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

extern int16_T filtLowPass_l(int16_T rtu_u, uint16_T rtu_coef, DW_filtLowPass
  *localDW);

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

/* Output and update for atomic system: '<Root>/filtLowPass' */
int16_T filtLowPass_l(int16_T rtu_u, uint16_T rtu_coef, DW_filtLowPass *localDW)
{
  int32_T tmp;
  int16_T rty_y_0;

  /* Outputs for Atomic SubSystem: '<S1>/Low_Pass_Filter1' */
  /* Sum: '<S2>/Sum1' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion'
   *  Product: '<S2>/Divide1'
   *  Product: '<S2>/Divide2'
   *  Sum: '<S2>/Sum5'
   *  UnitDelay: '<S2>/UnitDelay3'
   */
  tmp = (((int16_T)(rtu_u << 4) * rtu_coef) >> 16) + (((int32_T)(65535U -
    rtu_coef) * localDW->UnitDelay3_DSTATE) >> 16);
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  rty_y_0 = (int16_T)tmp;

  /* Update for UnitDelay: '<S2>/UnitDelay3' incorporates:
   *  Sum: '<S2>/Sum1'
   */
  localDW->UnitDelay3_DSTATE = (int16_T)tmp;

  /* End of Outputs for SubSystem: '<S1>/Low_Pass_Filter1' */
  return rty_y_0;
}

/* Model step function */
void filtLowPass_step(RT_MODEL *const rtM)
{
  DW *rtDW = ((DW *) rtM->dwork);
  ExtU *rtU = (ExtU *) rtM->inputs;
  ExtY *rtY = (ExtY *) rtM->outputs;

  /* Outputs for Atomic SubSystem: '<Root>/filtLowPass' */

  /* Outport: '<Root>/y' incorporates:
   *  Inport: '<Root>/coef'
   *  Inport: '<Root>/u'
   */
  rtY->y = (int16_T) filtLowPass_l(rtU->u, rtU->coef, &rtDW->filtLowPass_l2);

  /* End of Outputs for SubSystem: '<Root>/filtLowPass' */
}

/* Model initialize function */
void filtLowPass_initialize(RT_MODEL *const rtM)
{
  /* (no initialization code required) */
  UNUSED_PARAMETER(rtM);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
