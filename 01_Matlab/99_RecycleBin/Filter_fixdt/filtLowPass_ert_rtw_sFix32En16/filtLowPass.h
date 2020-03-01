/*
 * File: filtLowPass.h
 *
 * Code generated for Simulink model 'filtLowPass'.
 *
 * Model version                  : 1.1257
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Mon Feb 24 20:31:06 2020
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

#ifndef RTW_HEADER_filtLowPass_h_
#define RTW_HEADER_filtLowPass_h_
#ifndef filtLowPass_COMMON_INCLUDES_
# define filtLowPass_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* filtLowPass_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (auto storage) for system '<Root>/filtLowPass' */
typedef struct {
  int32_T UnitDelay1_DSTATE;           /* '<S2>/UnitDelay1' */
} DW_filtLowPass;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_filtLowPass filtLowPass_l2;       /* '<Root>/filtLowPass' */
} DW;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  int32_T u;                           /* '<Root>/u' */
  uint16_T coef;                       /* '<Root>/coef' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int32_T y;                           /* '<Root>/y' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  ExtU *inputs;
  ExtY *outputs;
  DW *dwork;
};

/* Model entry point functions */
extern void filtLowPass_initialize(RT_MODEL *const rtM);
extern void filtLowPass_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Scope1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/filtLowPass')    - opens subsystem BLDCmotorControl_FOC_R2017b_fixdt/filtLowPass
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/filtLowPass/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotorControl_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/filtLowPass'
 * '<S2>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/filtLowPass/Low_Pass_Filter'
 */
#endif                                 /* RTW_HEADER_filtLowPass_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
