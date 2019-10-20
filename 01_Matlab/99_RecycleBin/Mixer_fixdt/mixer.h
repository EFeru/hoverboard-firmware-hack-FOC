/*
 * File: mixer.h
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

#ifndef RTW_HEADER_mixer_h_
#define RTW_HEADER_mixer_h_
#ifndef mixer_COMMON_INCLUDES_
# define mixer_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* mixer_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  int16_T speed;                       /* '<Root>/speed' */
  int16_T steer;                       /* '<Root>/steer' */
  int16_T speed_coef;                  /* '<Root>/speed_coef' */
  int16_T steer_coef;                  /* '<Root>/steer_coef' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_T speedR;                      /* '<Root>/speedR' */
  int16_T speedL;                      /* '<Root>/speedL' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  ExtU *inputs;
  ExtY *outputs;
};

/* Model entry point functions */
extern void mixer_initialize(RT_MODEL *const rtM);
extern void mixer_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Conversion4' : Unused code path elimination
 * Block '<S1>/Display' : Unused code path elimination
 * Block '<S1>/Display1' : Unused code path elimination
 * Block '<S1>/Display3' : Unused code path elimination
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
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/mixer')    - opens subsystem BLDCmotorControl_FOC_R2017b_fixdt/mixer
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/mixer/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotorControl_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/mixer'
 */
#endif                                 /* RTW_HEADER_mixer_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
