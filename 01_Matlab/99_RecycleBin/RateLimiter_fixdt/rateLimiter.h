/*
 * File: rateLimiter.h
 *
 * Code generated for Simulink model 'rateLimiter'.
 *
 * Model version                  : 1.1186
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sun Oct 27 16:29:07 2019
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

#ifndef RTW_HEADER_rateLimiter_h_
#define RTW_HEADER_rateLimiter_h_
#ifndef rateLimiter_COMMON_INCLUDES_
# define rateLimiter_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* rateLimiter_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (auto storage) for system '<Root>/rateLimiter' */
typedef struct {
  int16_T UnitDelay_DSTATE;            /* '<S2>/UnitDelay' */
} DW_rateLimiter;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_rateLimiter rateLimiter_j0;       /* '<Root>/rateLimiter' */
} DW;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  int16_T u;                           /* '<Root>/u' */
  int16_T rate;                        /* '<Root>/rate' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_T y;                           /* '<Root>/y' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  ExtU *inputs;
  ExtY *outputs;
  DW *dwork;
};

/* Model entry point functions */
extern void rateLimiter_initialize(RT_MODEL *const rtM);
extern void rateLimiter_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Propagation' : Unused code path elimination
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
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter')    - opens subsystem BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotorControl_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter'
 * '<S2>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter/Rate_Limiter'
 * '<S3>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/rateLimiter/Rate_Limiter/Saturation Dynamic'
 */
#endif                                 /* RTW_HEADER_rateLimiter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
