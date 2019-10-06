/*
 * File: filtLowPass.h
 *
 * Code generated for Simulink model 'filtLowPass'.
 *
 * Model version                  : 1.1160
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Fri Sep 27 08:03:25 2019
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
  int16_T UnitDelay3_DSTATE;           /* '<S2>/UnitDelay3' */
} DW_filtLowPass;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_filtLowPass filtLowPass_l2;       /* '<Root>/filtLowPass' */
} DW;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  int16_T u;                           /* '<Root>/u' */
  uint16_T coef;                       /* '<Root>/coef' */
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
extern void filtLowPass_initialize(RT_MODEL *const rtM);
extern void filtLowPass_step(RT_MODEL *const rtM);

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
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/filtLowPass')    - opens subsystem BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/filtLowPass
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/filtLowPass/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control'
 * '<S1>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/filtLowPass'
 * '<S2>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/filtLowPass/Low_Pass_Filter1'
 */
#endif                                 /* RTW_HEADER_filtLowPass_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
