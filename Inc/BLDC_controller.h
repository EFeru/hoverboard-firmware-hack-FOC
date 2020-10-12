/*
 * File: BLDC_controller.h
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

#ifndef RTW_HEADER_BLDC_controller_h_
#define RTW_HEADER_BLDC_controller_h_
#include "rtwtypes.h"
#ifndef BLDC_controller_COMMON_INCLUDES_
# define BLDC_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* BLDC_controller_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (auto storage) for system '<S12>/Counter' */
typedef struct {
  int16_T UnitDelay_DSTATE;            /* '<S17>/UnitDelay' */
} DW_Counter;

/* Block signals and states (auto storage) for system '<S57>/PI_clamp_fixdt' */
typedef struct {
  int32_T ResettableDelay_DSTATE;      /* '<S71>/Resettable Delay' */
  uint8_T icLoad;                      /* '<S71>/Resettable Delay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S68>/UnitDelay1' */
} DW_PI_clamp_fixdt;

/* Block signals and states (auto storage) for system '<S55>/PI_clamp_fixdt' */
typedef struct {
  int32_T ResettableDelay_DSTATE;      /* '<S61>/Resettable Delay' */
  uint8_T icLoad;                      /* '<S61>/Resettable Delay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S59>/UnitDelay1' */
} DW_PI_clamp_fixdt_i;

/* Block signals and states (auto storage) for system '<S56>/PI_clamp_fixdt' */
typedef struct {
  int16_T ResettableDelay_DSTATE;      /* '<S66>/Resettable Delay' */
  uint8_T icLoad;                      /* '<S66>/Resettable Delay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S63>/UnitDelay1' */
} DW_PI_clamp_fixdt_e;

/* Block signals and states (auto storage) for system '<S43>/Low_Pass_Filter' */
typedef struct {
  int32_T UnitDelay1_DSTATE[2];        /* '<S53>/UnitDelay1' */
} DW_Low_Pass_Filter;

/* Block signals and states (auto storage) for system '<S76>/I_backCalc_fixdt' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S81>/UnitDelay' */
  int32_T UnitDelay_DSTATE_h;          /* '<S83>/UnitDelay' */
} DW_I_backCalc_fixdt;

/* Block signals and states (auto storage) for system '<S23>/Counter' */
typedef struct {
  uint16_T UnitDelay_DSTATE;           /* '<S28>/UnitDelay' */
} DW_Counter_l;

/* Block signals and states (auto storage) for system '<S19>/either_edge' */
typedef struct {
  boolean_T UnitDelay_DSTATE;          /* '<S24>/UnitDelay' */
} DW_either_edge;

/* Block signals and states (auto storage) for system '<S3>/Debounce_Filter' */
typedef struct {
  DW_either_edge either_edge_k;        /* '<S19>/either_edge' */
  DW_Counter_l Counter_h;              /* '<S22>/Counter' */
  DW_Counter_l Counter_i0;             /* '<S23>/Counter' */
  boolean_T UnitDelay_DSTATE;          /* '<S19>/UnitDelay' */
} DW_Debounce_Filter;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_either_edge either_edge_a;        /* '<S3>/either_edge' */
  DW_Debounce_Filter Debounce_Filter_f;/* '<S3>/Debounce_Filter' */
  DW_I_backCalc_fixdt I_backCalc_fixdt_g;/* '<S75>/I_backCalc_fixdt' */
  DW_I_backCalc_fixdt I_backCalc_fixdt1;/* '<S76>/I_backCalc_fixdt1' */
  DW_I_backCalc_fixdt I_backCalc_fixdt_i;/* '<S76>/I_backCalc_fixdt' */
  DW_Low_Pass_Filter Low_Pass_Filter_m;/* '<S43>/Low_Pass_Filter' */
  DW_PI_clamp_fixdt_e PI_clamp_fixdt_at;/* '<S56>/PI_clamp_fixdt' */
  DW_PI_clamp_fixdt_i PI_clamp_fixdt_oc;/* '<S55>/PI_clamp_fixdt' */
  DW_PI_clamp_fixdt PI_clamp_fixdt_k;  /* '<S57>/PI_clamp_fixdt' */
  DW_Counter Counter_e;                /* '<S12>/Counter' */
  int32_T Divide1;                     /* '<S74>/Divide1' */
  int32_T UnitDelay_DSTATE;            /* '<S38>/UnitDelay' */
  int16_T Gain4[3];                    /* '<S45>/Gain4' */
  int16_T DataTypeConversion[2];       /* '<S53>/Data Type Conversion' */
  int16_T z_counterRawPrev;            /* '<S16>/z_counterRawPrev' */
  int16_T Merge1;                      /* '<S31>/Merge1' */
  int16_T Divide3;                     /* '<S5>/Divide3' */
  int16_T Vd_max1;                     /* '<S47>/Vd_max1' */
  int16_T Gain3;                       /* '<S47>/Gain3' */
  int16_T Vq_max_M1;                   /* '<S47>/Vq_max_M1' */
  int16_T Gain5;                       /* '<S47>/Gain5' */
  int16_T i_max;                       /* '<S47>/i_max' */
  int16_T Divide1_a;                   /* '<S47>/Divide1' */
  int16_T Gain1;                       /* '<S47>/Gain1' */
  int16_T Gain4_c;                     /* '<S47>/Gain4' */
  int16_T Switch2;                     /* '<S80>/Switch2' */
  int16_T Switch2_l;                   /* '<S86>/Switch2' */
  int16_T Switch2_c;                   /* '<S84>/Switch2' */
  int16_T Merge;                       /* '<S44>/Merge' */
  int16_T Switch1;                     /* '<S72>/Switch1' */
  int16_T Divide11;                    /* '<S16>/Divide11' */
  int16_T UnitDelay3_DSTATE;           /* '<S12>/UnitDelay3' */
  int16_T UnitDelay4_DSTATE;           /* '<S16>/UnitDelay4' */
  int16_T UnitDelay2_DSTATE;           /* '<S16>/UnitDelay2' */
  int16_T UnitDelay3_DSTATE_o;         /* '<S16>/UnitDelay3' */
  int16_T UnitDelay5_DSTATE;           /* '<S16>/UnitDelay5' */
  int16_T UnitDelay4_DSTATE_e;         /* '<S12>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_eu;        /* '<S7>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_h;         /* '<S6>/UnitDelay4' */
  int8_T Switch2_e;                    /* '<S11>/Switch2' */
  int8_T UnitDelay2_DSTATE_b;          /* '<S11>/UnitDelay2' */
  int8_T If4_ActiveSubsystem;          /* '<S1>/If4' */
  int8_T If1_ActiveSubsystem;          /* '<S1>/If1' */
  int8_T If2_ActiveSubsystem;          /* '<S31>/If2' */
  int8_T If2_ActiveSubsystem_a;        /* '<S6>/If2' */
  int8_T If1_ActiveSubsystem_e;        /* '<S6>/If1' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S44>/Switch Case' */
  uint8_T z_ctrlMod;                   /* '<S4>/F03_02_Control_Mode_Manager' */
  uint8_T UnitDelay3_DSTATE_fy;        /* '<S9>/UnitDelay3' */
  uint8_T UnitDelay1_DSTATE;           /* '<S9>/UnitDelay1' */
  uint8_T UnitDelay2_DSTATE_f;         /* '<S9>/UnitDelay2' */
  uint8_T is_active_c1_BLDC_controller;/* '<S4>/F03_02_Control_Mode_Manager' */
  uint8_T is_c1_BLDC_controller;       /* '<S4>/F03_02_Control_Mode_Manager' */
  uint8_T is_ACTIVE;                   /* '<S4>/F03_02_Control_Mode_Manager' */
  boolean_T Merge_n;                   /* '<S19>/Merge' */
  boolean_T dz_cntTrnsDet;             /* '<S16>/dz_cntTrnsDet' */
  boolean_T UnitDelay2_DSTATE_g;       /* '<S8>/UnitDelay2' */
  boolean_T UnitDelay5_DSTATE_l;       /* '<S8>/UnitDelay5' */
  boolean_T UnitDelay6_DSTATE;         /* '<S8>/UnitDelay6' */
  boolean_T UnitDelay_DSTATE_b;        /* '<S37>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE_n;       /* '<S16>/UnitDelay1' */
  boolean_T n_commDeacv_Mode;          /* '<S12>/n_commDeacv' */
  boolean_T dz_cntTrnsDet_Mode;        /* '<S16>/dz_cntTrnsDet' */
} DW;

/* Constant parameters (auto storage) */
typedef struct {
  /* Computed Parameter: r_sin3PhaA_M1_Table
   * Referenced by: '<S89>/r_sin3PhaA_M1'
   */
  int16_T r_sin3PhaA_M1_Table[181];

  /* Computed Parameter: r_sin3PhaB_M1_Table
   * Referenced by: '<S89>/r_sin3PhaB_M1'
   */
  int16_T r_sin3PhaB_M1_Table[181];

  /* Computed Parameter: r_sin3PhaC_M1_Table
   * Referenced by: '<S89>/r_sin3PhaC_M1'
   */
  int16_T r_sin3PhaC_M1_Table[181];

  /* Computed Parameter: r_sin_M1_Table
   * Referenced by: '<S49>/r_sin_M1'
   */
  int16_T r_sin_M1_Table[181];

  /* Computed Parameter: r_cos_M1_Table
   * Referenced by: '<S49>/r_cos_M1'
   */
  int16_T r_cos_M1_Table[181];

  /* Computed Parameter: iq_maxSca_M1_Table
   * Referenced by: '<S47>/iq_maxSca_M1'
   */
  uint16_T iq_maxSca_M1_Table[50];

  /* Computed Parameter: z_commutMap_M1_table
   * Referenced by: '<S87>/z_commutMap_M1'
   */
  int8_T z_commutMap_M1_table[18];

  /* Computed Parameter: vec_hallToPos_Value
   * Referenced by: '<S10>/vec_hallToPos'
   */
  int8_T vec_hallToPos_Value[8];
} ConstP;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  boolean_T b_motEna;                  /* '<Root>/b_motEna' */
  uint8_T z_ctrlModReq;                /* '<Root>/z_ctrlModReq' */
  int16_T r_inpTgt;                    /* '<Root>/r_inpTgt' */
  uint8_T b_hallA;                     /* '<Root>/b_hallA ' */
  uint8_T b_hallB;                     /* '<Root>/b_hallB' */
  uint8_T b_hallC;                     /* '<Root>/b_hallC' */
  int16_T i_phaAB;                     /* '<Root>/i_phaAB' */
  int16_T i_phaBC;                     /* '<Root>/i_phaBC' */
  int16_T i_DCLink;                    /* '<Root>/i_DCLink' */
  int16_T a_mechAngle;                 /* '<Root>/a_mechAngle' */
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_T DC_phaA;                     /* '<Root>/DC_phaA' */
  int16_T DC_phaB;                     /* '<Root>/DC_phaB' */
  int16_T DC_phaC;                     /* '<Root>/DC_phaC' */
  uint8_T z_errCode;                   /* '<Root>/z_errCode' */
  int16_T n_mot;                       /* '<Root>/n_mot' */
  int16_T a_elecAngle;                 /* '<Root>/a_elecAngle' */
  int16_T iq;                          /* '<Root>/iq' */
  int16_T id;                          /* '<Root>/id' */
} ExtY;

/* Parameters (auto storage) */
struct P_ {
  int32_T dV_openRate;                 /* Variable: dV_openRate
                                        * Referenced by: '<S35>/dV_openRate'
                                        */
  int16_T dz_cntTrnsDetHi;             /* Variable: dz_cntTrnsDetHi
                                        * Referenced by: '<S16>/dz_cntTrnsDet'
                                        */
  int16_T dz_cntTrnsDetLo;             /* Variable: dz_cntTrnsDetLo
                                        * Referenced by: '<S16>/dz_cntTrnsDet'
                                        */
  int16_T n_cruiseMotTgt;              /* Variable: n_cruiseMotTgt
                                        * Referenced by: '<S55>/n_cruiseMotTgt'
                                        */
  int16_T z_maxCntRst;                 /* Variable: z_maxCntRst
                                        * Referenced by:
                                        *   '<S12>/Counter'
                                        *   '<S12>/z_maxCntRst'
                                        *   '<S12>/z_maxCntRst2'
                                        *   '<S12>/UnitDelay3'
                                        *   '<S16>/z_counter'
                                        */
  uint16_T cf_speedCoef;               /* Variable: cf_speedCoef
                                        * Referenced by: '<S16>/cf_speedCoef'
                                        */
  uint16_T t_errDequal;                /* Variable: t_errDequal
                                        * Referenced by: '<S3>/t_errDequal'
                                        */
  uint16_T t_errQual;                  /* Variable: t_errQual
                                        * Referenced by: '<S3>/t_errQual'
                                        */
  int16_T Vd_max;                      /* Variable: Vd_max
                                        * Referenced by:
                                        *   '<S47>/Vd_max1'
                                        *   '<S34>/Vd_max'
                                        */
  int16_T Vq_max_M1[46];               /* Variable: Vq_max_M1
                                        * Referenced by: '<S47>/Vq_max_M1'
                                        */
  int16_T Vq_max_XA[46];               /* Variable: Vq_max_XA
                                        * Referenced by: '<S47>/Vq_max_XA'
                                        */
  int16_T a_phaAdvMax;                 /* Variable: a_phaAdvMax
                                        * Referenced by: '<S5>/a_phaAdvMax'
                                        */
  int16_T i_max;                       /* Variable: i_max
                                        * Referenced by:
                                        *   '<S47>/i_max'
                                        *   '<S34>/i_max'
                                        */
  int16_T id_fieldWeakMax;             /* Variable: id_fieldWeakMax
                                        * Referenced by: '<S5>/id_fieldWeakMax'
                                        */
  int16_T n_commAcvLo;                 /* Variable: n_commAcvLo
                                        * Referenced by: '<S12>/n_commDeacv'
                                        */
  int16_T n_commDeacvHi;               /* Variable: n_commDeacvHi
                                        * Referenced by: '<S12>/n_commDeacv'
                                        */
  int16_T n_fieldWeakAuthHi;           /* Variable: n_fieldWeakAuthHi
                                        * Referenced by: '<S5>/n_fieldWeakAuthHi'
                                        */
  int16_T n_fieldWeakAuthLo;           /* Variable: n_fieldWeakAuthLo
                                        * Referenced by: '<S5>/n_fieldWeakAuthLo'
                                        */
  int16_T n_max;                       /* Variable: n_max
                                        * Referenced by:
                                        *   '<S47>/n_max1'
                                        *   '<S34>/n_max'
                                        */
  int16_T n_stdStillDet;               /* Variable: n_stdStillDet
                                        * Referenced by: '<S12>/n_stdStillDet'
                                        */
  int16_T r_errInpTgtThres;            /* Variable: r_errInpTgtThres
                                        * Referenced by: '<S3>/r_errInpTgtThres'
                                        */
  int16_T r_fieldWeakHi;               /* Variable: r_fieldWeakHi
                                        * Referenced by: '<S5>/r_fieldWeakHi'
                                        */
  int16_T r_fieldWeakLo;               /* Variable: r_fieldWeakLo
                                        * Referenced by: '<S5>/r_fieldWeakLo'
                                        */
  uint16_T cf_KbLimProt;               /* Variable: cf_KbLimProt
                                        * Referenced by:
                                        *   '<S75>/cf_KbLimProt'
                                        *   '<S76>/cf_KbLimProt'
                                        */
  uint16_T cf_idKp;                    /* Variable: cf_idKp
                                        * Referenced by: '<S57>/cf_idKp1'
                                        */
  uint16_T cf_iqKp;                    /* Variable: cf_iqKp
                                        * Referenced by: '<S56>/cf_iqKp'
                                        */
  uint16_T cf_nKp;                     /* Variable: cf_nKp
                                        * Referenced by: '<S55>/cf_nKp'
                                        */
  uint16_T cf_currFilt;                /* Variable: cf_currFilt
                                        * Referenced by: '<S43>/cf_currFilt'
                                        */
  uint16_T cf_idKi;                    /* Variable: cf_idKi
                                        * Referenced by: '<S57>/cf_idKi1'
                                        */
  uint16_T cf_iqKi;                    /* Variable: cf_iqKi
                                        * Referenced by: '<S56>/cf_iqKi'
                                        */
  uint16_T cf_iqKiLimProt;             /* Variable: cf_iqKiLimProt
                                        * Referenced by:
                                        *   '<S74>/cf_iqKiLimProt'
                                        *   '<S76>/cf_iqKiLimProt'
                                        */
  uint16_T cf_nKi;                     /* Variable: cf_nKi
                                        * Referenced by: '<S55>/cf_nKi'
                                        */
  uint16_T cf_nKiLimProt;              /* Variable: cf_nKiLimProt
                                        * Referenced by:
                                        *   '<S75>/cf_nKiLimProt'
                                        *   '<S76>/cf_nKiLimProt'
                                        */
  uint8_T n_polePairs;                 /* Variable: n_polePairs
                                        * Referenced by: '<S14>/n_polePairs'
                                        */
  uint8_T z_ctrlTypSel;                /* Variable: z_ctrlTypSel
                                        * Referenced by: '<S1>/z_ctrlTypSel'
                                        */
  uint8_T z_selPhaCurMeasABC;          /* Variable: z_selPhaCurMeasABC
                                        * Referenced by: '<S42>/z_selPhaCurMeasABC'
                                        */
  boolean_T b_angleMeasEna;            /* Variable: b_angleMeasEna
                                        * Referenced by:
                                        *   '<S2>/b_angleMeasEna'
                                        *   '<S12>/b_angleMeasEna'
                                        */
  boolean_T b_cruiseCtrlEna;           /* Variable: b_cruiseCtrlEna
                                        * Referenced by: '<S1>/b_cruiseCtrlEna'
                                        */
  boolean_T b_diagEna;                 /* Variable: b_diagEna
                                        * Referenced by: '<S1>/b_diagEna'
                                        */
  boolean_T b_fieldWeakEna;            /* Variable: b_fieldWeakEna
                                        * Referenced by:
                                        *   '<S1>/b_fieldWeakEna'
                                        *   '<S90>/b_fieldWeakEna'
                                        */
};

/* Parameters (auto storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
  P *defaultParam;
  ExtU *inputs;
  ExtY *outputs;
  DW *dwork;
};

/* Constant parameters (auto storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void BLDC_controller_initialize(RT_MODEL *const rtM);
extern void BLDC_controller_step(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S12>/Scope2' : Unused code path elimination
 * Block '<S13>/Scope' : Unused code path elimination
 * Block '<S39>/Data Type Duplicate' : Unused code path elimination
 * Block '<S39>/Data Type Propagation' : Unused code path elimination
 * Block '<S40>/Data Type Duplicate' : Unused code path elimination
 * Block '<S40>/Data Type Propagation' : Unused code path elimination
 * Block '<S41>/Data Type Duplicate' : Unused code path elimination
 * Block '<S41>/Data Type Propagation' : Unused code path elimination
 * Block '<S64>/Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Propagation' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Data Type Duplicate' : Unused code path elimination
 * Block '<S73>/Data Type Propagation' : Unused code path elimination
 * Block '<S77>/Data Type Duplicate' : Unused code path elimination
 * Block '<S77>/Data Type Propagation' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Propagation' : Unused code path elimination
 * Block '<S84>/Data Type Duplicate' : Unused code path elimination
 * Block '<S84>/Data Type Propagation' : Unused code path elimination
 * Block '<S86>/Data Type Duplicate' : Unused code path elimination
 * Block '<S86>/Data Type Propagation' : Unused code path elimination
 * Block '<S6>/Scope12' : Unused code path elimination
 * Block '<S6>/Scope8' : Unused code path elimination
 * Block '<S90>/Scope' : Unused code path elimination
 * Block '<S66>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S8>/Data Type Conversion' : Eliminate redundant data type conversion
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
 * hilite_system('BLDCmotor_FOC_R2017b_fixdt/BLDC_controller')    - opens subsystem BLDCmotor_FOC_R2017b_fixdt/BLDC_controller
 * hilite_system('BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotor_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller'
 * '<S2>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations'
 * '<S3>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics'
 * '<S4>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager'
 * '<S5>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening'
 * '<S6>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control'
 * '<S7>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management'
 * '<S8>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/Task_Scheduler'
 * '<S9>'   : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Edge_Detector'
 * '<S10>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_02_Position_Calculation'
 * '<S11>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_03_Direction_Detection'
 * '<S12>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation'
 * '<S13>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Electrical_Angle_Estimation'
 * '<S14>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_06_Electrical_Angle_Measurement'
 * '<S15>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter'
 * '<S16>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Raw_Motor_Speed_Estimation'
 * '<S17>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter/rst_Delay'
 * '<S18>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_06_Electrical_Angle_Measurement/Modulo_fixdt'
 * '<S19>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter'
 * '<S20>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/either_edge'
 * '<S21>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Default'
 * '<S22>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification'
 * '<S23>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification'
 * '<S24>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/either_edge'
 * '<S25>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter'
 * '<S26>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter/rst_Delay'
 * '<S27>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter'
 * '<S28>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter/rst_Delay'
 * '<S29>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_01_Mode_Transition_Calculation'
 * '<S30>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_02_Control_Mode_Manager'
 * '<S31>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis'
 * '<S32>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Control_Type'
 * '<S33>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Mode'
 * '<S34>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/FOC_Control_Type'
 * '<S35>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode'
 * '<S36>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter'
 * '<S37>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/rising_edge_init'
 * '<S38>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Delay_Init1'
 * '<S39>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Saturation Dynamic'
 * '<S40>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening/Saturation Dynamic'
 * '<S41>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Weakening/Saturation Dynamic1'
 * '<S42>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Transform'
 * '<S43>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Current_Filtering'
 * '<S44>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC'
 * '<S45>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Inv_Clarke_Transform'
 * '<S46>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Inv_Park_Transform'
 * '<S47>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations'
 * '<S48>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Park_Transform'
 * '<S49>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Sine_Cosine_Approximation'
 * '<S50>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesAB'
 * '<S51>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesAC'
 * '<S52>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesBC'
 * '<S53>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Current_Filtering/Low_Pass_Filter'
 * '<S54>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Open_Mode'
 * '<S55>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Speed_Mode'
 * '<S56>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode'
 * '<S57>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation'
 * '<S58>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Voltage_Mode'
 * '<S59>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Speed_Mode/PI_clamp_fixdt'
 * '<S60>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Speed_Mode/PI_clamp_fixdt/Clamping_circuit'
 * '<S61>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Speed_Mode/PI_clamp_fixdt/Integrator'
 * '<S62>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Speed_Mode/PI_clamp_fixdt/Saturation_hit'
 * '<S63>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode/PI_clamp_fixdt'
 * '<S64>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode/Saturation Dynamic1'
 * '<S65>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode/PI_clamp_fixdt/Clamping_circuit'
 * '<S66>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode/PI_clamp_fixdt/Integrator'
 * '<S67>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Torque_Mode/PI_clamp_fixdt/Saturation_hit'
 * '<S68>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation/PI_clamp_fixdt'
 * '<S69>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation/Saturation Dynamic'
 * '<S70>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation/PI_clamp_fixdt/Clamping_circuit'
 * '<S71>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation/PI_clamp_fixdt/Integrator'
 * '<S72>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Vd_Calculation/PI_clamp_fixdt/Saturation_hit'
 * '<S73>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/FOC/Voltage_Mode/Saturation Dynamic1'
 * '<S74>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Speed_Mode_Protection'
 * '<S75>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Torque_Mode_Protection'
 * '<S76>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection'
 * '<S77>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Speed_Mode_Protection/Saturation Dynamic'
 * '<S78>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Torque_Mode_Protection/I_backCalc_fixdt'
 * '<S79>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Torque_Mode_Protection/I_backCalc_fixdt/Integrator'
 * '<S80>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Torque_Mode_Protection/I_backCalc_fixdt/Saturation Dynamic1'
 * '<S81>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt'
 * '<S82>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt1'
 * '<S83>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt/Integrator'
 * '<S84>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt/Saturation Dynamic1'
 * '<S85>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt1/Integrator'
 * '<S86>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F05_Field_Oriented_Control/Motor_Limitations/Voltage_Mode_Protection/I_backCalc_fixdt1/Saturation Dynamic1'
 * '<S87>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/COM_Method'
 * '<S88>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/FOC_Method'
 * '<S89>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method'
 * '<S90>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method/Final_Phase_Advance_Calculation'
 * '<S91>'  : 'BLDCmotor_FOC_R2017b_fixdt/BLDC_controller/F06_Control_Type_Management/SIN_Method/Final_Phase_Advance_Calculation/Modulo_fixdt'
 */
#endif                                 /* RTW_HEADER_BLDC_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
