/*
 * File: BLDC_controller.h
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 1.1187
 * Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
 * C/C++ source code generated on : Sun Oct 27 17:31:20 2019
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

/* Block signals and states (auto storage) for system '<S11>/Counter' */
typedef struct {
  int16_T UnitDelay_DSTATE;            /* '<S17>/UnitDelay' */
} DW_Counter;

/* Block signals and states (auto storage) for system '<S31>/Low_Pass_Filter' */
typedef struct {
  int16_T UnitDelay3_DSTATE[2];        /* '<S44>/UnitDelay3' */
} DW_Low_Pass_Filter;

/* Block signals and states (auto storage) for system '<S40>/PI_backCalc_fixdt_Id' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S61>/UnitDelay' */
  int32_T UnitDelay_DSTATE_l;          /* '<S63>/UnitDelay' */
} DW_PI_backCalc_fixdt;

/* Block signals and states (auto storage) for system '<S38>/PI_backCalc_fixdt_n' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S53>/UnitDelay' */
  int32_T UnitDelay_DSTATE_h;          /* '<S55>/UnitDelay' */
} DW_PI_backCalc_fixdt_f;

/* Block signals and states (auto storage) for system '<S36>/Rate_Limiter' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S51>/UnitDelay' */
} DW_Rate_Limiter;

/* Block signals and states (auto storage) for system '<S22>/Counter' */
typedef struct {
  uint16_T UnitDelay_DSTATE;           /* '<S27>/UnitDelay' */
} DW_Counter_l;

/* Block signals and states (auto storage) for system '<S18>/either_edge' */
typedef struct {
  boolean_T UnitDelay_DSTATE;          /* '<S23>/UnitDelay' */
} DW_either_edge;

/* Block signals and states (auto storage) for system '<S3>/Debounce_Filter' */
typedef struct {
  DW_either_edge either_edge_k;        /* '<S18>/either_edge' */
  DW_Counter_l Counter_h;              /* '<S21>/Counter' */
  DW_Counter_l Counter_i0;             /* '<S22>/Counter' */
  boolean_T UnitDelay_DSTATE;          /* '<S18>/UnitDelay' */
} DW_Debounce_Filter;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_either_edge either_edge_a;        /* '<S3>/either_edge' */
  DW_Debounce_Filter Debounce_Filter_f;/* '<S3>/Debounce_Filter' */
  DW_Rate_Limiter Rate_Limiter_l;      /* '<S36>/Rate_Limiter' */
  DW_PI_backCalc_fixdt PI_backCalc_fixdt_Iq;/* '<S39>/PI_backCalc_fixdt_Iq' */
  DW_PI_backCalc_fixdt_f PI_backCalc_fixdt_n_p;/* '<S38>/PI_backCalc_fixdt_n' */
  DW_PI_backCalc_fixdt PI_backCalc_fixdt_Id;/* '<S40>/PI_backCalc_fixdt_Id' */
  DW_Low_Pass_Filter Low_Pass_Filter_m;/* '<S31>/Low_Pass_Filter' */
  DW_Counter Counter_e;                /* '<S11>/Counter' */
  int16_T Gain4[3];                    /* '<S33>/Gain4' */
  int16_T Sum1[2];                     /* '<S44>/Sum1' */
  int16_T z_counterRawPrev;            /* '<S16>/z_counterRawPrev' */
  int16_T Merge;                       /* '<S5>/Merge' */
  int16_T Divide1;                     /* '<S46>/Divide1' */
  int16_T Divide4;                     /* '<S45>/Divide4' */
  int16_T Switch2;                     /* '<S64>/Switch2' */
  int16_T Divide11;                    /* '<S16>/Divide11' */
  int16_T UnitDelay3_DSTATE;           /* '<S11>/UnitDelay3' */
  int16_T UnitDelay4_DSTATE;           /* '<S6>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_p;         /* '<S16>/UnitDelay4' */
  int16_T UnitDelay2_DSTATE;           /* '<S16>/UnitDelay2' */
  int16_T UnitDelay3_DSTATE_o;         /* '<S16>/UnitDelay3' */
  int16_T UnitDelay5_DSTATE;           /* '<S16>/UnitDelay5' */
  int16_T UnitDelay4_DSTATE_e;         /* '<S11>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_er;        /* '<S5>/UnitDelay4' */
  int8_T Switch2_e;                    /* '<S10>/Switch2' */
  int8_T UnitDelay2_DSTATE_b;          /* '<S10>/UnitDelay2' */
  int8_T If2_ActiveSubsystem;          /* '<S1>/If2' */
  int8_T If1_ActiveSubsystem;          /* '<S1>/If1' */
  int8_T If1_ActiveSubsystem_h;        /* '<S5>/If1' */
  int8_T If1_ActiveSubsystem_f;        /* '<S35>/If1' */
  int8_T If2_ActiveSubsystem_c;        /* '<S35>/If2' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S5>/Switch Case' */
  uint8_T UnitDelay1_DSTATE;           /* '<S4>/UnitDelay1' */
  uint8_T UnitDelay3_DSTATE_fy;        /* '<S8>/UnitDelay3' */
  uint8_T UnitDelay1_DSTATE_m;         /* '<S8>/UnitDelay1' */
  uint8_T UnitDelay2_DSTATE_f;         /* '<S8>/UnitDelay2' */
  uint8_T UnitDelay_DSTATE;            /* '<S3>/UnitDelay' */
  uint8_T is_active_c1_BLDC_controller;/* '<S4>/F02_02_Control_Mode_Manager' */
  uint8_T is_c1_BLDC_controller;       /* '<S4>/F02_02_Control_Mode_Manager' */
  uint8_T is_ACTIVE;                   /* '<S4>/F02_02_Control_Mode_Manager' */
  boolean_T Merge_n;                   /* '<S18>/Merge' */
  boolean_T dz_cntTrnsDet;             /* '<S16>/dz_cntTrnsDet' */
  boolean_T UnitDelay_DSTATE_g;        /* '<S50>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE_n;       /* '<S16>/UnitDelay1' */
  boolean_T n_commDeacv_Mode;          /* '<S11>/n_commDeacv' */
  boolean_T n_fieldWeakAuth_Mode;      /* '<S32>/n_fieldWeakAuth' */
  boolean_T dz_cntTrnsDet_Mode;        /* '<S16>/dz_cntTrnsDet' */
} DW;

/* Constant parameters (auto storage) */
typedef struct {
  /* Computed Parameter: z_commutMap_M1_table
   * Referenced by: '<S6>/z_commutMap_M1'
   */
  int16_T z_commutMap_M1_table[18];

  /* Computed Parameter: r_sin_M1_Table
   * Referenced by: '<S32>/r_sin_M1'
   */
  int16_T r_sin_M1_Table[181];

  /* Computed Parameter: r_cos_M1_Table
   * Referenced by: '<S32>/r_cos_M1'
   */
  int16_T r_cos_M1_Table[181];

  /* Computed Parameter: vec_hallToPos_Value
   * Referenced by: '<S9>/vec_hallToPos'
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
} ExtU;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int16_T DC_phaA;                     /* '<Root>/DC_phaA' */
  int16_T DC_phaB;                     /* '<Root>/DC_phaB' */
  int16_T DC_phaC;                     /* '<Root>/DC_phaC' */
  uint8_T z_errCode;                   /* '<Root>/z_errCode' */
  int16_T n_mot;                       /* '<Root>/n_mot' */
  int16_T a_elecAngle;                 /* '<Root>/a_elecAngle' */
  int16_T r_devSignal1;                /* '<Root>/r_devSignal1' */
  int16_T r_devSignal2;                /* '<Root>/r_devSignal2' */
} ExtY;

/* Parameters (auto storage) */
struct P_ {
  int32_T dV_openRate;                 /* Variable: dV_openRate
                                        * Referenced by: '<S36>/dV_openRate'
                                        */
  int16_T dz_cntTrnsDetHi;             /* Variable: dz_cntTrnsDetHi
                                        * Referenced by: '<S16>/dz_cntTrnsDet'
                                        */
  int16_T dz_cntTrnsDetLo;             /* Variable: dz_cntTrnsDetLo
                                        * Referenced by: '<S16>/dz_cntTrnsDet'
                                        */
  int16_T r_errInpTgtThres;            /* Variable: r_errInpTgtThres
                                        * Referenced by: '<S3>/r_errInpTgtThres'
                                        */
  int16_T z_maxCntRst;                 /* Variable: z_maxCntRst
                                        * Referenced by:
                                        *   '<S11>/Counter'
                                        *   '<S11>/z_maxCntRst'
                                        *   '<S11>/z_maxCntRst2'
                                        *   '<S11>/UnitDelay3'
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
  uint16_T cf_idKp;                    /* Variable: cf_idKp
                                        * Referenced by: '<S40>/cf_idKp'
                                        */
  uint16_T cf_iqKp;                    /* Variable: cf_iqKp
                                        * Referenced by: '<S39>/cf_iqKp'
                                        */
  uint16_T cf_nKp;                     /* Variable: cf_nKp
                                        * Referenced by: '<S38>/cf_nKp'
                                        */
  int16_T Vd_max;                      /* Variable: Vd_max
                                        * Referenced by:
                                        *   '<S35>/Vd_max1'
                                        *   '<S14>/Vd_max'
                                        */
  int16_T Vq_max_M1[46];               /* Variable: Vq_max_M1
                                        * Referenced by: '<S35>/Vq_max_M1'
                                        */
  int16_T Vq_max_XA[46];               /* Variable: Vq_max_XA
                                        * Referenced by: '<S35>/Vq_max_XA'
                                        */
  int16_T i_max;                       /* Variable: i_max
                                        * Referenced by:
                                        *   '<S35>/i_max'
                                        *   '<S14>/i_max'
                                        */
  int16_T id_fieldWeak_M1[12];         /* Variable: id_fieldWeak_M1
                                        * Referenced by: '<S32>/id_fieldWeak_M1'
                                        */
  int16_T iq_max_M1[51];               /* Variable: iq_max_M1
                                        * Referenced by: '<S35>/iq_max_M1'
                                        */
  int16_T iq_max_XA[51];               /* Variable: iq_max_XA
                                        * Referenced by: '<S35>/iq_max_XA'
                                        */
  int16_T n_commAcvLo;                 /* Variable: n_commAcvLo
                                        * Referenced by: '<S11>/n_commDeacv'
                                        */
  int16_T n_commDeacvHi;               /* Variable: n_commDeacvHi
                                        * Referenced by: '<S11>/n_commDeacv'
                                        */
  int16_T n_fieldWeakAuthHi;           /* Variable: n_fieldWeakAuthHi
                                        * Referenced by: '<S32>/n_fieldWeakAuth'
                                        */
  int16_T n_fieldWeakAuthLo;           /* Variable: n_fieldWeakAuthLo
                                        * Referenced by: '<S32>/n_fieldWeakAuth'
                                        */
  int16_T n_max;                       /* Variable: n_max
                                        * Referenced by:
                                        *   '<S35>/n_max1'
                                        *   '<S14>/n_max'
                                        */
  int16_T n_stdStillDet;               /* Variable: n_stdStillDet
                                        * Referenced by: '<S11>/n_stdStillDet'
                                        */
  int16_T r_fieldWeak_XA[12];          /* Variable: r_fieldWeak_XA
                                        * Referenced by: '<S32>/r_fieldWeak_XA'
                                        */
  uint16_T cf_currFilt;                /* Variable: cf_currFilt
                                        * Referenced by: '<S31>/cf_currFilt'
                                        */
  uint16_T cf_idKb;                    /* Variable: cf_idKb
                                        * Referenced by: '<S40>/cf_idKb'
                                        */
  uint16_T cf_idKi;                    /* Variable: cf_idKi
                                        * Referenced by: '<S40>/cf_idKi'
                                        */
  uint16_T cf_iqKb;                    /* Variable: cf_iqKb
                                        * Referenced by: '<S39>/cf_iqKb'
                                        */
  uint16_T cf_iqKi;                    /* Variable: cf_iqKi
                                        * Referenced by: '<S39>/cf_iqKi'
                                        */
  uint16_T cf_iqKiLimProt;             /* Variable: cf_iqKiLimProt
                                        * Referenced by: '<S38>/cf_iqKiLimProt'
                                        */
  uint16_T cf_nKb;                     /* Variable: cf_nKb
                                        * Referenced by: '<S38>/cf_nKb'
                                        */
  uint16_T cf_nKi;                     /* Variable: cf_nKi
                                        * Referenced by: '<S38>/cf_nKi'
                                        */
  uint16_T cf_iqKpLimProt;             /* Variable: cf_iqKpLimProt
                                        * Referenced by: '<S45>/cf_iqKpLimProt'
                                        */
  uint16_T cf_nKpLimProt;              /* Variable: cf_nKpLimProt
                                        * Referenced by: '<S46>/cf_nKpLimProt'
                                        */
  uint8_T z_ctrlTypSel;                /* Variable: z_ctrlTypSel
                                        * Referenced by: '<S1>/z_ctrlTypSel1'
                                        */
  boolean_T b_diagEna;                 /* Variable: b_diagEna
                                        * Referenced by: '<S1>/b_diagEna'
                                        */
  boolean_T b_fieldWeakEna;            /* Variable: b_fieldWeakEna
                                        * Referenced by: '<S32>/b_fieldWeakEna'
                                        */
  boolean_T b_selPhaABCurrMeas;        /* Variable: b_selPhaABCurrMeas
                                        * Referenced by: '<S30>/b_selPhaABCurrMeas'
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
 * Block '<S11>/Scope2' : Unused code path elimination
 * Block '<S12>/Scope' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Propagation' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Data Type Propagation' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S52>/Data Type Propagation' : Unused code path elimination
 * Block '<S5>/Scope12' : Unused code path elimination
 * Block '<S5>/Scope8' : Unused code path elimination
 * Block '<S5>/Scope9' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Propagation' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Propagation' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Propagation' : Unused code path elimination
 * Block '<S64>/Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Data Type Propagation' : Unused code path elimination
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
 * Block '<S65>/Data Type Duplicate' : Unused code path elimination
 * Block '<S65>/Data Type Propagation' : Unused code path elimination
 * Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion6' : Eliminate redundant data type conversion
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
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller')    - opens subsystem BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller
 * hilite_system('BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDCmotorControl_FOC_R2017b_fixdt'
 * '<S1>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller'
 * '<S2>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations'
 * '<S3>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics'
 * '<S4>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager'
 * '<S5>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control'
 * '<S6>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management'
 * '<S7>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Input_Scaling'
 * '<S8>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_02_Edge_Detector'
 * '<S9>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_03_Position_Calculation'
 * '<S10>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Direction_Detection'
 * '<S11>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Speed_Estimation'
 * '<S12>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_06_Electrical_Angle_Estimation'
 * '<S13>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Input_Scaling/Commutation_Control_Type'
 * '<S14>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Input_Scaling/FOC_Control_Type'
 * '<S15>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Speed_Estimation/Counter'
 * '<S16>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Speed_Estimation/Raw_Motor_Speed_Estimation'
 * '<S17>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Speed_Estimation/Counter/rst_Delay'
 * '<S18>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter'
 * '<S19>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/either_edge'
 * '<S20>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Default'
 * '<S21>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification'
 * '<S22>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification'
 * '<S23>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/either_edge'
 * '<S24>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter'
 * '<S25>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter/rst_Delay'
 * '<S26>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter'
 * '<S27>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter/rst_Delay'
 * '<S28>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F02_01_Mode_Transition_Calculation'
 * '<S29>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F02_02_Control_Mode_Manager'
 * '<S30>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform'
 * '<S31>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Current_Filtering'
 * '<S32>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Field_Weakening'
 * '<S33>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Inv_Clarke_Transform'
 * '<S34>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Inv_Park_Transform'
 * '<S35>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations'
 * '<S36>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode'
 * '<S37>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Park_Transform'
 * '<S38>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode'
 * '<S39>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode'
 * '<S40>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation'
 * '<S41>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Voltage_Mode'
 * '<S42>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesAB'
 * '<S43>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesBC'
 * '<S44>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Current_Filtering/Low_Pass_Filter'
 * '<S45>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Current_Limit_Protection'
 * '<S46>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Speed_Limit_Protection'
 * '<S47>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Current_Limit_Protection/Saturation Dynamic'
 * '<S48>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Speed_Limit_Protection/Saturation Dynamic1'
 * '<S49>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode/Rate_Limiter'
 * '<S50>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode/rising_edge_init'
 * '<S51>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode/Rate_Limiter/Delay_Init1'
 * '<S52>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode/Rate_Limiter/Saturation Dynamic'
 * '<S53>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_backCalc_fixdt_n'
 * '<S54>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/Saturation Dynamic1'
 * '<S55>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_backCalc_fixdt_n/Integrator'
 * '<S56>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_backCalc_fixdt_n/Saturation Dynamic1'
 * '<S57>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_backCalc_fixdt_Iq'
 * '<S58>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/Saturation Dynamic'
 * '<S59>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_backCalc_fixdt_Iq/Integrator'
 * '<S60>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_backCalc_fixdt_Iq/Saturation Dynamic1'
 * '<S61>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_backCalc_fixdt_Id'
 * '<S62>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/Saturation Dynamic'
 * '<S63>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_backCalc_fixdt_Id/Integrator'
 * '<S64>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_backCalc_fixdt_Id/Saturation Dynamic1'
 * '<S65>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Voltage_Mode/Saturation Dynamic1'
 */
#endif                                 /* RTW_HEADER_BLDC_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
