/*
 * File: BLDC_controller.h
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

/* Block signals and states (auto storage) for system '<S10>/Counter' */
typedef struct {
  int16_T UnitDelay_DSTATE;            /* '<S14>/UnitDelay' */
} DW_Counter;

/* Block signals and states (auto storage) for system '<S46>/PI_clamp_fixdt_id' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S67>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S64>/UnitDelay1' */
} DW_PI_clamp_fixdt;

/* Block signals and states (auto storage) for system '<S37>/Low_Pass_Filter' */
typedef struct {
  int16_T UnitDelay3_DSTATE[2];        /* '<S50>/UnitDelay3' */
} DW_Low_Pass_Filter;

/* Block signals and states (auto storage) for system '<S44>/PI_clamp_fixdt_n' */
typedef struct {
  int32_T UnitDelay_DSTATE;            /* '<S57>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE;         /* '<S55>/UnitDelay1' */
} DW_PI_clamp_fixdt_c;

/* Block signals and states (auto storage) for system '<S19>/Counter' */
typedef struct {
  uint16_T UnitDelay_DSTATE;           /* '<S24>/UnitDelay' */
} DW_Counter_l;

/* Block signals and states (auto storage) for system '<S15>/either_edge' */
typedef struct {
  boolean_T UnitDelay_DSTATE;          /* '<S20>/UnitDelay' */
} DW_either_edge;

/* Block signals and states (auto storage) for system '<S3>/Debounce_Filter' */
typedef struct {
  DW_either_edge either_edge_k;        /* '<S15>/either_edge' */
  DW_Counter_l Counter_h;              /* '<S18>/Counter' */
  DW_Counter_l Counter_i0;             /* '<S19>/Counter' */
  boolean_T UnitDelay_DSTATE;          /* '<S15>/UnitDelay' */
} DW_Debounce_Filter;

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  DW_either_edge either_edge_a;        /* '<S3>/either_edge' */
  DW_Debounce_Filter Debounce_Filter_f;/* '<S3>/Debounce_Filter' */
  DW_PI_clamp_fixdt PI_clamp_fixdt_iq; /* '<S45>/PI_clamp_fixdt_iq' */
  DW_PI_clamp_fixdt_c PI_clamp_fixdt_n_o;/* '<S44>/PI_clamp_fixdt_n' */
  DW_Low_Pass_Filter Low_Pass_Filter_m;/* '<S37>/Low_Pass_Filter' */
  DW_PI_clamp_fixdt PI_clamp_fixdt_id; /* '<S46>/PI_clamp_fixdt_id' */
  DW_Counter Counter_e;                /* '<S10>/Counter' */
  int32_T UnitDelay_DSTATE;            /* '<S34>/UnitDelay' */
  int16_T Gain4[3];                    /* '<S39>/Gain4' */
  int16_T Sum1[2];                     /* '<S50>/Sum1' */
  int16_T z_counterRawPrev;            /* '<S13>/z_counterRawPrev' */
  int16_T Merge;                       /* '<S5>/Merge' */
  int16_T Divide1;                     /* '<S52>/Divide1' */
  int16_T Divide4;                     /* '<S51>/Divide4' */
  int16_T Switch1;                     /* '<S68>/Switch1' */
  int16_T Divide11;                    /* '<S13>/Divide11' */
  int16_T UnitDelay3_DSTATE;           /* '<S10>/UnitDelay3' */
  int16_T UnitDelay4_DSTATE;           /* '<S13>/UnitDelay4' */
  int16_T UnitDelay2_DSTATE;           /* '<S13>/UnitDelay2' */
  int16_T UnitDelay3_DSTATE_o;         /* '<S13>/UnitDelay3' */
  int16_T UnitDelay5_DSTATE;           /* '<S13>/UnitDelay5' */
  int16_T UnitDelay4_DSTATE_e;         /* '<S10>/UnitDelay4' */
  int16_T UnitDelay4_DSTATE_eu;        /* '<S6>/UnitDelay4' */
  int8_T Switch2;                      /* '<S9>/Switch2' */
  int8_T UnitDelay2_DSTATE_b;          /* '<S9>/UnitDelay2' */
  int8_T If2_ActiveSubsystem;          /* '<S1>/If2' */
  int8_T If2_ActiveSubsystem_j;        /* '<S27>/If2' */
  int8_T If1_ActiveSubsystem;          /* '<S1>/If1' */
  int8_T If2_ActiveSubsystem_a;        /* '<S5>/If2' */
  int8_T If1_ActiveSubsystem_e;        /* '<S5>/If1' */
  int8_T If1_ActiveSubsystem_f;        /* '<S41>/If1' */
  int8_T If2_ActiveSubsystem_c;        /* '<S41>/If2' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S5>/Switch Case' */
  uint8_T UnitDelay3_DSTATE_fy;        /* '<S7>/UnitDelay3' */
  uint8_T UnitDelay1_DSTATE;           /* '<S7>/UnitDelay1' */
  uint8_T UnitDelay2_DSTATE_f;         /* '<S7>/UnitDelay2' */
  uint8_T UnitDelay1_DSTATE_p;         /* '<S4>/UnitDelay1' */
  uint8_T UnitDelay_DSTATE_c;          /* '<S3>/UnitDelay' */
  uint8_T is_active_c1_BLDC_controller;/* '<S4>/F03_02_Control_Mode_Manager' */
  uint8_T is_c1_BLDC_controller;       /* '<S4>/F03_02_Control_Mode_Manager' */
  uint8_T is_ACTIVE;                   /* '<S4>/F03_02_Control_Mode_Manager' */
  boolean_T Merge_n;                   /* '<S15>/Merge' */
  boolean_T dz_cntTrnsDet;             /* '<S13>/dz_cntTrnsDet' */
  boolean_T UnitDelay_DSTATE_e;        /* '<S33>/UnitDelay' */
  boolean_T UnitDelay1_DSTATE_n;       /* '<S13>/UnitDelay1' */
  boolean_T n_commDeacv_Mode;          /* '<S10>/n_commDeacv' */
  boolean_T n_fieldWeakAuth_Mode;      /* '<S38>/n_fieldWeakAuth' */
  boolean_T n_fieldWeakAuth_Mode_m;    /* '<S73>/n_fieldWeakAuth' */
  boolean_T dz_cntTrnsDet_Mode;        /* '<S13>/dz_cntTrnsDet' */
} DW;

/* Constant parameters (auto storage) */
typedef struct {
  /* Computed Parameter: r_sin3PhaA_M1_Table
   * Referenced by: '<S71>/r_sin3PhaA_M1'
   */
  int16_T r_sin3PhaA_M1_Table[181];

  /* Computed Parameter: r_sin3PhaB_M1_Table
   * Referenced by: '<S71>/r_sin3PhaB_M1'
   */
  int16_T r_sin3PhaB_M1_Table[181];

  /* Computed Parameter: r_sin3PhaC_M1_Table
   * Referenced by: '<S71>/r_sin3PhaC_M1'
   */
  int16_T r_sin3PhaC_M1_Table[181];

  /* Computed Parameter: r_sin_M1_Table
   * Referenced by: '<S38>/r_sin_M1'
   */
  int16_T r_sin_M1_Table[181];

  /* Computed Parameter: r_cos_M1_Table
   * Referenced by: '<S38>/r_cos_M1'
   */
  int16_T r_cos_M1_Table[181];

  /* Computed Parameter: iq_maxSca_M1_Table
   * Referenced by: '<S41>/iq_maxSca_M1'
   */
  uint16_T iq_maxSca_M1_Table[50];

  /* Computed Parameter: z_commutMap_M1_table
   * Referenced by: '<S70>/z_commutMap_M1'
   */
  int8_T z_commutMap_M1_table[18];

  /* Computed Parameter: vec_hallToPos_Value
   * Referenced by: '<S8>/vec_hallToPos'
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
                                        * Referenced by: '<S31>/dV_openRate'
                                        */
  int16_T dz_cntTrnsDetHi;             /* Variable: dz_cntTrnsDetHi
                                        * Referenced by: '<S13>/dz_cntTrnsDet'
                                        */
  int16_T dz_cntTrnsDetLo;             /* Variable: dz_cntTrnsDetLo
                                        * Referenced by: '<S13>/dz_cntTrnsDet'
                                        */
  int16_T z_maxCntRst;                 /* Variable: z_maxCntRst
                                        * Referenced by:
                                        *   '<S10>/Counter'
                                        *   '<S10>/z_maxCntRst'
                                        *   '<S10>/z_maxCntRst2'
                                        *   '<S10>/UnitDelay3'
                                        *   '<S13>/z_counter'
                                        */
  uint16_T cf_speedCoef;               /* Variable: cf_speedCoef
                                        * Referenced by: '<S13>/cf_speedCoef'
                                        */
  uint16_T t_errDequal;                /* Variable: t_errDequal
                                        * Referenced by: '<S3>/t_errDequal'
                                        */
  uint16_T t_errQual;                  /* Variable: t_errQual
                                        * Referenced by: '<S3>/t_errQual'
                                        */
  int16_T Vd_max;                      /* Variable: Vd_max
                                        * Referenced by:
                                        *   '<S41>/Vd_max1'
                                        *   '<S30>/Vd_max'
                                        */
  int16_T Vq_max_M1[46];               /* Variable: Vq_max_M1
                                        * Referenced by: '<S41>/Vq_max_M1'
                                        */
  int16_T Vq_max_XA[46];               /* Variable: Vq_max_XA
                                        * Referenced by: '<S41>/Vq_max_XA'
                                        */
  int16_T i_max;                       /* Variable: i_max
                                        * Referenced by:
                                        *   '<S41>/i_max'
                                        *   '<S30>/i_max'
                                        */
  int16_T id_fieldWeak_M1[12];         /* Variable: id_fieldWeak_M1
                                        * Referenced by: '<S38>/id_fieldWeak_M1'
                                        */
  int16_T n_commAcvLo;                 /* Variable: n_commAcvLo
                                        * Referenced by: '<S10>/n_commDeacv'
                                        */
  int16_T n_commDeacvHi;               /* Variable: n_commDeacvHi
                                        * Referenced by: '<S10>/n_commDeacv'
                                        */
  int16_T n_fieldWeakAuthHi;           /* Variable: n_fieldWeakAuthHi
                                        * Referenced by:
                                        *   '<S38>/n_fieldWeakAuth'
                                        *   '<S73>/n_fieldWeakAuth'
                                        */
  int16_T n_fieldWeakAuthLo;           /* Variable: n_fieldWeakAuthLo
                                        * Referenced by:
                                        *   '<S38>/n_fieldWeakAuth'
                                        *   '<S73>/n_fieldWeakAuth'
                                        */
  int16_T n_max;                       /* Variable: n_max
                                        * Referenced by:
                                        *   '<S41>/n_max1'
                                        *   '<S30>/n_max'
                                        */
  int16_T n_stdStillDet;               /* Variable: n_stdStillDet
                                        * Referenced by: '<S10>/n_stdStillDet'
                                        */
  int16_T r_errInpTgtThres;            /* Variable: r_errInpTgtThres
                                        * Referenced by: '<S3>/r_errInpTgtThres'
                                        */
  int16_T r_fieldWeak_XA[12];          /* Variable: r_fieldWeak_XA
                                        * Referenced by: '<S38>/r_fieldWeak_XA'
                                        */
  int16_T r_phaAdv_XA[11];             /* Variable: r_phaAdv_XA
                                        * Referenced by: '<S73>/r_phaAdv_XA'
                                        */
  uint16_T cf_idKp;                    /* Variable: cf_idKp
                                        * Referenced by: '<S46>/cf_idKp1'
                                        */
  uint16_T cf_iqKp;                    /* Variable: cf_iqKp
                                        * Referenced by: '<S45>/cf_iqKp'
                                        */
  uint16_T cf_nKp;                     /* Variable: cf_nKp
                                        * Referenced by: '<S44>/cf_nKp'
                                        */
  uint16_T cf_currFilt;                /* Variable: cf_currFilt
                                        * Referenced by: '<S37>/cf_currFilt'
                                        */
  uint16_T cf_idKi;                    /* Variable: cf_idKi
                                        * Referenced by: '<S46>/cf_idKi1'
                                        */
  uint16_T cf_iqKi;                    /* Variable: cf_iqKi
                                        * Referenced by: '<S45>/cf_iqKi'
                                        */
  uint16_T cf_iqKiLimProt;             /* Variable: cf_iqKiLimProt
                                        * Referenced by: '<S44>/cf_iqKiLimProt'
                                        */
  uint16_T cf_nKi;                     /* Variable: cf_nKi
                                        * Referenced by: '<S44>/cf_nKi'
                                        */
  uint16_T cf_iqKpLimProt;             /* Variable: cf_iqKpLimProt
                                        * Referenced by: '<S51>/cf_iqKpLimProt'
                                        */
  uint16_T cf_nKpLimProt;              /* Variable: cf_nKpLimProt
                                        * Referenced by: '<S52>/cf_nKpLimProt'
                                        */
  int16_T a_phaAdv_M1[11];             /* Variable: a_phaAdv_M1
                                        * Referenced by: '<S73>/a_phaAdv_M1'
                                        */
  uint8_T z_ctrlTypSel;                /* Variable: z_ctrlTypSel
                                        * Referenced by: '<S1>/z_ctrlTypSel1'
                                        */
  boolean_T b_diagEna;                 /* Variable: b_diagEna
                                        * Referenced by: '<S1>/b_diagEna'
                                        */
  boolean_T b_fieldWeakEna;            /* Variable: b_fieldWeakEna
                                        * Referenced by:
                                        *   '<S38>/b_fieldWeakEna'
                                        *   '<S73>/b_fieldWeakEna'
                                        */
  boolean_T b_selPhaABCurrMeas;        /* Variable: b_selPhaABCurrMeas
                                        * Referenced by: '<S36>/b_selPhaABCurrMeas'
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
 * Block '<S10>/Scope2' : Unused code path elimination
 * Block '<S11>/Scope' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate' : Unused code path elimination
 * Block '<S35>/Data Type Propagation' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Propagation' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Propagation' : Unused code path elimination
 * Block '<S5>/Scope12' : Unused code path elimination
 * Block '<S5>/Scope8' : Unused code path elimination
 * Block '<S5>/Scope9' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Propagation' : Unused code path elimination
 * Block '<S65>/Data Type Duplicate' : Unused code path elimination
 * Block '<S65>/Data Type Propagation' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Scope' : Unused code path elimination
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
 * '<S7>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_01_Edge_Detector'
 * '<S8>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_02_Position_Calculation'
 * '<S9>'   : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_03_Direction_Detection'
 * '<S10>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation'
 * '<S11>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_05_Electrical_Angle_Estimation'
 * '<S12>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter'
 * '<S13>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Raw_Motor_Speed_Estimation'
 * '<S14>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F01_Estimations/F01_04_Speed_Estimation/Counter/rst_Delay'
 * '<S15>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter'
 * '<S16>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/either_edge'
 * '<S17>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Default'
 * '<S18>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification'
 * '<S19>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification'
 * '<S20>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/either_edge'
 * '<S21>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter'
 * '<S22>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Dequalification/Counter/rst_Delay'
 * '<S23>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter'
 * '<S24>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F02_Diagnostics/Debounce_Filter/Qualification/Counter/rst_Delay'
 * '<S25>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_01_Mode_Transition_Calculation'
 * '<S26>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_02_Control_Mode_Manager'
 * '<S27>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis'
 * '<S28>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Control_Type'
 * '<S29>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Default_Mode'
 * '<S30>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/FOC_Control_Type'
 * '<S31>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode'
 * '<S32>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter'
 * '<S33>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/rising_edge_init'
 * '<S34>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Delay_Init1'
 * '<S35>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F03_Control_Mode_Manager/F03_03_Input_Target_Synthesis/Open_Mode/Rate_Limiter/Saturation Dynamic'
 * '<S36>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform'
 * '<S37>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Current_Filtering'
 * '<S38>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Field_Weakening'
 * '<S39>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Inv_Clarke_Transform'
 * '<S40>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Inv_Park_Transform'
 * '<S41>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations'
 * '<S42>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Open_Mode'
 * '<S43>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Park_Transform'
 * '<S44>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode'
 * '<S45>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode'
 * '<S46>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation'
 * '<S47>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Voltage_Mode'
 * '<S48>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesAB'
 * '<S49>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Clarke_Transform/Clarke_PhasesBC'
 * '<S50>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Current_Filtering/Low_Pass_Filter'
 * '<S51>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Current_Limit_Protection'
 * '<S52>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Speed_Limit_Protection'
 * '<S53>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Current_Limit_Protection/Saturation Dynamic'
 * '<S54>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Motor_Limitations/Speed_Limit_Protection/Saturation Dynamic1'
 * '<S55>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_clamp_fixdt_n'
 * '<S56>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_clamp_fixdt_n/Clamping_circuit'
 * '<S57>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_clamp_fixdt_n/Integrator'
 * '<S58>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Speed_Mode/PI_clamp_fixdt_n/Saturation_hit'
 * '<S59>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_clamp_fixdt_iq'
 * '<S60>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/Saturation Dynamic'
 * '<S61>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_clamp_fixdt_iq/Clamping_circuit'
 * '<S62>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_clamp_fixdt_iq/Integrator'
 * '<S63>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Torque_Mode/PI_clamp_fixdt_iq/Saturation_hit'
 * '<S64>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_clamp_fixdt_id'
 * '<S65>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/Saturation Dynamic'
 * '<S66>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_clamp_fixdt_id/Clamping_circuit'
 * '<S67>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_clamp_fixdt_id/Integrator'
 * '<S68>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Vd_Calculation/PI_clamp_fixdt_id/Saturation_hit'
 * '<S69>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F04_Field_Oriented_Control/Voltage_Mode/Saturation Dynamic1'
 * '<S70>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management/F05_00_COM_Method'
 * '<S71>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management/F05_01_SIN_Method'
 * '<S72>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management/F05_02_FOC_Method'
 * '<S73>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management/F05_01_SIN_Method/Phase_Advance_Calculation'
 * '<S74>'  : 'BLDCmotorControl_FOC_R2017b_fixdt/BLDC_controller/F05_Control_Type_Management/F05_01_SIN_Method/Phase_Advance_Calculation/Modulo_fixdt'
 */
#endif                                 /* RTW_HEADER_BLDC_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
