/*
 * File: CyberDog_Motor_FOC.h
 *
 * Code generated for Simulink model 'CyberDog_Motor_FOC'.
 *
 * Model version                  : 1.25
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sun Aug 16 02:05:08 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CyberDog_Motor_FOC_h_
#define CyberDog_Motor_FOC_h_
#ifndef CyberDog_Motor_FOC_COMMON_INCLUDES_
#define CyberDog_Motor_FOC_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* CyberDog_Motor_FOC_COMMON_INCLUDES_ */

#include "CyberDog_Motor_FOC_types.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T Saturation;                 /* '<S288>/Saturation' */
} B_CyberDog_Motor_FOC_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Delay_DSTATE;               /* '<S78>/Delay' */
  real32_T Delay_DSTATE_h;             /* '<S102>/Delay' */
  real32_T Integrator_DSTATE;          /* '<S47>/Integrator' */
  real32_T Delay_DSTATE_d;             /* '<S103>/Delay' */
  real32_T Integrator_DSTATE_l;        /* '<S171>/Integrator' */
  real32_T Delay_DSTATE_k;             /* '<S12>/Delay' */
  real32_T Integrator_DSTATE_b;        /* '<S281>/Integrator' */
  real32_T Integrator_DSTATE_k;        /* '<S223>/Integrator' */
  real32_T Delay_DSTATE_o;             /* '<S95>/Delay' */
  real32_T Delay2_DSTATE;              /* '<S90>/Delay2' */
  real32_T Delay3_DSTATE;              /* '<S90>/Delay3' */
  real32_T Delay2_DSTATE_i;            /* '<S89>/Delay2' */
  real32_T Delay3_DSTATE_h;            /* '<S89>/Delay3' */
  real32_T Delay_DSTATE_dt;            /* '<S98>/Delay' */
  real32_T Delay1_DSTATE;              /* '<S82>/Delay1' */
  real32_T Delay_DSTATE_ov;            /* '<S82>/Delay' */
  real32_T Delay1_DSTATE_b;            /* '<S81>/Delay1' */
  real32_T Delay_DSTATE_kp;            /* '<S81>/Delay' */
  int8_T Integrator_PrevResetState;    /* '<S47>/Integrator' */
  boolean_T icLoad;                    /* '<S78>/Delay' */
  boolean_T icLoad_o;                  /* '<S103>/Delay' */
  boolean_T icLoad_d;                  /* '<S12>/Delay' */
  boolean_T icLoad_d0;                 /* '<S95>/Delay' */
  boolean_T icLoad_i;                  /* '<S98>/Delay' */
  boolean_T icLoad_l;                  /* '<S85>/Delay' */
  boolean_T icLoad_k;                  /* '<S88>/Delay' */
} DW_CyberDog_Motor_FOC_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Delay_Reset_ZCE;          /* '<S102>/Delay' */
  ZCSigState Delay2_Reset_ZCE;         /* '<S90>/Delay2' */
  ZCSigState Delay3_Reset_ZCE;         /* '<S90>/Delay3' */
  ZCSigState Delay2_Reset_ZCE_o;       /* '<S89>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_o;       /* '<S89>/Delay3' */
  ZCSigState Delay1_Reset_ZCE;         /* '<S82>/Delay1' */
  ZCSigState Delay_Reset_ZCE_e;        /* '<S82>/Delay' */
  ZCSigState Delay1_Reset_ZCE_c;       /* '<S81>/Delay1' */
  ZCSigState Delay_Reset_ZCE_k;        /* '<S81>/Delay' */
} PrevZCX_CyberDog_Motor_FOC_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S113>/sine_table_values'
   *   '<S304>/sine_table_values'
   *   '<S105>/sine_table_values'
   */
  real32_T pooled5[1002];
} ConstP_CyberDog_Motor_FOC_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T eleangle;                   /* '<Root>/eleangle' */
  real32_T iq_ref;                     /* '<Root>/iq_ref' */
  real32_T id_ref;                     /* '<Root>/id_ref' */
  real32_T pll_reset;                  /* '<Root>/pll_reset' */
  real32_T ref_speed;                  /* '<Root>/ref_speed' */
  real32_T ctrl_mode;                  /* '<Root>/ctrl_mode' */
} ExtU_CyberDog_Motor_FOC_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T duty_u;                     /* '<Root>/duty_u' */
  real32_T duty_v;                     /* '<Root>/duty_v' */
  real32_T duty_w;                     /* '<Root>/duty_w' */
  real32_T theta_elec_filt;            /* '<Root>/theta_elec_filt' */
  real32_T speed_meas_rps;             /* '<Root>/speed_meas_rps' */
} ExtY_CyberDog_Motor_FOC_T;

/* Real-time Model Data Structure */
struct tag_RTM_CyberDog_Motor_FOC_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_B;

/* Block states (default storage) */
extern DW_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_PrevZCX;

/* External inputs (root inport signals with default storage) */
extern ExtU_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_Y;

/* Constant parameters (default storage) */
extern const ConstP_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T CoggingFF_En;          /* Variable: CoggingFF_En
                                        * Referenced by: '<S125>/Const_CogEn'
                                        * 齿槽转矩前馈使能(0/1)
                                        */
extern real32_T Cogging_Lut_Angle[32]; /* Variable: Cogging_Lut_Angle
                                        * Referenced by: '<S125>/LUT_cog'
                                        * 齿槽前馈LUT 电角度断点(rad)
                                        */
extern real32_T Cogging_Lut_V[32];     /* Variable: Cogging_Lut_V
                                        * Referenced by: '<S125>/LUT_cog'
                                        * 齿槽前馈LUT iq补偿(A)
                                        */
extern real32_T CurrD_Ki;              /* Variable: CurrD_Ki
                                        * Referenced by: '<S168>/Integral Gain'
                                        * d轴电流环积分增益
                                        */
extern real32_T CurrD_Kp;              /* Variable: CurrD_Kp
                                        * Referenced by: '<S176>/Proportional Gain'
                                        * d轴电流环比例增益
                                        */
extern real32_T CurrD_MaxOut;          /* Variable: CurrD_MaxOut
                                        * Referenced by:
                                        *   '<S171>/Integrator'
                                        *   '<S178>/Saturation'
                                        * d轴电流环输出上限(V)
                                        */
extern real32_T CurrD_MinOut;          /* Variable: CurrD_MinOut
                                        * Referenced by:
                                        *   '<S171>/Integrator'
                                        *   '<S178>/Saturation'
                                        * d轴电流环输出下限(V)
                                        */
extern real32_T CurrQ_Ki;              /* Variable: CurrQ_Ki
                                        * Referenced by: '<S220>/Integral Gain'
                                        * q轴电流环积分增益
                                        */
extern real32_T CurrQ_Kp;              /* Variable: CurrQ_Kp
                                        * Referenced by: '<S228>/Proportional Gain'
                                        * q轴电流环比例增益
                                        */
extern real32_T Curr_MaxOut;           /* Variable: Curr_MaxOut
                                        * Referenced by:
                                        *   '<S223>/Integrator'
                                        *   '<S230>/Saturation'
                                        * 电流环输出上限(V)
                                        */
extern real32_T Curr_MinOut;           /* Variable: Curr_MinOut
                                        * Referenced by:
                                        *   '<S223>/Integrator'
                                        *   '<S230>/Saturation'
                                        * 电流环输出下限(V)
                                        */
extern real32_T DeadComp_En;           /* Variable: DeadComp_En
                                        * Referenced by: '<S123>/Const_DeadEn'
                                        * 死区补偿使能(0/1)
                                        */
extern real32_T DeadComp_Lut_I[16];    /* Variable: DeadComp_Lut_I
                                        * Referenced by: '<S123>/LUT_dead'
                                        * 死区补偿LUT 电流断点(A)
                                        */
extern real32_T DeadComp_Lut_V[16];    /* Variable: DeadComp_Lut_V
                                        * Referenced by: '<S123>/LUT_dead'
                                        * 死区补偿LUT 补偿电压(V)
                                        */
extern real32_T InvTwoPiPolePairs;     /* Variable: InvTwoPiPolePairs
                                        * Referenced by: '<S1>/G_mech_rps'
                                        * 电角速度->机械RPS 1/(2*pi*PolePairs)
                                        */
extern real32_T InvVbus;               /* Variable: InvVbus
                                        * Referenced by:
                                        *   '<S127>/G_1_vbus'
                                        *   '<S127>/G_ubeta'
                                        * 母线电压倒数 1/Vbus
                                        */
extern real32_T Speed_Ki;              /* Variable: Speed_Ki
                                        * Referenced by: '<S278>/Integral Gain'
                                        * 速度环积分增益(1kHz+UseI*Ts 每周期增量)
                                        */
extern real32_T Speed_Kp;              /* Variable: Speed_Kp
                                        * Referenced by: '<S286>/Proportional Gain'
                                        * 速度环比例增益
                                        */
extern real32_T Speed_MaxOut;          /* Variable: Speed_MaxOut
                                        * Referenced by:
                                        *   '<S281>/Integrator'
                                        *   '<S288>/Saturation'
                                        * 速度环输出上限(A, iq_ref)
                                        */
extern real32_T Speed_MinOut;          /* Variable: Speed_MinOut
                                        * Referenced by:
                                        *   '<S281>/Integrator'
                                        *   '<S288>/Saturation'
                                        * 速度环输出下限(A, iq_ref)
                                        */
extern real32_T VmaxCoeff;             /* Variable: VmaxCoeff
                                        * Referenced by: '<S129>/G_vmax'
                                        * 电压限幅系数 Vbus/√3·MaxMod
                                        */

/* Model entry point functions */
extern void CyberDog_Motor_FOC_initialize(void);
extern void CyberDog_Motor_FOC_step(void);
extern void CyberDog_Motor_FOC_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CyberDog_Motor_FOC_T *const CyberDog_Motor_FOC_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S3>/Data Type Conversion' : Unused code path elimination
 * Block '<S3>/Data Type Conversion1' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S65>/I1CnstDT' : Unused code path elimination
 * Block '<S65>/I2CnstDT' : Unused code path elimination
 * Block '<S73>/Bias' : Unused code path elimination
 * Block '<S73>/Bias1' : Unused code path elimination
 * Block '<S73>/Constant' : Unused code path elimination
 * Block '<S73>/Data Type Duplicate' : Unused code path elimination
 * Block '<S73>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Data Type Propagation1' : Unused code path elimination
 * Block '<S73>/Divide' : Unused code path elimination
 * Block '<S73>/Divide1' : Unused code path elimination
 * Block '<S73>/OD1Cnst' : Unused code path elimination
 * Block '<S73>/OD1CnstDT' : Unused code path elimination
 * Block '<S73>/Product' : Unused code path elimination
 * Block '<S73>/Unary Minus' : Unused code path elimination
 * Block '<S65>/MaxFreqCnst' : Unused code path elimination
 * Block '<S65>/OD1CnstDT' : Unused code path elimination
 * Block '<S65>/OD2CnstDT' : Unused code path elimination
 * Block '<S74>/Bias' : Unused code path elimination
 * Block '<S74>/Constant' : Unused code path elimination
 * Block '<S74>/Data Type Duplicate' : Unused code path elimination
 * Block '<S74>/Data Type Propagation' : Unused code path elimination
 * Block '<S74>/Data Type Propagation1' : Unused code path elimination
 * Block '<S74>/Data Type Propagation2' : Unused code path elimination
 * Block '<S74>/Divide' : Unused code path elimination
 * Block '<S74>/OD1Cnst' : Unused code path elimination
 * Block '<S74>/OD1CnstDT' : Unused code path elimination
 * Block '<S74>/OD1CnstDT1' : Unused code path elimination
 * Block '<S74>/Product' : Unused code path elimination
 * Block '<S74>/Product1' : Unused code path elimination
 * Block '<S74>/SampleTime' : Unused code path elimination
 * Block '<S75>/Const' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Propagation1' : Unused code path elimination
 * Block '<S75>/MaxFreqDT' : Unused code path elimination
 * Block '<S75>/Product' : Unused code path elimination
 * Block '<S66>/Data Type Duplicate' : Unused code path elimination
 * Block '<S67>/Bias' : Unused code path elimination
 * Block '<S67>/Constant' : Unused code path elimination
 * Block '<S67>/Data Type Propagation' : Unused code path elimination
 * Block '<S67>/Data Type Propagation1' : Unused code path elimination
 * Block '<S67>/Divide' : Unused code path elimination
 * Block '<S67>/Product' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S68>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S79>/Data Type Duplicate' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Propagation' : Unused code path elimination
 * Block '<S81>/Data Type Propagation1' : Unused code path elimination
 * Block '<S82>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Propagation' : Unused code path elimination
 * Block '<S82>/Data Type Propagation1' : Unused code path elimination
 * Block '<S71>/Data Type Duplicate' : Unused code path elimination
 * Block '<S89>/Data Type Duplicate' : Unused code path elimination
 * Block '<S89>/Data Type Propagation' : Unused code path elimination
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S91>/Data Type Duplicate' : Unused code path elimination
 * Block '<S92>/Data Type Duplicate' : Unused code path elimination
 * Block '<S72>/Bias' : Unused code path elimination
 * Block '<S72>/Constant' : Unused code path elimination
 * Block '<S72>/Data Type Propagation' : Unused code path elimination
 * Block '<S72>/Data Type Propagation1' : Unused code path elimination
 * Block '<S72>/Divide' : Unused code path elimination
 * Block '<S72>/Product' : Unused code path elimination
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S102>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S102>/Data Type Propagation' : Unused code path elimination
 * Block '<S103>/BooleanTypeCast' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate3' : Unused code path elimination
 * Block '<S103>/Data Type Propagation' : Unused code path elimination
 * Block '<S103>/InpGainDatatype' : Unused code path elimination
 * Block '<S103>/OutGainDatatype' : Unused code path elimination
 * Block '<S103>/Reset' : Unused code path elimination
 * Block '<S105>/Data Type Duplicate' : Unused code path elimination
 * Block '<S105>/Data Type Propagation' : Unused code path elimination
 * Block '<S110>/Data Type Duplicate' : Unused code path elimination
 * Block '<S111>/Data Type Duplicate' : Unused code path elimination
 * Block '<S113>/Data Type Duplicate' : Unused code path elimination
 * Block '<S113>/Data Type Propagation' : Unused code path elimination
 * Block '<S118>/Data Type Duplicate' : Unused code path elimination
 * Block '<S119>/Data Type Duplicate' : Unused code path elimination
 * Block '<S134>/Data Type Duplicate' : Unused code path elimination
 * Block '<S244>/Data Type Duplicate' : Unused code path elimination
 * Block '<S244>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S302>/Data Type Duplicate' : Unused code path elimination
 * Block '<S302>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S304>/Data Type Duplicate' : Unused code path elimination
 * Block '<S304>/Data Type Propagation' : Unused code path elimination
 * Block '<S309>/Data Type Duplicate' : Unused code path elimination
 * Block '<S310>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/FreqGain' : Eliminated nontunable gain of 1
 * Block '<S5>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S7>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S66>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S79>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S80>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S91>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S92>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S102>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S102>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S102>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S105>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S107>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S112>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S113>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S120>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S133>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S133>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S163>/Kb' : Eliminated nontunable gain of 1
 * Block '<S215>/Kb' : Eliminated nontunable gain of 1
 * Block '<S273>/Kb' : Eliminated nontunable gain of 1
 * Block '<S304>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S311>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S5>/One' : Unused code path elimination
 * Block '<S5>/Reset' : Unused code path elimination
 * Block '<S5>/Sum' : Unused code path elimination
 * Block '<S5>/UseInputPort' : Unused code path elimination
 * Block '<S5>/a' : Unused code path elimination
 * Block '<S66>/FilterConstant' : Unused code path elimination
 * Block '<S66>/OneMinusFilterConstant' : Unused code path elimination
 * Block '<S66>/Reset' : Unused code path elimination
 * Block '<S66>/UseInputPort' : Unused code path elimination
 * Block '<S245>/Offset' : Unused code path elimination
 * Block '<S245>/Unary_Minus' : Unused code path elimination
 * Block '<S303>/Offset' : Unused code path elimination
 * Block '<S303>/Unary_Minus' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CyberDog_Motor_FOC'
 * '<S1>'   : 'CyberDog_Motor_FOC/EncoderPLL'
 * '<S2>'   : 'CyberDog_Motor_FOC/FOC_Algorithm'
 * '<S3>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF'
 * '<S4>'   : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele'
 * '<S5>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter'
 * '<S6>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller'
 * '<S7>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward'
 * '<S8>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem'
 * '<S9>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO'
 * '<S10>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter'
 * '<S11>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter/Low-pass'
 * '<S12>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S13>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Anti-windup'
 * '<S14>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/D Gain'
 * '<S15>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/External Derivative'
 * '<S16>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter'
 * '<S17>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter ICs'
 * '<S18>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/I Gain'
 * '<S19>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain'
 * '<S20>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain Fdbk'
 * '<S21>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator'
 * '<S22>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator ICs'
 * '<S23>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Copy'
 * '<S24>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Gain'
 * '<S25>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/P Copy'
 * '<S26>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Parallel P Gain'
 * '<S27>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Reset Signal'
 * '<S28>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation'
 * '<S29>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation Fdbk'
 * '<S30>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum'
 * '<S31>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum Fdbk'
 * '<S32>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode'
 * '<S33>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode Sum'
 * '<S34>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Integral'
 * '<S35>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Ngain'
 * '<S36>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/postSat Signal'
 * '<S37>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preInt Signal'
 * '<S38>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preSat Signal'
 * '<S39>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Anti-windup/Passthrough'
 * '<S40>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/D Gain/Disabled'
 * '<S41>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/External Derivative/Disabled'
 * '<S42>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter/Disabled'
 * '<S43>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter ICs/Disabled'
 * '<S44>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/I Gain/Internal Parameters'
 * '<S45>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain/Passthrough'
 * '<S46>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S47>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator/Discrete'
 * '<S48>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator ICs/Internal IC'
 * '<S49>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S50>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Gain/Disabled'
 * '<S51>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/P Copy/Disabled'
 * '<S52>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S53>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Reset Signal/External Reset'
 * '<S54>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation/Passthrough'
 * '<S55>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation Fdbk/Disabled'
 * '<S56>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum/Sum_PI'
 * '<S57>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum Fdbk/Disabled'
 * '<S58>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode/Disabled'
 * '<S59>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S60>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S61>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S62>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/postSat Signal/Forward_Path'
 * '<S63>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preInt Signal/Internal PreInt'
 * '<S64>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preSat Signal/Forward_Path'
 * '<S65>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem'
 * '<S66>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter'
 * '<S67>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/InpFilterFunc'
 * '<S68>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod'
 * '<S69>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/Nofunc'
 * '<S70>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/NofuncSpd'
 * '<S71>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod'
 * '<S72>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/SpdFilterFunc'
 * '<S73>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/IGains'
 * '<S74>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/ODGains'
 * '<S75>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/OutDTSetup'
 * '<S76>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter'
 * '<S77>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass'
 * '<S78>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S79>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1'
 * '<S80>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3'
 * '<S81>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/Integrator'
 * '<S82>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/Integrator2'
 * '<S83>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter'
 * '<S84>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S85>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S86>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter'
 * '<S87>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S88>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S89>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator'
 * '<S90>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator1'
 * '<S91>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1'
 * '<S92>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3'
 * '<S93>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter'
 * '<S94>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S95>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S96>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter'
 * '<S97>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S98>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S99>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem/If Action Subsystem'
 * '<S100>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem/If Action Subsystem1'
 * '<S101>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem'
 * '<S102>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased'
 * '<S103>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Position Generator'
 * '<S104>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup'
 * '<S105>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S106>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S107>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S108>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S109>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S110>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S111>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S112>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype backpropogation'
 * '<S113>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup'
 * '<S114>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/Interpolation'
 * '<S115>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp'
 * '<S116>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/datatype'
 * '<S117>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S118>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S119>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S120>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/datatype/datatype backpropogation'
 * '<S121>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke'
 * '<S122>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController'
 * '<S123>' : 'CyberDog_Motor_FOC/FOC_Algorithm/DeadTimeComp'
 * '<S124>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark'
 * '<S125>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop'
 * '<S126>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park'
 * '<S127>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SVPWM'
 * '<S128>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th'
 * '<S129>' : 'CyberDog_Motor_FOC/FOC_Algorithm/VoltageLimiter'
 * '<S130>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant'
 * '<S131>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb'
 * '<S132>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform'
 * '<S133>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform/Two phase input'
 * '<S134>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform/Two phase input/Two phase CRL wrap'
 * '<S135>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d'
 * '<S136>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q'
 * '<S137>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Anti-windup'
 * '<S138>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/D Gain'
 * '<S139>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/External Derivative'
 * '<S140>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter'
 * '<S141>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter ICs'
 * '<S142>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/I Gain'
 * '<S143>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain'
 * '<S144>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain Fdbk'
 * '<S145>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator'
 * '<S146>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator ICs'
 * '<S147>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Copy'
 * '<S148>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Gain'
 * '<S149>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/P Copy'
 * '<S150>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Parallel P Gain'
 * '<S151>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Reset Signal'
 * '<S152>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation'
 * '<S153>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation Fdbk'
 * '<S154>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum'
 * '<S155>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum Fdbk'
 * '<S156>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode'
 * '<S157>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode Sum'
 * '<S158>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Integral'
 * '<S159>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Ngain'
 * '<S160>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/postSat Signal'
 * '<S161>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preInt Signal'
 * '<S162>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preSat Signal'
 * '<S163>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Anti-windup/Back Calculation'
 * '<S164>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/D Gain/Disabled'
 * '<S165>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/External Derivative/Disabled'
 * '<S166>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter/Disabled'
 * '<S167>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter ICs/Disabled'
 * '<S168>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/I Gain/Internal Parameters'
 * '<S169>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain/Passthrough'
 * '<S170>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain Fdbk/Disabled'
 * '<S171>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator/Discrete'
 * '<S172>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator ICs/Internal IC'
 * '<S173>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Copy/Disabled wSignal Specification'
 * '<S174>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Gain/Disabled'
 * '<S175>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/P Copy/Disabled'
 * '<S176>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Parallel P Gain/Internal Parameters'
 * '<S177>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Reset Signal/Disabled'
 * '<S178>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation/Enabled'
 * '<S179>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation Fdbk/Disabled'
 * '<S180>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum/Sum_PI'
 * '<S181>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum Fdbk/Disabled'
 * '<S182>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode/Disabled'
 * '<S183>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode Sum/Passthrough'
 * '<S184>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Integral/TsSignalSpecification'
 * '<S185>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Ngain/Passthrough'
 * '<S186>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/postSat Signal/Forward_Path'
 * '<S187>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preInt Signal/Internal PreInt'
 * '<S188>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preSat Signal/Forward_Path'
 * '<S189>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Anti-windup'
 * '<S190>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/D Gain'
 * '<S191>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/External Derivative'
 * '<S192>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter'
 * '<S193>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter ICs'
 * '<S194>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/I Gain'
 * '<S195>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain'
 * '<S196>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain Fdbk'
 * '<S197>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator'
 * '<S198>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator ICs'
 * '<S199>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Copy'
 * '<S200>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Gain'
 * '<S201>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/P Copy'
 * '<S202>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Parallel P Gain'
 * '<S203>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Reset Signal'
 * '<S204>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation'
 * '<S205>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation Fdbk'
 * '<S206>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum'
 * '<S207>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum Fdbk'
 * '<S208>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode'
 * '<S209>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode Sum'
 * '<S210>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Integral'
 * '<S211>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Ngain'
 * '<S212>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/postSat Signal'
 * '<S213>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preInt Signal'
 * '<S214>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preSat Signal'
 * '<S215>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Anti-windup/Back Calculation'
 * '<S216>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/D Gain/Disabled'
 * '<S217>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/External Derivative/Disabled'
 * '<S218>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter/Disabled'
 * '<S219>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter ICs/Disabled'
 * '<S220>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/I Gain/Internal Parameters'
 * '<S221>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain/Passthrough'
 * '<S222>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain Fdbk/Disabled'
 * '<S223>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator/Discrete'
 * '<S224>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator ICs/Internal IC'
 * '<S225>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Copy/Disabled wSignal Specification'
 * '<S226>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Gain/Disabled'
 * '<S227>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/P Copy/Disabled'
 * '<S228>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Parallel P Gain/Internal Parameters'
 * '<S229>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Reset Signal/Disabled'
 * '<S230>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation/Enabled'
 * '<S231>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation Fdbk/Disabled'
 * '<S232>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum/Sum_PI'
 * '<S233>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum Fdbk/Disabled'
 * '<S234>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode/Disabled'
 * '<S235>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode Sum/Passthrough'
 * '<S236>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Integral/TsSignalSpecification'
 * '<S237>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Ngain/Passthrough'
 * '<S238>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/postSat Signal/Forward_Path'
 * '<S239>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preInt Signal/Internal PreInt'
 * '<S240>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preSat Signal/Forward_Path'
 * '<S241>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant'
 * '<S242>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb'
 * '<S243>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform'
 * '<S244>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform/Two inputs CRL'
 * '<S245>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S246>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed'
 * '<S247>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Anti-windup'
 * '<S248>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/D Gain'
 * '<S249>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/External Derivative'
 * '<S250>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Filter'
 * '<S251>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Filter ICs'
 * '<S252>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/I Gain'
 * '<S253>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Ideal P Gain'
 * '<S254>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Ideal P Gain Fdbk'
 * '<S255>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Integrator'
 * '<S256>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Integrator ICs'
 * '<S257>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/N Copy'
 * '<S258>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/N Gain'
 * '<S259>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/P Copy'
 * '<S260>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Parallel P Gain'
 * '<S261>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Reset Signal'
 * '<S262>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Saturation'
 * '<S263>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Saturation Fdbk'
 * '<S264>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Sum'
 * '<S265>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Sum Fdbk'
 * '<S266>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tracking Mode'
 * '<S267>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tracking Mode Sum'
 * '<S268>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tsamp - Integral'
 * '<S269>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tsamp - Ngain'
 * '<S270>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/postSat Signal'
 * '<S271>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/preInt Signal'
 * '<S272>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/preSat Signal'
 * '<S273>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Anti-windup/Back Calculation'
 * '<S274>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/D Gain/Disabled'
 * '<S275>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/External Derivative/Disabled'
 * '<S276>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Filter/Disabled'
 * '<S277>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Filter ICs/Disabled'
 * '<S278>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/I Gain/Internal Parameters'
 * '<S279>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Ideal P Gain/Passthrough'
 * '<S280>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Ideal P Gain Fdbk/Disabled'
 * '<S281>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Integrator/Discrete'
 * '<S282>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Integrator ICs/Internal IC'
 * '<S283>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/N Copy/Disabled wSignal Specification'
 * '<S284>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/N Gain/Disabled'
 * '<S285>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/P Copy/Disabled'
 * '<S286>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Parallel P Gain/Internal Parameters'
 * '<S287>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Reset Signal/Disabled'
 * '<S288>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Saturation/Enabled'
 * '<S289>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Saturation Fdbk/Disabled'
 * '<S290>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Sum/Sum_PI'
 * '<S291>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Sum Fdbk/Disabled'
 * '<S292>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tracking Mode/Disabled'
 * '<S293>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tracking Mode Sum/Passthrough'
 * '<S294>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tsamp - Integral/TsSignalSpecification'
 * '<S295>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/Tsamp - Ngain/Passthrough'
 * '<S296>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/postSat Signal/Forward_Path'
 * '<S297>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/preInt Signal/Internal PreInt'
 * '<S298>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PID_speed/preSat Signal/Forward_Path'
 * '<S299>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant'
 * '<S300>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb'
 * '<S301>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform'
 * '<S302>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform/Two inputs CRL'
 * '<S303>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S304>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup'
 * '<S305>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/Interpolation'
 * '<S306>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp'
 * '<S307>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/datatype'
 * '<S308>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S309>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S310>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S311>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/datatype/datatype backpropogation'
 */
#endif                                 /* CyberDog_Motor_FOC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
