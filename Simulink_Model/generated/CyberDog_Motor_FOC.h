/*
 * File: CyberDog_Motor_FOC.h
 *
 * Code generated for Simulink model 'CyberDog_Motor_FOC'.
 *
 * Model version                  : 1.56
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Sep 24 01:32:40 2026
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
#include "rtGetNaN.h"
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
  real32_T Saturation;                 /* '<S291>/Saturation' */
  real32_T Saturation_f;               /* '<S344>/Saturation' */
} B_CyberDog_Motor_FOC_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Delay_DSTATE;               /* '<S79>/Delay' */
  real32_T Delay_DSTATE_h;             /* '<S103>/Delay' */
  real32_T Integrator_DSTATE;          /* '<S48>/Integrator' */
  real32_T Delay_DSTATE_d;             /* '<S104>/Delay' */
  real32_T Integrator_DSTATE_l;        /* '<S172>/Integrator' */
  real32_T Filter_DSTATE;              /* '<S279>/Filter' */
  real32_T Delay_DSTATE_k;             /* '<S13>/Delay' */
  real32_T Integrator_DSTATE_m;        /* '<S337>/Integrator' */
  real32_T ESO_UD_z2_DSTATE;           /* '<S248>/ESO_UD_z2' */
  real32_T Integrator_DSTATE_k;        /* '<S224>/Integrator' */
  real32_T v_prev_DSTATE;              /* '<S1>/v_prev' */
  real32_T ESO_UD_lpf_DSTATE;          /* '<S248>/ESO_UD_lpf' */
  real32_T ESO_UD_z1_DSTATE;           /* '<S248>/ESO_UD_z1' */
  real32_T Delay_DSTATE_o;             /* '<S96>/Delay' */
  real32_T Delay2_DSTATE;              /* '<S91>/Delay2' */
  real32_T Delay3_DSTATE;              /* '<S91>/Delay3' */
  real32_T Delay2_DSTATE_i;            /* '<S90>/Delay2' */
  real32_T Delay3_DSTATE_h;            /* '<S90>/Delay3' */
  real32_T Delay_DSTATE_dt;            /* '<S99>/Delay' */
  real32_T Delay1_DSTATE;              /* '<S83>/Delay1' */
  real32_T Delay_DSTATE_ov;            /* '<S83>/Delay' */
  real32_T Delay1_DSTATE_b;            /* '<S82>/Delay1' */
  real32_T Delay_DSTATE_kp;            /* '<S82>/Delay' */
  int8_T Integrator_PrevResetState;    /* '<S48>/Integrator' */
  boolean_T icLoad;                    /* '<S79>/Delay' */
  boolean_T icLoad_o;                  /* '<S104>/Delay' */
  boolean_T icLoad_d;                  /* '<S13>/Delay' */
  boolean_T icLoad_d0;                 /* '<S96>/Delay' */
  boolean_T icLoad_i;                  /* '<S99>/Delay' */
  boolean_T icLoad_l;                  /* '<S86>/Delay' */
  boolean_T icLoad_k;                  /* '<S89>/Delay' */
} DW_CyberDog_Motor_FOC_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Delay_Reset_ZCE;          /* '<S103>/Delay' */
  ZCSigState Delay2_Reset_ZCE;         /* '<S91>/Delay2' */
  ZCSigState Delay3_Reset_ZCE;         /* '<S91>/Delay3' */
  ZCSigState Delay2_Reset_ZCE_o;       /* '<S90>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_o;       /* '<S90>/Delay3' */
  ZCSigState Delay1_Reset_ZCE;         /* '<S83>/Delay1' */
  ZCSigState Delay_Reset_ZCE_e;        /* '<S83>/Delay' */
  ZCSigState Delay1_Reset_ZCE_c;       /* '<S82>/Delay1' */
  ZCSigState Delay_Reset_ZCE_k;        /* '<S82>/Delay' */
} PrevZCX_CyberDog_Motor_FOC_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S114>/sine_table_values'
   *   '<S360>/sine_table_values'
   *   '<S106>/sine_table_values'
   */
  real32_T pooled6[1002];
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
  real32_T coast;                      /* '<Root>/coast' */
  real32_T pos_ref;                    /* '<Root>/pos_ref' */
  real32_T pos_fbk;                    /* '<Root>/pos_fbk' */
  real32_T vd_ref;                     /* '<Root>/vd_ref' */
  real32_T vq_ref;                     /* '<Root>/vq_ref' */
  real32_T v_alpha_ref;                /* '<Root>/v_alpha_ref' */
  real32_T v_beta_ref;                 /* '<Root>/v_beta_ref' */
  real32_T we_elec;                    /* '<Root>/we_elec' */
  real32_T ff_en;                      /* '<Root>/ff_en' */
  real32_T ref_accel;                  /* '<Root>/ref_accel' */
  real32_T vel_ff;                     /* '<Root>/vel_ff' */
} ExtU_CyberDog_Motor_FOC_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T duty_u;                     /* '<Root>/duty_u' */
  real32_T duty_v;                     /* '<Root>/duty_v' */
  real32_T duty_w;                     /* '<Root>/duty_w' */
  real32_T theta_elec_filt;            /* '<Root>/theta_elec_filt' */
  real32_T speed_meas_rps;             /* '<Root>/speed_meas_rps' */
  real32_T i_alpha_meas;               /* '<Root>/i_alpha_meas' */
  real32_T i_beta_meas;                /* '<Root>/i_beta_meas' */
  real32_T id_meas;                    /* '<Root>/id_meas' */
  real32_T iq_meas;                    /* '<Root>/iq_meas' */
  real32_T vd_meas;                    /* '<Root>/vd_meas' */
  real32_T vq_meas;                    /* '<Root>/vq_meas' */
  real32_T accel_meas_rps2;            /* '<Root>/accel_meas_rps2' */
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
extern MotorCfgBus MotorCfg;           /* Variable: MotorCfg
                                        * Referenced by:
                                        *   '<S123>/G_MFlux'
                                        *   '<S123>/G_MLd'
                                        *   '<S123>/G_MLq'
                                        *   '<S248>/ESO_C_b0'
                                        *   '<S248>/ESO_C_b1'
                                        *   '<S248>/ESO_G_b0'
                                        *   '<S248>/ESO_G_b1'
                                        *   '<S248>/ESO_G_b2'
                                        *   '<S248>/ESO_G_inv'
                                        *   '<S248>/G_FFB'
                                        *   '<S248>/G_FFFc'
                                        *   '<S248>/G_FFJ'
                                        * 固件打包传入：电机参数 + 电流/速度前馈系数 + LESO + 规划时间
                                        */
extern real32_T CoggingFF_En;          /* Variable: CoggingFF_En
                                        * Referenced by: '<S126>/Const_CogEn'
                                        * 齿槽转矩前馈使能(0/1)
                                        */
extern real32_T Cogging_Lut_Angle[360];/* Variable: Cogging_Lut_Angle
                                        * Referenced by: '<S126>/LUT_cog'
                                        * 齿槽前馈LUT 电角度断点(rad)
                                        */
extern real32_T Cogging_Lut_V[360];    /* Variable: Cogging_Lut_V
                                        * Referenced by: '<S126>/LUT_cog'
                                        * 齿槽前馈LUT iq补偿(A)
                                        */
extern real32_T CurrD_Ki;              /* Variable: CurrD_Ki
                                        * Referenced by: '<S169>/Integral Gain'
                                        * d轴电流环积分增益
                                        */
extern real32_T CurrD_Kp;              /* Variable: CurrD_Kp
                                        * Referenced by: '<S177>/Proportional Gain'
                                        * d轴电流环比例增益
                                        */
extern real32_T CurrD_MaxOut;          /* Variable: CurrD_MaxOut
                                        * Referenced by:
                                        *   '<S172>/Integrator'
                                        *   '<S179>/Saturation'
                                        * d轴电流环输出上限(V)
                                        */
extern real32_T CurrD_MinOut;          /* Variable: CurrD_MinOut
                                        * Referenced by:
                                        *   '<S172>/Integrator'
                                        *   '<S179>/Saturation'
                                        * d轴电流环输出下限(V)
                                        */
extern real32_T CurrQ_Ki;              /* Variable: CurrQ_Ki
                                        * Referenced by: '<S221>/Integral Gain'
                                        * q轴电流环积分增益
                                        */
extern real32_T CurrQ_Kp;              /* Variable: CurrQ_Kp
                                        * Referenced by: '<S229>/Proportional Gain'
                                        * q轴电流环比例增益
                                        */
extern real32_T Curr_MaxOut;           /* Variable: Curr_MaxOut
                                        * Referenced by:
                                        *   '<S224>/Integrator'
                                        *   '<S231>/Saturation'
                                        * 电流环输出上限(V)
                                        */
extern real32_T Curr_MinOut;           /* Variable: Curr_MinOut
                                        * Referenced by:
                                        *   '<S224>/Integrator'
                                        *   '<S231>/Saturation'
                                        * 电流环输出下限(V)
                                        */
extern real32_T DeadComp_En;           /* Variable: DeadComp_En
                                        * Referenced by: '<S124>/Const_DeadEn'
                                        * 死区补偿使能(0/1)
                                        */
extern real32_T DeadComp_Lut_I[20];    /* Variable: DeadComp_Lut_I
                                        * Referenced by: '<S124>/LUT_dead'
                                        * 死区补偿LUT 电流断点(A)
                                        */
extern real32_T DeadComp_Lut_V[20];    /* Variable: DeadComp_Lut_V
                                        * Referenced by: '<S124>/LUT_dead'
                                        * 死区补偿LUT 补偿电压(V)
                                        */
extern real32_T InvTwoPiPolePairs;     /* Variable: InvTwoPiPolePairs
                                        * Referenced by: '<S2>/G_mech_rps'
                                        * 电角速度->机械RPS 1/(2*pi*PolePairs)
                                        */
extern real32_T InvVbus;               /* Variable: InvVbus
                                        * Referenced by:
                                        *   '<S128>/G_1_vbus'
                                        *   '<S128>/G_ubeta'
                                        * 母线电压倒数 1/Vbus
                                        */
extern real32_T Pos_Kd;                /* Variable: Pos_Kd
                                        * Referenced by: '<S277>/Derivative Gain'
                                        * 位置环微分增益(预留, 先0)
                                        */
extern real32_T Pos_Kp;                /* Variable: Pos_Kp
                                        * Referenced by: '<S289>/Proportional Gain'
                                        * 位置环比例增益(RPS/rad)
                                        */
extern real32_T Pos_MaxOut;            /* Variable: Pos_MaxOut
                                        * Referenced by: '<S291>/Saturation'
                                        * 位置环输出上限(RPS, 作速度环参考)
                                        */
extern real32_T Pos_MinOut;            /* Variable: Pos_MinOut
                                        * Referenced by: '<S291>/Saturation'
                                        * 位置环输出下限(RPS)
                                        */
extern real32_T Speed_Ki;              /* Variable: Speed_Ki
                                        * Referenced by: '<S334>/Integral Gain'
                                        * 速度环积分增益(1kHz+UseI*Ts 每周期增量)
                                        */
extern real32_T Speed_Kp;              /* Variable: Speed_Kp
                                        * Referenced by: '<S342>/Proportional Gain'
                                        * 速度环比例增益
                                        */
extern real32_T Speed_MaxOut;          /* Variable: Speed_MaxOut
                                        * Referenced by: '<S344>/Saturation'
                                        * 速度环输出上限(A, iq_ref)
                                        */
extern real32_T Speed_MinOut;          /* Variable: Speed_MinOut
                                        * Referenced by: '<S344>/Saturation'
                                        * 速度环输出下限(A, iq_ref)
                                        */
extern real32_T VmaxCoeff;             /* Variable: VmaxCoeff
                                        * Referenced by: '<S130>/G_vmax'
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
 * Block '<S4>/Data Type Duplicate' : Unused code path elimination
 * Block '<S4>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S4>/Data Type Conversion' : Unused code path elimination
 * Block '<S4>/Data Type Conversion1' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S8>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S66>/I1CnstDT' : Unused code path elimination
 * Block '<S66>/I2CnstDT' : Unused code path elimination
 * Block '<S74>/Bias' : Unused code path elimination
 * Block '<S74>/Bias1' : Unused code path elimination
 * Block '<S74>/Constant' : Unused code path elimination
 * Block '<S74>/Data Type Duplicate' : Unused code path elimination
 * Block '<S74>/Data Type Propagation' : Unused code path elimination
 * Block '<S74>/Data Type Propagation1' : Unused code path elimination
 * Block '<S74>/Divide' : Unused code path elimination
 * Block '<S74>/Divide1' : Unused code path elimination
 * Block '<S74>/OD1Cnst' : Unused code path elimination
 * Block '<S74>/OD1CnstDT' : Unused code path elimination
 * Block '<S74>/Product' : Unused code path elimination
 * Block '<S74>/Unary Minus' : Unused code path elimination
 * Block '<S66>/MaxFreqCnst' : Unused code path elimination
 * Block '<S66>/OD1CnstDT' : Unused code path elimination
 * Block '<S66>/OD2CnstDT' : Unused code path elimination
 * Block '<S75>/Bias' : Unused code path elimination
 * Block '<S75>/Constant' : Unused code path elimination
 * Block '<S75>/Data Type Duplicate' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Propagation1' : Unused code path elimination
 * Block '<S75>/Data Type Propagation2' : Unused code path elimination
 * Block '<S75>/Divide' : Unused code path elimination
 * Block '<S75>/OD1Cnst' : Unused code path elimination
 * Block '<S75>/OD1CnstDT' : Unused code path elimination
 * Block '<S75>/OD1CnstDT1' : Unused code path elimination
 * Block '<S75>/Product' : Unused code path elimination
 * Block '<S75>/Product1' : Unused code path elimination
 * Block '<S75>/SampleTime' : Unused code path elimination
 * Block '<S76>/Const' : Unused code path elimination
 * Block '<S76>/Data Type Propagation' : Unused code path elimination
 * Block '<S76>/Data Type Propagation1' : Unused code path elimination
 * Block '<S76>/MaxFreqDT' : Unused code path elimination
 * Block '<S76>/Product' : Unused code path elimination
 * Block '<S67>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Bias' : Unused code path elimination
 * Block '<S68>/Constant' : Unused code path elimination
 * Block '<S68>/Data Type Propagation' : Unused code path elimination
 * Block '<S68>/Data Type Propagation1' : Unused code path elimination
 * Block '<S68>/Divide' : Unused code path elimination
 * Block '<S68>/Product' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Propagation' : Unused code path elimination
 * Block '<S82>/Data Type Propagation1' : Unused code path elimination
 * Block '<S83>/Data Type Duplicate' : Unused code path elimination
 * Block '<S83>/Data Type Propagation' : Unused code path elimination
 * Block '<S83>/Data Type Propagation1' : Unused code path elimination
 * Block '<S72>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S91>/Data Type Duplicate' : Unused code path elimination
 * Block '<S91>/Data Type Propagation' : Unused code path elimination
 * Block '<S92>/Data Type Duplicate' : Unused code path elimination
 * Block '<S93>/Data Type Duplicate' : Unused code path elimination
 * Block '<S73>/Bias' : Unused code path elimination
 * Block '<S73>/Constant' : Unused code path elimination
 * Block '<S73>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Data Type Propagation1' : Unused code path elimination
 * Block '<S73>/Divide' : Unused code path elimination
 * Block '<S73>/Product' : Unused code path elimination
 * Block '<S9>/Data Type Duplicate' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S103>/Data Type Propagation' : Unused code path elimination
 * Block '<S104>/BooleanTypeCast' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate3' : Unused code path elimination
 * Block '<S104>/Data Type Propagation' : Unused code path elimination
 * Block '<S104>/InpGainDatatype' : Unused code path elimination
 * Block '<S104>/OutGainDatatype' : Unused code path elimination
 * Block '<S104>/Reset' : Unused code path elimination
 * Block '<S106>/Data Type Duplicate' : Unused code path elimination
 * Block '<S106>/Data Type Propagation' : Unused code path elimination
 * Block '<S111>/Data Type Duplicate' : Unused code path elimination
 * Block '<S112>/Data Type Duplicate' : Unused code path elimination
 * Block '<S114>/Data Type Duplicate' : Unused code path elimination
 * Block '<S114>/Data Type Propagation' : Unused code path elimination
 * Block '<S119>/Data Type Duplicate' : Unused code path elimination
 * Block '<S120>/Data Type Duplicate' : Unused code path elimination
 * Block '<S135>/Data Type Duplicate' : Unused code path elimination
 * Block '<S245>/Data Type Duplicate' : Unused code path elimination
 * Block '<S245>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S358>/Data Type Duplicate' : Unused code path elimination
 * Block '<S358>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S360>/Data Type Duplicate' : Unused code path elimination
 * Block '<S360>/Data Type Propagation' : Unused code path elimination
 * Block '<S365>/Data Type Duplicate' : Unused code path elimination
 * Block '<S366>/Data Type Duplicate' : Unused code path elimination
 * Block '<S4>/FreqGain' : Eliminated nontunable gain of 1
 * Block '<S6>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S8>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S67>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S80>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S81>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S92>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S93>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S103>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S103>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S103>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S106>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S108>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S113>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S114>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S121>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S134>/Kalpha' : Eliminated nontunable gain of 1
 * Block '<S134>/Kbeta' : Eliminated nontunable gain of 1
 * Block '<S164>/Kb' : Eliminated nontunable gain of 1
 * Block '<S216>/Kb' : Eliminated nontunable gain of 1
 * Block '<S329>/Kb' : Eliminated nontunable gain of 1
 * Block '<S360>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S367>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S6>/One' : Unused code path elimination
 * Block '<S6>/Reset' : Unused code path elimination
 * Block '<S6>/Sum' : Unused code path elimination
 * Block '<S6>/UseInputPort' : Unused code path elimination
 * Block '<S6>/a' : Unused code path elimination
 * Block '<S67>/FilterConstant' : Unused code path elimination
 * Block '<S67>/OneMinusFilterConstant' : Unused code path elimination
 * Block '<S67>/Reset' : Unused code path elimination
 * Block '<S67>/UseInputPort' : Unused code path elimination
 * Block '<S246>/Offset' : Unused code path elimination
 * Block '<S246>/Unary_Minus' : Unused code path elimination
 * Block '<S359>/Offset' : Unused code path elimination
 * Block '<S359>/Unary_Minus' : Unused code path elimination
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
 * '<S1>'   : 'CyberDog_Motor_FOC/AccelEst'
 * '<S2>'   : 'CyberDog_Motor_FOC/EncoderPLL'
 * '<S3>'   : 'CyberDog_Motor_FOC/FOC_Algorithm'
 * '<S4>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF'
 * '<S5>'   : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele'
 * '<S6>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter'
 * '<S7>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller'
 * '<S8>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward'
 * '<S9>'   : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem'
 * '<S10>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO'
 * '<S11>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter'
 * '<S12>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter/Low-pass'
 * '<S13>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S14>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Anti-windup'
 * '<S15>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/D Gain'
 * '<S16>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/External Derivative'
 * '<S17>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter'
 * '<S18>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter ICs'
 * '<S19>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/I Gain'
 * '<S20>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain'
 * '<S21>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain Fdbk'
 * '<S22>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator'
 * '<S23>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator ICs'
 * '<S24>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Copy'
 * '<S25>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Gain'
 * '<S26>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/P Copy'
 * '<S27>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Parallel P Gain'
 * '<S28>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Reset Signal'
 * '<S29>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation'
 * '<S30>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation Fdbk'
 * '<S31>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum'
 * '<S32>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum Fdbk'
 * '<S33>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode'
 * '<S34>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode Sum'
 * '<S35>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Integral'
 * '<S36>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Ngain'
 * '<S37>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/postSat Signal'
 * '<S38>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preInt Signal'
 * '<S39>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preSat Signal'
 * '<S40>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Anti-windup/Passthrough'
 * '<S41>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/D Gain/Disabled'
 * '<S42>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/External Derivative/Disabled'
 * '<S43>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter/Disabled'
 * '<S44>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Filter ICs/Disabled'
 * '<S45>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/I Gain/Internal Parameters'
 * '<S46>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain/Passthrough'
 * '<S47>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator/Discrete'
 * '<S49>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Integrator ICs/Internal IC'
 * '<S50>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S51>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/N Gain/Disabled'
 * '<S52>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/P Copy/Disabled'
 * '<S53>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S54>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Reset Signal/External Reset'
 * '<S55>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation/Passthrough'
 * '<S56>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Saturation Fdbk/Disabled'
 * '<S57>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum/Sum_PI'
 * '<S58>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Sum Fdbk/Disabled'
 * '<S59>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode/Disabled'
 * '<S60>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S61>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S62>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S63>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/postSat Signal/Forward_Path'
 * '<S64>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preInt Signal/Internal PreInt'
 * '<S65>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/PID Controller/preSat Signal/Forward_Path'
 * '<S66>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem'
 * '<S67>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter'
 * '<S68>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/InpFilterFunc'
 * '<S69>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod'
 * '<S70>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/Nofunc'
 * '<S71>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/NofuncSpd'
 * '<S72>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod'
 * '<S73>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/SpdFilterFunc'
 * '<S74>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/IGains'
 * '<S75>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/ODGains'
 * '<S76>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/GainSystem/OutDTSetup'
 * '<S77>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter'
 * '<S78>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass'
 * '<S79>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S80>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1'
 * '<S81>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3'
 * '<S82>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/Integrator'
 * '<S83>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/Integrator2'
 * '<S84>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter'
 * '<S85>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S86>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S87>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter'
 * '<S88>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S89>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S90>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator'
 * '<S91>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator1'
 * '<S92>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1'
 * '<S93>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3'
 * '<S94>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter'
 * '<S95>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S96>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S97>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter'
 * '<S98>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S99>'  : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S100>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem/If Action Subsystem'
 * '<S101>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/Subsystem/If Action Subsystem1'
 * '<S102>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem'
 * '<S103>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased'
 * '<S104>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Position Generator'
 * '<S105>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup'
 * '<S106>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S107>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S108>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S109>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S110>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S111>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S112>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S113>' : 'CyberDog_Motor_FOC/EncoderPLL/PLL_FF/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype backpropogation'
 * '<S114>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup'
 * '<S115>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/Interpolation'
 * '<S116>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp'
 * '<S117>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/datatype'
 * '<S118>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S119>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S120>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S121>' : 'CyberDog_Motor_FOC/EncoderPLL/SinCos_ele/Sine-Cosine Lookup/datatype/datatype backpropogation'
 * '<S122>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke'
 * '<S123>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController'
 * '<S124>' : 'CyberDog_Motor_FOC/FOC_Algorithm/DeadTimeComp'
 * '<S125>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark'
 * '<S126>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop'
 * '<S127>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park'
 * '<S128>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SVPWM'
 * '<S129>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th'
 * '<S130>' : 'CyberDog_Motor_FOC/FOC_Algorithm/VoltageLimiter'
 * '<S131>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant'
 * '<S132>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb'
 * '<S133>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform'
 * '<S134>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform/Two phase input'
 * '<S135>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Clarke/Variant/mcb/Clarke Transform/Two phase input/Two phase CRL wrap'
 * '<S136>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d'
 * '<S137>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q'
 * '<S138>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Anti-windup'
 * '<S139>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/D Gain'
 * '<S140>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/External Derivative'
 * '<S141>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter'
 * '<S142>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter ICs'
 * '<S143>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/I Gain'
 * '<S144>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain'
 * '<S145>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain Fdbk'
 * '<S146>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator'
 * '<S147>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator ICs'
 * '<S148>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Copy'
 * '<S149>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Gain'
 * '<S150>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/P Copy'
 * '<S151>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Parallel P Gain'
 * '<S152>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Reset Signal'
 * '<S153>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation'
 * '<S154>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation Fdbk'
 * '<S155>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum'
 * '<S156>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum Fdbk'
 * '<S157>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode'
 * '<S158>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode Sum'
 * '<S159>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Integral'
 * '<S160>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Ngain'
 * '<S161>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/postSat Signal'
 * '<S162>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preInt Signal'
 * '<S163>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preSat Signal'
 * '<S164>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Anti-windup/Back Calculation'
 * '<S165>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/D Gain/Disabled'
 * '<S166>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/External Derivative/Disabled'
 * '<S167>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter/Disabled'
 * '<S168>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Filter ICs/Disabled'
 * '<S169>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/I Gain/Internal Parameters'
 * '<S170>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain/Passthrough'
 * '<S171>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Ideal P Gain Fdbk/Disabled'
 * '<S172>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator/Discrete'
 * '<S173>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Integrator ICs/Internal IC'
 * '<S174>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Copy/Disabled wSignal Specification'
 * '<S175>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/N Gain/Disabled'
 * '<S176>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/P Copy/Disabled'
 * '<S177>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Parallel P Gain/Internal Parameters'
 * '<S178>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Reset Signal/Disabled'
 * '<S179>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation/Enabled'
 * '<S180>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Saturation Fdbk/Disabled'
 * '<S181>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum/Sum_PI'
 * '<S182>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Sum Fdbk/Disabled'
 * '<S183>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode/Disabled'
 * '<S184>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tracking Mode Sum/Passthrough'
 * '<S185>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Integral/TsSignalSpecification'
 * '<S186>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/Tsamp - Ngain/Passthrough'
 * '<S187>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/postSat Signal/Forward_Path'
 * '<S188>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preInt Signal/Internal PreInt'
 * '<S189>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_d/preSat Signal/Forward_Path'
 * '<S190>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Anti-windup'
 * '<S191>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/D Gain'
 * '<S192>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/External Derivative'
 * '<S193>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter'
 * '<S194>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter ICs'
 * '<S195>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/I Gain'
 * '<S196>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain'
 * '<S197>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain Fdbk'
 * '<S198>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator'
 * '<S199>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator ICs'
 * '<S200>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Copy'
 * '<S201>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Gain'
 * '<S202>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/P Copy'
 * '<S203>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Parallel P Gain'
 * '<S204>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Reset Signal'
 * '<S205>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation'
 * '<S206>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation Fdbk'
 * '<S207>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum'
 * '<S208>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum Fdbk'
 * '<S209>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode'
 * '<S210>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode Sum'
 * '<S211>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Integral'
 * '<S212>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Ngain'
 * '<S213>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/postSat Signal'
 * '<S214>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preInt Signal'
 * '<S215>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preSat Signal'
 * '<S216>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Anti-windup/Back Calculation'
 * '<S217>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/D Gain/Disabled'
 * '<S218>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/External Derivative/Disabled'
 * '<S219>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter/Disabled'
 * '<S220>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Filter ICs/Disabled'
 * '<S221>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/I Gain/Internal Parameters'
 * '<S222>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain/Passthrough'
 * '<S223>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Ideal P Gain Fdbk/Disabled'
 * '<S224>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator/Discrete'
 * '<S225>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Integrator ICs/Internal IC'
 * '<S226>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Copy/Disabled wSignal Specification'
 * '<S227>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/N Gain/Disabled'
 * '<S228>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/P Copy/Disabled'
 * '<S229>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Parallel P Gain/Internal Parameters'
 * '<S230>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Reset Signal/Disabled'
 * '<S231>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation/Enabled'
 * '<S232>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Saturation Fdbk/Disabled'
 * '<S233>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum/Sum_PI'
 * '<S234>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Sum Fdbk/Disabled'
 * '<S235>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode/Disabled'
 * '<S236>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tracking Mode Sum/Passthrough'
 * '<S237>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Integral/TsSignalSpecification'
 * '<S238>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/Tsamp - Ngain/Passthrough'
 * '<S239>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/postSat Signal/Forward_Path'
 * '<S240>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preInt Signal/Internal PreInt'
 * '<S241>' : 'CyberDog_Motor_FOC/FOC_Algorithm/CurrentController/PID_q/preSat Signal/Forward_Path'
 * '<S242>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant'
 * '<S243>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb'
 * '<S244>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform'
 * '<S245>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform/Two inputs CRL'
 * '<S246>' : 'CyberDog_Motor_FOC/FOC_Algorithm/InvPark/Variant/mcb/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S247>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController'
 * '<S248>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController'
 * '<S249>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos'
 * '<S250>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Anti-windup'
 * '<S251>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/D Gain'
 * '<S252>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/External Derivative'
 * '<S253>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Filter'
 * '<S254>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Filter ICs'
 * '<S255>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/I Gain'
 * '<S256>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Ideal P Gain'
 * '<S257>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Ideal P Gain Fdbk'
 * '<S258>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Integrator'
 * '<S259>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Integrator ICs'
 * '<S260>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/N Copy'
 * '<S261>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/N Gain'
 * '<S262>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/P Copy'
 * '<S263>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Parallel P Gain'
 * '<S264>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Reset Signal'
 * '<S265>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Saturation'
 * '<S266>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Saturation Fdbk'
 * '<S267>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Sum'
 * '<S268>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Sum Fdbk'
 * '<S269>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tracking Mode'
 * '<S270>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tracking Mode Sum'
 * '<S271>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tsamp - Integral'
 * '<S272>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tsamp - Ngain'
 * '<S273>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/postSat Signal'
 * '<S274>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/preInt Signal'
 * '<S275>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/preSat Signal'
 * '<S276>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Anti-windup/Disabled'
 * '<S277>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/D Gain/Internal Parameters'
 * '<S278>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/External Derivative/Error'
 * '<S279>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Filter/Disc. Forward Euler Filter'
 * '<S280>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Filter ICs/Internal IC - Filter'
 * '<S281>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/I Gain/Disabled'
 * '<S282>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Ideal P Gain/Passthrough'
 * '<S283>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Ideal P Gain Fdbk/Disabled'
 * '<S284>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Integrator/Disabled'
 * '<S285>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Integrator ICs/Disabled'
 * '<S286>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/N Copy/Disabled'
 * '<S287>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/N Gain/Internal Parameters'
 * '<S288>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/P Copy/Disabled'
 * '<S289>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Parallel P Gain/Internal Parameters'
 * '<S290>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Reset Signal/Disabled'
 * '<S291>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Saturation/Enabled'
 * '<S292>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Saturation Fdbk/Disabled'
 * '<S293>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Sum/Sum_PD'
 * '<S294>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Sum Fdbk/Disabled'
 * '<S295>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tracking Mode/Disabled'
 * '<S296>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tracking Mode Sum/Passthrough'
 * '<S297>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tsamp - Integral/TsSignalSpecification'
 * '<S298>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/Tsamp - Ngain/Passthrough'
 * '<S299>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/postSat Signal/Forward_Path'
 * '<S300>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/preInt Signal/Internal PreInt'
 * '<S301>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/PositionController/PID_pos/preSat Signal/Forward_Path'
 * '<S302>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed'
 * '<S303>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Anti-windup'
 * '<S304>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/D Gain'
 * '<S305>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/External Derivative'
 * '<S306>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Filter'
 * '<S307>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Filter ICs'
 * '<S308>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/I Gain'
 * '<S309>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Ideal P Gain'
 * '<S310>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Ideal P Gain Fdbk'
 * '<S311>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Integrator'
 * '<S312>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Integrator ICs'
 * '<S313>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/N Copy'
 * '<S314>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/N Gain'
 * '<S315>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/P Copy'
 * '<S316>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Parallel P Gain'
 * '<S317>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Reset Signal'
 * '<S318>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Saturation'
 * '<S319>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Saturation Fdbk'
 * '<S320>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Sum'
 * '<S321>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Sum Fdbk'
 * '<S322>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tracking Mode'
 * '<S323>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tracking Mode Sum'
 * '<S324>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tsamp - Integral'
 * '<S325>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tsamp - Ngain'
 * '<S326>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/postSat Signal'
 * '<S327>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/preInt Signal'
 * '<S328>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/preSat Signal'
 * '<S329>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Anti-windup/Back Calculation'
 * '<S330>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/D Gain/Disabled'
 * '<S331>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/External Derivative/Disabled'
 * '<S332>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Filter/Disabled'
 * '<S333>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Filter ICs/Disabled'
 * '<S334>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/I Gain/Internal Parameters'
 * '<S335>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Ideal P Gain/Passthrough'
 * '<S336>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Ideal P Gain Fdbk/Disabled'
 * '<S337>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Integrator/Discrete'
 * '<S338>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Integrator ICs/Internal IC'
 * '<S339>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/N Copy/Disabled wSignal Specification'
 * '<S340>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/N Gain/Disabled'
 * '<S341>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/P Copy/Disabled'
 * '<S342>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Parallel P Gain/Internal Parameters'
 * '<S343>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Reset Signal/Disabled'
 * '<S344>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Saturation/Enabled'
 * '<S345>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Saturation Fdbk/Disabled'
 * '<S346>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Sum/Sum_PI'
 * '<S347>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Sum Fdbk/Disabled'
 * '<S348>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tracking Mode/Disabled'
 * '<S349>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tracking Mode Sum/Passthrough'
 * '<S350>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tsamp - Integral/TsSignalSpecification'
 * '<S351>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/Tsamp - Ngain/Passthrough'
 * '<S352>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/postSat Signal/Forward_Path'
 * '<S353>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/preInt Signal/Internal PreInt'
 * '<S354>' : 'CyberDog_Motor_FOC/FOC_Algorithm/OuterLoop/SpeedController/PID_speed/preSat Signal/Forward_Path'
 * '<S355>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant'
 * '<S356>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb'
 * '<S357>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform'
 * '<S358>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform/Two inputs CRL'
 * '<S359>' : 'CyberDog_Motor_FOC/FOC_Algorithm/Park/Variant/mcb/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S360>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup'
 * '<S361>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/Interpolation'
 * '<S362>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp'
 * '<S363>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/datatype'
 * '<S364>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S365>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S366>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S367>' : 'CyberDog_Motor_FOC/FOC_Algorithm/SinCos_th/Sine-Cosine Lookup/datatype/datatype backpropogation'
 */
#endif                                 /* CyberDog_Motor_FOC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
