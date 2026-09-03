/*
 * File: CyberDog_Motor_FOC.c
 *
 * Code generated for Simulink model 'CyberDog_Motor_FOC'.
 *
 * Model version                  : 1.36
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Mon Aug 31 00:45:23 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CyberDog_Motor_FOC.h"
#include "rtwtypes.h"
#include "CyberDog_Motor_FOC_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "zero_crossing_types.h"

/* Exported block parameters */
real32_T CoggingFF_En = 0.0F;          /* Variable: CoggingFF_En
                                        * Referenced by: '<S125>/Const_CogEn'
                                        * 齿槽转矩前馈使能(0/1)
                                        */
real32_T Cogging_Lut_Angle[360] = { 0.0F, 0.0175019093F, 0.0350038186F,
  0.0525057279F, 0.0700076371F, 0.0875095427F, 0.105011456F, 0.122513361F,
  0.140015274F, 0.15751718F, 0.175019085F, 0.192520991F, 0.210022911F,
  0.227524817F, 0.245026723F, 0.262528628F, 0.280030549F, 0.297532439F,
  0.31503436F, 0.33253628F, 0.350038171F, 0.367540091F, 0.385041982F,
  0.402543902F, 0.420045823F, 0.437547714F, 0.455049634F, 0.472551554F,
  0.490053445F, 0.507555366F, 0.525057256F, 0.542559206F, 0.560061097F,
  0.577563F, 0.595064878F, 0.612566829F, 0.630068719F, 0.64757061F, 0.66507256F,
  0.682574451F, 0.700076342F, 0.717578292F, 0.735080183F, 0.752582073F,
  0.770083964F, 0.787585914F, 0.805087805F, 0.822589695F, 0.840091646F,
  0.857593536F, 0.875095427F, 0.892597377F, 0.910099268F, 0.927601159F,
  0.945103109F, 0.962605F, 0.98010689F, 0.997608781F, 1.01511073F, 1.03261268F,
  1.05011451F, 1.06761646F, 1.08511841F, 1.10262024F, 1.12012219F, 1.13762403F,
  1.15512598F, 1.17262793F, 1.19012976F, 1.20763171F, 1.22513366F, 1.24263549F,
  1.26013744F, 1.27763939F, 1.29514122F, 1.31264317F, 1.33014512F, 1.34764695F,
  1.3651489F, 1.38265085F, 1.40015268F, 1.41765463F, 1.43515658F, 1.45265841F,
  1.47016037F, 1.48766232F, 1.50516415F, 1.5226661F, 1.54016793F, 1.55766988F,
  1.57517183F, 1.59267366F, 1.61017561F, 1.62767756F, 1.64517939F, 1.66268134F,
  1.68018329F, 1.69768512F, 1.71518707F, 1.73268902F, 1.75019085F, 1.7676928F,
  1.78519475F, 1.80269659F, 1.82019854F, 1.83770049F, 1.85520232F, 1.87270427F,
  1.89020622F, 1.90770805F, 1.92521F, 1.94271195F, 1.96021378F, 1.97771573F,
  1.99521756F, 2.01271963F, 2.03022146F, 2.04772329F, 2.06522536F, 2.08272719F,
  2.10022902F, 2.11773109F, 2.13523293F, 2.15273476F, 2.17023683F, 2.18773866F,
  2.20524049F, 2.22274232F, 2.24024439F, 2.25774622F, 2.27524805F, 2.29275012F,
  2.31025195F, 2.32775378F, 2.34525585F, 2.36275768F, 2.38025951F, 2.39776158F,
  2.41526341F, 2.43276525F, 2.45026731F, 2.46776915F, 2.48527098F, 2.50277305F,
  2.52027488F, 2.53777671F, 2.55527878F, 2.57278061F, 2.59028244F, 2.60778451F,
  2.62528634F, 2.64278817F, 2.66029024F, 2.67779207F, 2.6952939F, 2.71279597F,
  2.7302978F, 2.74779963F, 2.7653017F, 2.78280354F, 2.80030537F, 2.81780744F,
  2.83530927F, 2.8528111F, 2.87031317F, 2.887815F, 2.90531683F, 2.9228189F,
  2.94032073F, 2.95782256F, 2.97532463F, 2.99282646F, 3.01032829F, 3.02783036F,
  3.04533219F, 3.06283402F, 3.08033586F, 3.09783792F, 3.11533976F, 3.13284159F,
  3.15034366F, 3.16784549F, 3.18534732F, 3.20284939F, 3.22035122F, 3.23785305F,
  3.25535512F, 3.27285695F, 3.29035878F, 3.30786085F, 3.32536268F, 3.34286451F,
  3.36036658F, 3.37786841F, 3.39537024F, 3.41287231F, 3.43037415F, 3.44787598F,
  3.46537805F, 3.48287988F, 3.50038171F, 3.51788378F, 3.53538561F, 3.55288744F,
  3.57038951F, 3.58789134F, 3.60539317F, 3.62289524F, 3.64039707F, 3.6578989F,
  3.67540097F, 3.6929028F, 3.71040463F, 3.7279067F, 3.74540854F, 3.76291037F,
  3.78041244F, 3.79791427F, 3.8154161F, 3.83291817F, 3.85042F, 3.86792183F,
  3.8854239F, 3.90292573F, 3.92042756F, 3.93792939F, 3.95543146F, 3.97293329F,
  3.99043512F, 4.00793695F, 4.02543926F, 4.04294109F, 4.06044292F, 4.07794476F,
  4.09544659F, 4.11294842F, 4.13045073F, 4.14795256F, 4.16545439F, 4.18295622F,
  4.20045805F, 4.21796F, 4.23546219F, 4.25296402F, 4.27046585F, 4.28796768F,
  4.30546951F, 4.32297134F, 4.34047365F, 4.35797548F, 4.37547731F, 4.39297915F,
  4.41048098F, 4.42798281F, 4.44548464F, 4.46298695F, 4.48048878F, 4.49799061F,
  4.51549244F, 4.53299427F, 4.5504961F, 4.56799841F, 4.58550024F, 4.60300207F,
  4.6205039F, 4.63800573F, 4.65550756F, 4.67301F, 4.6905117F, 4.70801353F,
  4.72551537F, 4.7430172F, 4.76051903F, 4.77802134F, 4.79552317F, 4.813025F,
  4.83052683F, 4.84802866F, 4.86553049F, 4.8830328F, 4.90053463F, 4.91803646F,
  4.93553829F, 4.95304F, 4.97054195F, 4.98804426F, 5.00554609F, 5.02304792F,
  5.04054976F, 5.05805159F, 5.07555342F, 5.09305573F, 5.11055756F, 5.12805939F,
  5.14556122F, 5.16306305F, 5.18056488F, 5.19806719F, 5.21556902F, 5.23307085F,
  5.25057268F, 5.26807451F, 5.28557634F, 5.30307817F, 5.32058048F, 5.33808231F,
  5.35558414F, 5.37308598F, 5.39058781F, 5.40808964F, 5.42559195F, 5.44309378F,
  5.46059561F, 5.47809744F, 5.49559927F, 5.5131011F, 5.53060341F, 5.54810524F,
  5.56560707F, 5.5831089F, 5.60061073F, 5.61811256F, 5.63561487F, 5.6531167F,
  5.67061853F, 5.68812037F, 5.7056222F, 5.72312403F, 5.74062634F, 5.75812817F,
  5.77563F, 5.79313183F, 5.81063366F, 5.82813549F, 5.8456378F, 5.86313963F,
  5.88064146F, 5.89814329F, 5.91564512F, 5.93314695F, 5.95064926F, 5.96815109F,
  5.98565292F, 6.00315475F, 6.02065659F, 6.03815842F, 6.05566072F, 6.07316256F,
  6.09066439F, 6.10816622F, 6.12566805F, 6.14317F, 6.16067171F, 6.17817402F,
  6.19567585F, 6.21317768F, 6.23067951F, 6.24818134F, 6.26568317F, 6.28318548F }
;                                      /* Variable: Cogging_Lut_Angle
                                        * Referenced by: '<S125>/LUT_cog'
                                        * 齿槽前馈LUT 电角度断点(rad)
                                        */

real32_T Cogging_Lut_V[360] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F } ;                             /* Variable: Cogging_Lut_V
                                        * Referenced by: '<S125>/LUT_cog'
                                        * 齿槽前馈LUT iq补偿(A)
                                        */

real32_T CurrD_Ki = 0.0618F;           /* Variable: CurrD_Ki
                                        * Referenced by: '<S168>/Integral Gain'
                                        * d轴电流环积分增益
                                        */
real32_T CurrD_Kp = 0.618F;            /* Variable: CurrD_Kp
                                        * Referenced by: '<S176>/Proportional Gain'
                                        * d轴电流环比例增益
                                        */
real32_T CurrD_MaxOut = 3.0F;          /* Variable: CurrD_MaxOut
                                        * Referenced by:
                                        *   '<S171>/Integrator'
                                        *   '<S178>/Saturation'
                                        * d轴电流环输出上限(V)
                                        */
real32_T CurrD_MinOut = -3.0F;         /* Variable: CurrD_MinOut
                                        * Referenced by:
                                        *   '<S171>/Integrator'
                                        *   '<S178>/Saturation'
                                        * d轴电流环输出下限(V)
                                        */
real32_T CurrQ_Ki = 0.0618F;           /* Variable: CurrQ_Ki
                                        * Referenced by: '<S220>/Integral Gain'
                                        * q轴电流环积分增益
                                        */
real32_T CurrQ_Kp = 0.618F;            /* Variable: CurrQ_Kp
                                        * Referenced by: '<S228>/Proportional Gain'
                                        * q轴电流环比例增益
                                        */
real32_T Curr_MaxOut = 13.8564062F;    /* Variable: Curr_MaxOut
                                        * Referenced by:
                                        *   '<S223>/Integrator'
                                        *   '<S230>/Saturation'
                                        * 电流环输出上限(V)
                                        */
real32_T Curr_MinOut = -13.8564062F;   /* Variable: Curr_MinOut
                                        * Referenced by:
                                        *   '<S223>/Integrator'
                                        *   '<S230>/Saturation'
                                        * 电流环输出下限(V)
                                        */
real32_T DeadComp_En = 1.0F;           /* Variable: DeadComp_En
                                        * Referenced by: '<S123>/Const_DeadEn'
                                        * 死区补偿使能(0/1)
                                        */
real32_T DeadComp_Lut_I[20] = { 0.05F, 0.1F, 0.15F, 0.2F, 0.25F, 0.3F, 0.35F,
  0.4F, 0.45F, 0.5F, 0.6F, 0.7F, 0.8F, 0.9F, 1.0F, 1.1F, 1.2F, 1.3F, 1.4F, 1.5F
} ;                                    /* Variable: DeadComp_Lut_I
                                        * Referenced by: '<S123>/LUT_dead'
                                        * 死区补偿LUT 电流断点(A)
                                        */

real32_T DeadComp_Lut_V[20] = { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
  0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F } ;/* Variable: DeadComp_Lut_V
                                                                      * Referenced by: '<S123>/LUT_dead'
                                                                      * 死区补偿LUT 补偿电压(V)
                                                                      */

real32_T InvTwoPiPolePairs = 0.0227364209F;/* Variable: InvTwoPiPolePairs
                                            * Referenced by: '<S1>/G_mech_rps'
                                            * 电角速度->机械RPS 1/(2*pi*PolePairs)
                                            */
real32_T InvVbus = 0.0416666679F;      /* Variable: InvVbus
                                        * Referenced by:
                                        *   '<S127>/G_1_vbus'
                                        *   '<S127>/G_ubeta'
                                        * 母线电压倒数 1/Vbus
                                        */
real32_T Pos_Kd = 0.0F;                /* Variable: Pos_Kd
                                        * Referenced by: '<S276>/Derivative Gain'
                                        * 位置环微分增益(预留, 先0)
                                        */
real32_T Pos_Kp = 10.0F;               /* Variable: Pos_Kp
                                        * Referenced by: '<S288>/Proportional Gain'
                                        * 位置环比例增益(RPS/rad)
                                        */
real32_T Pos_MaxOut = 5.0F;            /* Variable: Pos_MaxOut
                                        * Referenced by: '<S290>/Saturation'
                                        * 位置环输出上限(RPS, 作速度环参考)
                                        */
real32_T Pos_MinOut = -5.0F;           /* Variable: Pos_MinOut
                                        * Referenced by: '<S290>/Saturation'
                                        * 位置环输出下限(RPS)
                                        */
real32_T Speed_Ki = 0.0006F;           /* Variable: Speed_Ki
                                        * Referenced by: '<S333>/Integral Gain'
                                        * 速度环积分增益(1kHz+UseI*Ts 每周期增量)
                                        */
real32_T Speed_Kp = 0.3F;              /* Variable: Speed_Kp
                                        * Referenced by: '<S341>/Proportional Gain'
                                        * 速度环比例增益
                                        */
real32_T Speed_MaxOut = 5.0F;          /* Variable: Speed_MaxOut
                                        * Referenced by: '<S343>/Saturation'
                                        * 速度环输出上限(A, iq_ref)
                                        */
real32_T Speed_MinOut = -5.0F;         /* Variable: Speed_MinOut
                                        * Referenced by: '<S343>/Saturation'
                                        * 速度环输出下限(A, iq_ref)
                                        */
real32_T VmaxCoeff = 13.1635857F;      /* Variable: VmaxCoeff
                                        * Referenced by: '<S129>/G_vmax'
                                        * 电压限幅系数 Vbus/√3·MaxMod
                                        */

/* Block signals (default storage) */
B_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_B;

/* Block states (default storage) */
DW_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_PrevZCX;

/* External inputs (root inport signals with default storage) */
ExtU_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_Y;

/* Real-time model */
static RT_MODEL_CyberDog_Motor_FOC_T CyberDog_Motor_FOC_M_;
RT_MODEL_CyberDog_Motor_FOC_T *const CyberDog_Motor_FOC_M =
  &CyberDog_Motor_FOC_M_;
static void rate_scheduler(void);
real32_T look1_iflf_binlxpw(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
{
  real32_T frac;
  real32_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (CyberDog_Motor_FOC_M->Timing.TaskCounters.TID[1])++;
  if ((CyberDog_Motor_FOC_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    CyberDog_Motor_FOC_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/*
 * Output and update for action system:
 *    '<S107>/If Action Subsystem'
 *    '<S115>/If Action Subsystem'
 *    '<S361>/If Action Subsystem'
 */
real32_T CyberDog_Moto_IfActionSubsystem(real32_T rtu_In1)
{
  int32_T tmp_0;
  real32_T tmp;

  /* DataTypeConversion: '<S110>/Convert_uint16' */
  tmp = floorf(rtu_In1);
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = fmodf(tmp, 65536.0F);
  }

  if (tmp < 0.0F) {
    tmp_0 = (int16_T)-(int16_T)(uint16_T)-tmp;
  } else {
    tmp_0 = (int16_T)(uint16_T)tmp;
  }

  /* Sum: '<S110>/Sum' incorporates:
   *  DataTypeConversion: '<S110>/Convert_back'
   *  DataTypeConversion: '<S110>/Convert_uint16'
   */
  return rtu_In1 - (real32_T)tmp_0;
}

/*
 * Output and update for action system:
 *    '<S107>/If Action Subsystem1'
 *    '<S115>/If Action Subsystem1'
 *    '<S361>/If Action Subsystem1'
 */
real32_T CyberDog_Mot_IfActionSubsystem1(real32_T rtu_In1)
{
  int32_T tmp_0;
  real32_T tmp;

  /* DataTypeConversion: '<S111>/Convert_uint16' */
  tmp = truncf(rtu_In1);
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = fmodf(tmp, 65536.0F);
  }

  if (tmp < 0.0F) {
    tmp_0 = (int16_T)-(int16_T)(uint16_T)-tmp;
  } else {
    tmp_0 = (int16_T)(uint16_T)tmp;
  }

  /* Sum: '<S111>/Sum' incorporates:
   *  DataTypeConversion: '<S111>/Convert_back'
   *  DataTypeConversion: '<S111>/Convert_uint16'
   */
  return rtu_In1 - (real32_T)tmp_0;
}

/* Model step function */
void CyberDog_Motor_FOC_step(void)
{
  real32_T rtb_FilterCoefficient;
  real32_T rtb_Integrator_l;
  real32_T rtb_LUT_dead;
  real32_T rtb_MM_max;
  real32_T rtb_Merge_idx_0;
  real32_T rtb_Merge_idx_1;
  real32_T rtb_Saturation_h;
  real32_T rtb_Sum4_m;
  real32_T rtb_Sum6_f;
  real32_T rtb_SumI4_m;
  real32_T rtb_Sum_g;
  real32_T rtb_Sw_cog;
  real32_T rtb_Sw_dVd;
  real32_T rtb_Switch1_c_idx_0;
  real32_T rtb_Switch1_c_idx_1;
  real32_T rtb_convert_pu;
  uint16_T rtb_Get_Integer;
  boolean_T rtb_Rel_dead;
  boolean_T rtb_Rel_dqvol;
  boolean_T tmp;

  /* RelationalOperator: '<S2>/Rel_dqvol' incorporates:
   *  Constant: '<S2>/Const_3'
   *  Inport: '<Root>/ctrl_mode'
   */
  rtb_Rel_dqvol = (CyberDog_Motor_FOC_U.ctrl_mode == 3.0F);

  /* Outputs for Atomic SubSystem: '<S133>/Two phase CRL wrap' */
  /* Gain: '<S134>/one_by_sqrt3' incorporates:
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Sum: '<S134>/a_plus_2b'
   */
  rtb_MM_max = ((CyberDog_Motor_FOC_U.ia + CyberDog_Motor_FOC_U.ib) +
                CyberDog_Motor_FOC_U.ib) * 0.577350259F;

  /* Outport: '<Root>/i_beta_meas' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S134>/a16'
   */
  CyberDog_Motor_FOC_Y.i_beta_meas = rtb_MM_max;

  /* End of Outputs for SubSystem: '<S133>/Two phase CRL wrap' */

  /* Gain: '<S115>/convert_pu' incorporates:
   *  Inport: '<Root>/eleangle'
   */
  rtb_convert_pu = 0.159154937F * CyberDog_Motor_FOC_U.eleangle;

  /* If: '<S115>/If' incorporates:
   *  Constant: '<S117>/Constant'
   *  RelationalOperator: '<S117>/Compare'
   */
  if (rtb_convert_pu < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S115>/If Action Subsystem' incorporates:
     *  ActionPort: '<S118>/Action Port'
     */
    rtb_convert_pu = CyberDog_Moto_IfActionSubsystem(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S115>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S115>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S119>/Action Port'
     */
    rtb_convert_pu = CyberDog_Mot_IfActionSubsystem1(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S115>/If Action Subsystem1' */
  }

  /* End of If: '<S115>/If' */

  /* Gain: '<S113>/indexing' */
  rtb_convert_pu *= 800.0F;

  /* DataTypeConversion: '<S113>/Get_Integer' */
  rtb_Sw_dVd = truncf(rtb_convert_pu);
  if (rtIsNaNF(rtb_Sw_dVd) || rtIsInfF(rtb_Sw_dVd)) {
    rtb_Sw_dVd = 0.0F;
  } else {
    rtb_Sw_dVd = fmodf(rtb_Sw_dVd, 65536.0F);
  }

  if (rtb_Sw_dVd < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sw_dVd;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sw_dVd;
  }

  /* End of DataTypeConversion: '<S113>/Get_Integer' */

  /* Sum: '<S113>/Sum2' incorporates:
   *  DataTypeConversion: '<S113>/Data Type Conversion1'
   */
  rtb_Integrator_l = rtb_convert_pu - (real32_T)rtb_Get_Integer;

  /* SwitchCase: '<S8>/Switch Case' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion2'
   *  Inport: '<Root>/pll_reset'
   */
  if (CyberDog_Motor_FOC_U.pll_reset != 0.0F) {
    /* Outputs for IfAction SubSystem: '<S8>/If Action Subsystem' incorporates:
     *  ActionPort: '<S99>/Action Port'
     */
    /* SignalConversion generated from: '<S99>/Out1' incorporates:
     *  Constant: '<S8>/Constant'
     *  SignalConversion generated from: '<S99>/In1'
     */
    rtb_Merge_idx_0 = 0.0F;
    rtb_Merge_idx_1 = 0.0F;

    /* End of Outputs for SubSystem: '<S8>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S8>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S100>/Action Port'
     */
    /* SignalConversion generated from: '<S100>/In1' incorporates:
     *  Constant: '<S113>/offset'
     *  Constant: '<S113>/sine_table_values'
     *  Product: '<S114>/Product'
     *  Product: '<S114>/Product1'
     *  Selector: '<S113>/Lookup'
     *  Sum: '<S113>/Sum'
     *  Sum: '<S114>/Sum3'
     *  Sum: '<S114>/Sum4'
     *  Sum: '<S114>/Sum5'
     *  Sum: '<S114>/Sum6'
     */
    rtb_Merge_idx_0 = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
                       (rtb_Get_Integer + 1U)] -
                       CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer]) *
      rtb_Integrator_l + CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer];
    rtb_Merge_idx_1 = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
                       (rtb_Get_Integer + 201U)] -
                       CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
                       (rtb_Get_Integer + 200U)]) * rtb_Integrator_l +
      CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 200U)];

    /* End of Outputs for SubSystem: '<S8>/If Action Subsystem1' */
  }

  /* End of SwitchCase: '<S8>/Switch Case' */

  /* Outputs for IfAction SubSystem: '<S7>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S71>/Action Port'
   */
  /* SwitchCase: '<S7>/Switch Case' incorporates:
   *  Constant: '<S65>/OD1Cnst'
   *  Constant: '<S65>/OD2Cnst'
   *  Constant: '<S91>/IC'
   *  Constant: '<S92>/IC'
   *  DataTypeConversion: '<S3>/Data Type Conversion2'
   *  Delay: '<S89>/Delay2'
   *  Delay: '<S89>/Delay3'
   *  Delay: '<S90>/Delay2'
   *  Delay: '<S90>/Delay3'
   *  Delay: '<S95>/Delay'
   *  Delay: '<S98>/Delay'
   *  Inport: '<Root>/pll_reset'
   *  Product: '<S89>/Product'
   *  Product: '<S89>/Product1'
   *  Product: '<S90>/Product'
   *  Product: '<S90>/Product1'
   *  Product: '<S95>/Product'
   *  Product: '<S95>/Product1'
   *  Product: '<S98>/Product'
   *  Product: '<S98>/Product1'
   *  Sum: '<S89>/Sum1'
   *  Sum: '<S90>/Sum1'
   *  Sum: '<S95>/Add1'
   *  Sum: '<S98>/Add1'
   *  Switch: '<S95>/Switch'
   *  Switch: '<S98>/Switch'
   * */
  if (CyberDog_Motor_FOC_DW.icLoad_d0) {
    CyberDog_Motor_FOC_DW.Delay_DSTATE_o = 0.0F;
  }

  CyberDog_Motor_FOC_DW.Delay_DSTATE_o = 0.979220271F * rtb_Merge_idx_1 +
    0.0207797289F * CyberDog_Motor_FOC_DW.Delay_DSTATE_o;
  rtb_convert_pu = CyberDog_Motor_FOC_DW.Delay_DSTATE_o * 2.3561945E+6F;
  if ((((CyberDog_Motor_FOC_U.pll_reset == 0.0F) ==
        (CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE == POS_ZCSIG)) &&
       (CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE != UNINITIALIZED_ZCSIG)) ||
      (CyberDog_Motor_FOC_U.pll_reset != 0.0F)) {
    CyberDog_Motor_FOC_DW.Delay2_DSTATE = 0.0F;
  }

  CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE = (ZCSigState)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);
  if ((((CyberDog_Motor_FOC_U.pll_reset == 0.0F) ==
        (CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE == POS_ZCSIG)) &&
       (CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE != UNINITIALIZED_ZCSIG)) ||
      (CyberDog_Motor_FOC_U.pll_reset != 0.0F)) {
    CyberDog_Motor_FOC_DW.Delay3_DSTATE = 0.0F;
  }

  CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE = (ZCSigState)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);
  CyberDog_Motor_FOC_DW.Delay3_DSTATE = ((CyberDog_Motor_FOC_DW.Delay3_DSTATE +
    rtb_convert_pu) - CyberDog_Motor_FOC_DW.Delay2_DSTATE) * 0.00422619516F;
  if ((((CyberDog_Motor_FOC_U.pll_reset == 0.0F) ==
        (CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE_o == POS_ZCSIG)) &&
       (CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE_o != UNINITIALIZED_ZCSIG)) ||
      (CyberDog_Motor_FOC_U.pll_reset != 0.0F)) {
    CyberDog_Motor_FOC_DW.Delay2_DSTATE_i = 0.0F;
  }

  CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE_o = (ZCSigState)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);
  rtb_Integrator_l = CyberDog_Motor_FOC_DW.Delay2_DSTATE_i;
  if ((((CyberDog_Motor_FOC_U.pll_reset == 0.0F) ==
        (CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE_o == POS_ZCSIG)) &&
       (CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE_o != UNINITIALIZED_ZCSIG)) ||
      (CyberDog_Motor_FOC_U.pll_reset != 0.0F)) {
    CyberDog_Motor_FOC_DW.Delay3_DSTATE_h = 0.0F;
  }

  CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE_o = (ZCSigState)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);
  if (CyberDog_Motor_FOC_DW.icLoad_i) {
    CyberDog_Motor_FOC_DW.Delay_DSTATE_dt = 0.0F;
  }

  CyberDog_Motor_FOC_DW.Delay_DSTATE_dt = rtb_Merge_idx_0 * 0.979220271F +
    0.0207797289F * CyberDog_Motor_FOC_DW.Delay_DSTATE_dt;
  CyberDog_Motor_FOC_DW.Delay2_DSTATE_i = CyberDog_Motor_FOC_DW.Delay_DSTATE_dt *
    2.3561945E+6F;
  CyberDog_Motor_FOC_DW.Delay3_DSTATE_h =
    ((CyberDog_Motor_FOC_DW.Delay3_DSTATE_h +
      CyberDog_Motor_FOC_DW.Delay2_DSTATE_i) - rtb_Integrator_l) *
    0.00422619516F;
  CyberDog_Motor_FOC_DW.icLoad_d0 = false;
  CyberDog_Motor_FOC_DW.Delay2_DSTATE = rtb_convert_pu;
  CyberDog_Motor_FOC_DW.icLoad_i = false;

  /* End of Outputs for SubSystem: '<S7>/OptimizedDifferentiationMethod' */

  /* Delay: '<S78>/Delay' */
  if (CyberDog_Motor_FOC_DW.icLoad) {
    /* Sum: '<S78>/Add1' incorporates:
     *  Constant: '<S66>/IC'
     */
    CyberDog_Motor_FOC_DW.Delay_DSTATE = 0.0F;
  }

  /* Outputs for IfAction SubSystem: '<S7>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S71>/Action Port'
   */
  /* SwitchCase: '<S7>/Switch Case' incorporates:
   *  Delay: '<S78>/Delay'
   *  Delay: '<S89>/Delay3'
   *  Delay: '<S90>/Delay3'
   *  Product: '<S71>/Product'
   *  Product: '<S71>/Product1'
   *  Product: '<S78>/Product'
   *  Product: '<S78>/Product1'
   *  Sum: '<S71>/Sum'
   *  Sum: '<S78>/Add1'
   */
  CyberDog_Motor_FOC_DW.Delay_DSTATE = (CyberDog_Motor_FOC_DW.Delay3_DSTATE_h *
    rtb_Merge_idx_1 - CyberDog_Motor_FOC_DW.Delay3_DSTATE * rtb_Merge_idx_0) *
    0.111635208F + 0.888364792F * CyberDog_Motor_FOC_DW.Delay_DSTATE;

  /* End of Outputs for SubSystem: '<S7>/OptimizedDifferentiationMethod' */

  /* Delay: '<S102>/Delay' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion2'
   *  Inport: '<Root>/pll_reset'
   */
  if ((((CyberDog_Motor_FOC_U.pll_reset == 0.0F) ==
        (CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE == POS_ZCSIG)) &&
       (CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE != UNINITIALIZED_ZCSIG)) ||
      (CyberDog_Motor_FOC_U.pll_reset != 0.0F)) {
    /* Sum: '<S103>/Sum' */
    CyberDog_Motor_FOC_DW.Delay_DSTATE_h = 0.0F;
  }

  CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE = (ZCSigState)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);

  /* If: '<S107>/If' incorporates:
   *  Constant: '<S109>/Constant'
   *  Delay: '<S102>/Delay'
   *  RelationalOperator: '<S109>/Compare'
   */
  if (CyberDog_Motor_FOC_DW.Delay_DSTATE_h < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S107>/If Action Subsystem' incorporates:
     *  ActionPort: '<S110>/Action Port'
     */
    rtb_Integrator_l = CyberDog_Moto_IfActionSubsystem
      (CyberDog_Motor_FOC_DW.Delay_DSTATE_h);

    /* End of Outputs for SubSystem: '<S107>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S107>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S111>/Action Port'
     */
    rtb_Integrator_l = CyberDog_Mot_IfActionSubsystem1
      (CyberDog_Motor_FOC_DW.Delay_DSTATE_h);

    /* End of Outputs for SubSystem: '<S107>/If Action Subsystem1' */
  }

  /* End of If: '<S107>/If' */

  /* Gain: '<S105>/indexing' */
  rtb_Integrator_l *= 800.0F;

  /* DataTypeConversion: '<S105>/Get_Integer' */
  rtb_Sw_dVd = truncf(rtb_Integrator_l);
  if (rtIsNaNF(rtb_Sw_dVd) || rtIsInfF(rtb_Sw_dVd)) {
    rtb_Sw_dVd = 0.0F;
  } else {
    rtb_Sw_dVd = fmodf(rtb_Sw_dVd, 65536.0F);
  }

  if (rtb_Sw_dVd < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sw_dVd;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sw_dVd;
  }

  /* End of DataTypeConversion: '<S105>/Get_Integer' */

  /* Sum: '<S105>/Sum2' incorporates:
   *  DataTypeConversion: '<S105>/Data Type Conversion1'
   */
  rtb_convert_pu = rtb_Integrator_l - (real32_T)rtb_Get_Integer;

  /* Sum: '<S3>/Sum' incorporates:
   *  Constant: '<S105>/offset'
   *  Constant: '<S105>/sine_table_values'
   *  Product: '<S106>/Product'
   *  Product: '<S106>/Product1'
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product1'
   *  Selector: '<S105>/Lookup'
   *  Sum: '<S105>/Sum'
   *  Sum: '<S106>/Sum3'
   *  Sum: '<S106>/Sum4'
   *  Sum: '<S106>/Sum5'
   *  Sum: '<S106>/Sum6'
   */
  rtb_convert_pu = ((CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer
    + 201U)] - CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer +
    200U)]) * rtb_convert_pu + CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
                    (rtb_Get_Integer + 200U)]) * rtb_Merge_idx_0 -
    ((CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 1U)] -
      CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer]) * rtb_convert_pu +
     CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer]) * rtb_Merge_idx_1;

  /* DiscreteIntegrator: '<S47>/Integrator' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion2'
   *  Inport: '<Root>/pll_reset'
   */
  if ((CyberDog_Motor_FOC_U.pll_reset != 0.0F) ||
      (CyberDog_Motor_FOC_DW.Integrator_PrevResetState != 0)) {
    CyberDog_Motor_FOC_DW.Integrator_DSTATE = 0.0F;
  }

  /* DiscreteIntegrator: '<S47>/Integrator' incorporates:
   *  Gain: '<S44>/Integral Gain'
   */
  CyberDog_Motor_FOC_DW.Integrator_DSTATE += 3947.8418F * rtb_convert_pu;

  /* Sum: '<S3>/Sum1' incorporates:
   *  Gain: '<S52>/Proportional Gain'
   *  Sum: '<S56>/Sum'
   */
  rtb_Integrator_l = (12566.3711F * rtb_convert_pu +
                      CyberDog_Motor_FOC_DW.Integrator_DSTATE) +
    CyberDog_Motor_FOC_DW.Delay_DSTATE;

  /* Product: '<S103>/Product' incorporates:
   *  Gain: '<S102>/Gain'
   */
  rtb_convert_pu = 1.59154952E-5F * rtb_Integrator_l;

  /* Delay: '<S103>/Delay' incorporates:
   *  Constant: '<S103>/InputGain'
   *  Constant: '<S103>/Offset'
   *  Product: '<S103>/Product1'
   *  Sum: '<S103>/Sum1'
   */
  if (CyberDog_Motor_FOC_DW.icLoad_o) {
    CyberDog_Motor_FOC_DW.Delay_DSTATE_d = 0.0F - rtb_convert_pu;
  }

  /* Sum: '<S103>/Sum' incorporates:
   *  Delay: '<S103>/Delay'
   */
  CyberDog_Motor_FOC_DW.Delay_DSTATE_h = rtb_convert_pu +
    CyberDog_Motor_FOC_DW.Delay_DSTATE_d;

  /* Switch: '<S103>/Switch1' */
  if (!(CyberDog_Motor_FOC_DW.Delay_DSTATE_h > 0.0F)) {
    /* Sum: '<S103>/Sum' incorporates:
     *  Constant: '<S103>/Bias1'
     *  Sum: '<S103>/Sum2'
     */
    CyberDog_Motor_FOC_DW.Delay_DSTATE_h++;
  }

  /* End of Switch: '<S103>/Switch1' */

  /* Switch: '<S103>/Switch2' */
  if (CyberDog_Motor_FOC_DW.Delay_DSTATE_h >= 1.0F) {
    /* Sum: '<S103>/Sum' incorporates:
     *  Sum: '<S103>/Sum3'
     */
    CyberDog_Motor_FOC_DW.Delay_DSTATE_h--;
  }

  /* End of Switch: '<S103>/Switch2' */

  /* Gain: '<S3>/PositionGain' incorporates:
   *  Delay: '<S102>/Delay'
   */
  CyberDog_Motor_FOC_Y.theta_elec_filt = 6.28318548F *
    CyberDog_Motor_FOC_DW.Delay_DSTATE_h;

  /* Gain: '<S361>/convert_pu' */
  rtb_convert_pu = 0.159154937F * CyberDog_Motor_FOC_Y.theta_elec_filt;

  /* If: '<S361>/If' incorporates:
   *  Constant: '<S363>/Constant'
   *  RelationalOperator: '<S363>/Compare'
   */
  if (rtb_convert_pu < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S361>/If Action Subsystem' incorporates:
     *  ActionPort: '<S364>/Action Port'
     */
    rtb_convert_pu = CyberDog_Moto_IfActionSubsystem(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S361>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S361>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S365>/Action Port'
     */
    rtb_convert_pu = CyberDog_Mot_IfActionSubsystem1(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S361>/If Action Subsystem1' */
  }

  /* End of If: '<S361>/If' */

  /* Gain: '<S359>/indexing' */
  rtb_convert_pu *= 800.0F;

  /* DataTypeConversion: '<S359>/Get_Integer' */
  rtb_Sw_dVd = truncf(rtb_convert_pu);
  if (rtIsNaNF(rtb_Sw_dVd) || rtIsInfF(rtb_Sw_dVd)) {
    rtb_Sw_dVd = 0.0F;
  } else {
    rtb_Sw_dVd = fmodf(rtb_Sw_dVd, 65536.0F);
  }

  if (rtb_Sw_dVd < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sw_dVd;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sw_dVd;
  }

  /* End of DataTypeConversion: '<S359>/Get_Integer' */

  /* Sum: '<S359>/Sum2' incorporates:
   *  DataTypeConversion: '<S359>/Data Type Conversion1'
   */
  rtb_Merge_idx_0 = rtb_convert_pu - (real32_T)rtb_Get_Integer;

  /* Sum: '<S360>/Sum4' incorporates:
   *  Constant: '<S359>/offset'
   *  Constant: '<S359>/sine_table_values'
   *  Product: '<S360>/Product'
   *  Selector: '<S359>/Lookup'
   *  Sum: '<S359>/Sum'
   *  Sum: '<S360>/Sum3'
   */
  rtb_Sum4_m = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 1U)]
                - CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer]) *
    rtb_Merge_idx_0 + CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer];

  /* Sum: '<S360>/Sum6' incorporates:
   *  Constant: '<S359>/offset'
   *  Constant: '<S359>/sine_table_values'
   *  Product: '<S360>/Product1'
   *  Selector: '<S359>/Lookup'
   *  Sum: '<S359>/Sum'
   *  Sum: '<S360>/Sum5'
   */
  rtb_Sum6_f = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer +
    201U)] - CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 200U)])
    * rtb_Merge_idx_0 + CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
    (rtb_Get_Integer + 200U)];

  /* Outputs for Atomic SubSystem: '<S356>/Two inputs CRL' */
  /* Outputs for Atomic SubSystem: '<S133>/Two phase CRL wrap' */
  /* Switch: '<S358>/Switch' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S134>/a16'
   *  Inport: '<Root>/ia'
   *  Product: '<S357>/acos'
   *  Product: '<S357>/asin'
   *  Product: '<S357>/bcos'
   *  Product: '<S357>/bsin'
   *  Sum: '<S357>/sum_Ds'
   *  Sum: '<S357>/sum_Qs'
   */
  rtb_Switch1_c_idx_0 = CyberDog_Motor_FOC_U.ia * rtb_Sum6_f + rtb_MM_max *
    rtb_Sum4_m;
  rtb_Switch1_c_idx_1 = rtb_MM_max * rtb_Sum6_f - CyberDog_Motor_FOC_U.ia *
    rtb_Sum4_m;

  /* End of Outputs for SubSystem: '<S133>/Two phase CRL wrap' */

  /* Outport: '<Root>/id_meas' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
   */
  CyberDog_Motor_FOC_Y.id_meas = rtb_Switch1_c_idx_0;

  /* Outport: '<Root>/iq_meas' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
   */
  CyberDog_Motor_FOC_Y.iq_meas = rtb_Switch1_c_idx_1;

  /* Sum: '<S122>/Sum_errd' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
   *  Inport: '<Root>/id_ref'
   */
  rtb_Merge_idx_0 = CyberDog_Motor_FOC_U.id_ref - rtb_Switch1_c_idx_0;

  /* End of Outputs for SubSystem: '<S356>/Two inputs CRL' */

  /* Sum: '<S180>/Sum' incorporates:
   *  DiscreteIntegrator: '<S171>/Integrator'
   *  Gain: '<S176>/Proportional Gain'
   */
  rtb_convert_pu = CurrD_Kp * rtb_Merge_idx_0 +
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_l;

  /* Saturate: '<S178>/Saturation' */
  if (rtb_convert_pu > CurrD_MaxOut) {
    rtb_Merge_idx_1 = CurrD_MaxOut;
  } else if (rtb_convert_pu < CurrD_MinOut) {
    rtb_Merge_idx_1 = CurrD_MinOut;
  } else {
    rtb_Merge_idx_1 = rtb_convert_pu;
  }

  /* End of Saturate: '<S178>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S356>/Two inputs CRL' */
  /* MinMax: '<S123>/Max_mag_floor' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
   *  Constant: '<S123>/Const_floor'
   *  Product: '<S123>/P_id2'
   *  Product: '<S123>/P_iq2'
   *  Sqrt: '<S123>/Sqrt_mag'
   *  Sum: '<S123>/S_mag2'
   */
  rtb_MM_max = fmaxf(sqrtf(rtb_Switch1_c_idx_0 * rtb_Switch1_c_idx_0 +
    rtb_Switch1_c_idx_1 * rtb_Switch1_c_idx_1), 1.0E-6F);

  /* End of Outputs for SubSystem: '<S356>/Two inputs CRL' */

  /* Lookup_n-D: '<S123>/LUT_dead' incorporates:
   *  MinMax: '<S123>/Max_mag_floor'
   */
  rtb_LUT_dead = look1_iflf_binlxpw(rtb_MM_max, DeadComp_Lut_I, DeadComp_Lut_V,
    19U);

  /* RelationalOperator: '<S123>/Rel_dead' incorporates:
   *  Constant: '<S123>/Const_0d'
   *  Constant: '<S123>/Const_DeadEn'
   */
  rtb_Rel_dead = (DeadComp_En > 0.0F);

  /* Switch: '<S2>/Sw_dq_vd' incorporates:
   *  Switch: '<S123>/Sw_dVd'
   */
  if (rtb_Rel_dqvol) {
    /* Switch: '<S2>/Sw_dq_vd' incorporates:
     *  Inport: '<Root>/vd_ref'
     */
    CyberDog_Motor_FOC_Y.vd_meas = CyberDog_Motor_FOC_U.vd_ref;
  } else {
    if (rtb_Rel_dead) {
      /* Outputs for Atomic SubSystem: '<S356>/Two inputs CRL' */
      /* Switch: '<S123>/Sw_dVd' incorporates:
       *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
       *  Product: '<S123>/D_id'
       *  Product: '<S123>/P_dVd'
       */
      rtb_Sw_dVd = rtb_Switch1_c_idx_0 / rtb_MM_max * rtb_LUT_dead;

      /* End of Outputs for SubSystem: '<S356>/Two inputs CRL' */
    } else {
      /* Switch: '<S123>/Sw_dVd' incorporates:
       *  Constant: '<S123>/Const_0d'
       */
      rtb_Sw_dVd = 0.0F;
    }

    /* Switch: '<S2>/Sw_dq_vd' incorporates:
     *  Sum: '<S123>/S_vd'
     */
    CyberDog_Motor_FOC_Y.vd_meas = rtb_Merge_idx_1 + rtb_Sw_dVd;
  }

  /* End of Switch: '<S2>/Sw_dq_vd' */
  tmp = (CyberDog_Motor_FOC_M->Timing.TaskCounters.TID[1] == 0);
  if (tmp) {
    /* Sum: '<S246>/Sum_pos_err' incorporates:
     *  Inport: '<Root>/pos_fbk'
     *  Inport: '<Root>/pos_ref'
     */
    rtb_Sw_dVd = CyberDog_Motor_FOC_U.pos_ref - CyberDog_Motor_FOC_U.pos_fbk;

    /* Gain: '<S286>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S278>/Filter'
     *  Gain: '<S276>/Derivative Gain'
     *  Sum: '<S278>/SumD'
     */
    rtb_FilterCoefficient = (Pos_Kd * rtb_Sw_dVd -
      CyberDog_Motor_FOC_DW.Filter_DSTATE) * 100.0F;

    /* Sum: '<S292>/Sum' incorporates:
     *  Gain: '<S288>/Proportional Gain'
     */
    CyberDog_Motor_FOC_B.Saturation = Pos_Kp * rtb_Sw_dVd +
      rtb_FilterCoefficient;

    /* Saturate: '<S290>/Saturation' */
    if (CyberDog_Motor_FOC_B.Saturation > Pos_MaxOut) {
      /* Sum: '<S292>/Sum' incorporates:
       *  Saturate: '<S290>/Saturation'
       */
      CyberDog_Motor_FOC_B.Saturation = Pos_MaxOut;
    } else if (CyberDog_Motor_FOC_B.Saturation < Pos_MinOut) {
      /* Sum: '<S292>/Sum' incorporates:
       *  Saturate: '<S290>/Saturation'
       */
      CyberDog_Motor_FOC_B.Saturation = Pos_MinOut;
    }

    /* End of Saturate: '<S290>/Saturation' */
  }

  /* Product: '<S12>/Product' */
  rtb_Integrator_l *= 0.111635208F;

  /* Delay: '<S12>/Delay' */
  if (CyberDog_Motor_FOC_DW.icLoad_d) {
    /* Sum: '<S12>/Add1' incorporates:
     *  Constant: '<S5>/IC'
     */
    CyberDog_Motor_FOC_DW.Delay_DSTATE_k = 0.0F;
  }

  /* Sum: '<S12>/Add1' incorporates:
   *  Delay: '<S12>/Delay'
   *  Product: '<S12>/Product1'
   */
  CyberDog_Motor_FOC_DW.Delay_DSTATE_k = 0.888364792F *
    CyberDog_Motor_FOC_DW.Delay_DSTATE_k + rtb_Integrator_l;

  /* Gain: '<S1>/G_mech_rps' */
  CyberDog_Motor_FOC_Y.speed_meas_rps = InvTwoPiPolePairs *
    CyberDog_Motor_FOC_DW.Delay_DSTATE_k;

  /* Switch: '<S247>/Sw_speedref' incorporates:
   *  Constant: '<S247>/Const_2'
   *  Inport: '<Root>/ctrl_mode'
   *  Inport: '<Root>/ref_speed'
   *  RelationalOperator: '<S247>/Rel_pos'
   */
  if (CyberDog_Motor_FOC_U.ctrl_mode == 2.0F) {
    rtb_Sw_dVd = CyberDog_Motor_FOC_B.Saturation;
  } else {
    rtb_Sw_dVd = CyberDog_Motor_FOC_U.ref_speed;
  }

  /* Sum: '<S247>/Sum_errs' incorporates:
   *  Switch: '<S247>/Sw_speedref'
   */
  rtb_Integrator_l = rtb_Sw_dVd - CyberDog_Motor_FOC_Y.speed_meas_rps;
  if (tmp) {
    /* Sum: '<S345>/Sum' incorporates:
     *  DiscreteIntegrator: '<S336>/Integrator'
     *  Gain: '<S341>/Proportional Gain'
     */
    rtb_Sum_g = Speed_Kp * rtb_Integrator_l +
      CyberDog_Motor_FOC_DW.Integrator_DSTATE_m;

    /* Saturate: '<S343>/Saturation' */
    if (rtb_Sum_g > Speed_MaxOut) {
      /* Saturate: '<S343>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation_f = Speed_MaxOut;
    } else if (rtb_Sum_g < Speed_MinOut) {
      /* Saturate: '<S343>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation_f = Speed_MinOut;
    } else {
      /* Saturate: '<S343>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation_f = rtb_Sum_g;
    }

    /* End of Saturate: '<S343>/Saturation' */
  }

  /* Switch: '<S125>/Sw_coast_iq' incorporates:
   *  Constant: '<S125>/Const_0'
   *  Constant: '<S125>/Const_CogEn'
   *  Inport: '<Root>/coast'
   *  RelationalOperator: '<S125>/Rel_cog'
   *  Sum: '<S125>/Sum_iqff'
   *  Switch: '<S125>/Sw_cog'
   *  Switch: '<S125>/Sw_mode'
   */
  if (CyberDog_Motor_FOC_U.coast != 0.0F) {
    rtb_Sw_dVd = 0.0F;
  } else {
    if (CoggingFF_En > 0.0F) {
      /* Switch: '<S125>/Sw_cog' incorporates:
       *  Gain: '<S3>/PositionGain'
       *  Lookup_n-D: '<S125>/LUT_cog'
       */
      rtb_Sw_cog = look1_iflf_binlxpw(CyberDog_Motor_FOC_Y.theta_elec_filt,
        Cogging_Lut_Angle, Cogging_Lut_V, 359U);
    } else {
      /* Switch: '<S125>/Sw_cog' incorporates:
       *  Constant: '<S125>/Const_0'
       */
      rtb_Sw_cog = 0.0F;
    }

    /* Switch: '<S125>/Sw_mode' incorporates:
     *  Constant: '<S125>/Const_1'
     *  Inport: '<Root>/ctrl_mode'
     *  Inport: '<Root>/iq_ref'
     *  RelationalOperator: '<S125>/Rel_mode'
     */
    if (CyberDog_Motor_FOC_U.ctrl_mode >= 1.0F) {
      rtb_Sw_dVd = CyberDog_Motor_FOC_B.Saturation_f;
    } else {
      rtb_Sw_dVd = CyberDog_Motor_FOC_U.iq_ref;
    }

    rtb_Sw_dVd += rtb_Sw_cog;
  }

  /* End of Switch: '<S125>/Sw_coast_iq' */

  /* Outputs for Atomic SubSystem: '<S356>/Two inputs CRL' */
  /* Sum: '<S122>/Sum_errq' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
   */
  rtb_Sw_dVd -= rtb_Switch1_c_idx_1;

  /* End of Outputs for SubSystem: '<S356>/Two inputs CRL' */

  /* Sum: '<S232>/Sum' incorporates:
   *  DiscreteIntegrator: '<S223>/Integrator'
   *  Gain: '<S228>/Proportional Gain'
   */
  rtb_Sw_cog = CurrQ_Kp * rtb_Sw_dVd + CyberDog_Motor_FOC_DW.Integrator_DSTATE_k;

  /* Saturate: '<S230>/Saturation' */
  if (rtb_Sw_cog > Curr_MaxOut) {
    rtb_Saturation_h = Curr_MaxOut;
  } else if (rtb_Sw_cog < Curr_MinOut) {
    rtb_Saturation_h = Curr_MinOut;
  } else {
    rtb_Saturation_h = rtb_Sw_cog;
  }

  /* End of Saturate: '<S230>/Saturation' */

  /* Switch: '<S2>/Sw_dq_vq' incorporates:
   *  Switch: '<S123>/Sw_dVq'
   */
  if (rtb_Rel_dqvol) {
    /* Switch: '<S2>/Sw_dq_vq' incorporates:
     *  Inport: '<Root>/vq_ref'
     */
    CyberDog_Motor_FOC_Y.vq_meas = CyberDog_Motor_FOC_U.vq_ref;
  } else {
    if (rtb_Rel_dead) {
      /* Outputs for Atomic SubSystem: '<S356>/Two inputs CRL' */
      /* Switch: '<S123>/Sw_dVq' incorporates:
       *  AlgorithmDescriptorDelegate generated from: '<S357>/a16'
       *  Product: '<S123>/D_iq'
       *  Product: '<S123>/P_dVq'
       */
      rtb_MM_max = rtb_Switch1_c_idx_1 / rtb_MM_max * rtb_LUT_dead;

      /* End of Outputs for SubSystem: '<S356>/Two inputs CRL' */
    } else {
      /* Switch: '<S123>/Sw_dVq' incorporates:
       *  Constant: '<S123>/Const_0d'
       */
      rtb_MM_max = 0.0F;
    }

    /* Switch: '<S2>/Sw_dq_vq' incorporates:
     *  Sum: '<S123>/S_vq'
     */
    CyberDog_Motor_FOC_Y.vq_meas = rtb_Saturation_h + rtb_MM_max;
  }

  /* End of Switch: '<S2>/Sw_dq_vq' */

  /* Outputs for Atomic SubSystem: '<S243>/Two inputs CRL' */
  /* Switch: '<S245>/Switch' incorporates:
   *  Product: '<S244>/dcos'
   *  Product: '<S244>/dsin'
   *  Product: '<S244>/qcos'
   *  Product: '<S244>/qsin'
   *  Sum: '<S244>/sum_alpha'
   *  Sum: '<S244>/sum_beta'
   */
  rtb_Switch1_c_idx_0 = CyberDog_Motor_FOC_Y.vd_meas * rtb_Sum6_f -
    CyberDog_Motor_FOC_Y.vq_meas * rtb_Sum4_m;
  rtb_Switch1_c_idx_1 = CyberDog_Motor_FOC_Y.vq_meas * rtb_Sum6_f +
    CyberDog_Motor_FOC_Y.vd_meas * rtb_Sum4_m;

  /* Sum: '<S129>/Sum_mag2' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S244>/a16'
   *  Product: '<S129>/P_va2'
   *  Product: '<S129>/P_vb2'
   */
  rtb_Sum6_f = rtb_Switch1_c_idx_0 * rtb_Switch1_c_idx_0 + rtb_Switch1_c_idx_1 *
    rtb_Switch1_c_idx_1;

  /* End of Outputs for SubSystem: '<S243>/Two inputs CRL' */

  /* Product: '<S129>/Div_factor' incorporates:
   *  Constant: '<S129>/Const_1e6'
   *  Gain: '<S129>/G_vmax'
   *  MinMax: '<S129>/Max_mag_floor'
   *  Sqrt: '<S129>/Sqrt_mag'
   */
  rtb_LUT_dead = VmaxCoeff / fmaxf(sqrtf(rtb_Sum6_f), 1.0E-6F);

  /* Switch: '<S2>/Sw_ab_valpha' incorporates:
   *  Constant: '<S2>/Const_4'
   *  Gain: '<S129>/G_vmax'
   *  Inport: '<Root>/ctrl_mode'
   *  Inport: '<Root>/v_alpha_ref'
   *  Inport: '<Root>/v_beta_ref'
   *  Product: '<S129>/P_vmax2'
   *  RelationalOperator: '<S129>/Rel_over'
   *  RelationalOperator: '<S2>/Rel_abvol'
   *  Switch: '<S129>/Sw_va'
   *  Switch: '<S129>/Sw_vb'
   *  Switch: '<S2>/Sw_ab_vbeta'
   */
  if (CyberDog_Motor_FOC_U.ctrl_mode == 4.0F) {
    rtb_Switch1_c_idx_0 = CyberDog_Motor_FOC_U.v_alpha_ref;
    rtb_Switch1_c_idx_1 = CyberDog_Motor_FOC_U.v_beta_ref;
  } else if (rtb_Sum6_f > VmaxCoeff * VmaxCoeff) {
    /* Outputs for Atomic SubSystem: '<S243>/Two inputs CRL' */
    /* Switch: '<S129>/Sw_va' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S244>/a16'
     *  Product: '<S129>/P_vaf'
     */
    rtb_Switch1_c_idx_0 *= rtb_LUT_dead;

    /* Switch: '<S129>/Sw_vb' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S244>/a16'
     *  Product: '<S129>/P_vbf'
     */
    rtb_Switch1_c_idx_1 *= rtb_LUT_dead;

    /* End of Outputs for SubSystem: '<S243>/Two inputs CRL' */
  }

  /* Gain: '<S127>/G_1_vbus' incorporates:
   *  Switch: '<S2>/Sw_ab_valpha'
   */
  rtb_Sum4_m = InvVbus * rtb_Switch1_c_idx_0;

  /* Gain: '<S127>/G_m05' */
  rtb_MM_max = -0.5F * rtb_Sum4_m;

  /* Gain: '<S127>/G_ubeta' incorporates:
   *  Switch: '<S2>/Sw_ab_vbeta'
   */
  rtb_Sum6_f = InvVbus * rtb_Switch1_c_idx_1;

  /* Sum: '<S127>/Sum_vbc' incorporates:
   *  Gain: '<S127>/G_0866'
   */
  rtb_Switch1_c_idx_1 = 0.866025388F * rtb_Sum6_f + rtb_MM_max;

  /* Sum: '<S127>/Sum_vc' incorporates:
   *  Gain: '<S127>/G_m0866'
   */
  rtb_MM_max += -0.866025388F * rtb_Sum6_f;

  /* Gain: '<S127>/G_05' incorporates:
   *  MinMax: '<S127>/MM_max'
   *  MinMax: '<S127>/MM_min'
   *  Sum: '<S127>/Sum_v0'
   */
  rtb_Sum6_f = (fmaxf(fmaxf(rtb_Sum4_m, rtb_Switch1_c_idx_1), rtb_MM_max) +
                fminf(fminf(rtb_Sum4_m, rtb_Switch1_c_idx_1), rtb_MM_max)) *
    0.5F;

  /* Switch: '<S2>/Sw_duty_u' incorporates:
   *  Inport: '<Root>/coast'
   *  Switch: '<S2>/Sw_duty_v'
   *  Switch: '<S2>/Sw_duty_w'
   */
  if (CyberDog_Motor_FOC_U.coast != 0.0F) {
    /* Outport: '<Root>/duty_u' incorporates:
     *  Constant: '<S2>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_u = 0.5F;

    /* Outport: '<Root>/duty_v' incorporates:
     *  Constant: '<S2>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_v = 0.5F;

    /* Outport: '<Root>/duty_w' incorporates:
     *  Constant: '<S2>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_w = 0.5F;
  } else {
    /* Outport: '<Root>/duty_u' incorporates:
     *  Constant: '<S127>/Const_05'
     *  Sum: '<S127>/Sum_du'
     *  Sum: '<S127>/Sum_va_dash'
     */
    CyberDog_Motor_FOC_Y.duty_u = (rtb_Sum4_m - rtb_Sum6_f) + 0.5F;

    /* Outport: '<Root>/duty_v' incorporates:
     *  Constant: '<S127>/Const_05'
     *  Sum: '<S127>/Sum_dv'
     *  Sum: '<S127>/Sum_vb_dash'
     */
    CyberDog_Motor_FOC_Y.duty_v = (rtb_Switch1_c_idx_1 - rtb_Sum6_f) + 0.5F;

    /* Outport: '<Root>/duty_w' incorporates:
     *  Constant: '<S127>/Const_05'
     *  Sum: '<S127>/Sum_dw'
     *  Sum: '<S127>/Sum_vc_dash'
     */
    CyberDog_Motor_FOC_Y.duty_w = (rtb_MM_max - rtb_Sum6_f) + 0.5F;
  }

  /* End of Switch: '<S2>/Sw_duty_u' */

  /* Outport: '<Root>/i_alpha_meas' incorporates:
   *  Inport: '<Root>/ia'
   */
  CyberDog_Motor_FOC_Y.i_alpha_meas = CyberDog_Motor_FOC_U.ia;
  if (tmp) {
    /* Sum: '<S328>/SumI4' incorporates:
     *  Gain: '<S333>/Integral Gain'
     *  Sum: '<S328>/SumI2'
     */
    rtb_SumI4_m = (CyberDog_Motor_FOC_B.Saturation_f - rtb_Sum_g) + Speed_Ki *
      rtb_Integrator_l;
  }

  /* Update for Delay: '<S78>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad = false;

  /* Update for DiscreteIntegrator: '<S47>/Integrator' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion2'
   *  Inport: '<Root>/pll_reset'
   */
  CyberDog_Motor_FOC_DW.Integrator_PrevResetState = (int8_T)
    (CyberDog_Motor_FOC_U.pll_reset != 0.0F);

  /* Update for Delay: '<S103>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_o = false;
  CyberDog_Motor_FOC_DW.Delay_DSTATE_d = CyberDog_Motor_FOC_DW.Delay_DSTATE_h;

  /* Update for DiscreteIntegrator: '<S171>/Integrator' incorporates:
   *  Gain: '<S168>/Integral Gain'
   *  Sum: '<S163>/SumI2'
   *  Sum: '<S163>/SumI4'
   */
  CyberDog_Motor_FOC_DW.Integrator_DSTATE_l += (rtb_Merge_idx_1 - rtb_convert_pu)
    + CurrD_Ki * rtb_Merge_idx_0;
  if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_l > CurrD_MaxOut) {
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_l = CurrD_MaxOut;
  } else if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_l < CurrD_MinOut) {
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_l = CurrD_MinOut;
  }

  /* End of Update for DiscreteIntegrator: '<S171>/Integrator' */
  if (tmp) {
    /* Update for DiscreteIntegrator: '<S278>/Filter' */
    CyberDog_Motor_FOC_DW.Filter_DSTATE += 0.001F * rtb_FilterCoefficient;

    /* Update for DiscreteIntegrator: '<S336>/Integrator' */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_m += rtb_SumI4_m;
  }

  /* Update for Delay: '<S12>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_d = false;

  /* Update for DiscreteIntegrator: '<S223>/Integrator' incorporates:
   *  Gain: '<S220>/Integral Gain'
   *  Sum: '<S215>/SumI2'
   *  Sum: '<S215>/SumI4'
   */
  CyberDog_Motor_FOC_DW.Integrator_DSTATE_k += (rtb_Saturation_h - rtb_Sw_cog) +
    CurrQ_Ki * rtb_Sw_dVd;
  if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_k > Curr_MaxOut) {
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_k = Curr_MaxOut;
  } else if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_k < Curr_MinOut) {
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_k = Curr_MinOut;
  }

  /* End of Update for DiscreteIntegrator: '<S223>/Integrator' */
  rate_scheduler();
}

/* Model initialize function */
void CyberDog_Motor_FOC_initialize(void)
{
  CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay1_Reset_ZCE = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE_e = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay1_Reset_ZCE_c = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay_Reset_ZCE_k = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay2_Reset_ZCE_o = UNINITIALIZED_ZCSIG;
  CyberDog_Motor_FOC_PrevZCX.Delay3_Reset_ZCE_o = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Delay: '<S78>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S103>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_o = true;

  /* InitializeConditions for Delay: '<S12>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_d = true;

  /* SystemInitialize for IfAction SubSystem: '<S7>/IntegralMethod' */
  /* InitializeConditions for Delay: '<S85>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_l = true;

  /* InitializeConditions for Delay: '<S88>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_k = true;

  /* End of SystemInitialize for SubSystem: '<S7>/IntegralMethod' */

  /* SystemInitialize for IfAction SubSystem: '<S7>/OptimizedDifferentiationMethod' */
  /* InitializeConditions for Delay: '<S95>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_d0 = true;

  /* InitializeConditions for Delay: '<S98>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_i = true;

  /* End of SystemInitialize for SubSystem: '<S7>/OptimizedDifferentiationMethod' */
}

/* Model terminate function */
void CyberDog_Motor_FOC_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
