/*
 * File: CyberDog_Motor_FOC.c
 *
 * Code generated for Simulink model 'CyberDog_Motor_FOC'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Tue Aug 25 01:00:16 2026
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
real32_T Cogging_Lut_Angle[32] = { 0.0F, 0.202683404F, 0.405366808F,
  0.608050168F, 0.810733616F, 1.01341701F, 1.21610034F, 1.41878378F, 1.62146723F,
  1.82415056F, 2.02683401F, 2.22951746F, 2.43220067F, 2.63488412F, 2.83756757F,
  3.04025102F, 3.24293447F, 3.44561768F, 3.64830112F, 3.85098457F, 4.05366802F,
  4.25635147F, 4.45903492F, 4.66171837F, 4.86440134F, 5.06708479F, 5.26976824F,
  5.47245169F, 5.67513514F, 5.87781858F, 6.08050203F, 6.28318548F } ;/* Variable: Cogging_Lut_Angle
                                                                      * Referenced by: '<S125>/LUT_cog'
                                                                      * 齿槽前馈LUT 电角度断点(rad)
                                                                      */

real32_T Cogging_Lut_V[32] = { 0.0F, 0.059153378F, 0.108718917F, 0.140662819F,
  0.149807483F, 0.134670675F, 0.0977058709F, 0.0449044667F, -0.0151752485F,
  -0.0727952942F, -0.118616357F, -0.145211563F, -0.148270249F, -0.127296641F,
  -0.0856902301F, -0.030194778F, 0.030194778F, 0.0856902301F, 0.127296641F,
  0.148270249F, 0.145211563F, 0.118616357F, 0.0727952942F, 0.0151752485F,
  -0.0449044667F, -0.0977058709F, -0.134670675F, -0.149807483F, -0.140662819F,
  -0.108718917F, -0.059153378F, -7.34788112E-17F } ;/* Variable: Cogging_Lut_V
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
real32_T DeadComp_Lut_I[16] = { 0.0F, 0.2F, 0.4F, 0.6F, 0.8F, 1.0F, 1.2F, 1.4F,
  1.6F, 1.8F, 2.0F, 2.2F, 2.4F, 2.6F, 2.8F, 3.0F } ;/* Variable: DeadComp_Lut_I
                                                     * Referenced by: '<S123>/LUT_dead'
                                                     * 死区补偿LUT 电流断点(A)
                                                     */

real32_T DeadComp_Lut_V[16] = { 0.0F, 0.04F, 0.08F, 0.12F, 0.16F, 0.2F, 0.24F,
  0.28F, 0.32F, 0.36F, 0.4F, 0.44F, 0.48F, 0.52F, 0.56F, 0.6F } ;/* Variable: DeadComp_Lut_V
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
real32_T Speed_Ki = 0.0006F;           /* Variable: Speed_Ki
                                        * Referenced by: '<S278>/Integral Gain'
                                        * 速度环积分增益(1kHz+UseI*Ts 每周期增量)
                                        */
real32_T Speed_Kp = 0.3F;              /* Variable: Speed_Kp
                                        * Referenced by: '<S286>/Proportional Gain'
                                        * 速度环比例增益
                                        */
real32_T Speed_MaxOut = 5.0F;          /* Variable: Speed_MaxOut
                                        * Referenced by:
                                        *   '<S281>/Integrator'
                                        *   '<S288>/Saturation'
                                        * 速度环输出上限(A, iq_ref)
                                        */
real32_T Speed_MinOut = -5.0F;         /* Variable: Speed_MinOut
                                        * Referenced by:
                                        *   '<S281>/Integrator'
                                        *   '<S288>/Saturation'
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
 *    '<S306>/If Action Subsystem'
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
 *    '<S306>/If Action Subsystem1'
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
  real32_T Sum_errs;
  real32_T rtb_Integrator_l;
  real32_T rtb_LUT_dead;
  real32_T rtb_MM_max;
  real32_T rtb_Max_mag_floor;
  real32_T rtb_Merge_idx_0;
  real32_T rtb_Merge_idx_1;
  real32_T rtb_Saturation_h;
  real32_T rtb_Sum4_m;
  real32_T rtb_Sum6_f;
  real32_T rtb_SumI4_ms;
  real32_T rtb_Sum_nb;
  real32_T rtb_Sum_om;
  real32_T rtb_Switch1_c_idx_0;
  real32_T rtb_Switch1_c_idx_1;
  real32_T rtb_convert_pu;
  uint16_T rtb_Get_Integer;
  boolean_T rtb_Rel_over;
  boolean_T tmp;

  /* Outputs for Atomic SubSystem: '<S133>/Two phase CRL wrap' */
  /* Gain: '<S134>/one_by_sqrt3' incorporates:
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Sum: '<S134>/a_plus_2b'
   */
  rtb_MM_max = ((CyberDog_Motor_FOC_U.ia + CyberDog_Motor_FOC_U.ib) +
                CyberDog_Motor_FOC_U.ib) * 0.577350259F;

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
  rtb_Sum_nb = truncf(rtb_convert_pu);
  if (rtIsNaNF(rtb_Sum_nb) || rtIsInfF(rtb_Sum_nb)) {
    rtb_Sum_nb = 0.0F;
  } else {
    rtb_Sum_nb = fmodf(rtb_Sum_nb, 65536.0F);
  }

  if (rtb_Sum_nb < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sum_nb;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sum_nb;
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
  rtb_Sum_nb = truncf(rtb_Integrator_l);
  if (rtIsNaNF(rtb_Sum_nb) || rtIsInfF(rtb_Sum_nb)) {
    rtb_Sum_nb = 0.0F;
  } else {
    rtb_Sum_nb = fmodf(rtb_Sum_nb, 65536.0F);
  }

  if (rtb_Sum_nb < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sum_nb;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sum_nb;
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

  /* Gain: '<S306>/convert_pu' */
  rtb_convert_pu = 0.159154937F * CyberDog_Motor_FOC_Y.theta_elec_filt;

  /* If: '<S306>/If' incorporates:
   *  Constant: '<S308>/Constant'
   *  RelationalOperator: '<S308>/Compare'
   */
  if (rtb_convert_pu < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S306>/If Action Subsystem' incorporates:
     *  ActionPort: '<S309>/Action Port'
     */
    rtb_convert_pu = CyberDog_Moto_IfActionSubsystem(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S306>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S306>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S310>/Action Port'
     */
    rtb_convert_pu = CyberDog_Mot_IfActionSubsystem1(rtb_convert_pu);

    /* End of Outputs for SubSystem: '<S306>/If Action Subsystem1' */
  }

  /* End of If: '<S306>/If' */

  /* Gain: '<S304>/indexing' */
  rtb_convert_pu *= 800.0F;

  /* DataTypeConversion: '<S304>/Get_Integer' */
  rtb_Sum_nb = truncf(rtb_convert_pu);
  if (rtIsNaNF(rtb_Sum_nb) || rtIsInfF(rtb_Sum_nb)) {
    rtb_Sum_nb = 0.0F;
  } else {
    rtb_Sum_nb = fmodf(rtb_Sum_nb, 65536.0F);
  }

  if (rtb_Sum_nb < 0.0F) {
    rtb_Get_Integer = (uint16_T)-(int16_T)(uint16_T)-rtb_Sum_nb;
  } else {
    rtb_Get_Integer = (uint16_T)rtb_Sum_nb;
  }

  /* End of DataTypeConversion: '<S304>/Get_Integer' */

  /* Sum: '<S304>/Sum2' incorporates:
   *  DataTypeConversion: '<S304>/Data Type Conversion1'
   */
  rtb_Merge_idx_0 = rtb_convert_pu - (real32_T)rtb_Get_Integer;

  /* Sum: '<S305>/Sum4' incorporates:
   *  Constant: '<S304>/offset'
   *  Constant: '<S304>/sine_table_values'
   *  Product: '<S305>/Product'
   *  Selector: '<S304>/Lookup'
   *  Sum: '<S304>/Sum'
   *  Sum: '<S305>/Sum3'
   */
  rtb_Sum4_m = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 1U)]
                - CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer]) *
    rtb_Merge_idx_0 + CyberDog_Motor_FOC_ConstP.pooled5[rtb_Get_Integer];

  /* Sum: '<S305>/Sum6' incorporates:
   *  Constant: '<S304>/offset'
   *  Constant: '<S304>/sine_table_values'
   *  Product: '<S305>/Product1'
   *  Selector: '<S304>/Lookup'
   *  Sum: '<S304>/Sum'
   *  Sum: '<S305>/Sum5'
   */
  rtb_Sum6_f = (CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer +
    201U)] - CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)(rtb_Get_Integer + 200U)])
    * rtb_Merge_idx_0 + CyberDog_Motor_FOC_ConstP.pooled5[(int32_T)
    (rtb_Get_Integer + 200U)];

  /* Outputs for Atomic SubSystem: '<S301>/Two inputs CRL' */
  /* Outputs for Atomic SubSystem: '<S133>/Two phase CRL wrap' */
  /* Switch: '<S303>/Switch' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S134>/a16'
   *  Inport: '<Root>/ia'
   *  Product: '<S302>/acos'
   *  Product: '<S302>/asin'
   *  Product: '<S302>/bcos'
   *  Product: '<S302>/bsin'
   *  Sum: '<S302>/sum_Ds'
   *  Sum: '<S302>/sum_Qs'
   */
  rtb_Switch1_c_idx_0 = CyberDog_Motor_FOC_U.ia * rtb_Sum6_f + rtb_MM_max *
    rtb_Sum4_m;
  rtb_Switch1_c_idx_1 = rtb_MM_max * rtb_Sum6_f - CyberDog_Motor_FOC_U.ia *
    rtb_Sum4_m;

  /* End of Outputs for SubSystem: '<S133>/Two phase CRL wrap' */

  /* Sum: '<S122>/Sum_errd' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S302>/a16'
   *  Inport: '<Root>/id_ref'
   */
  rtb_Merge_idx_0 = CyberDog_Motor_FOC_U.id_ref - rtb_Switch1_c_idx_0;

  /* End of Outputs for SubSystem: '<S301>/Two inputs CRL' */

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

  /* Outputs for Atomic SubSystem: '<S301>/Two inputs CRL' */
  /* MinMax: '<S123>/Max_mag_floor' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S302>/a16'
   *  Constant: '<S123>/Const_floor'
   *  Product: '<S123>/P_id2'
   *  Product: '<S123>/P_iq2'
   *  Sqrt: '<S123>/Sqrt_mag'
   *  Sum: '<S123>/S_mag2'
   */
  rtb_Max_mag_floor = fmaxf(sqrtf(rtb_Switch1_c_idx_0 * rtb_Switch1_c_idx_0 +
    rtb_Switch1_c_idx_1 * rtb_Switch1_c_idx_1), 1.0E-6F);

  /* End of Outputs for SubSystem: '<S301>/Two inputs CRL' */

  /* Lookup_n-D: '<S123>/LUT_dead' incorporates:
   *  MinMax: '<S123>/Max_mag_floor'
   */
  rtb_LUT_dead = look1_iflf_binlxpw(rtb_Max_mag_floor, DeadComp_Lut_I,
    DeadComp_Lut_V, 15U);

  /* RelationalOperator: '<S123>/Rel_dead' incorporates:
   *  Constant: '<S123>/Const_0d'
   *  Constant: '<S123>/Const_DeadEn'
   */
  rtb_Rel_over = (DeadComp_En > 0.0F);

  /* Switch: '<S123>/Sw_dVd' */
  if (rtb_Rel_over) {
    /* Outputs for Atomic SubSystem: '<S301>/Two inputs CRL' */
    /* Gain: '<S1>/G_mech_rps' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S302>/a16'
     *  Product: '<S123>/D_id'
     *  Product: '<S123>/P_dVd'
     */
    CyberDog_Motor_FOC_Y.speed_meas_rps = rtb_Switch1_c_idx_0 /
      rtb_Max_mag_floor * rtb_LUT_dead;

    /* End of Outputs for SubSystem: '<S301>/Two inputs CRL' */
  } else {
    /* Gain: '<S1>/G_mech_rps' incorporates:
     *  Constant: '<S123>/Const_0d'
     */
    CyberDog_Motor_FOC_Y.speed_meas_rps = 0.0F;
  }

  /* End of Switch: '<S123>/Sw_dVd' */

  /* Sum: '<S123>/S_vd' */
  rtb_MM_max = rtb_Merge_idx_1 + CyberDog_Motor_FOC_Y.speed_meas_rps;

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

  /* Sum: '<S125>/Sum_errs' incorporates:
   *  Inport: '<Root>/ref_speed'
   */
  Sum_errs = CyberDog_Motor_FOC_U.ref_speed -
    CyberDog_Motor_FOC_Y.speed_meas_rps;
  tmp = (CyberDog_Motor_FOC_M->Timing.TaskCounters.TID[1] == 0);
  if (tmp) {
    /* Sum: '<S290>/Sum' incorporates:
     *  DiscreteIntegrator: '<S281>/Integrator'
     *  Gain: '<S286>/Proportional Gain'
     */
    rtb_Sum_om = Speed_Kp * Sum_errs + CyberDog_Motor_FOC_DW.Integrator_DSTATE_b;

    /* Saturate: '<S288>/Saturation' */
    if (rtb_Sum_om > Speed_MaxOut) {
      /* Saturate: '<S288>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation = Speed_MaxOut;
    } else if (rtb_Sum_om < Speed_MinOut) {
      /* Saturate: '<S288>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation = Speed_MinOut;
    } else {
      /* Saturate: '<S288>/Saturation' */
      CyberDog_Motor_FOC_B.Saturation = rtb_Sum_om;
    }

    /* End of Saturate: '<S288>/Saturation' */
  }

  /* Switch: '<S125>/Sw_cog' incorporates:
   *  Constant: '<S125>/Const_0'
   *  Constant: '<S125>/Const_CogEn'
   *  Gain: '<S3>/PositionGain'
   *  Lookup_n-D: '<S125>/LUT_cog'
   *  RelationalOperator: '<S125>/Rel_cog'
   */
  if (CoggingFF_En > 0.0F) {
    rtb_Integrator_l = look1_iflf_binlxpw(CyberDog_Motor_FOC_Y.theta_elec_filt,
      Cogging_Lut_Angle, Cogging_Lut_V, 31U);
  } else {
    rtb_Integrator_l = 0.0F;
  }

  /* End of Switch: '<S125>/Sw_cog' */

  /* Switch: '<S125>/Sw_mode' incorporates:
   *  Constant: '<S125>/Const_1'
   *  Inport: '<Root>/ctrl_mode'
   *  Inport: '<Root>/iq_ref'
   *  RelationalOperator: '<S125>/Rel_mode'
   */
  if (CyberDog_Motor_FOC_U.ctrl_mode >= 1.0F) {
    rtb_Sum_nb = CyberDog_Motor_FOC_B.Saturation;
  } else {
    rtb_Sum_nb = CyberDog_Motor_FOC_U.iq_ref;
  }

  /* Outputs for Atomic SubSystem: '<S301>/Two inputs CRL' */
  /* Sum: '<S122>/Sum_errq' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S302>/a16'
   *  Sum: '<S125>/Sum_iqff'
   *  Switch: '<S125>/Sw_mode'
   */
  rtb_Integrator_l = (rtb_Sum_nb + rtb_Integrator_l) - rtb_Switch1_c_idx_1;

  /* End of Outputs for SubSystem: '<S301>/Two inputs CRL' */

  /* Sum: '<S232>/Sum' incorporates:
   *  DiscreteIntegrator: '<S223>/Integrator'
   *  Gain: '<S228>/Proportional Gain'
   */
  rtb_Sum_nb = CurrQ_Kp * rtb_Integrator_l +
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_k;

  /* Saturate: '<S230>/Saturation' */
  if (rtb_Sum_nb > Curr_MaxOut) {
    rtb_Saturation_h = Curr_MaxOut;
  } else if (rtb_Sum_nb < Curr_MinOut) {
    rtb_Saturation_h = Curr_MinOut;
  } else {
    rtb_Saturation_h = rtb_Sum_nb;
  }

  /* End of Saturate: '<S230>/Saturation' */

  /* Switch: '<S123>/Sw_dVq' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S302>/a16'
   *  Constant: '<S123>/Const_0d'
   *  Product: '<S123>/D_iq'
   *  Product: '<S123>/P_dVq'
   */
  if (rtb_Rel_over) {
    /* Outputs for Atomic SubSystem: '<S301>/Two inputs CRL' */
    rtb_Switch1_c_idx_1 = rtb_Switch1_c_idx_1 / rtb_Max_mag_floor * rtb_LUT_dead;

    /* End of Outputs for SubSystem: '<S301>/Two inputs CRL' */
  } else {
    rtb_Switch1_c_idx_1 = 0.0F;
  }

  /* End of Switch: '<S123>/Sw_dVq' */

  /* Sum: '<S123>/S_vq' */
  rtb_Switch1_c_idx_1 += rtb_Saturation_h;

  /* Outputs for Atomic SubSystem: '<S243>/Two inputs CRL' */
  /* Switch: '<S245>/Switch' incorporates:
   *  Product: '<S244>/dcos'
   *  Product: '<S244>/dsin'
   *  Product: '<S244>/qcos'
   *  Product: '<S244>/qsin'
   *  Sum: '<S244>/sum_alpha'
   *  Sum: '<S244>/sum_beta'
   */
  rtb_Switch1_c_idx_0 = rtb_MM_max * rtb_Sum6_f - rtb_Switch1_c_idx_1 *
    rtb_Sum4_m;
  rtb_Switch1_c_idx_1 = rtb_Switch1_c_idx_1 * rtb_Sum6_f + rtb_MM_max *
    rtb_Sum4_m;

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
  rtb_Max_mag_floor = VmaxCoeff / fmaxf(sqrtf(rtb_Sum6_f), 1.0E-6F);

  /* Switch: '<S129>/Sw_va' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S244>/a16'
   *  Gain: '<S129>/G_vmax'
   *  Product: '<S129>/P_vaf'
   *  Product: '<S129>/P_vbf'
   *  Product: '<S129>/P_vmax2'
   *  RelationalOperator: '<S129>/Rel_over'
   *  Switch: '<S129>/Sw_vb'
   */
  if (rtb_Sum6_f > VmaxCoeff * VmaxCoeff) {
    /* Outputs for Atomic SubSystem: '<S243>/Two inputs CRL' */
    rtb_Switch1_c_idx_0 *= rtb_Max_mag_floor;
    rtb_Switch1_c_idx_1 *= rtb_Max_mag_floor;

    /* End of Outputs for SubSystem: '<S243>/Two inputs CRL' */
  }

  /* Gain: '<S127>/G_1_vbus' incorporates:
   *  Switch: '<S129>/Sw_va'
   */
  rtb_Sum4_m = InvVbus * rtb_Switch1_c_idx_0;

  /* Gain: '<S127>/G_m05' */
  rtb_MM_max = -0.5F * rtb_Sum4_m;

  /* Gain: '<S127>/G_ubeta' incorporates:
   *  Switch: '<S129>/Sw_vb'
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

  /* Switch: '<Root>/Sw_duty_u' incorporates:
   *  Inport: '<Root>/coast'
   *  Switch: '<Root>/Sw_duty_v'
   *  Switch: '<Root>/Sw_duty_w'
   */
  if (CyberDog_Motor_FOC_U.coast != 0.0F) {
    /* Outport: '<Root>/duty_u' incorporates:
     *  Constant: '<Root>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_u = 0.5;

    /* Outport: '<Root>/duty_v' incorporates:
     *  Constant: '<Root>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_v = 0.5;

    /* Outport: '<Root>/duty_w' incorporates:
     *  Constant: '<Root>/Const_coast05'
     */
    CyberDog_Motor_FOC_Y.duty_w = 0.5;
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

  /* End of Switch: '<Root>/Sw_duty_u' */
  if (tmp) {
    /* Sum: '<S273>/SumI4' incorporates:
     *  Gain: '<S278>/Integral Gain'
     *  Sum: '<S273>/SumI2'
     */
    rtb_SumI4_ms = (CyberDog_Motor_FOC_B.Saturation - rtb_Sum_om) + Speed_Ki *
      Sum_errs;
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

  /* Update for Delay: '<S12>/Delay' */
  CyberDog_Motor_FOC_DW.icLoad_d = false;
  if (tmp) {
    /* Update for DiscreteIntegrator: '<S281>/Integrator' */
    CyberDog_Motor_FOC_DW.Integrator_DSTATE_b += rtb_SumI4_ms;
    if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_b > Speed_MaxOut) {
      CyberDog_Motor_FOC_DW.Integrator_DSTATE_b = Speed_MaxOut;
    } else if (CyberDog_Motor_FOC_DW.Integrator_DSTATE_b < Speed_MinOut) {
      CyberDog_Motor_FOC_DW.Integrator_DSTATE_b = Speed_MinOut;
    }

    /* End of Update for DiscreteIntegrator: '<S281>/Integrator' */
  }

  /* Update for DiscreteIntegrator: '<S223>/Integrator' incorporates:
   *  Gain: '<S220>/Integral Gain'
   *  Sum: '<S215>/SumI2'
   *  Sum: '<S215>/SumI4'
   */
  CyberDog_Motor_FOC_DW.Integrator_DSTATE_k += (rtb_Saturation_h - rtb_Sum_nb) +
    CurrQ_Ki * rtb_Integrator_l;
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
