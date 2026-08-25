/*
 * File: CyberDog_Motor_FOC_private.h
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

#ifndef CyberDog_Motor_FOC_private_h_
#define CyberDog_Motor_FOC_private_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "CyberDog_Motor_FOC_types.h"
#include "CyberDog_Motor_FOC.h"

extern real32_T look1_iflf_binlxpw(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
extern real32_T CyberDog_Moto_IfActionSubsystem(real32_T rtu_In1);
extern real32_T CyberDog_Mot_IfActionSubsystem1(real32_T rtu_In1);

#endif                                 /* CyberDog_Motor_FOC_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
