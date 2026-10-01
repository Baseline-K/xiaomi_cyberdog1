/*
 * File: CyberDog_Motor_FOC_types.h
 *
 * Code generated for Simulink model 'CyberDog_Motor_FOC'.
 *
 * Model version                  : 1.56
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Fri Sep 11 01:24:13 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CyberDog_Motor_FOC_types_h_
#define CyberDog_Motor_FOC_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_MotorCfgBus_
#define DEFINED_TYPEDEF_FOR_MotorCfgBus_

typedef struct {
  real32_T Ld;
  real32_T Lq;
  real32_T Flux;
  real32_T Kt;
  real32_T J;
  real32_T ffd_ld;
  real32_T ffd_lq;
  real32_T ffd_flux;
  real32_T ff_j2pikt;
  real32_T ff_b;
  real32_T ff_fc;
  real32_T leso_b0;
  real32_T leso_invB0;
  real32_T leso_beta1;
  real32_T leso_beta2;
  real32_T plT_ms;
} MotorCfgBus;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_CyberDog_Motor_FOC_T RT_MODEL_CyberDog_Motor_FOC_T;

#endif                                 /* CyberDog_Motor_FOC_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
