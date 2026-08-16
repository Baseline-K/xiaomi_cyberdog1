/**
  ******************************************************************************
  * @file     core_cm3.h
  * @brief    GD32_PATCH: Redirect core_cm3.h → core_cm4.h for GD32F303 (Cortex-M4F)
  *
  *          The GD32F303 uses a Cortex-M4F core but runs STM32F103-compatible
  *          firmware. The STM32F1xx device headers hardcode #include "core_cm3.h"
  *          which does NOT support FPU and will error if __FPU_PRESENT=1 or
  *          -mfloat-abi=hard is set.
  *
  *          This file sits in Core/Inc/ with BEFORE priority in the include
  *          path, so it's found before the real CMSIS core_cm3.h and redirects
  *          to the Cortex-M4F core header instead.
  *
  *          CubeMX does NOT generate this file, so it won't be overwritten.
  ******************************************************************************
  */
#ifndef __CORE_CM3_H
#define __CORE_CM3_H

/* GD32_PATCH: Ensure __FPU_PRESENT is set BEFORE core_cm4.h checks it.
   Some build targets (e.g. STM32_Drivers OBJECT library) may not inherit
   the command-line -D__FPU_PRESENT=1U from the main executable target. */
#ifndef __FPU_PRESENT
#define __FPU_PRESENT 1U
#endif

#include "core_cm4.h"

#endif /* __CORE_CM3_H */
