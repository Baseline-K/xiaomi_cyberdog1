#include "stm32f1xx_hal.h"
#include "FOC_run.h"

typedef struct 
{
  int16_t qV_Component1;
  int16_t qV_Component2;
} Volt_Components;

void RevPark_Circle_Limitation(int32_t * const Voltage_Alpha, int32_t * const Voltage_Beta);
