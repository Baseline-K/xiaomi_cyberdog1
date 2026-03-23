#ifndef SOFT_I2C_PORT_H
#define SOFT_I2C_PORT_H

#include "Soft_IIC.h"


/* 默认 等待SCL空闲上限（以 CPU 周期计）——将被 softi2c_init 覆盖 */
#ifndef SOFTI2C_DEFAULT_STRETCH_TIMEOUT_NS
    #define SOFTI2C_DEFAULT_STRETCH_TIMEOUT_NS 2000U // 2 us -> 对应 CPU 周期由 init 转换
#endif

#define I2C_UNSTICK_MAX_CLOCKS  18  // 最多发送 18 个 SCL 脉冲

extern softi2c_t g_softi2c;
void SoftI2C_RegisterAndInit(uint32_t SystemCoreClock);



#endif

