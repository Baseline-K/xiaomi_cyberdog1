#include <string.h>
#include "Soft_IIC_port.h"

#include "stm32f1xx_hal.h"  
#include "i2c.h"


/* DWT delay helpers （请确保在系统 init 时启用 DWT） */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline void delay_cycles_dwt(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

//count=1 0.26us  count=2  0.36us  count=5 0.6us
static inline void delay_loop_iter(uint32_t count)
{
    if (count == 0) return;

#if defined(__GNUC__) || defined(__clang__)
    __asm__ volatile (
        "1: subs %0, #1\n"
        "   bne 1b\n"
        : "+r" (count)
        :
        : "cc"
    );
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
    while (count--) { __nop(); }   // __nop() 是 ARMCC 内置
#elif defined(__ICCARM__)
    // IAR: use simple loop with NOP (portable)
    while (count--) { __asm("nop"); }
#else
    // generic fallback
    while (count--) { __asm__ volatile("nop"); }
#endif
}

/********************* SoftIIC Adapter *************************/			
/* 替换为你实际的端口/引脚（CubeMX 中的 SCL/SDA） */
#define SOFT_SCL_PORT GPIOB
#define SOFT_SCL_PIN  (1<<8)   // GPIO_PIN_8 -> bit mask   PB8 (I2C1_SCL)
#define SOFT_SDA_PORT GPIOB
#define SOFT_SDA_PIN  (1<<9)   //  PB9 (I2C1_SDA)


static inline void scl_high_direct(void) { SOFT_SCL_PORT->BSRR = SOFT_SCL_PIN; }
static inline void scl_low_direct (void) {  SOFT_SCL_PORT->BSRR = ((uint32_t)SOFT_SCL_PIN) << 16U;  } 
static inline void sda_high_direct(void) { SOFT_SDA_PORT->BSRR = SOFT_SDA_PIN; }
static inline void sda_low_direct (void) { SOFT_SDA_PORT->BSRR = ((uint32_t)SOFT_SDA_PIN) << 16U; }
static inline int  sda_read_direct(void) { return ( (SOFT_SDA_PORT->IDR & SOFT_SDA_PIN) ? 1 : 0 ); }
static inline int  scl_read_direct(void) { return ( (SOFT_SCL_PORT->IDR & SOFT_SCL_PIN) ? 1 : 0 ); }
static inline void half_delay_cycles(uint32_t cycles)  { delay_cycles_dwt(cycles);}  // delay_loop_iter(1);
static inline void irq_disable_impl(void){ __disable_irq(); }
static inline void irq_enable_impl(void) { __enable_irq(); }

static inline void  gpio_set_out_od(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_NOPULL;   //外部上拉
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pin = SOFT_SCL_PIN;
    HAL_GPIO_Init(SOFT_SCL_PORT, &gpio_init);
    gpio_init.Pin = SOFT_SDA_PIN;
    HAL_GPIO_Init(SOFT_SDA_PORT, &gpio_init);
}

/* 恢复为 AF Open-Drain（I2C）——请根据你的 I2C 实例来选择 Alternate */
static inline void gpio_restore_af(void)
{
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;   //外部上拉
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    /* 假设是 I2C1 */
    //gpio_init.Alternate = GPIO_AF4_I2C1; // 在 F1 系列这个字段无效；CubeMX 设置 AF
    gpio_init.Pin = SOFT_SCL_PIN;
    HAL_GPIO_Init(SOFT_SCL_PORT, &gpio_init);
    gpio_init.Pin = SOFT_SDA_PIN;
    HAL_GPIO_Init(SOFT_SDA_PORT, &gpio_init);
}

static inline void gpio_deinit_hw_i2c(void)
{
    HAL_I2C_DeInit(&hi2c1);

    // 2) 可选：关闭 I2C 外设时钟（节电），若你希望节省电流
    // __HAL_RCC_I2C1_CLK_DISABLE(); // 根据具体系列调用
}

static inline void gpio_reinit_hw_i2c(void)
{
    HAL_I2C_Init(&hi2c1);
}


/* 全局软 I2C 句柄 */
softi2c_t g_softi2c;
void SoftI2C_RegisterAndInit(uint32_t SystemCoreClock)
{
    //dwt_init();

    // 注册 GPIO 回调
    g_softi2c.gpio.scl_high = scl_high_direct;
    g_softi2c.gpio.scl_low = scl_low_direct;
    g_softi2c.gpio.sda_high = sda_high_direct;
    g_softi2c.gpio.sda_low = sda_low_direct;
    g_softi2c.gpio.sda_read = sda_read_direct;
	  g_softi2c.gpio.scl_read = scl_read_direct;
    g_softi2c.gpio.delay_cycles = half_delay_cycles;
    g_softi2c.gpio.irq_disable = irq_disable_impl;
    g_softi2c.gpio.irq_enable = irq_enable_impl;
    g_softi2c.gpio.set_pins_out_od = gpio_set_out_od;
    g_softi2c.gpio.restore_pins_af = gpio_restore_af;
    g_softi2c.gpio.deinit_hw_i2c = gpio_deinit_hw_i2c;
    g_softi2c.gpio.reinit_hw_i2c = gpio_reinit_hw_i2c;

	  // 初始化软 I2C 句柄（例如 1 MHz）实际中由于软件指令有延时，
	  //你给1MHZ，通讯速率会小于1MHZ，因此这里target_freq_hz最好结合示波器给一个值
    softi2c_init(&g_softi2c, 1000000, true, SystemCoreClock);

    // 现在 g_softi2c 可作为任何 I2C 设备驱动的底层句柄传入
}
