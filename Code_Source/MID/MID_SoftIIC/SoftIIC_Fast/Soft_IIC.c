#include "Soft_IIC.h"
#include <stdint.h>
#include <string.h>


/* 默认 等待SCL空闲上限（以 CPU 周期计）——将被 softi2c_init 覆盖 */
#ifndef SOFTI2C_DEFAULT_STRETCH_TIMEOUT_NS
#define SOFTI2C_DEFAULT_STRETCH_TIMEOUT_NS 2000U // 2 us -> 对应 CPU 周期由 init 转换
#endif

#define I2C_UNSTICK_MAX_CLOCKS  18  // 最多发送 18 个 SCL 脉冲

/********************* SoftIIC Adapter *************************/
#include "stm32f1xx_hal.h"  
//#include "i2c.h"
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

//Stm32F103C8T6 armM3-72Mhz count=1 0.26us  count=2  0.36us  count=5 0.6us
//GD32F303RET6 armM4-120Mhz  count=1 0.21us  count=2 0.24us count=5 0.317us 
static inline __attribute__((always_inline)) void delay_loop_iter(uint32_t count)
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

			
/* 替换为你实际的端口/引脚（CubeMX 中的 SCL/SDA） */
#define SOFT_SCL_PORT GPIOC
#define SOFT_SCL_PIN  (1<<4)   // GPIO_PIN_4 -> bit mask   PC4 (I2C1_SCL)
#define SOFT_SDA_PORT GPIOA
#define SOFT_SDA_PIN  (1<<5)   //  PA5 (I2C1_SDA)

//Prevent the compiler from reordering operations when reading or writing registers, especially under high optimization settings.
static inline __attribute__((always_inline)) void mem_barrier(void) { __asm__ volatile("" ::: "memory"); }  

static inline __attribute__((always_inline)) void scl_high_direct(void) { SOFT_SCL_PORT->BSRR = SOFT_SCL_PIN; mem_barrier();}
static inline __attribute__((always_inline)) void scl_low_direct (void) {  SOFT_SCL_PORT->BSRR = ((uint32_t)SOFT_SCL_PIN) << 16U;  mem_barrier();} 
static inline __attribute__((always_inline)) void sda_high_direct(void) { SOFT_SDA_PORT->BSRR = SOFT_SDA_PIN; mem_barrier();}
static inline __attribute__((always_inline)) void sda_low_direct (void) { SOFT_SDA_PORT->BSRR = ((uint32_t)SOFT_SDA_PIN) << 16U; mem_barrier();}
static inline __attribute__((always_inline)) int  sda_read_direct(void) { return ( (SOFT_SDA_PORT->IDR & SOFT_SDA_PIN) ? 1 : 0 ); mem_barrier();}
static inline __attribute__((always_inline)) int  scl_read_direct(void) { return ( (SOFT_SCL_PORT->IDR & SOFT_SCL_PIN) ? 1 : 0 ); mem_barrier();}
static inline __attribute__((always_inline)) void half_delay_cycles(uint32_t cycles) {  delay_loop_iter(2); mem_barrier();}  //delay_cycles_dwt(cycles);
static inline __attribute__((always_inline)) void irq_disable_impl(void){ __disable_irq(); mem_barrier();}
static inline __attribute__((always_inline)) void irq_enable_impl(void) { __enable_irq(); mem_barrier();}


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
    //HAL_I2C_DeInit(&hi2c1);

    // 2) 可选：关闭 I2C 外设时钟（节电），若你希望节省电流
    // __HAL_RCC_I2C1_CLK_DISABLE(); // 根据具体系列调用
}

static inline void gpio_reinit_hw_i2c(void)
{
    //HAL_I2C_Init(&hi2c1);
}



/* 全局软 I2C 句柄（或在你的驱动里静态） */
softi2c_t g_softi2c;
void SoftI2C_RegisterAndInit(uint32_t target_freq_hz, uint32_t SystemCoreClock)
{
    //dwt_init();

	  // 初始化软 I2C 句柄（例如 1 MHz）实际中由于软件指令有延时，
	  //你给1MHZ，通讯速率会小于1MHZ，因此这里target_freq_hz最好结合示波器给一个值
    softi2c_init(&g_softi2c, target_freq_hz, true, SystemCoreClock);
}


/********************* SoftIIC *************************/
/* Internal: half period delay */
static inline  __attribute__((always_inline)) void half_delay(softi2c_t *i2c)
{
    half_delay_cycles(i2c->half_period_cycles);
}

static inline  __attribute__((always_inline)) void one_quarter_delay(softi2c_t *i2c)
{
    half_delay_cycles(i2c->one_quarter_period_cycles);
}

static inline  __attribute__((always_inline)) void one_eighth_delay(softi2c_t *i2c)
{
    half_delay_cycles(i2c->one_eighth_period_cycles);

}

static inline  __attribute__((always_inline)) void one_sixteenth_delay(softi2c_t *i2c)
{
    half_delay_cycles(i2c->one_sixteenth_period_cycles);
	 
}

//使外设的IIC通讯退出死锁,在IIC设备初始化之前调用
void I2C_Unstick(softi2c_t *i2c)
{
    gpio_deinit_hw_i2c();

    gpio_set_out_od();

    /* 1) 先释放总线（把 SCL/SDA 设为高——open-drain release） */
    scl_high_direct();
    sda_high_direct();
    half_delay(i2c);

    /* 2) 如果 SDA 已经是高（总线空闲），直接返回成功 */
    if (sda_read_direct() != 0) {
        return;
    }
   
    /* 3) SDA 仍然为低，可能是从设备挂起在某个状态（例如：SDA 被拉低等待 ACK）；
     *    我们通过产生最多I2C_UNSTICK_MAX_CLOCKS 个 SCL 脉冲来尝试释放它（9 个字节 + ACK）
     */
    for (int i = 0; i < I2C_UNSTICK_MAX_CLOCKS; i++) {
        // 产生 SCL 脉冲
        scl_low_direct();
        half_delay(i2c);
        scl_high_direct();
        half_delay(i2c);

        // 每次都检测 SDA 是否释放
        if (sda_read_direct() != 0)
            break;
    }
 
    // 4. 模拟 STOP 条件释放总线（SCL 高时，SDA 从低拉到高）
    sda_low_direct();
    half_delay(i2c);
    scl_high_direct();
    half_delay(i2c);
    sda_high_direct();
    half_delay(i2c);

    gpio_reinit_hw_i2c();
}

/* Helper: wait for SCL high (clock stretching) */
static inline bool wait_scl_high(softi2c_t *i2c)
{
    uint32_t t = i2c->scl_stretch_timeout_cycles;
    while (t--) {
        if (scl_read_direct()) return true;
        // else fallback to tiny delay
        half_delay_cycles(1);
    }
    return false;
}

/* Init computes half_period_cycles from SystemCoreClock and target freq.
   We cannot directly access SystemCoreClock here (user's platform should set it),
   so we require the user make delay_cycles accurate. For convenience, caller
   should set half_period_cycles before calling init (but we calculate via freq using DWT if available). */

/* Public API */

softi2c_status_t softi2c_init(softi2c_t *i2c, 
                              uint32_t scl_freq_hz, bool disable_irq_during_xfer, uint32_t SystemCoreClock)
{
    if (!i2c ) {
        return SOFTI2C_ARG_ERROR;
    }

    I2C_Unstick(i2c);  // 确保总线空闲
    gpio_deinit_hw_i2c();
    gpio_set_out_od();

    memset(i2c, 0, sizeof(*i2c));
    i2c->scl_freq_hz = scl_freq_hz;
    i2c->disable_irq_during_xfer = disable_irq_during_xfer;

    // Compute half_period_cycles = SystemCoreClock/(2 * scl_freq_hz)
    // We use a runtime-known macro SystemCoreClock if available. If not, user-supplied delay_cycles must match expectation.
   if( SystemCoreClock != 0){
			uint32_t sc = SystemCoreClock;
			if (scl_freq_hz == 0) return SOFTI2C_ARG_ERROR;
			uint32_t cycles = (sc + (scl_freq_hz)) / (2 * scl_freq_hz); // round
			if (cycles < 1) cycles = 1;
			i2c->half_period_cycles = cycles;
		  (cycles / 2u >= 1) ? (i2c->one_quarter_period_cycles = cycles / 2u) : (i2c->one_quarter_period_cycles = 1);  
		  (cycles / 4u >= 1) ? (i2c->one_eighth_period_cycles = cycles / 4u) : (i2c->one_eighth_period_cycles = 1);
		  (cycles / 8u >= 1) ? (i2c->one_sixteenth_period_cycles = cycles / 8u) : (i2c->one_sixteenth_period_cycles = 1);
	 }
   else{
     // fallback: user must set gpio.delay_cycles to implement appropriate timing; set default small delay
     i2c->half_period_cycles = 1;
		 i2c->one_quarter_period_cycles = 1;
		 i2c->one_eighth_period_cycles = 1;
	   i2c->one_sixteenth_period_cycles = 1;
	 }

    // set stretch timeout cycles by converting default ns timeout to cycles
    if( SystemCoreClock != 0){
      uint64_t cycles_timeout = ((uint64_t)SystemCoreClock * SOFTI2C_DEFAULT_STRETCH_TIMEOUT_NS) / 1000000000ULL;
      if (cycles_timeout == 0) cycles_timeout = 1;
      i2c->scl_stretch_timeout_cycles = (uint32_t)cycles_timeout;
	  }
	  else{
			i2c->scl_stretch_timeout_cycles = 1000;
		}

    // Ensure bus idle (SCL/SDA high)
    scl_high_direct();
    sda_high_direct();
    half_delay(i2c);

    return SOFTI2C_OK;
}

/* Start condition: SDA high->low while SCL high */
static inline __attribute__((always_inline)) void softi2c_start(softi2c_t *i2c)
{
    if (i2c->disable_irq_during_xfer) irq_disable_impl();

    sda_high_direct();
    scl_high_direct();
    half_delay(i2c);

    sda_low_direct();
    half_delay(i2c);
    scl_low_direct();
    half_delay(i2c);
}

/* Stop: SDA low -> high while SCL high */
static inline __attribute__((always_inline)) void softi2c_stop(softi2c_t *i2c)
{
    sda_low_direct();
    half_delay(i2c);
    scl_high_direct();
    half_delay(i2c);
    sda_high_direct();
    half_delay(i2c);

    if (i2c->disable_irq_during_xfer) irq_enable_impl();
}

/* Write one byte, return OK if ACK received (ACK=0). */
static inline __attribute__((always_inline)) softi2c_status_t softi2c_write_byte(softi2c_t *i2c, uint8_t b)
{
    if (!i2c) return SOFTI2C_ARG_ERROR;
    
    for (int i = 0; i < 8; ++i) {
        // 设置数据位
        if (b & 0x80) sda_high_direct(); 
        else sda_low_direct();
        b <<= 1;
        
        half_delay(i2c);        // SCL低电平保持时间
        scl_high_direct();   
        one_quarter_delay(i2c);        // SCL高电平保持时间  一般IIC高电平时间比低电平短
        scl_low_direct();
        // 移除末尾的 half_delay - 下一次循环的数据设置可以在此期间完成
    }
    
    // ACK检测
    sda_high_direct();
    half_delay(i2c);
    scl_high_direct();
    one_eighth_delay(i2c);     
    int ack = sda_read_direct();
		one_eighth_delay(i2c); 
    //half_delay(i2c);
    scl_low_direct();
    
    return (ack == 0) ? SOFTI2C_OK : SOFTI2C_NACK;
}

static inline __attribute__((always_inline)) uint8_t softi2c_read_byte(softi2c_t *i2c, bool send_nack)
{
    uint8_t v = 0;
    sda_high_direct();
    
    for (int i = 0; i < 8; ++i) {
        half_delay(i2c);
        scl_high_direct();
        one_eighth_delay(i2c);  // 减少采样延时
        if (sda_read_direct()) v |= (1 << (7-i));  // 直接移位到正确位置
			  //one_sixteenth_delay(i2c); 
        scl_low_direct();
    }
    
    // ACK/NACK
    (send_nack) ? sda_high_direct() : sda_low_direct();
    half_delay(i2c);
    scl_high_direct();
    one_quarter_delay(i2c);
    scl_low_direct();
    sda_high_direct();
    
    return v;
}

/* Low-level master write of raw bytes (no mem addr). dev_addr7 is 7-bit address. */
softi2c_status_t softi2c_master_write(softi2c_t *i2c, uint8_t dev_addr7,
                                      const uint8_t *data, uint16_t len)
{
    if (!i2c) return SOFTI2C_ARG_ERROR;

    softi2c_start(i2c);
    // send address (7-bit <<1 | 0)
    uint8_t addr_byte = (uint8_t)((dev_addr7 << 1) | 0x0);
    softi2c_status_t st = softi2c_write_byte(i2c, addr_byte);
    if (st != SOFTI2C_OK) {
        softi2c_stop(i2c);
        return st;
    }

    for (uint16_t i = 0; i < len; ++i) {
        st = softi2c_write_byte(i2c, data[i]);
        if (st != SOFTI2C_OK) {
            softi2c_stop(i2c);
            return st;
        }
    }

    softi2c_stop(i2c);
    return SOFTI2C_OK;
}

/* Low-level master read of raw bytes (no mem addr). */
softi2c_status_t softi2c_master_read(softi2c_t *i2c, uint8_t dev_addr7,
                                     uint8_t *data, uint16_t len)
{
    if (!i2c) return SOFTI2C_ARG_ERROR;

    softi2c_start(i2c);
    uint8_t addr_byte = (uint8_t)((dev_addr7 << 1) | 0x1);
    softi2c_status_t st = softi2c_write_byte(i2c, addr_byte);
    if (st != SOFTI2C_OK) {
        softi2c_stop(i2c);
        return st;
    }

    for (uint16_t i = 0; i < len; ++i) {
        bool nack = (i == (len - 1));
        data[i] = softi2c_read_byte(i2c, nack);
    }

    softi2c_stop(i2c);
    return SOFTI2C_OK;
}

/* Master mem write (mem_addr_size = 1 or 2) */
softi2c_status_t softi2c_master_mem_write(softi2c_t *i2c, uint8_t dev_addr7,
                                          uint16_t mem_addr, uint8_t mem_addr_size,
                                          const uint8_t *data, uint16_t len)
{
    if (mem_addr_size != 1 && mem_addr_size != 2) return SOFTI2C_ARG_ERROR;
    if (!i2c) return SOFTI2C_ARG_ERROR;

    softi2c_start(i2c);
    uint8_t addr_byte = (uint8_t)((dev_addr7 << 1) | 0x0);
    softi2c_status_t st = softi2c_write_byte(i2c, addr_byte);
    if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }

    if (mem_addr_size == 2) {
        st = softi2c_write_byte(i2c, (uint8_t)(mem_addr >> 8));
        if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }
    }
    st = softi2c_write_byte(i2c, (uint8_t)(mem_addr & 0xFF));
    if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }

    for (uint16_t i = 0; i < len; ++i) {
        st = softi2c_write_byte(i2c, data[i]);
        if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }
    }

    softi2c_stop(i2c);
    return SOFTI2C_OK;
}

/* Master mem read: write memaddr (1 or 2 bytes) then repeated start + read len bytes */
softi2c_status_t softi2c_master_mem_read(softi2c_t *i2c, uint8_t dev_addr7,
                                         uint16_t mem_addr, uint8_t mem_addr_size,
                                         uint8_t *data, uint16_t len)
{
    if (mem_addr_size != 1 && mem_addr_size != 2) return SOFTI2C_ARG_ERROR;
    if (!i2c) return SOFTI2C_ARG_ERROR;

    // Write memaddr
    softi2c_start(i2c);
    uint8_t addr_byte = (uint8_t)((dev_addr7 << 1) | 0x0);
    softi2c_status_t st = softi2c_write_byte(i2c, addr_byte);
    if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }

    if (mem_addr_size == 2) {
        st = softi2c_write_byte(i2c, (uint8_t)(mem_addr >> 8));
        if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }
    }
    st = softi2c_write_byte(i2c, (uint8_t)(mem_addr & 0xFF));
    if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }

    // repeated start + read
    // do not stop between
    // repeated start:
    sda_high_direct();
    half_delay(i2c);
    scl_high_direct();
    half_delay(i2c);
    sda_low_direct();
    half_delay(i2c);
    scl_low_direct();
    half_delay(i2c);

    // send address with read bit
    addr_byte = (uint8_t)((dev_addr7 << 1) | 0x1);
    st = softi2c_write_byte(i2c, addr_byte);
    if (st != SOFTI2C_OK) { softi2c_stop(i2c); return st; }

    for (uint16_t i = 0; i < len; ++i) {
        bool nack = (i == (len - 1));
        data[i] = softi2c_read_byte(i2c, nack);
    }
    softi2c_stop(i2c);
    return SOFTI2C_OK;
}
