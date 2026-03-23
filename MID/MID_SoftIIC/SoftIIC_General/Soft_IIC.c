#include "Soft_IIC.h"
#include "Soft_IIC_port.h"
#include <stdint.h>
#include <string.h>


/********************* SoftIIC *************************/

/* Internal: half period delay */
static inline void half_delay(softi2c_t *i2c)
{
    i2c->gpio.delay_cycles(i2c->half_period_cycles);
}

static inline void one_quarter_delay(softi2c_t *i2c)
{
    i2c->gpio.delay_cycles(i2c->one_quarter_period_cycles);
	 
}

static inline void one_eighth_delay(softi2c_t *i2c)
{
    i2c->gpio.delay_cycles(i2c->one_eighth_period_cycles);
	 
}

static inline void one_sixteenth_delay(softi2c_t *i2c)
{
    i2c->gpio.delay_cycles(i2c->one_sixteenth_period_cycles);
}

/*********************************使外设的IIC通讯退出死锁***************************/
//在IIC设备初始化之前调用
//IIC死锁解决办法： https://shequ.stmicroelectronics.cn/thread-623081-1-1.html

void I2C_Unstick(softi2c_t *i2c)
{
  // 1) 释放并反初始化硬件 I2C（若正在使用，请先确保安全停止）
    if(i2c->gpio.deinit_hw_i2c != NULL){
        i2c->gpio.deinit_hw_i2c();
    }

    i2c->gpio.set_pins_out_od();

    /* 1) 先释放总线（把 SCL/SDA 设为高——open-drain release） */
    i2c->gpio.scl_high();
    i2c->gpio.sda_high();
    half_delay(i2c);

    /* 2) 如果 SDA 已经是高（总线空闲），直接返回成功 */
    if (i2c->gpio.sda_read() != 0) {
        return;
    }
   
    /* 3) SDA 仍然为低，可能是从设备挂起在某个状态（例如：SDA 被拉低等待 ACK）；
     *    我们通过产生最多I2C_UNSTICK_MAX_CLOCKS 个 SCL 脉冲来尝试释放它（9 个字节 + ACK）
     */
    for (int i = 0; i < I2C_UNSTICK_MAX_CLOCKS; i++) {
        // 产生 SCL 脉冲
        i2c->gpio.scl_low(); 
        half_delay(i2c);
        i2c->gpio.scl_high();
        half_delay(i2c);

        // 每次都检测 SDA 是否释放
        if (i2c->gpio.sda_read() != 0)
            break;
    }
 
    // 4. 模拟 STOP 条件释放总线（SCL 高时，SDA 从低拉到高）
    i2c->gpio.sda_low();
    half_delay(i2c);
    i2c->gpio.scl_high();
    half_delay(i2c);
    i2c->gpio.sda_high();
    half_delay(i2c);

    if(i2c->gpio.reinit_hw_i2c != NULL){
        i2c->gpio.reinit_hw_i2c();
    }
}

/* Helper: wait for SCL high (clock stretching) */
static inline bool wait_scl_high(softi2c_t *i2c)
{
    // 如果用户提供了 scl_read 回调（可选），使用它做 clock-stretch 检测
    if (i2c->gpio.scl_read == NULL) return false; 

    uint32_t t = i2c->scl_stretch_timeout_cycles;
    while (t--) {
        if (i2c->gpio.scl_read()) return true;
        // else fallback to tiny delay
        i2c->gpio.delay_cycles(1);
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
    if (!i2c  || !i2c->gpio.scl_high || !i2c->gpio.scl_low ||
        !i2c->gpio.sda_high || !i2c->gpio.sda_low || !i2c->gpio.sda_read || !i2c->gpio.delay_cycles
        || !i2c->gpio.set_pins_out_od || !i2c->gpio.irq_enable || !i2c->gpio.irq_disable) {
        return SOFTI2C_ARG_ERROR;
    }

    I2C_Unstick(i2c);  // 确保总线空闲

    // 1) 释放并反初始化硬件 I2C（若正在使用，请先确保安全停止）
    if(i2c->gpio.deinit_hw_i2c != NULL){
        i2c->gpio.deinit_hw_i2c();
    }
    
    // 2) 配置 SCL/SDA 为 Open-Drain 输出（GPIO）
    i2c->gpio.set_pins_out_od();

		 /* 保存 gpio 回调到本地临时结构，防止下面的清零把它们擦掉 */
    softi2c_gpio_if_t saved_gpio;
    memcpy(&saved_gpio, &i2c->gpio, sizeof(saved_gpio));
		
		/* 清除除 gpio 之外的其他字段 */
    memset(i2c, 0, sizeof(*i2c));
    /* 恢复 gpio 指针 */
    memcpy(&i2c->gpio, &saved_gpio, sizeof(saved_gpio));
		
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
    i2c->gpio.scl_high();
    i2c->gpio.sda_high();
    half_delay(i2c);

    return SOFTI2C_OK;
}

/* Start condition: SDA high->low while SCL high */
void softi2c_start(softi2c_t *i2c)
{
    if (i2c->disable_irq_during_xfer) i2c->gpio.irq_disable();

    i2c->gpio.sda_high();
    i2c->gpio.scl_high();
    one_quarter_delay(i2c);

    i2c->gpio.sda_low();
    half_delay(i2c);
    i2c->gpio.scl_low();
    half_delay(i2c);
}

/* Stop: SDA low -> high while SCL high */
void softi2c_stop(softi2c_t *i2c)
{
    i2c->gpio.sda_low();
    half_delay(i2c);
    i2c->gpio.scl_high();
    one_quarter_delay(i2c);
    i2c->gpio.sda_high();
    one_quarter_delay(i2c);

    if (i2c->disable_irq_during_xfer) i2c->gpio.irq_enable();
}

/* Write one byte, return OK if ACK received (ACK=0). */
softi2c_status_t softi2c_write_byte(softi2c_t *i2c, uint8_t b)
{
    if (!i2c) return SOFTI2C_ARG_ERROR;
    
    for (int i = 0; i < 8; ++i) {
        // 设置数据位
        if (b & 0x80) i2c->gpio.sda_high(); 
        else i2c->gpio.sda_low();
        b <<= 1;
        
        half_delay(i2c);        // SCL低电平保持时间
        i2c->gpio.scl_high();   
        one_quarter_delay(i2c);        // SCL高电平保持时间  一般IIC高电平时间比低电平短
        i2c->gpio.scl_low();
        // 移除末尾的 half_delay - 下一次循环的数据设置可以在此期间完成
    }
    
    // ACK检测
    i2c->gpio.sda_high();
    half_delay(i2c);
    i2c->gpio.scl_high();
    one_eighth_delay(i2c);     
    int ack = i2c->gpio.sda_read();
	one_eighth_delay(i2c); 
    //half_delay(i2c);
    i2c->gpio.scl_low();
    
    return (ack == 0) ? SOFTI2C_OK : SOFTI2C_NACK;
}

uint8_t softi2c_read_byte(softi2c_t *i2c, bool send_nack)
{
    uint8_t v = 0;
    i2c->gpio.sda_high();
    
    for (int i = 0; i < 8; ++i) {
        half_delay(i2c);
        i2c->gpio.scl_high();
        one_quarter_delay(i2c);  // 减少采样延时
        if (i2c->gpio.sda_read()) v |= (1 << (7-i));  // 直接移位到正确位置
		//one_sixteenth_delay(i2c); 
        i2c->gpio.scl_low();
    }
    
    // ACK/NACK
    (send_nack) ? i2c->gpio.sda_high() : i2c->gpio.sda_low();
    half_delay(i2c);
    i2c->gpio.scl_high();
    one_quarter_delay(i2c);
    i2c->gpio.scl_low();
    i2c->gpio.sda_high();
    
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
    i2c->gpio.sda_high();
    half_delay(i2c);
    i2c->gpio.scl_high();
    half_delay(i2c);
    i2c->gpio.sda_low();
    half_delay(i2c);
    i2c->gpio.scl_low();
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
