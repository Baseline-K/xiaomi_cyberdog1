#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>
#include <stdbool.h>

/* 返回码 */
typedef enum {
    SOFTI2C_OK = 0,
    SOFTI2C_NACK,
    SOFTI2C_TIMEOUT,
    SOFTI2C_BUS_ERROR,
    SOFTI2C_ARG_ERROR
} softi2c_status_t;


/* 主句柄 */
typedef struct {
    uint32_t half_period_cycles; // 半个 SCL 周期对应的 CPU 周期数 (calculated from freq)
	  uint32_t one_quarter_period_cycles;
	  uint32_t one_eighth_period_cycles;
	  uint32_t one_sixteenth_period_cycles;
    uint32_t scl_freq_hz;        // e.g. 1000000 for 1 MHz
    uint32_t scl_stretch_timeout_cycles; // 等待 SCL 被释放（cycle）超时
    bool disable_irq_during_xfer; // 若 true 在 byte 发送/接收关键段禁中断
} softi2c_t;
extern softi2c_t g_softi2c;

/* 初始化：
 *  - iface: 由用户填充的 gpio 回调集合
 *  - scl_freq_hz: 目标 SCL 频率（建议 100000..1000000）
 *  - disable_irq_during_xfer: 对 1MHz 建议 true
 */
softi2c_status_t softi2c_init(softi2c_t *i2c, 
                              uint32_t scl_freq_hz, bool disable_irq_during_xfer, uint32_t SystemCoreClock);
void SoftI2C_RegisterAndInit(uint32_t target_freq_hz, uint32_t SystemCoreClock);
/* 基本操作 */
void I2C_Unstick(softi2c_t *i2c);
void softi2c_start(softi2c_t *i2c);
void softi2c_stop(softi2c_t *i2c);
softi2c_status_t softi2c_write_byte(softi2c_t *i2c, uint8_t b); // 返回 SOFTI2C_OK 或 SOFTI2C_NACK/Timeout
uint8_t softi2c_read_byte(softi2c_t *i2c, bool send_nack);

/* 高层：master mem write / mem read（支持 8bit memaddr 与 16bit memaddr） */
/* mem_addr_size = 1 or 2 */
softi2c_status_t softi2c_master_mem_write(softi2c_t *i2c, uint8_t dev_addr7,
                                          uint16_t mem_addr, uint8_t mem_addr_size,
                                          const uint8_t *data, uint16_t len);
softi2c_status_t softi2c_master_mem_read(softi2c_t *i2c, uint8_t dev_addr7,
                                         uint16_t mem_addr, uint8_t mem_addr_size,
                                         uint8_t *data, uint16_t len);

/* 直接 write/read 原始 bytes（dev_addr7 为 7-bit 地址） */
softi2c_status_t softi2c_master_write(softi2c_t *i2c, uint8_t dev_addr7,
                                      const uint8_t *data, uint16_t len);
softi2c_status_t softi2c_master_read(softi2c_t *i2c, uint8_t dev_addr7,
                                     uint8_t *data, uint16_t len);

#endif // SOFT_I2C_H

								
