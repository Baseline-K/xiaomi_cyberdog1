#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "main.h"
//#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>


#define AS5600_I2C_HANDLE hi2c1

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

/*
注意:AS5600的地址0x36是指的是原始7位设备地址,而ST I2C库中的设备地址是指原始设备地址左移一位得到的设备地址
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)




#define AS5600_RESOLUTION 4096U //12bit Resolution

#define AS5600_CONF_HIGH_REGISTER  0x07U
#define AS5600_CONF_LOW_REGISTER   0x08U
#define AS5600_CONF_MASK           0x3FFFU
#define AS5600_CONF_RESERVED_MASK  0xC000U
#define AS5600_RAW_ANGLE_REGISTER  0x0CU  //only read

#define AS5600_STATUS_REGISTER  0x0BU              //only read
#define AS5600_STATUS_MD_MASK   0x20U              //magnet detected
#define AS5600_STATUS_ML_MASK   0x10U              //magnet too weak
#define AS5600_STATUS_MH_MASK   0x08U              //magnet too strong

#define AS5600_ZPOS11_8_REGISTER 0x01
#define AS5600_ZPOS7_0_REGISTER 0x02

#define AS5600_BURN_REGISTER  0xFF               //only write
#define Burn_Angle  0x80
#define Burn_Setting  0x40
//在寄存器0xFF内写入值0x80来执行烧录角度命令。烧录角度命令最多可以执行三次  
//在寄存器0xFF内写入值0x40来执行烧录设置命令。只有在ZPOS和MPOS从未被永久性写入时(ZMCO = 00)才可对MANG进行写入。 烧录设置命令只能被执行一次。 

/* CONF register fields. Numeric values match the AS5600 datasheet. */
typedef enum {
    AS5600_POWER_MODE_NOM  = 0U,
    AS5600_POWER_MODE_LPM1 = 1U,
    AS5600_POWER_MODE_LPM2 = 2U,
    AS5600_POWER_MODE_LPM3 = 3U
} AS5600_PowerMode_t;

typedef enum {
    AS5600_HYSTERESIS_OFF   = 0U,
    AS5600_HYSTERESIS_1_LSB = 1U,
    AS5600_HYSTERESIS_2_LSB = 2U,
    AS5600_HYSTERESIS_3_LSB = 3U
} AS5600_Hysteresis_t;

typedef enum {
    AS5600_OUTPUT_ANALOG_FULL    = 0U,
    AS5600_OUTPUT_ANALOG_REDUCED = 1U,
    AS5600_OUTPUT_PWM            = 2U
} AS5600_OutputStage_t;

typedef enum {
    AS5600_PWM_115_HZ = 0U,
    AS5600_PWM_230_HZ = 1U,
    AS5600_PWM_460_HZ = 2U,
    AS5600_PWM_920_HZ = 3U
} AS5600_PwmFrequency_t;

typedef enum {
    AS5600_SLOW_FILTER_16X = 0U,
    AS5600_SLOW_FILTER_8X  = 1U,
    AS5600_SLOW_FILTER_4X  = 2U,
    AS5600_SLOW_FILTER_2X  = 3U
} AS5600_SlowFilter_t;

typedef enum {
    AS5600_FAST_FILTER_SLOW_ONLY = 0U,
    AS5600_FAST_FILTER_6_LSB     = 1U,
    AS5600_FAST_FILTER_7_LSB     = 2U,
    AS5600_FAST_FILTER_9_LSB     = 3U,
    AS5600_FAST_FILTER_18_LSB    = 4U,
    AS5600_FAST_FILTER_21_LSB    = 5U,
    AS5600_FAST_FILTER_24_LSB    = 6U,
    AS5600_FAST_FILTER_10_LSB    = 7U
} AS5600_FastFilterThreshold_t;

typedef struct {
    AS5600_PowerMode_t power_mode;
    AS5600_Hysteresis_t hysteresis;
    AS5600_OutputStage_t output_stage;
    AS5600_PwmFrequency_t pwm_frequency;
    AS5600_SlowFilter_t slow_filter;
    AS5600_FastFilterThreshold_t fast_filter_threshold;
    bool watchdog_enable;
} AS5600_Config_t;

typedef struct {
    uint16_t config_raw;
    uint8_t status;
    int filter_config_result;
    int config_read_result;
    int status_read_result;
} AS5600_StartupDiagnostics_t;

extern AS5600_StartupDiagnostics_t AS5600_StartupDiagnostics;



typedef struct {
			volatile float angle;        
			volatile float angle_last;
	    float angle_init;   //初始角度
	    float angle_zero_offset;       //机械零点和电气零点偏移量
	    float eleangle;     //根据电机极对数换算的电角度
	
	    float       Speed_angle_deta;   //速度角度值（计算速度），即两次角度之差
	    float       Speed_angle_deta_last;
	    float       Speed_angle_deta_Max;
	    float       Speed_angle_deta_Min;
	    float       Speed_RPM;         //电机旋转速度  
	    float       Speed_RPM_last;       	
	    
    	uint8_t       Move_State;        //电机旋转状态  1为正转 0为反转
	    uint8_t       Sample_Mode;        //采样模式，分为速度采样和位置采样、
	    uint32_t      rotation_circles;   //转过的整圈数  只算一个方向，正转一圈加1，反转一圈减1
	    float         CCW_angle_total;   //Forward rotation angle
			float         CW_angle_total;    //Reverse direction angle
	    uint32_t      rotation_circles_absolute;   //转过的整圈数，正反转都增加，为正值
	    float         rotation_angle_absolute;


	
	    /*** DMA variable***/
	    volatile bool dma_transfer_complete;
	    uint8_t DMA_rxbuffer[2];
	    uint8_t DMA_cnt;                    // 等待DMA获取次数
	    uint8_t n_timeout;                  // 超时阈值（n）
     
			
}Encoder_AS5600_t;
extern   Encoder_AS5600_t   Encoder_AS5600;
				 
		 
#define  AS5600_DEFAULTS {0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,0,  2,0,0,0,0,0} // 初始化参数

int AS5600_ReadAngle_SoftI2C(volatile float *Angle);	
	
/*
 * Call once before the FOC sampling ISR starts. Initializes 1 MHz Soft-I2C,
 * applies volatile low-latency SF=2x/FTH=slow-only configuration with readback,
 * caches CONF/STATUS diagnostics, then restores the RAW_ANGLE read pointer.
 */
int bsp_as5600HwInit(void);
/* Software-state reset only; performs no I2C transaction. */
void bsp_as5600ResetTracking(void);
//int bsp_as5600GetRawAngle(uint16_t *raw_angle) ;
int  bsp_as5600GetAngle(volatile float *Angle) ;
/* Call configuration/register APIs only while the FOC ISR angle read is stopped. */
int AS5600_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length);
int AS5600_WriteRegisters(uint8_t reg, const uint8_t *data, uint16_t length);
/* Raw read returns all 16 register bits, including factory-reserved bits. */
int AS5600_ReadConfigRaw(uint16_t *raw_config);
/* Raw write changes only defined CONF bits and preserves factory-reserved bits. */
int AS5600_WriteConfigRaw(uint16_t raw_config, bool verify);
int AS5600_ReadConfig(AS5600_Config_t *config);
int AS5600_WriteConfig(const AS5600_Config_t *config, bool verify);
int AS5600_SetFilterConfig(AS5600_SlowFilter_t slow_filter,
                           AS5600_FastFilterThreshold_t fast_filter_threshold,
                           bool verify);
int AS5600_ReadStatus(uint8_t *status);
int AS5600_BurnAngle(void);
int bsp_AS5600_SetZero(uint8_t Burn);
float bsp_as5600GetSpeed(float AS5600_SpeedFactor);
int bsp_as5600_DMAGetAngle(volatile float *Angle) ;
#endif /* __BSP_AS5600_H */





