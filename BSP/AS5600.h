#ifndef __BSP_AS5600_H
#define __BSP_AS5600_H

#include "main.h"
//#include "i2c.h"
#include <stdbool.h>


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




#define AS5600_RESOLUTION 4096 //12bit Resolution 
#define AS5600_RAW_ANGLE_REGISTER  0x0C  //only read

#define AS5600_STATUS_REGISTER  0x0B               //only read
// Many implementations use 0x20 for MD (bit5). If unsure, confirm with your datasheet revision.
#define AS5600_STATUS_MD_MASK   0x20U

#define AS5600_ZPOS11_8_REGISTER 0x01
#define AS5600_ZPOS7_0_REGISTER 0x02

#define AS5600_BURN_REGISTER  0xFF               //only write
#define Burn_Angle  0x80
#define Burn_Setting  0x80
//在寄存器0xFF内写入值0x80来执行烧录角度命令。烧录角度命令最多可以执行三次  
//在寄存器0xFF内写入值0x40来执行烧录设置命令。只有在ZPOS和MPOS从未被永久性写入时(ZMCO = 00)才可对MANG进行写入。 烧录设置命令只能被执行一次。 



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
	
void bsp_as5600Init(void);
//int bsp_as5600GetRawAngle(uint16_t *raw_angle) ;
int  bsp_as5600GetAngle(volatile float *Angle) ;
int bsp_AS5600_SetZero(uint8_t Burn);
float bsp_as5600GetSpeed(float AS5600_SpeedFactor);
int bsp_as5600_DMAGetAngle(volatile float *Angle) ;
#endif /* __BSP_AS5600_H */





