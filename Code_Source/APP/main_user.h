//############################################################
// FILE:  Task_function.h
// Created on: 2017年1月5日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef MAIN_USER_H
#define MAIN_USER_H

  
#include "main.h"
#include "arm_math.h"

#include "filter.h"

typedef struct {
	     int32_t   BUS_Voltage ;  //母线电压DC Bus  Voltage	     
	     int32_t   RP_speed_Voltage ;   // 电位器电压 RP1_Voltage
}ADCSamp;
extern    ADCSamp     ADCSampPare;


extern uint8_t aRxBuffer[100];

extern uint16_t  ADC_ConvertedValue[2];

/* ===== ADC DMA 缓冲（2026-09-27 双编码器改造后）=====
 * ADC1 规则组 6 通道：输出端 6 路线性霍尔（**连续转换 + DMA1_CH1 循环填充**）。
 *   缓冲下标 = ADC1 规则组 rank 顺序 = CH4, CH5, CH6, CH7, CH14, CH15
 *   -> PA4, PA5, PA6, PA7, PC4, PC5
 * 母线电压不在这里：它走 **ADC3 注入 rank2**（与电流一起被 TIM1_TRGO 触发、
 * PWM 同步），由 FOC ISR 读进全局 Vbus，慢安全也直接用那个值。
 * ADC3 的规则组**故意不启动** —— 见 main_user.c 里的说明。 */
extern volatile uint16_t Hall_ADC_DMA[6];


void main_user(void);
void Offset_CurrentReading(void);
/* 返回 0 = 零偏校准完成；-1 = 未采满（已退中点 + 上报 FAULT_CURRENT_SENSE_MASK）。
 * 调用方必须检查：失败时禁止继续强拖 / 使能功率。 */
int  Offset_Current_Start(void);
void ADC_Sample(void);
void Param_init(void);
void Param_deinit(void);
/*********** HAL extern*************/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;   /* 2026-09-27：母线电压（注入组给 FOC / 规则组给慢安全） */

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


extern PLL *Encoder_PLL_mecFilter;
extern PLL *Encoder_PLL_eleFilter;

/* --- 电流零偏校准自检（RTT `get` 的 cur 行） -------------------------------
 * done  = 1 表示上电校准采满 OFFSET_CAL_SAMPLES 拍并算完均值；
 *         0 = 采样链没起来（超时），此时已退回中点 ADC_MID_SCALE 并上报
 *         FAULT_CURRENT_SENSE_MASK，main_user 会跳过强拖与编码器零位校准。
 * count = 实际采到的拍数（诊断用，done=1 时应等于 OFFSET_CAL_SAMPLES）。
 * 定义在 main_user.c；这里只做只读声明，其它模块（RTT_Cmd）不要写。 */
extern volatile uint8_t  g_offset_cal_done;
extern volatile uint16_t g_offset_cal_count;
#endif  // end of Task_function_H definition

//===========================================================================
// End of file.
//===========================================================================
