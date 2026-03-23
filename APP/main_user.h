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


void main_user(void);
void Offset_CurrentReading(void);
void Offset_Current_Start(void);
void ADC_Sample(void);
void Param_init(void);
void Param_deinit(void);
/*********** HAL extern*************/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


extern PLL *Encoder_PLL_mecFilter;
extern PLL *Encoder_PLL_eleFilter;
#endif  // end of Task_function_H definition

//===========================================================================
// End of file.
//===========================================================================
