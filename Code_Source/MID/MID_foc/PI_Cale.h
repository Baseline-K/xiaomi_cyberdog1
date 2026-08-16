//############################################################
// FILE:  PI_Cale.h
// Created on: 2017年1月11日
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

#ifndef  PI_Cale_H
#define  PI_Cale_H

#include "IQ_math.h"
#include "stm32f1xx_hal.h"
#include "FOC_run.h"
#include "ThreeHall.h"

typedef struct {
			 int32_t  Kp;		 // PI控制的比例参数 	 
			 int32_t  Ki;		 // PI控制的积分参数 	  
			 int32_t  Umax;	 // PI控制的输出限制最大幅值参数 	  	 
			 int32_t  Umin;	 // PI控制的输出限制最小幅值参数 	 
	
	     int32_t  Ref;   // PI控制的给定参数，速度环即给定的速度单位是rpm
			 int32_t  Fbk;   // PI控制的反馈参数，速度环即检测的速度单位是rpm 	
	     int32_t  FbkF;  // PI控制的dq电流滤波参数
			 int32_t  Out;   // PI控制的限幅后输出参数 
			 //int32_t  OutF;  // PI控制的限幅后滤波输出参数，速度环输出作为电流环的pi_iq.Ref，电流环输出作为park逆变换的输入电压
	     int32_t  error; // 目标值和反馈值的误差
			 int32_t  up;		 // PI控制的比例项输出参数 
			 int32_t  ui;		 // PI控制的积分项输出参数 	  	 
			 int32_t  i1;		 // PI控制的历史积分项输出参数 	 		
       int32_t  v1;		 // PI控制的输出项参数 	

	     int32_t (*FF)();    //PI控制前馈项,由于是可变的，因此这里为函数指针
	   } PI_Control, *p_PI_Control ;



#define PI_Control_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,NULL}  // 初始化参数

void  PI_Controller(p_PI_Control  pV);  //PI控制算法函数
void PI_Controller_conditional(p_PI_Control pV);
void  PI_Pare_init(void );    //PI控制参数初始化
#endif /* PI_Cale*/
