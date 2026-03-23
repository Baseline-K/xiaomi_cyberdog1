//############################################################
// FILE: IQ_math.h
// Created on: 2017年1月18日
// Author: XQ
// summary: IQ_math_ 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
   
#ifndef _IQ_math_H
#define _IQ_math_H 

#include "main.h"
//共256个数
 
#define Abs(A)    ((A>=0)?A:-A)  // 绝对值函数
#define Min(A,B)  ((A<=B)?A:B)   // 求最小函数
#define Max(A,B)  ((A>=B)?A:B)   // 求最大函数

/* 这个A是一个带小数的浮点数，将其放大32768倍然后强制转为整数，计算完后再缩小32768倍强制转回浮点数，
   这样在尽可能不丢失精度的情况下将浮点数转为整数计算，没有单片机FPU的情况下能大大提高。
	 32位Q15格式定点数表示浮点数范围-32768------32767.999984741 ，当浮点数在这个范围内可以用Q15格式，精度1/32768 */
#define   _IQ15(A)       (int32_t)((A)<<15)  //左移15位 32768     
#define   _IQ(A)          _IQ15(A)           //定义IQ格式 32768
#define   _IQmpy(A,B)    (int32_t)((A*B)>>15)  //IQ格式相乘，这里的AB是已经转为IQ15格式的了，两个IQ15格式相乘为IQ30格式，需要再右移15为将结果转回IQ15格式
#define   _IQdiv(A,B)    (int32_t)((A<<15)/B)  //IQ格式相除，这里的AB是已经转为IQ15格式的了，两个IQ15格式相除为IQ0格式，需要再左移15为将结果转回IQ15格式
#define   _IQdiv2(A)     (int32_t)((A)>>1)     //除2
#define   _IQmpy2(A)     (int32_t)(A<<1)       //乘2

#define   _IQ10mpy(A,B)   (int32_t)((A*B)>>10) //IQ10格式相乘 




#define SIN_RAD     0x0300
#define U0_90       0x0000 
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300


typedef struct 	{ 
	        int32_t  IQAngle;  // 电机磁极位置角度0---65536即是0---360度 		
				  int32_t  IQSin;		 // IQ格式正弦参数，-32768---32767  -1到1 
				  int32_t  IQCos;		 // IQ格式余弦参数，-32768---32767  -1到1	 
				} IQSin_Cos , *p_IQSin_Cos;


#define IQSin_Cos_DEFAULTS  { 0,0,0} // 初始化参数

typedef struct 	{ 
	        int32_t  Alpha; 	//二相静止坐标系 Alpha 轴	 
				  int32_t  Beta;		//二相静止坐标系 Beta 轴	 	 
				  int32_t  IQTan;		//IQ格式正切 45度正切是1，IQ的格式是 
  				int32_t  IQAngle;	//IQ格式角度值 0---65536 == 0---360度 
	        int32_t  JZIQAngle; //矫正IQ格式角度值
         } IQAtan , *p_IQAtan;


#define IQAtan_DEFAULTS  {0,0,0,0,0}  // 初始化参数

uint32_t IQSqrt(uint32_t  M); // 开方函数
void  IQSin_Cos_Cale(p_IQSin_Cos pV); //将当前计算的IQ电角度对应的正余弦值用查表法找出最接近的
void  IQAtan_Cale(p_IQAtan  pV) ;  //根据Alpha和Beta轴的值算出tan正切值，再根据正切值查表找出最接近的角度
int32_t IQsat( int32_t Uint,int32_t  U_max, int32_t U_min); //限制赋值函数
#endif /* __IQ_math_H */

//===========================================================================
// No more.
//===========================================================================
