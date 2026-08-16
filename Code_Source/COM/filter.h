#ifndef __FILTER_H__
#define __FILTER_H__

#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <arm_math.h>

//卡尔曼滤波结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.00004
	  float R;//观测噪声协方差  
}KFP;   //Kalman Filter parameter
/*由于数据输入误差为10mV，运放+adc采样转换误差为5mv,假设两种误差符合正态分布
由于两种误差独立，则两种误差的方差相加约为0.0001，因此初始化值为0.0001
*/

//初始化参数设置
#define KFP_AD7691_voltage_Default  {0.02,0,0,0,0.00002,0.0001}


//一阶低通滤波结构体类型定义
typedef struct {
	    float      Samp_Time;     //采样时间间隔，单位s
			float	     Cut_Freq;      //截止频率，单位HZ
	    float      K1;            // 一阶低通滤波系数 K1，历史值权重
	    float      K2;            // 一阶低通滤波系数 K2，测量值权重
	    float      Fitter;        //滤波值
      float      Old;           //历史值
}LPF;   //Low-pass filter parameter

//初始化参数设置
#define LPF_AD7691_voltage_Default  {0.001,10,0,0,0,0}   //


//滑动平均滤波结构体类型定义
typedef struct {
    float *Date;         // 采样历史数据（动态数组）
    float Total_value;   // 数据总和值
    float Average_value; // 数据平均值
    int   WindowSize;    // 滑动窗口大小
    int   head;          // 环形队列头部索引

    int   downsample_n;    // 降采样因子，每接收 n 个数据才参与一次平均滤波
    int   downsample_cnt;  // 当前计数
} MAF;  //Moving-average filter parameter

//初始化参数设置
#define MAF_AD7691_voltage_Default  {{0},0,0}


#define MAX_WINDOW 30   // 允许的最大滤波窗口大小
//中值滤波结构体类型定义
typedef struct {
	  float *Date;     // 采样历史数据，动态分配
    float Median_value; // 数据中值
    int Index;         // 当前写入索引
    int WindowSize;    // 窗口大小
}MFP;   //Median_filter parameter

//初始化参数设置
#define MFP_AD7691_voltage_Default  {{0},0,0}



//锁相环PLL滤波结构体类型定义
typedef struct {

    float   eleangle_Sinx;   //对采样输入电角度的正弦余弦值进行计算
    float   eleangle_Cosx;
	  float   PLLAngle_Sinx;   //对锁相环电角度的正弦余弦值进行计算
    float   PLLAngle_Cosx;
	   
		float   PLL_Theta;          //PLL滤波后的电角度
		float   PLL_DeltTheta;      //PLL电角度和采样电角度的error差值
		float   PLL_DeltTheta_Sum;   //PLL电角度和采样电角度的error差值的积分
		float  PLL_DeltTheta_Sum_Maxlimit;        //PI环路滤波器积分项上限
	  float  PLL_DeltTheta_Sum_Minlimit;        //PI环路滤波器积分项下限
	
		float   PLL_Omega;           //PLL预测的角速度
		float   PLL_Omega_filtered;  //PLL预测的角速度滤波后值
	  float   PLL_Omega_Maxlimit;           //PLL预测的角速度上限
		float   PLL_Omega_Minlimit;           //PLL预测的角速度下限

	  uint8_t    initialized_Flag;          // 是否已初始化（第一次接收）
	  float  FOC_FREQ;
	  float  SpeedPllBandwidth;
	  float  SpeedPllKp;
		float  SpeedPllKi;
	  float  OmegaToTheta;            //电角速度转电角度系数
	  float  deadband;                //死区,避免噪声在小误差下触发积分/PI动作
		
		
		MFP *fliter_struct;
}PLL;

#define M_PI 3.141592653589793f

float kalmanfilter(KFP *kfp,float input);

float lowpass_filter(LPF *lpf,float input);
void  init_lowpass_filter(LPF *lpf, float Samp_Time, float Cut_Freq);

MAF* mov_average_filter_init(int windowSize, int downsample_n);
void mov_average_filter_deinit(MAF *maf);
void mov_average_filter_clear(MAF *maf);
float mov_average_filter(MAF *maf,float input);

MFP* median_filter_init(int window_size);
void median_filter_deinit(MFP *mfp);
float median_filter(MFP *mfp, float new_sample);
int compare(const void *a, const void *b);
void median_filter_fill(MFP *mfp, float seed);

PLL* Angle_PLL_filter_init(float FOC_FREQ, float PllBandwidth, float deadband, float Max_Angle_Speed);
float Angle_PLL_filter(PLL *PLL_Filter, float eleAngle_input);
void Angle_PLL_filter_clear(PLL *PLL_Filter) ;
void Angle_PLL_filter_deinit(PLL *pll);
#endif 

