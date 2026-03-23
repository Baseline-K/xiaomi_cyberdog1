//############################################################
// FILE:  Axis_transform.c
// Created on: 2017年1月11日
// Author: XQ
// summary:Axis_transform
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子  
//网址: https://shuolidianzi.taobao.com  
//Author-QQ: 616264123  
//电机控制QQ群：314306105  
//############################################################

#include "Axis_transform.h"
    
//Alpha = Iu
//Beta = (2^15*Ia + 2^15*2*Ib)/(√3*(2^15))

//void  CLARKE_Cale(p_CLARKE  pV)
//{
//	pV->Alpha = pV->As;
//	pV->Beta = _IQmpy((pV->As +_IQmpy2(pV->Bs)),18918);  //  _IQ(0.57735026918963)  1/√3放大2^15
//}

/*这里是等幅值变换，自动乘了系数2/3
  同时由于这里参与Q15定点数乘法_IQmpy计算的数1/√3为Q15类型定点数，
	因此CLARKE变换后的pV->Alpha和pV->Beta定点数格式与输入参数pV->As和pV->Bs一致*/
void  CLARKE_Cale(p_CLARKE  pV)      
{
	pV->Alpha = pV->As;
	pV->Beta = _IQmpy((pV->As +_IQmpy2(pV->Bs)),18918);  //  _IQ(0.57735026918963)
}

// Parking Id,Iq   d轴和alpha轴重合，q轴超前d轴90度
// Id = Ialpha*cos+Ibeta*sin
// Iq = Ibeta*cos-Ialpha*sin
/*由于这里参与Q15定点数乘法_IQmpy计算的数pV->Cosine和pV->Sine都为Q15类型定点数，
	因此PARK变换后的pV->Ds和pV->Qs定点数格式与输入参数pV->Alpha和pV->Beta一致*/
void  PARK_Cale(p_PARK pV)
{
	pV->Ds = _IQmpy(pV->Alpha,pV->Cosine) + _IQmpy(pV->Beta,pV->Sine);
  pV->Qs = _IQmpy(pV->Beta,pV->Cosine) - _IQmpy(pV->Alpha,pV->Sine);
}

//IParking Ia,Ib
// Ialpha = Id*cos-Iq*sin
// Ibeta = Iq*cos+Id*sin
/*由于这里参与Q15定点数乘法_IQmpy计算的数pV->Cosine和pV->Sine都为Q15类型定点数，
	因此IPARK变换后的pV->Alpha和pV->Beta定点数格式与输入参数pV->Ds和pV->Qs一致*/
void  IPARK_Cale(p_IPARK pV)    
{
	pV->Alpha = _IQmpy(pV->Ds,pV->Cosine) - _IQmpy( pV->Qs,pV->Sine);
	pV->Beta  = _IQmpy(pV->Qs,pV->Cosine) + _IQmpy(pV->Ds,pV->Sine);
}

//===========================================================================
// No more.
//===========================================================================

