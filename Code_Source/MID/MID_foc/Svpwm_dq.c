//############################################################
// FILE:  Svpwm_dq.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Svpwm_dq
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com       
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "Svpwm_dq.h"

/*由于这里参与Q15定点数乘法_IQmpy计算的数sqrt(3)/2为Q15类型定点数，
	因此SVPWM变换后的pV->Ta和pV->Tb、pV->Tc定点数格式与输入参数pV->Ubeta和pV->Ualpha一致*/
void  SVPWM_Cale(p_SVPWM  pV)
{
  pV->tmp1= pV->Ubeta;   // 相当于二相静止坐标--到三相静止变换出Uabc
	pV->tmp2= _IQdiv2(pV->Ubeta) + _IQmpy(28377,pV->Ualpha); //28377 = 0.866*32768=sqrt(3)/2*2^15
  pV->tmp3= pV->tmp2 - pV->tmp1;

	pV->VecSector=3;   // 根据三相电压符号计算矢量扇区
	pV->VecSector=(pV->tmp2> 0)?( pV->VecSector-1):pV->VecSector;  //可能扇区号  是2：否3
	pV->VecSector=(pV->tmp3> 0)?( pV->VecSector-1):pV->VecSector;  //可能扇区号  是是1：否是2   是否2：否否3
	pV->VecSector=(pV->tmp1< 0)?(7-pV->VecSector) :pV->VecSector;  //最终扇区号  是是是6：是是否1   否是是5：否是否2   是否是5：是否否2   否否是4：否否否3

	if  (pV->VecSector==1 || pV->VecSector==4)  // 根据矢量扇区计算矢量占空比Tabc，输出值为Q15定点数的-1到+1
	{  
		pV->Ta= pV->tmp2;
		pV->Tb= pV->tmp1-pV->tmp3;
		pV->Tc=-pV->tmp2;
	}
  else if(pV->VecSector==2 || pV->VecSector==5)
	{  
		pV->Ta= pV->tmp3+pV->tmp2;
		pV->Tb= pV->tmp1;
		pV->Tc=-pV->tmp1;
	}
  else
	{   
		pV->Ta= pV->tmp3;
		pV->Tb=-pV->tmp3;
		pV->Tc=-(pV->tmp1+pV->tmp2);
	}
	
	//输出限幅
	 if(pV->Ta > _IQ15(1)) 
		  pV->Ta = _IQ15(1);
	 else if(pV->Ta < -_IQ15(1))
	    pV->Ta = -_IQ15(1);
	 
	  if(pV->Tb > _IQ15(1)) 
		  pV->Tb = _IQ15(1);
	 else if(pV->Tb < -_IQ15(1))
	    pV->Tb = -_IQ15(1);
	 
	  if(pV->Tc > _IQ15(1)) 
		  pV->Tc = _IQ15(1);
	 else if(pV->Tc < -_IQ15(1))
	    pV->Tc = -_IQ15(1);
}

//===========================================================================
// No more.
//===========================================================================
