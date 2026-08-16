 //############################################################
// FILE:  PI_Cale.c
// Created on: 2017??1??4??
// Author: XQ
// summary: PI_Cale
//???????????????��????????????????????????��????
//??????��???????
//DSP/STM32????????????
//???????
//???: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//???????QQ???314306105
//############################################################

#include "PI_Cale.h"

extern  PI_Control   pi_spd ;
extern  PI_Control   pi_id  ;
extern  PI_Control   pi_iq  ;

#define PI_mode 1  //0：Q10定点数格式位置式模式   


/***/
void  PI_Controller(p_PI_Control  pV)   //位置式PI
{
    /* proportional term */
	pV->error = pV->Ref - pV->FbkF;     //计算误差err

	/* integral term */
  #if PI_mode == 0
	pV->ui = _IQ10mpy(pV->Ki, pV->up)+ pV->i1;        //(pV->Ki)* (pV->up) + pV->i1;//ki(n) = ki*err + ki(n-1) = ki*err(n)
	pV->ui = IQsat( pV->ui, pV->Umax,  pV->Umin);     //ki(n)积分抗饱和
	pV->i1 = pV->ui;                                  //ki(n-1) = ki(n) 

	/* control output */
	pV->v1 = _IQ10mpy(pV->Kp, (pV->up )) + pV->ui;	       //(pV->Kp)* (pV->up) + pV->ui;//PI输出 = Kp*err + ki*err(n) 
	pV->Out = IQsat( pV->v1, pV->Umax,  pV->Umin); //PI输出限幅

	#elif  PI_mode == 1
	pV->up = _IQ10mpy(pV->Kp, (pV->error));
	
	pV->ui = _IQ10mpy(pV->Ki, pV->error)+ pV->i1;        //(pV->Ki)* (pV->up) + pV->i1;//ki(n) = ki*err + ki(n-1) = ki*err(n)
	pV->ui = IQsat( pV->ui, pV->Umax,  pV->Umin);     //ki(n)积分抗饱和
	pV->i1 = pV->ui;                                  //ki(n-1) = ki(n) 

	/* control output */
	pV->v1 =  pV->up + pV->ui;	       //(pV->Kp)* (pV->up) + pV->ui;//PI输出 = Kp*err + ki*err(n) 
	pV->Out = IQsat( pV->v1, pV->Umax,  pV->Umin); //PI输出限幅
	
	#endif
}

// 位置式 PI (conditional integration anti-windup)
/* 前馈：在dq轴电压方程中，Ud = Ld​*id/dt​（PI控制） -ωe*​Lq​*iq + Rs*id ;Uq = Ld​*id/dt（PI控制） +ωe*Ld*id+ ωe​*ψf +Rs*iq
   做电流环前馈/解耦时，我们把除PI控制的其余项当作已知扰动直接用前馈抵消，于是把“目标电压”设为：     
   Ud' = Ud-pi + ωe*​Lq​*iq（反向补偿耦合扰动）+ Rs*id（正向补偿线阻压降）； Uq' = Uq-pi -（ωe*Ld*id+ ωe​*ψf）（反向补偿耦合扰动）+ Rs*iq（正向补偿线阻压降）
   上述目标电压前馈项符号是在常见约定：d轴和alpha轴重合，q轴超前d轴90度情况下，如果dq轴方向和约定不同那前馈项符号要在上述基础上取反*/

/* 前馈项取值：Rs*iq = 相电阻*pi_iq.Fbk = pi_iq.Fbk*7/10     Rs*id = pi_id.Fbk*7/10
               ωe*​Lq​*iq = 电角速度*这里近似为相电感*pi_iq.Fbk = (Hall_Three.Speed_RPM*Hall_Three.Poles*618/100)*(24/10000)*pi_iq.Fbk 
               ωe*​Ld*id = (Hall_Three.Speed_RPM*Hall_Three.Poles*618/100)*(24/10000)*pi_id.Fbk
               ωe​*ψf = 电角速度*磁链 = (Hall_Three.Speed_RPM*Hall_Three.Poles*618/100)* (17/1000)  */

int32_t ff;
void PI_Controller_conditional(p_PI_Control pV)
{
    /* 1. 误差 */
    pV->up = pV->Ref - pV->Fbk;    

    /* 2. 前馈项（可选，如果不使用，设置 pV->FF = 0） */
    ff = pV->FF();    // pV->FF 应与 PI输出v1 的 Q 格式一致

	  
    /* 3. 预测积分项、预测未限幅的输出 v_unclamped = Kp*err + (i1 + delta_i) + ff */  
	  int32_t ui = pV->i1 + _IQ10mpy(pV->Ki, pV->up);        // ki(n) = ki*err + ki(n-1) = ki*err(n)
    int64_t v_unclamped = _IQ10mpy(pV->Kp, pV->up) + ui + ff; // Kp*err + ki(n) + ff

    /* 4. 预测输出项限幅  */
    int64_t v_clamped = IQsat(v_unclamped, pV->Umax, pV->Umin);

    /* 5. 动态钳位修正积分器 */
    // 关键公式：i(k) = ui + (v_clamped - v_unclamped)
    int32_t ui_corrected = ui + (int32_t)(v_clamped - v_unclamped);
	  ui_corrected = IQsat(ui_corrected, (pV->Umax), (pV->Umin));
    pV->ui = ui_corrected;
	
    /* 6. 计算最终输出  */
    pV->i1 = pV->ui;  // ki(n-1) = ki(n) 当前积分项
    pV->v1 = _IQ10mpy(pV->Kp, pV->up) + pV->ui + ff;
    pV->Out = IQsat(pV->v1, pV->Umax, pV->Umin);
}


/*对于速度环，由于输入的速度为真实速度，而计算PI时使用的Q10定点乘法_IQmpy，且输出pV->Out没有
  定点数转为真实值，又由于速度环的输出为电流环的输入，为Q10格式的电流ref，因此我们认为这里速度环的PI参数都是Q10*Q10 = Q20定点数；

  对于电流环，由于输入的电流为Q10定点数，而计算PI时使用的Q10定点乘法_IQmpy，且输出pV->Out电压没有
  定点数转为真实值，因此我们认为这里电流环的PI参数都是Q15定点数；

  至于积分上限，一般取输出上限的0.5~1.0倍

  至于输出上限，
  由于速度环输出为目标电流，因此输出应该不超过电机额定电流的1.2到1.5倍，避免饱和，这里电流为Q10定点数  
  由于电流环输出为电压且是Q15定点数，这里设置上限为VBUS*2^15,大概为24*30000
*/
int32_t pi_spd_FF(void)
{
	return  0;
}

int32_t pi_iq_FF(void)
{
	//(pi_id.Fbk*Hall_Three.Speed_RPM*Hall_Three.Poles*618)*24/100/10000; //(Hall_Three.Speed_RPM*Hall_Three.Poles*618/100)* (17/1000); //+pi_iq.Fbk*7/10; 
	return  (int32_t)(-(int64_t)pi_id.Fbk*Hall_Three.Speed_RPM*297/10000);
}

int32_t pi_id_FF(void)
{
	//(pi_iq.Fbk*Hall_Three.Speed_RPM*Hall_Three.Poles*618)*24/100/10000; //+ pi_id.Fbk*7/10;
	return  (int32_t)((int64_t)pi_iq.Fbk*Hall_Three.Speed_RPM*297/10000);
}


/*对于速度环，由于输入的速度为真实速度，而计算PI时使用的Q10定点乘法_IQmpy，且输出pV->Out没有
  定点数转为真实值，因此我们认为这里速度环的PI参数都是Q10定点数；

  对于电流环，由于输入的电流为Q10定点数，而计算PI时使用的Q10定点乘法_IQmpy，且输出pV->Out电压没有
  定点数转为真实值，因此我们认为这里电流环的PI参数都是真实值；

  至于积分上限，一般取输出上限的0.5~1.0倍

  至于输出上限，
  由于速度环输出为目标电流，因此输出应该不超过电机额定电流的1.2到1.5倍，避免饱和，这里电流为Q10定点数  
  由于电流环输出为电压且是Q15定点数，这里设置上限为VBUS*2^15,大概为24*30000
*/
void  PI_Pare_init(void )
{
	#if PI_mode == 0

  pi_spd.Kp=18000;     // _ 
  pi_spd.Ki=260;  //    T* ??��???? /0.2
  pi_spd.Umax =30000;  //  
  pi_spd.Umin =0;

  pi_spd.Kp= 23173*2;     //  
  pi_spd.Ki= 346*2;     //   
  pi_spd.Umax = _IQ15(Motor_Params.VBUS) *577/1000 ;  // 直接作为电流环输出时
  pi_spd.Umin = -_IQ15(Motor_Params.VBUS) *577/1000;  // 直接作为电流环输出时

  pi_id.Kp= 34530;   // 
  pi_id.Ki= 720;     // 
  pi_id.Umax =200000;//  
  pi_id.Umin =-200000;    // 

  pi_iq.Kp=2680  ;
  pi_iq.Ki=236 ;
  pi_iq.Umax =30000 ;
  pi_iq.Umin =0 ;
	
	#elif  PI_mode == 1
	/*电流环带宽设置为采样频率的1/20 ，采样频率10KHZ时带宽Bandwidth为500HZ = 3141rad/s
	电流环Kp = L * Bandwidth = 5.7
	电流环Ki = R * Bandwidth = 1570   由于离散化因此Ki还需要乘一个Ts, Ts= 0.0001s，因此Ki*Ts = 0.157
	这里Bandwidth单位可以为HZ或者rad，两者相差6.28倍，
	不乘6.28的PI参数的阶跃响应波形更平滑，而乘以6.28的波形会略有超调，但电机并未产生震荡且能够正常工作。
	
	
	速度环带宽Bandwidth取电流环带宽的1/20,即150,
	速度环Kp = Bandwidth*转动惯量/(1.5*磁极对数*磁链) = 0.0221    Q20格式就是23173
	速度环Ki = Bandwidth*Kp = 3.3    由于离散化因此Ki还需要乘一个Ts, Ts= 0.0001s，因此Ki*Ts = 0.00033  Q20格式就是346
	*/
	


	pi_spd.FF = pi_spd_FF;
	pi_spd.Kp = 562;     // 25;  //
  pi_spd.Ki = 8;  //    3750;     //
	  pi_spd.Umax = (int32_t)(Motor_Params.Current_Rating * 1024.0f) * 3 / 2;  // _IQ15(Motor_Params.VBUS); //
  pi_spd.Umin =0;
	
	//输出限幅：防止Ud、Ud单个电压大于FOC矢量圆的最大半径（等幅值变换下为Udc/√3）
	 
	pi_id.FF = pi_id_FF; //+ pi_id.Fbk*7/10;    //
  pi_id.Kp=  20000;      //75300;  //6530;    //167116     10530  L = 1.9mH
  pi_id.Ki=  100; //400;    //300;     // 1970;   1500   R = 0.5 omu
  pi_id.Umax = _IQ15(Motor_Params.VBUS) *577/1000 / 2; //_IQ15(Motor_Params.VBUS) *577/1000 *2;  //40000; 
  pi_id.Umin = -_IQ15(Motor_Params.VBUS) *577/1000 / 2;//-_IQ15(Motor_Params.VBUS) *577/1000 *2; //-40000; 
 
	pi_iq.FF = pi_iq_FF;
  pi_iq.Kp = 20000;      //75300;   //6530  ; 
  pi_iq.Ki = 100;  //400;     //300 ;     
  pi_iq.Umax = _IQ15(Motor_Params.VBUS) *577/1000 / 2;  //350000;;
  pi_iq.Umin = -_IQ15(Motor_Params.VBUS) *577/1000 / 2; //-350000 ;
  #endif
}
   

//===========================================================================
// No more.
//===========================================================================

