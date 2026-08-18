#include "FOC_run.h"
#include "Timer.h"
#include "main_user.h"
#include "SEGGER_RTT_Port.h"

#include "foc.h"
#include "AS5600.h"

#define DEBUG 1
#if DEBUG

#else

#endif
float speed_ref_Test = 5;
float position_ref_Test = PI;


MotorState_t MotorState;

//extern   logic      logicContr;
//extern   TaskTime   TaskTimePare;  

extern ADC_HandleTypeDef hadc1, hadc2;
extern TIM_HandleTypeDef htim1;
	
uint32_t start,end,cycles;
float time_us;
void (* func_ptr)(ADC_HandleTypeDef *hadc) = NULL;
// 10khz  ADC注入组采样完成中断
uint32_t Predict_ThreeHallangle;
float Predict_eleangle;

float Phase_Ra,Phase_Rb,Phase_Rc, Vbus;
float Phase_La,Phase_Lb,Phase_Lc, Vbus;
void HAL_ADCEx_InjectedConvCpltCallback( ADC_HandleTypeDef *hadc)
{

	if( hadc->Instance == hadc1.Instance)
	{
		
		if(MotorState.run_state == RUNSTATE_CALIBRATING)
		{
			Offset_CurrentReading();
			  return;
		}


		
		
#if DEBUG	
		
		start = DWT->CYCCNT; // 记录起始时间
#else
										
#endif
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
		

		

		/***********************   1、Processing Eleangle  ***************************/
		//Predict_ThreeHallangle = ThreeHallanglecale();  
//		Predict_eleangle = (float)Predict_ThreeHallangle* 0.0000958752f;
//		Predict_eleangle = fmodf(Predict_eleangle, Two_PI);
			//Hall_Three.Hall_State = GET_HALLState_debounce_dynamic();
		
		//Predict_eleangle = VF_IF_RUN_Float(-600.0f, 6000.0f);  	//强拖判断同步电角度
		
		bsp_as5600GetAngle(&Encoder_AS5600.angle);
		Encoder_AS5600.eleangle	= fmodf(Encoder_AS5600.angle * Motor_Params.Pole_Pairs, Two_PI);
#if !USE_GENERATED_FOC
		/* 手写 PLL（仅 legacy 路径；USE_GENERATED_FOC=1 时由 Simulink 模型内的官方 MCB PLL 提供） */
		Predict_eleangle = Angle_PLL_filter(Encoder_PLL_eleFilter, Encoder_AS5600.eleangle);
		Encoder_PLL_eleFilter->PLL_Omega_filtered = Lpf(Encoder_PLL_eleFilter->PLL_Omega, Encoder_PLL_eleFilter->PLL_Omega_filtered, 0.7f);
#endif
		
		/***********************   2、Processing ADC  ***************************/
		
		foc_abc_current_i.adc_phase_ic =  ADC1->JDR1;   
	  foc_abc_current_i.adc_phase_ib  =  ADC2->JDR1;  
	

    foc_abc_current_i.ic  =  -((float)foc_abc_current_i.offset_phase_ic - foc_abc_current_i.adc_phase_ic) * I_SCALE;
		foc_abc_current_i.ib  =  -((float)foc_abc_current_i.offset_phase_ib - foc_abc_current_i.adc_phase_ib) * I_SCALE;	//3.3/4096 /(40V/V) / (0.001Ω)
		foc_abc_current_i.ia = - foc_abc_current_i.ic - foc_abc_current_i.ib;

    Vbus = ADC_ConvertedValue[0]* V_SCALE; // ADC1->JDR2 * V_SCALE;  
		/*********************  Processing EventTask   *************************/


		/* Keep the 10 kHz sampling interrupt alive while stopped, but do not run
		* the encoder, speed loop, current loop, or SVPWM calculations. */
		if(MotorState.run_state != RUNSTATE_RUNNING)
		{
			TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_HalfPerMax;
			
		}	
		else{

	
		
#if USE_GENERATED_FOC
		FOC_Generated_Step();
#else
       //Position_Loop_Handle(position_ref_Test,	Encoder_AS5600.CCW_angle_total - Encoder_AS5600.CW_angle_total);
		speed_Loop_Handle(3.0f, &speed_pid, 0.04f, 0.04f);	
//    speed_Loop_Handle(speed_ref_Test, &speed_pid, 0.04f, 0.04f);  //
   
		/***********************   FOC Algorithm   ***************************/
		Angle_Sin_Cos(Predict_eleangle, &foc_sin_cos);  //
		Clark(foc_abc_current_i, &foc_alpha_beta_i);
		Park(foc_alpha_beta_i, foc_sin_cos, &foc_dq_i);
//				foc_dq_i.iq = -foc_dq_i.iq;
//				foc_dq_i.id = -foc_dq_i.id;
	
		foc_dq_i.iq = Lpf(foc_dq_i.iq, foc_dq_i.iq_last,0.8f);
		foc_dq_i.iq_last = foc_dq_i.iq;
		
		foc_dq_i.id = Lpf(foc_dq_i.id, foc_dq_i.id_last,0.8f);
		foc_dq_i.id_last = foc_dq_i.id;
		
		
		current_q_pid.ref = speed_pid.output; //;
		current_d_pid.ref = 0.0f;
		current_q_pid.fbk = foc_dq_i.iq;
		current_d_pid.fbk = foc_dq_i.id;
		current_pid_Control(&current_q_pid);
		current_pid_Control(&current_d_pid);
		foc_dq_v.vq = current_q_pid.output;
    foc_dq_v.vd = current_d_pid.output;
// 		foc_dq_v.vq = -5.0f;
//    foc_dq_v.vd = 0.0f;
		
		Reverse_Park(foc_sin_cos, foc_dq_v, &foc_alpha_beta_v);
	 /*Overmodulation prevention*/
		RevPark_Circle_Limitation_cmsis(&foc_alpha_beta_v.v_alpha, &foc_alpha_beta_v.v_beta,
																				Motor_Params.VBUS, 0.95f);
		Foc_Svpwm(foc_alpha_beta_v, &foc_pwm, Motor_Params.VBUS, PWM_PERIOD);
		
		TIM1->CCR1 = foc_pwm.pwm_u;
		TIM1->CCR2 = foc_pwm.pwm_v;
		TIM1->CCR3 = foc_pwm.pwm_w;

		
		/*********************   FOC Algorithm END ***************************/
			 
#endif
		}
#if DEBUG	
		//SEGGER_RTT_Port_Write();
		JS_RTT_PLUS_SendData(); 
		
    end = DWT->CYCCNT;   // 记录结束时间	
		
    cycles  = end - start; // 计算运行的 CPU 周期数
		// 处理溢出情况（如果测量过程中发生了溢出）
    if (end < start) {
				cycles = (0x100000000ULL - start) + end;  // 处理溢出
		} else {
			cycles = end - start;
		}
		time_us  = (float)cycles / (SystemCoreClock / 1e6); // 转换为微秒
		func_ptr = HAL_ADCEx_InjectedConvCpltCallback;
#else
									
#endif		
		
	}
}

//VF爬坡强拖启动  跳过电流环、速度环、位置环，直接给定目标Ud = 0，Uq为一个值，然后匀速增加电角度
float Angular_Speed = 0 , Angular_Angle = 0;
uint32_t Angular_Angle_Q16 = 0;
uint64_t Angular_Speed_ref;
uint32_t IF_FOC_num;
uint32_t Angular_Speed_step;

uint32_t VF_IF_RUN(uint16_t RPM_ref, uint16_t Time_ms)
{
	
	  Angular_Speed_ref = (uint64_t)(RPM_ref)*65536ULL/60ULL*(uint64_t)Motor_Params.Pole_Pairs; 	//计算目标电角速度  0-360度换算为0-65535
    IF_FOC_num = (uint32_t)Time_ms*1000/(1000000/FOC_Frequency);   //爬坡过程FOC中断次数
	  Angular_Speed_step =Angular_Speed_ref / IF_FOC_num;  //每次FOC中断的爬坡电角速度步进值

    Angular_Speed += Angular_Speed_step;  //计算本次电角速度
	  if( Angular_Speed >= Angular_Speed_ref)
			Angular_Speed = Angular_Speed_ref;
			
    Angular_Angle = Angular_Angle + Angular_Speed * (float)(1000000/FOC_Frequency) / 1000000;  //计算本次电角度
    if(Angular_Angle >= 65536)
        Angular_Angle -= 65536;
    Angular_Angle_Q16 = (uint32_t)(Angular_Angle);  //将电角度转换为Q15格式

//		IparkU.Ds= 0;   									// id目标值为0
//		IparkU.Qs= Uq_ref;
	return Angular_Angle_Q16;  //返回Q16格式的电角度
}

//VF爬坡强拖启动  跳过电流环、速度环、位置环，直接给定目标Ud = 0，Uq为一个值，然后匀速增加电角度
float VF_IF_RUN_Float(float RPM_ref, float Time_ms)
{
		static float Angular_Speed = 0 , Angular_Angle = 0;
		static float Angular_Speed_ref;
		static float IF_FOC_num;
		static float Angular_Speed_step;
	
	  Angular_Speed_ref = RPM_ref*6.28f/60.0f*Motor_Params.Pole_Pairs; 	//计算目标电角速度
    IF_FOC_num = Time_ms * 1000 / 50.0f;   //爬坡过程FOC中断次数
	  Angular_Speed_step = Angular_Speed_ref / IF_FOC_num;  //每次FOC中断的爬坡电角速度步进值

    Angular_Speed += Angular_Speed_step;  //计算本次电角速度
	  if( Angular_Speed >= Angular_Speed_ref)
			Angular_Speed = Angular_Speed_ref;
			
    Angular_Angle = Angular_Angle + Angular_Speed * 50.0f / 1000000.0f;  //计算本次电角度
    if(Angular_Angle >= Two_PI)
        Angular_Angle -= Two_PI;

	 return  Angular_Angle;  //返回电角度
}

