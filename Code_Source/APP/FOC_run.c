#include "FOC_run.h"
#include "Timer.h"
#include "main_user.h"
#include "SEGGER_RTT_Port.h"

#include "foc.h"
#include "AS5600.h"
#include "Safety_Module.h"        /* Safety_FastStep */
#include "motor_state_machine.h"  /* MotorStateMachine_PostFault */
#include "identify.h"             /* 离线辨识：RUNSTATE_IDENTIFYING 时填电压输入 */

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
volatile uint32_t g_foc_wcet_us = 0U;   /* FOC ISR 最大耗时（us），DiagTask/Ozone 只读 */
volatile uint32_t g_foc_isr_us  = 0U;   /* FOC ISR 每周期耗时（us），Ozone 实时观察 */
void (* func_ptr)(ADC_HandleTypeDef *hadc) = NULL;
// 10khz  ADC注入组采样完成中断
uint32_t Predict_ThreeHallangle;
float Predict_eleangle;

/* 连续机械角(rad, 多圈)：10kHz ISR 累加，位置模式反馈（FOC_Generated_Step 读取） */
float g_mech_pos_rad = 0.0f;
static float g_last_angle_rad = -1.0f;   /* <0 = 首样本哨兵 */

float Phase_Ra,Phase_Rb,Phase_Rc, Vbus;
float Phase_La,Phase_Lb,Phase_Lc, Vbus;
void HAL_ADCEx_InjectedConvCpltCallback( ADC_HandleTypeDef *hadc)
{
	uint32_t _isr_t0 = DWT->CYCCNT;   /* FOC ISR 耗时测量（DWT 120MHz，SEGGER_RTT_Port 已使能） */

	if( hadc->Instance == hadc1.Instance)
	{

		if(MotorState.run_state == RUNSTATE_CALIBRATING)
		{
			Offset_CurrentReading();
			{   uint32_t _el = (DWT->CYCCNT - _isr_t0) / 120U;   /* cycles → µs @120MHz */
			    g_foc_isr_us = _el;
			    if (_el > g_foc_wcet_us) g_foc_wcet_us = _el; }
			  return;
		}
		
		
#if DEBUG	
		
		
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
		{   /* 连续机械位置：角度增量 + ±π 绕回检测（首样本定零点 = 电气零点对齐位置） */
			float ang = Encoder_AS5600.angle;
			float d = ang - g_last_angle_rad;
			if (g_last_angle_rad < 0.0f) {
				g_last_angle_rad = ang;
				g_mech_pos_rad   = ang;
			} else {
				if      (d >  PI)  d -= Two_PI;
				else if (d < -PI)  d += Two_PI;
				g_mech_pos_rad += d;
				g_last_angle_rad = ang;
			}
		}
		Encoder_AS5600.eleangle	= fmodf(Encoder_AS5600.angle * Motor_Params.Pole_Pairs, Two_PI);
		
		/***********************   2、Processing ADC  ***************************/
		
		foc_abc_current_i.adc_phase_ic =  ADC1->JDR1;   
	  foc_abc_current_i.adc_phase_ib  =  ADC2->JDR1;  
	

    foc_abc_current_i.ic  =  -((float)foc_abc_current_i.offset_phase_ic - foc_abc_current_i.adc_phase_ic) * I_SCALE;
		foc_abc_current_i.ib  =  -((float)foc_abc_current_i.offset_phase_ib - foc_abc_current_i.adc_phase_ib) * I_SCALE;	//3.3/4096 /(40V/V) / (0.001Ω)
		foc_abc_current_i.ia = - foc_abc_current_i.ic - foc_abc_current_i.ib;

    Vbus = ADC_ConvertedValue[0]* V_SCALE; // ADC1->JDR2 * V_SCALE;
    Motor_Params.VBUS = Vbus;   /* 母线电压实时化：模型 SVPWM/限幅/电流限随实测母线（FOC_Generated_Step 每步同步） */
		/*********************  Safety 快检（Phase 4） *************************/
		{
			SafetyFastInput_t fast_in = { Vbus, { foc_abc_current_i.ia, foc_abc_current_i.ib, foc_abc_current_i.ic } };
			fault_mask_t fast_faults = 0;
			Safety_FastStep(&fast_in, &Safety_Config, &fast_faults);
			if (fast_faults != 0U) {
				__HAL_TIM_MOE_DISABLE(&htim1);              /* 当场快速关 PWM 输出 */
				MotorStateMachine_PostFault(fast_faults);   /* 原子锁存，任务 1ms 巡检进 FAULT */
			}
		}
		/*********************  Processing EventTask   *************************/

		/* Phase 3b：始终调用 FOC_Generated_Step——模型 PLL 常跑给实时转速；
		 * coast/停机由模型 coast 输入处理（duty 强制 0.5），不再按 run_state 门控。 */
		if (MotorState.run_state == RUNSTATE_IDENTIFYING) {
			Identify_FocIsrStep();   /* 填模型电压输入（ctrl_mode=3/4），模型 step 再应用电压 */
		}
		FOC_Generated_Step();

#if DEBUG	
		
		//JS_RTT_PLUS_SendData();

#else
									
#endif
	}
	/* FOC ISR 每周期耗时记录（µs @120MHz），Ozone 加 g_foc_isr_us / g_foc_wcet_us */
	{
		uint32_t _el = (DWT->CYCCNT - _isr_t0) / 120U;
		g_foc_isr_us = _el;
		if (_el > g_foc_wcet_us) g_foc_wcet_us = _el;
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
