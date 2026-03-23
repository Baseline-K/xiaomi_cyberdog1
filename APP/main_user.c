//############################################################
// FILE: Task_function.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Task_function
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
#include "main_user.h"
#include "iwdg.h"

#include "FOC_run.h"
#include "Timer.h"   
#include "ThreeHall.h"  
#include "IQ_math.h"
#include "Task_function.h"

#include "foc_pid.h"
#include "speed_pid.h"
#include "position_pid.h"
#include "AS5600.h"
#include "Soft_IIC.h"
#include "drv83xx.h"
#include "dead_comp.h"

#include "SEGGER_RTT_Port.h"
#include "cm_backtrace.h"

void fault_test_by_div0(void) {
    volatile int * SCB_CCR = (volatile int *) 0xE000ED14; // SCB->CCR
    int x, y, z;

    *SCB_CCR |= (1 << 4); /* bit4: DIV_0_TRP. */

    x = 10;
    y = 0;
    z = x / y;
    printf("z:%d\n", z);
}

void DWT_Init(void);
void Uart_485_Init(void);

logic        logicContr = logic_DEFAULTS;
IQSin_Cos    AngleSin_Cos = IQSin_Cos_DEFAULTS ;


extern uint32_t start,end,cycles;
extern float time_us;

PLL *Encoder_PLL_mecFilter;
PLL *Encoder_PLL_eleFilter;




void main_user(void)
{
	//ADC校准
	HAL_ADCEx_Calibration_Start( &hadc1);
	HAL_ADCEx_Calibration_Start( &hadc2);
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
	//启动ADC1规则组的DMA连续转换
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_ConvertedValue, 1);
	__HAL_TIM_CLEAR_FLAG( &htim1, TIM_FLAG_BREAK);
  HAL_TIM_Base_Start_IT(&htim1);  // 启动定时器1中断

	
	/******* 用户区初始化  **********/
	//DWT_Init();    //调试计数器
	SEGGER_RTT_PLUS_Port_Init();   //J-Scope调试变量曲线
	SEGGER_RTT_Init();             //RTT Viewer打印数据
	printf("Segger RTT Init Success!\r\n");
	
  cm_backtrace_init("Xiaomi_CyberDog1", "V0.0", "V0.0");  //栈回溯
	// fault_test_by_div0();	
	
	DRV83XX_Init();
	Offset_Current_Start();  //校准作用,电流传感器的理论偏移值为1.65V
	
  bsp_as5600Init();
	//校准传感器与电机零点偏移角度
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		TIM1->CCR1 = PWM_HalfPerMax * 13/10;  //  A  U   // d轴强拖，形成SVPWM模型中的基础矢量1，即对应转子零度位置
		TIM1->CCR2 = PWM_HalfPerMax;  //  B  V
		TIM1->CCR3 = PWM_HalfPerMax;  //  C  W
		HAL_Delay(200);     
		float angle_zero_offset;
		bsp_as5600GetAngle(&angle_zero_offset);
		Encoder_AS5600.angle_zero_offset = - angle_zero_offset;

    TIM1->CCR1 = PWM_HalfPerMax;  //  A  U   // 松开电机
		TIM1->CCR2 = PWM_HalfPerMax;  //  B  V
		TIM1->CCR3 = PWM_HalfPerMax;  //  C  W
   
	 
	 Motor_Params.Current_Rating = 1.0f;
	 Motor_Params.Flux = 0.00499f;
	 Motor_Params.Phase_L = 0.0018f;
	 Motor_Params.Phase_R =  3.2f;
	 Motor_Params.Pole_Pairs = 7.0f;
	 Motor_Params.Rotor_inertia = 0.000004f;
	 Motor_Params.VBUS = 12.0f;
	 Motor_Params.Reduction_ratio = 1.0f;
   Motor_Params.Torque_Rating = 0.02f;
	 Motor_Params.RPM_Rating = 2400.0f;
	 /* init filter */
	 Encoder_PLL_mecFilter = Angle_PLL_filter_init(FOC_Frequency, 250.0f, 1.0f/4096.0f * 2 *PI * 0.1f, 2.0f * (Motor_Params.RPM_Rating/60) * 2*PI);
	 Encoder_PLL_eleFilter = Angle_PLL_filter_init(FOC_Frequency, 250.0f, 1.0f/4096.0f * 2 *PI * 0.1f * Motor_Params.Pole_Pairs, 
	                                               2.0f * (Motor_Params.RPM_Rating/60) * 2*PI * Motor_Params.Pole_Pairs);
	 
	 DeadTime_Compensation_Init(&Motor_Params);
		 
	 //MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t previousMillis = 0;
  const uint32_t interval = 10; // 单位 ms 间隔
	
		 
  while (1)
  {
		  
			 // 获取当前的系统时间
			 uint32_t currentMillis = HAL_GetTick();

		 // 检查是否已达到间隔时间
		 if (currentMillis - previousMillis >= interval) {
				previousMillis = currentMillis;  // 更新上次执行的时间
			 
			 //HAL_IWDG_Refresh(&hiwdg);
        // 调用要定时执行的函数
			 //ADCSampPare.RP_speed_Voltage = 300;
			 knob_control();   		//检测电位器
			 
//			 __disable_irq();
//				start = DWT->CYCCNT; // 记录起始时间
// 
//			  bsp_as5600GetAngle(&Encoder_AS5600.angle);
//			  //bsp_as5600_DMAGetAngle(&Encoder_AS5600.angle);
//			  Encoder_AS5600.eleangle	= fmodf(Encoder_AS5600.angle * Pole, Two_PI);
//			 	
//			 
//				end = DWT->CYCCNT;   // 记录结束时间
//				
//				cycles  = end - start; // 计算运行的 CPU 周期数
//				// 处理溢出情况（如果测量过程中发生了溢出）
//				if (end < start) {
//						cycles = (0x100000000ULL - start) + end;  // 处理溢出
//				} else {
//					cycles = end - start;
//				}
//				time_us  = (float)cycles / (SystemCoreClock / 1e6); // 转换为微秒
//				__enable_irq();
				
			
			 //disable_erq();   float FOC_time_us = time_us; void (* FOC_func_ptr)(ADC_HandleTypeDef *hadc) = func_ptr;enable_erq();
//			  myprintf("Function execution time: %.3f us\nFunction %p is in %s memory\n",\
//				FOC_time_us,FOC_func_ptr,((uint32_t)func_ptr >= SRAM_BASE) ? "RAM" : "Flash"); 
			 
			  //HAL_IWDG_Refresh(&hiwdg);  // 喂狗，重装载计数器
			// UploadData();	

		 }
		
  }
}


void Param_init(void)
{ 
	  current_q_pid_Init(&current_q_pid, Motor_Params);
	  current_d_pid_Init(&current_d_pid, Motor_Params);
		speed_pid_init(Motor_Params, N);
	  position_PID_Init(&position_pid,Motor_Params);
		//ThreeHallPara_init();   // 三霍尔角度传感器的参数初始化
	  bsp_as5600Init();
		SEGGER_RTT_TimeStamp_reset(); 
}

void Param_deinit(void)
{
	//memset(&TestPare, 0, sizeof(TestPare));
	//memset(&TaskTimePare, 0, sizeof(TaskTimePare));
	memset(&logicContr, 0, sizeof(logicContr));
	memset(&AngleSin_Cos, 0, sizeof(AngleSin_Cos));
	//memset(&Hall_Three, 0, sizeof(Hall_Three));
	memset(&Encoder_AS5600, 0, sizeof(Encoder_AS5600));
	
	memset(&foc_dq_v, 0, sizeof(foc_dq_v));
	memset(&foc_dq_i, 0, sizeof(foc_dq_i));
	memset(&current_q_pid, 0, sizeof(current_q_pid));
	memset(&current_d_pid, 0, sizeof(current_d_pid));
	memset(&speed_pid, 0, sizeof(speed_pid));
	memset(&position_pid, 0, sizeof(position_pid));
	
	Angle_PLL_filter_clear(Encoder_PLL_mecFilter);
	Angle_PLL_filter_clear(Encoder_PLL_eleFilter);
}

/**********************  Printf  ************************/
int fputc(int ch, FILE *f) {
    SEGGER_RTT_Write(0, (const char *)&ch, 1);
    return ch;
}

/**********************  DWT  ************************/
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT
    DWT->CYCCNT = 0; // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能 CYCCNT 计数器
}


/**********************  ADC  ************************/
ADCSamp      ADCSampPare;
uint16_t  ADC_ConvertedValue[2]={0};
//校准作用,电流传感器的理论偏移值为1.65V
//三电阻采样时可以用软件构建动态的偏移电压的ADC值，三相电流的ADC值相加是3倍的偏移电压ADC值的数学关系
void Offset_Current_Start(void)
{
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_3);
  HAL_Delay(50);
	MotorState.run_state = RUNSTATE_CALIBRATING;
	HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);
	HAL_ADCEx_InjectedStart_IT(&hadc1);  //开启FOC运行
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_Delay(300);
}

void Offset_CurrentReading(void)
{
	static uint16_t i;  
  i++;
	
	/* ADC Channel used for current reading are read  in order to get zero currents ADC values*/
	foc_abc_current_i.offset_phase_ib +=  ADC2->JDR1;;
  foc_abc_current_i.offset_phase_ic +=  ADC1->JDR1;		 


	if(i >= 1000)   
  {
		MotorState.run_state = RUNSTATE_STOPPED;
		i = 0;
		
		foc_abc_current_i.offset_phase_ib = foc_abc_current_i.offset_phase_ib / 1000; 
		foc_abc_current_i.offset_phase_ic =	foc_abc_current_i.offset_phase_ic / 1000;
		
		HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_4);
		HAL_ADCEx_InjectedStop_IT(&hadc1);  //停止FOC运行
		HAL_ADCEx_InjectedStop(&hadc2);	
  }
}

//===========================================================================
// No more.
//===========================================================================
