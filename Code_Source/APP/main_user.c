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
#include "CANopen_OD.h"
#include "MotorCtrl.h"
#include "CAN_bsp.h"
#include "RTT_Cmd.h"

#include "AS5600.h"
#include "Soft_IIC.h"
#include "drv83xx.h"

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
	SEGGER_RTT_Init();             //RTT Viewer打印数据（先初始化RTT，避免PLUS通道配置被重置）
	SEGGER_RTT_PLUS_Port_Init();   //J-Scope调试变量曲线
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
		PowerStage_Disable();          // 上电默认关闭三相功率输出
   
	 
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
	 
	 FOC_Generated_Init();    // Simulink 生成 FOC 算法参数初始化
	 MotorState.ctrl_mode = CTRL_MODE_SPEED;
	 CAN_bsp_Init();          // CAN 控制接口（过滤器/接收/回调）
	 MotorCtrl_Init();        // 电机控制层（默认停止）
	 FOC_Interrupt_Start();   // 上电后持续运行 10 kHz FOC/采样中断

	 //MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t previousMillis = 0;
	uint32_t prevCanMillis = 0;
  const uint32_t interval = 10; // 单位 ms 间隔
	
		 
  while (1)
  {
		  
			 // 获取当前的系统时间
			 uint32_t currentMillis = HAL_GetTick();

			 // 1ms：控制接口处理（CAN + RTT）
			 if (currentMillis - prevCanMillis >= 1U) {
				 prevCanMillis = currentMillis;
				 CANopen_OD_Process();   // CAN 弹帧→解析→写 MotorCtrl 影子
				 MotorCtrl_Process();    // start/stop 请求
				 RTT_Cmd_Process();      // RTT 命令 → MotorCtrl
			 }

		 // 检查是否已达到间隔时间
		 if (currentMillis - previousMillis >= interval) {
				previousMillis = currentMillis;  // 更新上次执行的时间
			 
			 //HAL_IWDG_Refresh(&hiwdg);
        // 调用要定时执行的函数
			 //ADCSampPare.RP_speed_Voltage = 300;
			 
//			 __disable_irq();
//				start = DWT->CYCCNT; // 记录起始时间
// 
			//   bsp_as5600GetAngle(&Encoder_AS5600.angle);
			//   //bsp_as5600_DMAGetAngle(&Encoder_AS5600.angle);
			//   Encoder_AS5600.eleangle	= fmodf(Encoder_AS5600.angle * Motor_Params.Pole_Pairs, Two_PI);

			 
			  //HAL_IWDG_Refresh(&hiwdg);  // 喂狗，重装载计数器

		 }

		
		
  }
}


void Param_init(void)
{
		// 注：Simulink 生成代码内部自带电流/速度/位置 PID，旧 MID_foc PID 结构已移除
		bsp_as5600Init();
		SEGGER_RTT_TimeStamp_reset();
}

void Param_deinit(void)
{
	memset(&foc_dq_v, 0, sizeof(foc_dq_v));
	memset(&foc_dq_i, 0, sizeof(foc_dq_i));

	Angle_PLL_filter_clear(Encoder_PLL_mecFilter);
	Angle_PLL_filter_clear(Encoder_PLL_eleFilter);
}

/**********************  Printf  ************************/
/* GCC + newlib(-nano)：CubeMX syscalls.c 的 _write 会调用它 */
int __io_putchar(int ch) {
    SEGGER_RTT_Write(0, (const char *)&ch, 1);
    return ch;
}

/* Keil / ARM C library：printf 输出会调用 fputc（两个都定义，谁被链接就调用谁） */
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
