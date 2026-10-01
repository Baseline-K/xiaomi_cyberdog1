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
#include "identify.h"          /* 离线辨识框架注册 */
#include "CANopen_OD.h"
#include "MotorCtrl.h"
#include "CAN_bsp.h"
#include "RTT_Cmd.h"

#include "Encoder_Adapt.h"
#include "drv83xx.h"
#include "drv83xx_reg.h"     /* CSA_GAIN_20 等寄存器常量（做增益一致性检查） */
#include "Safety_Module.h"   /* Safety_Config_Init / FAULT_CURRENT_SENSE_MASK */
#include "motor_state_machine.h"  /* MotorStateMachine_PostFault（零点校准失败上报） */
#include "tim.h"             /* htim4（调度器启动前的 IWDG 喂养定时器） */
#include "Task_Watchdog.h"   /* Twdt_ResetCauseInit / Twdt_* */

#include "SEGGER_RTT_Port.h"
#include "SEGGER_SYSVIEW.h"   /* SystemView：SEGGER_SYSVIEW_Conf */
#include "cm_backtrace.h"

#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"

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
static void Encoder_LogStartupInfo(void);
/* PLL 死区 = 1 LSB 的 10%，按当前编码器分辨率算（AS5600=4096，AS5047=16384） */
static float Encoder_Deadband(void);

extern uint32_t start,end,cycles;
extern float time_us;

PLL *Encoder_PLL_mecFilter;
PLL *Encoder_PLL_eleFilter;




void main_user(void)
{
	/* ===== 看门狗：调度器启动前的唯一喂养点 =====
	 * MX_IWDG_Init() 已在 main.c 里启动 IWDG（PSC32/Reload50 ≈ 40.8ms 超时），
	 * 从这里到调度器启动之间全是阻塞初始化（DRV83XX_Init / Offset_Current_Start /
	 * HAL_Delay(200) 等，合计约 670ms，最长单段 300ms），不喂狗必然复位。
	 * TIM4 更新中断周期 = WWDG_FEED_INTERVAL_100us × 100us = 5ms（Board_Config.h），
	 * 处理函数在 main.c 的 HAL_TIM_PeriodElapsedCallback（USER CODE Callback 1）里。
	 * 调度器起来之后必须关掉它（见本函数末尾的"交棒"），否则 SafetyTask 卡死查不出来。 */
	HAL_IWDG_Refresh(&hiwdg);
	HAL_TIM_Base_Start_IT(&htim4);

	//ADC校准
	HAL_ADCEx_Calibration_Start( &hadc1);
	HAL_ADCEx_Calibration_Start( &hadc2);
	HAL_ADCEx_Calibration_Start( &hadc3);   /* ADC3：电流C + Vbus 都在它的注入组（FOC ISR 源） */
	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG( &hadc3, ADC_FLAG_JEOC);
	//启动ADC1规则组的DMA（6 通道 = 输出端 6 路线性霍尔；规则组连续转换，DMA 循环填充）
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Hall_ADC_DMA, 6);
	/* 注意：**不要**启动 ADC3 的规则组。Vbus 走 ADC3 注入 rank2（与电流一起、PWM 同步）；
	 * 若再让 ADC3 规则组连续跑 Vbus，它的单次转换要 16.8us（239.5 周期采样），
	 * 会把注入序列（电流采样）推迟最多 16.8us —— 比霍尔那条还严重。
	 * 慢安全直接用 FOC ISR 发布的 Vbus 即可（见 app_tasks.c）。 */
	__HAL_TIM_CLEAR_FLAG( &htim1, TIM_FLAG_BREAK);
  HAL_TIM_Base_Start(&htim1);  // 启动定时器1计数器（**不带 IT**：更新中断无用，
                               // 而且它与 ADC1_2_IRQn 同为优先级 1 会挤占 FOC 采样中断。
                               // 参考工程同样是 HAL_TIM_Base_Start()。）

	
	/******* 用户区初始化  **********/
	/* DWT 周期计数器：**必须在这里使能**，不能指望下面那句 J-Scope 初始化。
	 * 依赖它的东西有一堆，而且全是"坏了不报错、只是数字静默变 0"的类型：
	 *   · AS5047 阻塞读的 SPI 超时（安全超时，见 AS5047.c as5047_dwt_ensure）
	 *   · FOC ISR 的 g_foc_isr_us / g_foc_wcet_us（§17 优化决策的依据）
	 *   · FreeRTOS 运行时间统计（vTaskSwitchContext / ulTaskGetRunTimeCounter）
	 *   · SEGGER_SYSVIEW / RTT TimeStamp 的时间戳
	 * 原来这行是注释掉的、靠 SEGGER_RTT_PLUS_Port_Init() 顺手使能 —— 那样只要
	 * 谁把那句调试曲线初始化删掉，上面全部静默失效。这里提前使能，去掉该耦合。
	 * （AS5047 驱动内部还有一次幂等探测 + 实测兜底，两层都留着。） */
	DWT_Init();    //调试计数器 / 各超时与 WCET 的时基
	/* GD32_NOTE: HAL 时基已迁移到 TIM6（SysTick 预留给 FreeRTOS），但调度器启动前
	 * 无人使能 SysTick 计数器。drv83xx.c 的 Delay_us() 直接读 SysTick->VAL 做微秒延时，
	 * 此处仅使能计数器、不开中断；vTaskStartScheduler() 后 FreeRTOS 会接管 SysTick 配置。 */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	SysTick->LOAD = 0x00FFFFFFUL;
	SysTick->VAL  = 0UL;
	SEGGER_RTT_Init();             //RTT Viewer打印数据（先初始化RTT，避免PLUS通道配置被重置）
	SEGGER_RTT_PLUS_Port_Init();   //J-Scope调试变量曲线
	SEGGER_SYSVIEW_Conf();         // SystemView：初始化 recorder，等待主机在 RTT 通道 3 发出 Start
	printf("Segger RTT Init Success!\r\n");

	/* 上一次复位原因：若是 IWDG_TIMEOUT，说明上一轮有喂狗点漏了。
	 * 当前策略：只打印 + 记录（供 RTT `get` 查询），**不**上报故障位 ——
	 * 台架上不希望一上电就被锁进故障态。
	 * 如果以后想让它也进故障上报，在 Twdt_ResetCauseInit() 里加一行
	 * MotorStateMachine_PostFault() 就行：S_IDLE 的 g_fault_bits 兜底检查
	 * （motor_state_machine.c 的 state_IDLE_do）会接住上电期间锁存的故障位。 */
	(void)Twdt_ResetCauseInit();

  cm_backtrace_init("Xiaomi_CyberDog1", "V0.0", "V0.0");  //栈回溯
	// fault_test_by_div0();	
	
	/* ===== 栅极驱动配置 + 回读校验 =====
	 * DRV83XX_Init 现在返回状态：write_reg 内部逐寄存器回读校验，非 0 就是没写进去。
	 * 写失败意味着 CSA 增益/保护阈值可能不是我们要的。 */
	if (0 != DRV83XX_Init())
	{
		printf("[DRV] ERROR: config write/readback verify FAILED\r\n");
	}
	DRV83XX_PrintConfig();   /* 打印读回的寄存器真值 */

	/* CSA 增益一致性（关键安全检查）：
	 * 软件 I_SCALE 用的增益是 FOC_run.h 的 CSA_GAIN_VV=20.0f，
	 * 硬件增益是 drv83xx.c 的 .CSA_GAIN = CSA_GAIN_20(0x2)。
	 * 两者不一致 → 所有电流读数与过流阈值成比例错（差 4 倍就是灾难）。 */
	if (stru_DRV8323Obj.drvCsa_obj.csaObj.CSA_GAIN != CSA_GAIN_20)
	{
		printf("[DRV] ERROR: CSACR.CSA_GAIN=%u != CSA_GAIN_20(%u) -> I_SCALE WRONG! "
		       "check drv83xx.c .CSA_GAIN vs FOC_run.h CSA_GAIN_VV\r\n",
		       (unsigned)stru_DRV8323Obj.drvCsa_obj.csaObj.CSA_GAIN, (unsigned)CSA_GAIN_20);
	}

	/* DRV 已配置完成、引脚电平已确定 → 现在才让 nFAULT 中断生效。
	 * 之前一直屏蔽：MX_GPIO_Init() 一上电就配好 PA12 的 EXTI 并使能 NVIC，
	 * 而那时 DRV 还没供电、引脚电平未定义，实测必现一次假故障
	 * （fault=0x00000008 但 FSR1/FSR2=0x0000、PA12 已释放）。 */
	MotorCtrl_DrvReady();

	/* ===== 电流零偏校准 + 上电强拖 + 编码器零电角度校准 =====
	 * ⚠️ 两者必须绑定：零偏算不出来时**绝不允许**继续强拖（会更危险）。
	 * 所以这里检查返回值，失败就把整段跳过、只保持功率级关闭。
	 * 失败时 Offset_Current_Start() 内部已做三件事：
	 *   ① 把零偏退回到理论中点 2048，避免 -(0-adc)×I_SCALE ≈ +82A 的假电流；
	 *   ② PostFault(FAULT_CURRENT_SENSE_MASK)，状态机进 IDLE 后第一拍就会转成 FAULT_NOW；
	 *   ③ 打印 `[CAL] ERROR` 并置 g_offset_cal_done=0（RTT `get` 可查）。 */
	if (Offset_Current_Start() != 0) {
		printf("[CAL] offset calibration FAILED -> skip strong-drag & encoder zero cal\r\n");
		PowerStage_Disable();
	} else {
  {
	/* ===== 编码器服务层 =====
	 * 按 Encoder_Adapt.h 的 ENCODER_SERVICE_DEFAULT_TYPE 挂载（默认 AS5047，
	 * 阻塞读取）。换编码器只改那一个宏，或在这里显式：
	 *     Encoder_Attach(ENCODER_TYPE_AS5600);
	 * 默认编码器挂不上时会自动回退到另一种，实际挂上的是谁看这行打印。 */
	Encoder_ServiceInit();
	printf("Encoder attach: %s (%lu counts/rev) status=%d\r\n",
	       Encoder_GetName(), (unsigned long)Encoder_GetResolution(),
	       (int)Encoder_GetStatus());
  }
	/* ===== 上电强拖 + 编码器零电角度校准 =====
	 * 原理：d 轴强拖锁转子到 A 相电气零位，读编码器角度记为零点偏移（angle_zero_offset）。
	 * 要求：强拖电流不能太小——需保证给电机施加"强拖电流"和"电机最大电流"时，
	 *      编码器角度都几乎不变（转子被磁力锁死、不随电流偏转），
	 *      才能说明此时校准的零电角度（零位）是准确的。
	 *      若强拖电流不足，施加更大电流时转子会被拉偏，导致零电角度误差。
	 * 当前：duty_u=1.0 → vα=4V → 强拖电流 ≈ 1.37A（R=2.92Ω），足以锁住转子。 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

		TIM1->CCR1 = PWM_PERIOD;  //  A  U   // d轴强拖：duty 1.0，vα=4V，强拖电流≈1.37A（R=2.92Ω），锁转子到零位
		TIM1->CCR2 = PWM_HalfPerMax;  //  B  V
		TIM1->CCR3 = PWM_HalfPerMax;  //  C  W
		HAL_Delay(200);     
		/* 此时零点偏移为 0，读数就是原始机械角；把它的负值设为零点偏移，
		 * 于是"强拖锁定的位置"读作 0 = 电气零点。 */
		{
			float angle_now = 0.0f;

			/* Task-context update obtains a fresh two-frame calibration sample;
			 * runtime FOC uses the single-frame pipeline. Never zero from stale data. */
			if (Encoder_Update() == ENCODER_OK && Encoder.data.sample_fresh &&
			    Encoder_GetMecAngle(&angle_now) == ENCODER_OK) {
				Encoder_SetZeroOffset(-angle_now);
				Encoder_ResetTracking();
			} else {
				PowerStage_Disable();
				MotorStateMachine_PostFault(FAULT_ENCODER_MASK);
				printf("[CAL] encoder fresh-angle read failed; zero not updated\r\n");
			}
		}

    TIM1->CCR1 = PWM_HalfPerMax;  //  A  U   // 松开电机
		TIM1->CCR2 = PWM_HalfPerMax;  //  B  V
		TIM1->CCR3 = PWM_HalfPerMax;  //  C  W
		PowerStage_Disable();          // 上电默认关闭三相功率输出
	}  /* end if (Offset_Current_Start() == 0) */
   
	 //2804云台电机
// 	 Motor_Params.Current_Rating = 1.0f;
// 	 Motor_Params.Current_Max = 3.0f;   // 最大电流（辨识安全上界 / 过流参考）
// 	 Motor_Params.Flux = 0.002896f; /* 15/20/25/30 RPS flux ID, four-point LS R^2 ~= 0.9993 */
// 	 Motor_Params.Kt = 0.0304f;   // 1.5*Ke
// 	 Motor_Params.Ke = 0.0203f;   // Ke = P*
// 	 Motor_Params.Ld = 0.0018f;   // d轴电感
// 	 Motor_Params.Lq = 0.0021f;   // q轴电感
// 	 Motor_Params.Phase_R =  2.92f;   // 离线辨识实测（identify r）2.92f
// 	 Motor_Params.Pole_Pairs = 7.0f;
// 	 Motor_Params.Rotor_inertia = 0.0000025f;   // 惯量辨识实测（identify j）2.5e-6
// 	 Motor_Params.friction_B_nm_rps   = 0.0f;   // 粘性 B（N·m/RPS）占位，由 identify flux 提交
// 	 Motor_Params.friction_Coulomb_nm = 0.0f;   // 库仑 fc（N·m）占位，由 identify flux 提交
// 	 Motor_Params.VBUS = 12.0f;
// 	 Motor_Params.Reduction_ratio = 1.0f;
//    Motor_Params.Torque_Rating = 0.02f;
// 	 Motor_Params.RPM_Rating = 2400.0f;

    //小米cyberdog电机
	 Motor_Params.Current_Rating = 5.0f;   /* 相电流峰值额定（1mΩ + CSA 增益20 → ±82.5A 量程） */
	 Motor_Params.Current_Max = 40.0f;      /* 峰值上限（辨识安全上界 / 过流参考）；软件过流 = ×1.2 = 48A */
	 /* ===== 以下为换 1mΩ 分流 + CSA 增益 20 + 对调相线后的完整辨识结果（2025 台架实测） =====
	  * identify flux：we=1403/1873/2349/2813 rad/s 四点最小二乘，段斜率
	  *   0.003741/0.003694/0.003641 漂移仅 2.7%，残差 ±0.015V（<0.15%），R²>0.9999；
	  *   we/(2π·21) 反算机械转速 10.63/14.19/17.80/21.32 RPS 与打印 spd 逐点吻合 → 极对数与编码器标度互相验证。
	  *   要点：iq 仅 0.18~0.21A 且 id≈0 → 相序/零点/方向都对（对调相线已生效）。 */
	 Motor_Params.Flux = 0.003694f;  /* identify flux: psi = 3.694 mWb */
	 Motor_Params.Kt = 0.116345f;    /* = 1.5·P·Flux = 1.5×21×0.003694 */
	 Motor_Params.Ke = 0.077564f;    /* = P·Flux = 21×0.003694 (V per mech rad/s, 相峰值) */
	 Motor_Params.Ld = 0.000195f;    /* identify l: Ld = 195 uH */
	 Motor_Params.Lq = 0.000393f;    /* identify l: Lq = 393 uH（Lq/Ld = 2.02，凸极性明显） */
	 Motor_Params.Phase_R =  0.2631f;   /* identify r: Rs = 0.2631 ohm */
	 Motor_Params.Pole_Pairs = 21.0f;   /* identify pole: pf=20.994 pr=21.007 */
	 /* identify j：J_accel=8.68e-5、J_decel=7.16e-5 → J=7.92e-5 kg·m²（err=17.5%，n=4/4）。
	  * 17.5% 的两段差来自摩擦模型不对称 + 编码器量化噪声（16384cpr 单拍速度 LSB 3.83 rad/s）
	  * 经高带宽 PLL（wn=1000Hz）放到加速度上。对速度环增益（Kp ∝ J）与 LESO 的 b0（∝1/J）
	  * 都在可接受范围；**若闭环后速度环振荡，优先怀疑这个值**（可用更慢的加速斜坡重测）。 */
	 Motor_Params.Rotor_inertia = 0.0000792f;
	 /* identify flux 同时给出摩擦模型；自检：B·21.3 RPS + fc = 0.0074+0.0170 = 0.0244 N·m
	  *  vs iq·Kt = 0.214×0.116345 = 0.0249 N·m → 吻合（B 单位是 N·m/RPS）。 */
	 Motor_Params.friction_B_nm_rps   = 0.000347f;   /* identify flux: B */
	 Motor_Params.friction_Coulomb_nm = 0.017025f;   /* identify flux: fc */
	 Motor_Params.VBUS = 24.0f;
	 Motor_Params.Reduction_ratio = 6.0f;
     Motor_Params.Torque_Rating = 24.0f;
	 /* !! 24V 母线下的电压上限提醒：SVPWM 线性区最大相电压峰值 = Vbus/√3 = 13.86V，
	  *    而 1800RPM(=30RPS) 时反电动势峰值 = Ke×2π×30 = 0.077564×188.5 = 14.62V > 13.86V
	  *    → 24V 下无载最高约 1706 RPM，30 RPS 目标不可达（会被电压限幅削顶）。
	  *    高速段电流能力同样受限（630Hz 时 ωLq=1.56Ω，10A 就要 15.6V）。
	  *    要跑满 1800RPM 需抬母线到 ~27V 以上（板级 OV 软闸 33V/硬闸 40V 有足够余量）。 */
	 Motor_Params.RPM_Rating = 1800.0f;
	 /* init filter（注：这两个 PLL 结构体已不再参与控制环，模型内部自带 EncoderPLL；
	  * 死区按编码器分辨率算，换编码器自动跟随） */
	 Encoder_Configure(Motor_Params.Pole_Pairs, Encoder.zero_offset);
	 Encoder_LogStartupInfo();
	 Encoder_PLL_mecFilter = Angle_PLL_filter_init(FOC_Frequency, 250.0f, Encoder_Deadband(), 2.0f * (Motor_Params.RPM_Rating/60) * 2*PI);
	 Encoder_PLL_eleFilter = Angle_PLL_filter_init(FOC_Frequency, 250.0f, Encoder_Deadband() * Motor_Params.Pole_Pairs,
	                                               2.0f * (Motor_Params.RPM_Rating/60) * 2*PI * Motor_Params.Pole_Pairs);
	 
	 FOC_Generated_Init();    // Simulink 生成 FOC 算法参数初始化
	 MotorState.ctrl_mode = MT_SPEED;
	 CAN_bsp_Init();          // CAN 控制接口（过滤器/接收/回调）
	 MotorCtrl_Init();        // 电机控制层（默认停止）
	 Identify_Init();         // 离线辨识框架（注册任务 handler）
	 Safety_Config_Init(&Safety_Config);   // 安全阈值从 Motor_Params 推算（运行期只读）
	 FOC_Interrupt_Start();   // 上电后持续运行 10 kHz FOC/采样中断

	 //MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/* ===== 看门狗交棒：TIM4 中断 → SafetyTask =====
	 * 关掉 TIM4 更新中断之后，运行期全系统只剩 SafetyTask 一处喂狗
	 * （app_tasks.c 的 SafetyTask_Entry）。若这里没真正关掉，
	 * "SafetyTask 卡死"会被 TIM4 掩盖，IWDG 就等于没接。
	 * 读回 DIER 确认，而不是"以为关掉了"。 */
	__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
	if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != 0U)
	{
		printf("[WDG] ERROR: TIM4 update IT still enabled -> IWDG masked by TIM4!\r\n");
	}
	HAL_IWDG_Refresh(&hiwdg);   /* 给"调度器启动 + SafetyTask 首次喂狗"留足余量 */

	// Phase 1：创建静态任务（BringUpTask + LegacyMainTask）并启动 FreeRTOS 调度器
	AppTasks_Init();
	vTaskStartScheduler();

	// 调度器正常不应返回；若返回（无任务/错误）则挂死便于调试
	while (1)
	{
	}
  /* USER CODE END WHILE */
}

/*
 * 打印当前挂载的编码器与配置。换编码器（AS5047 <-> AS5600）时靠这行确认
 * 到底挂上的是哪一个、分辨率多少、零点偏移多少。
 */
static void Encoder_LogStartupInfo(void)
{
	printf("Encoder: %s cpr=%lu pole_pairs=%.1f offset=%.4f rad status=%d\r\n",
	       Encoder_GetName(),
	       (unsigned long)Encoder_GetResolution(),
	       (double)Encoder.pole_pairs,
	       (double)Encoder.zero_offset,
	       (int)Encoder_GetStatus());
}

/* PLL 死区 = 1 LSB 的 10%，按当前编码器分辨率算（AS5600=4096，AS5047=16384），
 * 这样换编码器时不用手工改这里的常数。未挂载时退回 AS5600 的分辨率。 */
static float Encoder_Deadband(void)
{
	const float res = (Encoder_GetResolution() > 0U) ? (float)Encoder_GetResolution() : 4096.0f;

	return 1.0f / res * 2.0f * PI * 0.1f;
}


void Param_init(void)
{
		// 注：Simulink 生成代码内部自带电流/速度/位置 PID，旧 MID_foc PID 结构已移除
		Encoder_ResetTracking();
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
/* 见 main_user.h 的说明：ADC1 规则组 = 6 路输出端霍尔（连续 + DMA） */
volatile uint16_t Hall_ADC_DMA[6] = {0};
//校准作用,电流传感器的理论偏移值为1.65V
//三电阻采样时可以用软件构建动态的偏移电压的ADC值，三相电流的ADC值相加是3倍的偏移电压ADC值的数学关系
/* ===== 电流零偏校准（2026-09-27 加完成校验）=====
 * 原来只靠 `HAL_Delay(300)` 等 ISR 累计 1000 拍，**无法知道是否真的采到了**：
 * ISR 一旦不跑（触发源接错 / JEOCIE 被 HAL 关掉 / CEN 丢了），零偏就停在 0，
 * 而 `ic = -(0 - adc)×I_SCALE` 会算出 ≈ +82 A 的假电流 → 直接触发过流甚至更糟。
 * 现在：有界等待 + 完成标志 + 失败退中点 + 上报故障 + 返回值让调用方放弃强拖。 */
#define OFFSET_CAL_SAMPLES      1000U   /* 10kHz × 100ms */
#define OFFSET_CAL_TIMEOUT_MS   500U    /* 5 倍余量 */
#define ADC_MID_SCALE           2048    /* 1.65V/3.3V × 4095 ≈ 2047.5，电流传感器理论中点 */

static   uint16_t s_offset_cal_i     = 0U;   /* 已累计拍数（ISR 里自增） */
volatile uint8_t  g_offset_cal_done  = 0U;   /* 1 = 采满并算完 */
volatile uint16_t g_offset_cal_count = 0U;   /* 实际采到的拍数（诊断用） */

int Offset_Current_Start(void)
{
	/* 关三个 PWM 通道：**不要**用 HAL_TIM_PWM_Stop —— 它会清 TIM1->CR1.CEN，
	 * 而注入触发链（TIM1 -> OC4REF -> TRGO -> ADC）靠的就是计数器在跑。
	 * 这里只清 CCxE，计数器保持运行；后面 FOC_Interrupt_Start() 会重新起 CH4。 */
	TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
  HAL_Delay(50);

	s_offset_cal_i      = 0U;
	g_offset_cal_count  = 0U;
	g_offset_cal_done   = 0U;
	foc_abc_current_i.offset_phase_ib = 0;   /* int32_t，1000×2048 不会溢出 */
	foc_abc_current_i.offset_phase_ic = 0;

	MotorState.run_state = RUNSTATE_CALIBRATING;
	(void)HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_4);
	/* 电流C 已迁到 ADC3 注入 rank1（PC0 / ADC3_IN10），校准必须跟着走，
	 * 否则读的是 ADC1（现在是霍尔）→ 零偏会算成霍尔的读数，电流标度全废。
	 * 触发源是 TIM1_TRGO(OC4REF)，只要求 CEN=1（main_user 开头已 HAL_TIM_Base_Start），
	 * 不要求 CC4E/MOE，所以这里在 FOC_Interrupt_Start() 之前也能正常转换。 */
	HAL_ADCEx_InjectedStart_IT(&hadc3);  //电流C + Vbus（JEOC = FOC ISR 源）
	HAL_ADCEx_InjectedStart(&hadc2);     //电流B

	/* 有界等待采满：此处是调度器启动前的 main 上下文，HAL_GetTick() 可靠
	 * （TIM6 中断能正常进入），所以用 tick 轮询即可 —— 与 ISR 内的 DWT 超时不同。 */
	{
		uint32_t t0 = HAL_GetTick();
		while ((g_offset_cal_done == 0U) &&
		       ((HAL_GetTick() - t0) < OFFSET_CAL_TIMEOUT_MS)) {
			/* 等 ISR 累计 */
		}
	}

	/* 无论成败都停掉注入组；FOC_Interrupt_Start() 后面会重新启动它 */
	(void)HAL_TIM_PWM_Stop( &htim1, TIM_CHANNEL_4);
	HAL_ADCEx_InjectedStop_IT(&hadc3);
	HAL_ADCEx_InjectedStop(&hadc2);

	if (g_offset_cal_done == 0U) {
		/* 失败：绝不能让零偏停在 0。退回理论中点（读数≈0A）并上报故障。 */
		foc_abc_current_i.offset_phase_ib = ADC_MID_SCALE;
		foc_abc_current_i.offset_phase_ic = ADC_MID_SCALE;
		MotorStateMachine_PostFault(FAULT_CURRENT_SENSE_MASK);
		printf("[CAL] ERROR: current offset FAILED (n=%u/%u) -> fallback mid-scale %d, fault posted\r\n",
		       (unsigned)g_offset_cal_count, OFFSET_CAL_SAMPLES, ADC_MID_SCALE);
		return -1;
	}

	printf("[CAL] current offset ok: ib=%ld ic=%ld (n=%u)\r\n",
	       (long)foc_abc_current_i.offset_phase_ib,
	       (long)foc_abc_current_i.offset_phase_ic,
	       (unsigned)g_offset_cal_count);
	return 0;
}

void Offset_CurrentReading(void)
{
  s_offset_cal_i++;
	
	/* ADC Channel used for current reading are read  in order to get zero currents ADC values*/
	foc_abc_current_i.offset_phase_ib +=  ADC2->JDR1;
  foc_abc_current_i.offset_phase_ic +=  ADC3->JDR1;   /* 电流C：ADC3 注入 rank1 */


	if(s_offset_cal_i >= OFFSET_CAL_SAMPLES)
  {
		MotorState.run_state = RUNSTATE_STOPPED;
		g_offset_cal_count = s_offset_cal_i;
		s_offset_cal_i = 0U;

		foc_abc_current_i.offset_phase_ib = foc_abc_current_i.offset_phase_ib / (int32_t)OFFSET_CAL_SAMPLES;
		foc_abc_current_i.offset_phase_ic =	foc_abc_current_i.offset_phase_ic / (int32_t)OFFSET_CAL_SAMPLES;
		g_offset_cal_done = 1U;   /* 置位放在算完之后；停止注入组交给 Offset_Current_Start() */
  }
}

//===========================================================================
// No more.
//===========================================================================
