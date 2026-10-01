#include "FOC_run.h"
#include "Timer.h"
#include "main_user.h"
#include "SEGGER_RTT_Port.h"

#include "foc.h"
#include "Encoder_Adapt.h"        /* 编码器服务层：Encoder_Update / Encoder.data */
#include "Safety_Module.h"        /* Safety_FastStep */
#include "motor_state_machine.h"  /* MotorStateMachine_PostFault */
#include "identify.h"             /* 离线辨识：RUNSTATE_IDENTIFYING 时填电压输入 */
#include "MotorCtrl.h"            /* PowerStage_MoeOff（快检无条件关断） */

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
/* FOC ISR 进入次数。用途：`diag` 里直接看这个数有没有在涨，就能判定
 * "采样链路是否活着" —— 它是 TIM1_TRGO(OC4REF) → ADC 注入 → JEOC 整条链的存活指示。 */
volatile uint32_t g_foc_isr_cnt = 0U;
/* 输出端霍尔扫描分频：10kHz / 10 = 1kHz（见 ISR 内写 SWSTART 的那段） */
static uint8_t    s_hall_scan_div = 0U;
/* 霍尔扫描序号（seqlock）：ISR 在启动新一轮扫描前 +1；
 * 解码侧读前/读后各取一次，不相等就重读，避免读到"半新半旧"的 6 通道快照。 */
volatile uint32_t g_hall_scan_seq = 0U;
volatile uint32_t g_foc_isr_us  = 0U;   /* FOC ISR 每周期耗时（us），Ozone 实时观察 */
void (* func_ptr)(ADC_HandleTypeDef *hadc) = NULL;
// 10khz  ADC注入组采样完成中断
uint32_t Predict_ThreeHallangle;
float Predict_eleangle;

/* 连续机械角(rad, 多圈)：10kHz ISR 累加，位置模式反馈（FOC_Generated_Step 读取） */
float g_mech_pos_rad = 0.0f;
static float g_last_angle_rad = -1.0f;   /* <0 = 首样本哨兵 */

float Phase_Ra,Phase_Rb,Phase_Rc;
float Phase_La,Phase_Lb,Phase_Lc;
/* 母线电压。**初值必须给标称值**（原来两行都是无初值的暂定定义 → 上电为 0）：
 * 模型用 Motor_Params.VBUS 做 SVPWM 归一化，若 ADC3 还没出数就停在 0，
 * 会算出 Inf/NaN；而 NaN 会让 S_FAULT_OVER 的 fabsf(speed) < 0.1 恒为 false，
 * 状态机从此卡死在 FAULT_OVER 出不来（实测踩过）。 */
float Vbus = 24.0f;

/* 编码器一拍耗时（µs，DWT 实测，SystemCoreClock 推导）。g_enc_read_wcet_us 是
 * 上电以来的最大值。用途：证明/推翻"编码器读取占了多少 FOC 中断预算"这类估算。
 * 期望：AS5047 阻塞读（2 帧 @7.5MHz + 4 次 t_CSn/t_L 间隔）约 6 us。
 * 若明显更大，先查 as5047_cs_gap 与 SPI 分频，而不是去改协议。 */
volatile uint32_t g_enc_read_us      = 0U;
volatile uint32_t g_enc_read_wcet_us = 0U;
void HAL_ADCEx_InjectedConvCpltCallback( ADC_HandleTypeDef *hadc)
{
	uint32_t _isr_t0 = DWT->CYCCNT;   /* FOC ISR 耗时测量（DWT 120MHz，SEGGER_RTT_Port 已使能） */

	/* 进入次数必须记在**所有提前 return 之前**：下面的校准路径会直接 return，
	 * 而这个计数被 SafetyTask 用作"IWDG 是否允许喂狗"的判据 ——
	 * 只要 ISR 进了就该算数，否则会在校准期间误判"采样链路已死"。
	 * （当前 RUNSTATE_CALIBRATING 只在调度器启动前出现，由 TIM4 喂狗，暂时不会踩到；
	 *   但语义上必须这样，将来校准若移到任务域就不会出事。） */
	g_foc_isr_cnt++;

	if( hadc->Instance == hadc3.Instance)
	{
		/* ===== 必须每拍重新使能 JEOC 中断（否则只会进一次！）=====
		 * F1 HAL 的 HAL_ADC_IRQHandler 在注入 EOC 分支里有这么一段：
		 *     if (ADC_IS_SOFTWARE_START_INJECTED(hadc) ||
		 *         (JAUTO==0 && ADC_IS_SOFTWARE_START_REGULAR(hadc) &&
		 *          hadc->Init.ContinuousConvMode == DISABLE))
		 *     { __HAL_ADC_DISABLE_IT(hadc, ADC_IT_JEOC); }     // ← 关掉 JEOCIE
		 *     HAL_ADCEx_InjectedConvCpltCallback(hadc);         // ← 才调回调
		 *     __HAL_ADC_CLEAR_FLAG(hadc, JSTRT|JEOC);           // ← 回调之后才清标志
		 *
		 * 本工程三个 ADC 的规则组都是 ADC_SOFTWARE_START（宏为真），
		 * 而注入组是 T1_TRGO（不是 JSWSTART，宏为假），
		 * 所以上面那个条件**等价于 ContinuousConvMode == DISABLE**。
		 * 一旦成立，HAL 在第一次注入中断后就把 JEOCIE 关掉 →
		 * **FOC 中断只进一次，之后再无触发**（实测症状：INIT 里进一次，进 IDLE 后再没进）。
		 *
		 * HAL 是在调用本回调**之前**关的，所以在这里重新使能是最直接、最稳的修法 ——
		 * 不再依赖"某个 ADC 的 ContinuousConvMode 必须配成 ENABLE"这种隐式耦合。
		 * 另外 adc.c 里也把 ADC3 的 ContinuousConvMode 配成 ENABLE 做双保险。
		 * 注意不能依赖"读 JDR 会自动清 JEOC"这种未核实的行为：清标志由 HAL 在回调后显式做。 */
		__HAL_ADC_ENABLE_IT(hadc, ADC_IT_JEOC);

		if(MotorState.run_state == RUNSTATE_CALIBRATING)
		{
			Offset_CurrentReading();
			/* 注意 DWT->CYCCNT 计的是 HCLK（本板 120MHz）。这里用 SystemCoreClock 推导，
		 * 不要写死 120：改主频（或换回 72MHz 的 STM32F103）时 WCET 数字才不会静默失真。
		 * 与 AS5047 驱动的寄存器级超时用同一套时钟推导口径。 */
		{   uint32_t _el = (DWT->CYCCNT - _isr_t0) / (SystemCoreClock / 1000000U);
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
		
		/* 编码器服务层：取一拍。DMA 模式下这里同时"转换上一帧、发下一帧"，
		 * 所以每拍必须恰好调用一次。换编码器不改这里。 */
		/* DWT 分段计时：专量"编码器一拍"在 FOC 中断里到底花多少。
		 * 这是 §17 FOC 中断时间优化的第一手数据 —— 在此之前只能靠估算，
		 * 而估算错得离谱过：AS5047 的 CS 间隔原本写成 24 轮 volatile NOP 循环，
		 * 实际 ≈2us × 每样本 4~6 次，占掉 10kHz 周期的 12%。 */
		{
			uint32_t _et0 = DWT->CYCCNT;
			Encoder_Update();
			{
				uint32_t _eel = (DWT->CYCCNT - _et0) / (SystemCoreClock / 1000000U);
				g_enc_read_us = _eel;
				if (_eel > g_enc_read_wcet_us) g_enc_read_wcet_us = _eel;
			}
		}
		{   /* 连续机械位置：角度增量 + ±π 绕回检测（首样本定起点 = 电气零点对齐位置）
		     * 注意：多圈位置属于位置环自己的口径，故仍在本层累加。
		     * Encoder.ResetTracking()（Param_init 会调）只清服务层内部量，
		     * 不会重置这里的 g_mech_pos_rad，保持与迁移前一致的行为。 */
			float ang = Encoder.data.mec;
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
		
		/***********************   2、Processing ADC  ***************************/
		
		foc_abc_current_i.adc_phase_ic =  ADC3->JDR1;   /* 电流C：PC0 / ADC3_IN10，注入 rank1 */
	  foc_abc_current_i.adc_phase_ib  =  ADC2->JDR1;   /* 电流B：PC1 / ADC2_IN11，注入 rank1 */
	

    foc_abc_current_i.ic  =  -((float)foc_abc_current_i.offset_phase_ic - foc_abc_current_i.adc_phase_ic) * I_SCALE;
		foc_abc_current_i.ib  =  -((float)foc_abc_current_i.offset_phase_ib - foc_abc_current_i.adc_phase_ib) * I_SCALE;	//I_SCALE=(3.3/4095)/(Rs×Gain)=(3.3/4095)/(0.001Ω×20)
		foc_abc_current_i.ia = - foc_abc_current_i.ic - foc_abc_current_i.ib;

    /* ===== 母线电压：ADC3 注入 rank2（PC3 / ADC3_IN13）=====
     * 与两个电流同被 TIM1_TRGO(OC4REF) 触发，所以是 PWM 同步采样。
     * JEOC 是在整个注入序列（两个 rank）都转完之后才置位，而本 ISR 正是被它触发的，
     * 所以此刻 JDR1/JDR2 都已就绪，直接读即可。
     * 清标志由 HAL 在回调返回后显式做（__HAL_ADC_CLEAR_FLAG(JSTRT|JEOC)）；
     * **不要**依赖"读 JDR 自动清 JEOC"这种未核实的行为。 */
    Vbus = (float)ADC3->JDR2 * V_SCALE;

    /* ===== 输出端 6 路霍尔：启动 ADC1 规则组的一次 6 通道扫描（10kHz/10 = 1kHz）=====
     * ===== 输出端 6 路霍尔：ADC1 规则组，**单次扫描 + 软件触发** =====
     * 排布（2026-09-27 定稿）：
     *   ADC3 注入 = 电流C(rank1) + Vbus(rank2)  → JEOC = 本 ISR 源，**只有注入组**
     *   ADC2 注入 = 电流B(rank1)                → 无中断，这里轮询 JDR1
     *   ADC1 规则 = 6 路霍尔（单次扫描 + SWSTART，DMA 循环填 Hall_ADC_DMA[6]）
     *
     * 为什么不用连续自由跑：6 通道一轮 16.4us，而周期是 100us，100/16.4 = 6.1 **除不尽**
     * → 采样时刻相对 PWM 开关沿会**逐周期漂移**，霍尔读数会随机落在开关噪声上，
     * 既不准也不可复现。改成在 FOC ISR 里固定分频点写 SWSTART：
     *   - 触发时刻锁在 PWM 周期内的固定位置（ISR 本身由 TIM1_TRGO 触发）→ 确定性采样
     *   - 落在与电流采样同一个零矢量安静窗口 → 噪声小
     *   - 一轮扫描 16.4us 远小于 1ms 分频周期，下一拍触发前早已完成
     *
     * 数据新鲜度与"撕裂"：本 ISR 只负责"启动下一轮"，**解码不在 ISR 里**（那是 3 组
     * atan2 的 float 重活）。因为 buffer 只在 ISR 里被 DMA 改，任务侧读 6 个 halfword
     * 可能被本 ISR 的下一轮扫描打断 → 用 g_hall_scan_seq 做 seqlock 重试即可
     * （解码器还没写，见 §18.4）。 */
    if (++s_hall_scan_div >= 10U) {
        s_hall_scan_div = 0U;
        g_hall_scan_seq++;      /* 先递增序号，解码侧前后各读一次做撕裂检测 */
        /* 必须同时置 EXTTRIG —— F1 的规则组软件启动靠这两位配合，只写 SWSTART
         * 在 EXTSEL 已设为 SWSTART 但 EXTTRIG 未置时不会真正启动
         * （HAL 的 ADC_SoftwareStartConv 就是两个位一起写）。 */
        ADC1->CR2 |= (ADC_CR2_EXTTRIG | ADC_CR2_SWSTART);
    }
    Motor_Params.VBUS = Vbus;   /* 母线电压实时化：模型 SVPWM/限幅/电流限随实测母线（FOC_Generated_Step 每步同步） */
		/*********************  Safety 快检（Phase 4） *************************/
		{
			SafetyFastInput_t fast_in = { Vbus, { foc_abc_current_i.ia, foc_abc_current_i.ib, foc_abc_current_i.ic } };
			fault_mask_t fast_faults = 0;
			Safety_FastStep(&fast_in, &Safety_Config, &fast_faults);
			if (fast_faults != 0U) {
				/* 当场无条件关 PWM 输出。
				 * 原来这里是 __HAL_TIM_MOE_DISABLE(&htim1) —— 那个宏在 PWM 运行中
				 * 是空操作（CCER 的 CCxE 非 0 就不写 BDTR），所以"快速关断"其实
				 * 一直没生效，全靠 1ms 后状态机的 PowerStage_Disable() 兜底。
				 * 详见 MotorCtrl.h 的 PowerStage_MoeOff() 说明。 */
				PowerStage_MoeOff();
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
	/* FOC ISR 每周期耗时记录（µs，由 SystemCoreClock 推导），Ozone 加 g_foc_isr_us / g_foc_wcet_us */
	{
		uint32_t _el = (DWT->CYCCNT - _isr_t0) / (SystemCoreClock / 1000000U);
		g_foc_isr_us = _el;
		if (_el > g_foc_wcet_us) g_foc_wcet_us = _el;
		/* g_foc_isr_cnt 在 ISR 最前面自增（校准路径会提前 return，那里也必须计数） */
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
