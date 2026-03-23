#include "dead_comp.h"



/**************************************************************************************** */
// fast sin/cos helpers (use your optimized versions if available)
#ifndef Fast_arm_sin_f32
  #define sinf(x) arm_sin_f32(x) 
  #define cosf(x) arm_cos_f32(x)
  #define Fast_arm_sin_cos_f32(x, ps, pc) do { *(ps) = sinf(x); *(pc) = cosf(x); } while(0)
#endif




// init helper: choose Ts (s), initial_omega (rad/s, nominal grid/machine electrical freq), sogi_k (1..2), desired_pll_bw_Hz
void SP_SOGI_PLL_init(SP_SOGI_PLL_t *p,
                                    float Ts,
                                    float initial_omega,
                                    float sogi_k,
                                    float desired_pll_bw_Hz,
                                    float deadband,
                                    float I_min)
{
    p->Ts = Ts;
    p->k = sogi_k;
    p->v_alpha = 0.0f;
    p->v_beta  = 0.0f;

    p->theta = 0.0f;
    p->omega = initial_omega; // rad/s (e.g., 2*pi*electrical_freq)
    p->pll_sin = 0.0f; p->pll_cos = 1.0f;
    p->integ = 0.0f;

    // PLL gains from desired bandwidth (Hz) -> wn = 2*pi*BW(rad/s)
    // Continuous design: Kp = 2*zeta*wn, Ki_cont = wn^2 (rad^2/s^2)
    // For discrete integrator implemented as integ += error (no *Ts),
    // set Ki_discrete = wn^2 * Ts
    float wn = 2.0f * 3.14159265358979323846f * desired_pll_bw_Hz;
    float zeta = 0.707f;
    p->Kp = 2.0f * zeta * wn;         // proportional (on sin_error)
    p->Ki = wn * wn * Ts;             // discrete integral mapping

    p->deadband = deadband;
    p->I_min = I_min;

    p->omega_max = initial_omega * 5.0f; // safety limits, adjust as needed
    p->omega_min = initial_omega * -5.0f;
    p->integ_max = 1e6f; p->integ_min = -1e6f;

    p->amp = 0.0f;
    p->initialized = 0;
}


// Main update: input single-phase instantaneous current `i_in` (A) -> returns filtered current via pointer
// NOTE: this function updates internal SOGI and PLL states. Use at each FOC cycle.
void SP_SOGI_PLL_update(SP_SOGI_PLL_t *p, float i_in, float *i_filtered_out)
{
    // 1) SOGI differential eqn (forward Euler discretization)
    // continuous: dv_alpha/dt = k*omega*(input - v_alpha) + omega * v_beta
    //             dv_beta/dt  = -omega * v_alpha + k*omega*(0 - v_beta)  (commonly second eqn may include damping; here we use basic)
    // common simplified SOGI:
    //   v_alpha_dot = p->k * p->omega * (i_in - p->v_alpha) + p->omega * p->v_beta;
    //   v_beta_dot  = -p->omega * p->v_alpha;
    //
    // discretize:
    float omega = p->omega;
    float valpha_dot = p->k * omega * (i_in - p->v_alpha) + omega * p->v_beta;
    float vbeta_dot  = -omega * p->v_alpha;
    // Euler step
    p->v_alpha += p->Ts * valpha_dot;
    p->v_beta  += p->Ts * vbeta_dot;

    // 2) magnitude
    float I_mag = sqrtf(p->v_alpha * p->v_alpha + p->v_beta * p->v_beta);
    p->amp = I_mag;

    // initialization: if not initialized and mag large enough, init theta to measured angle
    if (!p->initialized) {
        if (I_mag > p->I_min && I_mag > 1e-6f) {
            p->theta = atan2f(p->v_beta, p->v_alpha);
            Fast_arm_sin_cos_f32(p->theta, &p->pll_sin, &p->pll_cos);
            p->initialized = 1;
        } else {
            // not enough amplitude: filtered output = v_alpha (best we can)
            if (i_filtered_out) *i_filtered_out = p->v_alpha;
            return;
        }
    }

    // 3) compute normalized measurement cos/sin (avoid atan2)
    float invI = 1.0f / (I_mag + 1e-12f); // protect divide
    float cos_m = p->v_alpha * invI;
    float sin_m = p->v_beta  * invI;

    // 4) phase error = sin(meas - pll) = sin_m * cos_pll - cos_m * sin_pll
    float sin_error = sin_m * p->pll_cos - cos_m * p->pll_sin;
    if (fabsf(sin_error) < p->deadband) sin_error = 0.0f;

    // 5) PLL PI: integ accumulate (discrete style)
    p->integ += sin_error * p->Ki;
    // clamp integral
    if (p->integ > p->integ_max) p->integ = p->integ_max;
    if (p->integ < p->integ_min) p->integ = p->integ_min;

    // delta_omega (rad/s)
    float delta_omega = p->Kp * sin_error + p->integ;
    // optional limit
    if (delta_omega > p->omega_max) delta_omega = p->omega_max;
    if (delta_omega < p->omega_min) delta_omega = p->omega_min;

    // 6) update pll omega and theta
    // If you have a feedforward known base freq, you can do p->omega = omega_ff + delta_omega;
    // Here we directly set p->omega = delta_omega (works if delta_omega near nominal). If you want a nominal,
    // supply p->omega with initial_omega and modify p->omega = initial + delta_omega. Adjust as needed.
    p->omega = delta_omega;
    p->theta += p->omega * p->Ts;

    // optionally normalize theta every N steps in background to avoid huge growth
    //  (omitted here for speed; add if desired)

    // 7) update cached sin/cos
    Fast_arm_sin_cos_f32(p->theta, &p->pll_sin, &p->pll_cos);

    // 8) reconstruct filtered single-phase current: use amp * sin(theta)
    float i_f = p->amp * p->pll_sin;
    if (i_filtered_out) *i_filtered_out = i_f;
}


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
}PLL_t;
/**
 *PLL滤波器，去除电角度噪声
 *@param PLL *PLL_Filter 锁相环滤波结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的值
 */
// 	#define PWM_FREQ 20000.0f
//	#define SpeedPllBandwidth 250.0f   //rad/s
PLL_t* Current_PLL_filter_init(float FOC_FREQ, float PllBandwidth, float deadband, float Max_Angle_Speed)
{
    // 参数合法性检查（可根据需要调整阈值）
    if (FOC_FREQ <= 0.0f || PllBandwidth <= 0.0f) {
#ifdef DEBUG
        DEBUG_OUT("PLL init param invalid: FOC_FREQ=%f, PllBandwidth=%f", FOC_FREQ, PllBandwidth);
#endif
        return NULL;
    }

    // 分配结构体
    PLL_t *pll = (PLL_t*)malloc(sizeof(PLL_t));
    if (!pll) {
#ifdef DEBUG
        DEBUG_OUT("PLL allocation failed\n");
#endif
        return NULL;
    }

    // 清零初始化
    memset(pll, 0, sizeof(PLL_t));

    // 基本参数赋值（与原静态 init 行为一致）
    pll->FOC_FREQ = FOC_FREQ;
    pll->SpeedPllBandwidth = PllBandwidth;
    // SpeedPllKp = 2 * Wn * zeta （这里默认 zeta = 1.0）
    pll->SpeedPllKp = (2.0f * pll->SpeedPllBandwidth * 1.0f);
    // SpeedPllKi = Wn^2 * Ts （Ts = 1 / FOC_FREQ）
    pll->SpeedPllKi = (pll->SpeedPllBandwidth * pll->SpeedPllBandwidth) / (pll->FOC_FREQ);
    // OmegaToTheta (电角速度 -> 电角度的系数) = 1 / FOC_FREQ （原实现）
    pll->OmegaToTheta = 1.0f / pll->FOC_FREQ;

    // 经验默认值：可按需调整或由调用者设置
    pll->deadband = deadband;
    pll->PLL_Omega_Maxlimit = Max_Angle_Speed;    //系统最大角速度 rad/s  默认给一个极大值
    pll->PLL_Omega_Minlimit = - pll->PLL_Omega_Minlimit;
    pll->PLL_DeltTheta_Sum_Maxlimit = pll->PLL_Omega_Maxlimit / pll->SpeedPllKi;   // PLL_Omega积分累计最大值，保证PLL_Omega积分项不过大
    pll->PLL_DeltTheta_Sum_Minlimit = - pll->PLL_DeltTheta_Sum_Maxlimit;  

    pll->initialized_Flag = 0;
		
//		pll->fliter_struct = median_filter_init(5);

#ifdef DEBUG
    DEBUG_OUT("PLL allocated %p FOC_FREQ=%f Bandwidth=%f Kp=%f Ki=%f",
              (void*)pll, pll->FOC_FREQ, pll->SpeedPllBandwidth, pll->SpeedPllKp, pll->SpeedPllKi);
#endif

    return pll;
}

static inline float Limit_Sat( float fdata,float fmax, float fmin) 
{
    fdata = (fdata > fmax) ? fmax : fdata;
    fdata = (fdata < fmin) ? fmin : fdata;

    return fdata;
}

#define Lpf(a,a_last,lpf)  (a * lpf + a_last * (1 - lpf))
float Current_PLL_filter(PLL_t *PLL_Filter, float eleAngle_input,float eleAngle_input_b, float eleAngle_input_c)
{
    /* 1. 首次初始化：直接把 PLL_Theta 设为输入，清积分*/
    if (!PLL_Filter->initialized_Flag) {
        PLL_Filter->PLL_Theta = eleAngle_input;
        PLL_Filter->initialized_Flag = 1;
        return PLL_Filter->PLL_Theta;
    }

		/*  2. 计算角度差  */
		PLL_Filter->eleangle_Sinx = arm_sin_f32(eleAngle_input);  
		PLL_Filter->eleangle_Cosx = arm_cos_f32(eleAngle_input);
		PLL_Filter->PLLAngle_Sinx = arm_sin_f32(PLL_Filter -> PLL_Theta);
		PLL_Filter->PLLAngle_Cosx = arm_cos_f32(PLL_Filter -> PLL_Theta);

    float sin_error = (PLL_Filter->eleangle_Sinx * PLL_Filter->PLLAngle_Cosx) - \
                                (PLL_Filter->eleangle_Cosx * PLL_Filter->PLLAngle_Sinx);
		PLL_Filter->PLL_DeltTheta = sin_error;  // 直接用 sin_error 作为误差值

     if (fabsf(PLL_Filter->PLL_DeltTheta) < PLL_Filter->deadband) {      // 死区处理，避免噪声在小误差下触发积分/PI动作
         PLL_Filter->PLL_DeltTheta = 0.0f;
     }

	
	/* 3. 计算角速度和角度  */
	PLL_Filter->PLL_DeltTheta_Sum += PLL_Filter->PLL_DeltTheta;  //PLL积分项
  Limit_Sat(PLL_Filter->PLL_DeltTheta_Sum ,PLL_Filter -> PLL_DeltTheta_Sum_Maxlimit, PLL_Filter -> PLL_DeltTheta_Sum_Minlimit) ;
		 
	PLL_Filter->PLL_Omega = (PLL_Filter -> SpeedPllKp * PLL_Filter->PLL_DeltTheta) + \
	                            (PLL_Filter -> SpeedPllKi * PLL_Filter->PLL_DeltTheta_Sum);
  Limit_Sat(PLL_Filter->PLL_Omega ,PLL_Filter -> PLL_Omega_Maxlimit, PLL_Filter -> PLL_Omega_Minlimit);
	PLL_Filter->PLL_Theta += (PLL_Filter->PLL_Omega* PLL_Filter -> OmegaToTheta);


				
		static float eleangle_Sinx_b = 
		static float eleangle_Cosx_b	=
		static float PLLangle_Sinx_b =
		static float PLLangle_Cosx_b	=
		static float PLL_DeltTheta_Sum_b, PLL_Omega_b, PLL_Theta_b;
		
		static float eleangle_Sinx_c = 
		static float eleangle_Cosx_c	=
		static float PLLangle_Sinx_c =
		static float PLLangle_Cosx_c	=	 
		static float PLL_DeltTheta_Sum_c, PLL_Omega_c, PLL_Theta_c;	
		float sin_error_b = eleangle_Sinx_b * PLLangle_Cosx_b - eleangle_Cosx_b * PLLangle_Sinx_b;
		float sin_error_c = eleangle_Sinx_c * PLLangle_Cosx_c - eleangle_Cosx_c * PLLangle_Sinx_c;	
		 if(fabsf(sin_error_b) < PLL_Filter->deadband){
			 sin_error_b = 0;
		 }
		 if(fabsf(sin_error_c) < PLL_Filter->deadband){
			 sin_error_c = 0;
		 }
		PLL_DeltTheta_Sum_b += sin_error_b;
		PLL_DeltTheta_Sum_c += sin_error_c; 
		Limit_Sat(PLL_DeltTheta_Sum_b ,PLL_Filter -> PLL_DeltTheta_Sum_Maxlimit, PLL_Filter -> PLL_DeltTheta_Sum_Minlimit) ;
		Limit_Sat(PLL_DeltTheta_Sum_c ,PLL_Filter -> PLL_DeltTheta_Sum_Maxlimit, PLL_Filter -> PLL_DeltTheta_Sum_Minlimit) ;
		 
		PLL_Omega_b = (PLL_Filter -> SpeedPllKp * PLL_Filter->PLL_DeltTheta) + \
	                            (PLL_Filter -> SpeedPllKi * PLL_Filter->PLL_DeltTheta_Sum_b);
		 	PLL_Omega_c = (PLL_Filter -> SpeedPllKp * PLL_Filter->PLL_DeltTheta) + \
	                            (PLL_Filter -> SpeedPllKi * PLL_Filter->PLL_DeltTheta_Sum_c);
  Limit_Sat(PLL_Omega_b ,PLL_Filter -> PLL_Omega_Maxlimit, PLL_Filter -> PLL_Omega_Minlimit);
		  Limit_Sat(PLL_Omega_c ,PLL_Filter -> PLL_Omega_Maxlimit, PLL_Filter -> PLL_Omega_Minlimit);
	  PLL_Theta_b += (PLL_Omega_b* PLL_Filter -> OmegaToTheta); 
	  PLL_Theta_c += (PLL_Omega_c* PLL_Filter -> OmegaToTheta); 
		 
	return PLL_Filter->PLL_Theta;
}

/***************************** Dead Zone Compensation *************************************/
#if Use_Dead_Compensation
FOC_DeadCompensation_t FOC_DeadComp = {
	  .tpwm = 50.0f,
    .td = 1.2f,
    .ton = 0.0f,
	  .toff = 0.0f,
		.uCE = 0.0f,
		.uD = 0.0f,
};


//Current_EPLL_t *Ia_PLL_currentFilter;
SP_SOGI_PLL_t Ia_PLL_SOGI;
PLL_t *PhaseA_PLL_currentFilter;

int DeadTime_Compensation_Init(Motor_Params_t *motor)
{
//	Ia_PLL_currentFilter = Current_EPLL_filter_Init(FOC_Frequency, 50.0f,
//																						1.0f * Two_PI * motor->Pole_Pairs / 60  * 0.1f, 
//																						motor->RPM_Max/60 * Two_PI * motor->Pole_Pairs,
//                                            1.5f * motor -> Current_Max);
//  SP_SOGI_PLL_init(&Ia_PLL_SOGI,
//                  1.0f / FOC_Frequency,
//                  50.0f,
//                  1.0f,
//                  50.0f,
//                  1.0f * Two_PI * motor->Pole_Pairs / 60 * 0.1f,
//                  0.0f);    
	
		PhaseA_PLL_currentFilter = Current_PLL_filter_init(FOC_Frequency, 250.0f, 1.0f/4096.0f * 2 *PI * 0.1f * Motor_Params.Pole_Pairs, 
																										 2.0f * (Motor_Params.RPM_Rating/60) * 2*PI * Motor_Params.Pole_Pairs);
	
	return 0;	
}

int DeadTime_Compensation_Deinit(void)
{
	
    //Current_EPLL_filter_deinit(Ia_PLL_currentFilter);
	 

    return 0;
}

float ia_test, ib_test, ic_test;
int get_weight_from_phasecurrent(float ia, float ib, float ic)
{
	// Current_EPLL_filter(Ia_PLL_currentFilter, ia, ib, ic);
  // ia_test = Ia_PLL_currentFilter->filtered_ia;
  // ib_test = Ia_PLL_currentFilter->filtered_ib;
  // ic_test = Ia_PLL_currentFilter->filtered_ic;

//   SP_SOGI_PLL_update(&Ia_PLL_SOGI, ia, &ia_test);
  ia_test = Current_PLL_filter(PhaseA_PLL_currentFilter, ia);
	return 0;
}

// ---------- 电流/扇区检测模块 ----------
/*
 * 输入:
 *   sector: 扇区号，接受 0..5
 * 输出:
 *   signA, signB, signC: 指向 int8_t 的指针，返回值为 +1 或 -1
 *
 * 说明:
 *   +1 表示相电流为正方向，-1 表示为负方向。
 */
void get_phase_polarities_from_sector(int sector,
                                      int8_t *signA,
                                      int8_t *signB,
                                      int8_t *signC)
{
    // 输入指针检查
    if (!signA || !signB || !signC) return;

    // 规范化为 0..5（对负数也工作）
    int s = sector % 6;
    if (s < 0) s += 6; // now s in 0..5

    // 映射表，index 0..5 对应 扇区 I..VI
    static const int8_t map[6][3] = {
        { +1, -1, -1 }, // sector 0 (I)
        { +1, +1, -1 }, // sector 1 (II)
        { -1, +1, -1 }, // sector 2 (III)
        { -1, +1, +1 }, // sector 3 (IV)
        { -1, -1, +1 }, // sector 4 (V)
        { +1, -1, +1 }  // sector 5 (VI)
    };

    *signA = map[s][0];
    *signB = map[s][1];
    *signC = map[s][2];
}

//// 基于 id, iq 以及电角度 theta_elec 得到扇区 0..5
//// 扇区定义： -30..30 -> 0, 30..90 -> 1, 90..150 -> 2, 150..210 -> 3, 210..270 -> 4, 270..330 -> 5
//#define Lpf(a,a_last,lpf)  (a * lpf + a_last * (1 - lpf))
//static inline float get_weight_from_id_iq(float id, float iq, float theta_elec)
//{
//	  // 1) 对iq、id进行低通滤波
//	  static float iq_last = 0.0f, id_last = 0.0f;
//		iq = Lpf(iq, iq_last,0.6f);
//		iq_last = iq;
//		id = Lpf(id, id_last,0.6f);
//		id_last = id;

//    // 2) 计算静止系电流角 theta_s
//	  if(fabsf(id) < 0.1f){
//			id = 0.0f;
//		}
//		Fast_arm_atan2_f32(iq, id, &FOC_DeadComp.theta_fei);
//	 
//		FOC_DeadComp.theta_s = fmodf(theta_elec + FOC_DeadComp.theta_fei, Two_PI); //PI/2.0f
//		if (FOC_DeadComp.theta_s < 0.0f) {
//				FOC_DeadComp.theta_s += Two_PI;
//		}

//    // 3) 映射到扇区，使用中心偏移 30deg
//    const float shift = PI / 6.0f;      // 30deg
//    const float sector_width = PI / 3.0f; // 60deg
//    int sector = (int)floorf((FOC_DeadComp.theta_s + shift) / sector_width) % 6;
//    if(sector < 0) sector += 6;
//		
//		get_phase_polarities_from_sector(sector, &FOC_DeadComp.signIa, &FOC_DeadComp.signIb, &FOC_DeadComp.signIc);
//   
//		
//		
//     /*计算权重*/		
//		const float plateau_deg = 15.0f;          // 中央平台半宽度（degrees），可调 5..12
//		const float plateau = plateau_deg * (PI/180.0f);
//    const float edge_deg = 30.0f;            // 扇区半宽（固定）
//    const float edge = edge_deg * (PI/180.0f);
//		
//		const float sector_w = PI/3.0f;    // 60 deg
//		float center = sector * sector_w;  // rad
//		 // 4) 局部角 phi (相对中心) 映射到 [-edge, edge]
//    float phi = FOC_DeadComp.theta_s - center;
//		phi = fmodf(phi, Two_PI);               // ensure in (-pi, pi]
//    if(phi <= -PI) phi += Two_PI;
//    else if(phi > PI) phi -= Two_PI;
//		
//		if(phi > edge) phi = edge;     // clamp to [-edge, edge] (should already be)
//    if(phi < -edge) phi = -edge;
//		
//		float weight;
//    float absphi = fabsf(phi);
//    if(absphi <= plateau){
//        weight = 1.0f;
//    } else if (absphi >= edge){
//        weight = 0.0f;
//    } else {
//        weight = (edge - absphi) / (edge - plateau); // linear ramp
//    }
//  
//		
//		return weight;
//}

// ---------- 死区补偿模块 ----------

//float u_err, delta_va,delta_vb,delta_vc;
//void deadtime_compensation_phase_polarities_based(
//    float id, float iq, float theta_elec,
//    float Udc, float *v_alpha, float *v_beta)
//{
//	 // 1) 得到电流极性
//	  float weight = get_weight_from_id_iq(id, iq, theta_elec);
//	
//	  // 2) 如果电流很小则不补偿（避免噪声导致切换）
//    float I_magnitude = id*id + iq*iq;
//    const float I_MIN_FOR_COMP = 0.2f * 0.2f; // A，阈值，可调
//    if (I_magnitude < I_MIN_FOR_COMP) {
//        return; // 不加补偿
//    }
//		
//		 // 3) 
//    u_err = Udc * FOC_DeadComp.td/ FOC_DeadComp.tpwm;

//    delta_va =  u_err * (float)FOC_DeadComp.signIa * weight;  // phase A voltage compensation (V)
//    delta_vb =  u_err * (float)FOC_DeadComp.signIb * weight;  // phase B
//    delta_vc =  u_err * (float)FOC_DeadComp.signIc * weight;  // phase C

//    //Clarke: convert Δva,Δvb,Δvc  -> Δv_alpha, Δv_beta
//    // Use conventional Clarke for phase voltages (alpha = va, beta = (vb - vc)*sqrt(3)/3)
//    float v_alpha_comp = delta_va;
//    float v_beta_comp  = (delta_vb - delta_vc) * ONE_BY_SQRT3;
////    float v_alpha_comp = u_err * 2.0f/3.0f * (FOC_DeadComp.signIa - 0.5f * FOC_DeadComp.signIb - 0.5f * FOC_DeadComp.signIc);
////    float v_beta_comp  = u_err * 2.0f/3.0f * (0 + SQRT3_BY_2 * FOC_DeadComp.signIb - SQRT3_BY_2 * FOC_DeadComp.signIc);
//		
//    // 4) 平滑补偿量（防止步进突变）—— 可选但推荐
////    static float v_alpha_comp_last = 0.0f, v_beta_comp_last = 0.0f;
////    const float comp_alpha = 0.3f; // 平滑系数，0..1
////    v_alpha_comp = Lpf(v_alpha_comp, v_alpha_comp_last,0.3f);  
////    v_beta_comp  = Lpf(v_beta_comp, v_beta_comp_last,0.3f);   
////    v_alpha_comp_last = v_alpha_comp;
////    v_beta_comp_last  = v_beta_comp;
//		
//   *v_alpha += v_alpha_comp;
//   *v_beta  += v_beta_comp;
//}
#else

#endif
