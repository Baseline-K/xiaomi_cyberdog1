#include "Para_identify.h"
#include "foc.h"


/******************************************** 死区补偿 ********************************************************/
void compensate(float vbus, float pwm_period, float* u_d, float* u_q)
{
	float U_PhaseA,U_PhaseB,U_PhaseC, U_alpha, U_beta, U_d, U_q;
	U_PhaseA =	vbus *  TIM1->CCR1/ pwm_period; 
	U_PhaseB =	vbus * 	TIM1->CCR2/ pwm_period; 
	U_PhaseC =	vbus * 	TIM1->CCR3/ pwm_period; 
	
	//Clark
	U_alpha = 0.6666667f * U_PhaseA - 0.3333333f * (U_PhaseB + U_PhaseC);
  U_beta  = (U_PhaseB - U_PhaseC) * 0.57735027f;
	
	//Park
	U_d = U_alpha * foc_sin_cos.cosx + U_beta * foc_sin_cos.sinx;
  U_q = -U_alpha * foc_sin_cos.sinx + U_beta * foc_sin_cos.cosx;
	
	* u_d = U_d;
	* u_q = U_q;
}

/******************************************** FFT ********************************************************/
// ---------- 配置与限制 ----------
#ifndef MAX_FFT_SIZE
#define MAX_FFT_SIZE 2048   // 根据板子内存酌情修改；HF_SAMPLE_N=512 应当没问题
#endif

// ---------- 内部静态缓存（避免每次都 malloc） ----------
static float *g_win = NULL;        // 窗函数数组，长度 N
static float *g_fft_in = NULL;     // 临时缓冲（实数输入，长度 N）
static float *g_fft_out = NULL;    // rfft 输出缓冲，长度 N (packed complex)
static arm_rfft_fast_instance_f32 g_rfft; // rfft 实例
static int g_rfft_size = 0;        // 当前已初始化的 FFT 长度

// 生成 Hann 窗并返回窗口和窗口和（sum_w）
// 若你希望使用矩形窗，把函数改成全部 1 即可
static float make_hann_window(int FFT_N, float **out_win)
{
    if (FFT_N <= 0) return 0.0f;
    if (g_win == NULL || g_rfft_size != FFT_N) {
        // 重新（re）分配缓存
        if (g_win) { free(g_win); g_win = NULL; }
        g_win = (float*)malloc(sizeof(float) * FFT_N);
        if (g_win == NULL) return 0.0f;
    }
    float sum = 0.0f;
    for (int n = 0; n < FFT_N; ++n) {
        // Hann: w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
        float w = 0.5f * (1.0f - cosf((2.0f * (float)PI * n) / (float)(FFT_N - 1)));
        g_win[n] = w;
        sum += w;
    }
    *out_win = g_win;
    return sum;
}

// 初始化临时缓冲与 RFFT（如果尚未初始化或 N 改变）
// 返回 0 成功，负为失败
static int ensure_rfft_buffers(int FFT_N)
{
    if (FFT_N <= 0 || FFT_N > MAX_FFT_SIZE) return -1;

    // 如果已初始化但尺寸不同，则释放旧缓冲
    if (g_rfft_size != FFT_N) {
        if (g_fft_in) { free(g_fft_in); g_fft_in = NULL; }
        if (g_fft_out) { free(g_fft_out); g_fft_out = NULL; }
        // alloc
        g_fft_in = (float*)malloc(sizeof(float) * FFT_N);
        g_fft_out = (float*)malloc(sizeof(float) * FFT_N); // CMSIS rfft fast packs N floats
        if (g_fft_in == NULL || g_fft_out == NULL) {
            if (g_fft_in) { free(g_fft_in); g_fft_in = NULL; }
            if (g_fft_out) { free(g_fft_out); g_fft_out = NULL; }
            return -2;
        }
        // init rfft instance
        if (arm_rfft_fast_init_f32(&g_rfft, FFT_N) != ARM_MATH_SUCCESS) {
            return -3;
        }
        g_rfft_size = FFT_N;
    }
    return 0;
}

/*
 * FFT_GetPhasor_CMSIS:
 *  - samples: 输入采样数组，长度 N（采样值最好已去直流）
 *  - FFT_N: FFT 长度（2^k） 
 *  - fs: 采样率（Hz）
 *  - freq: 想要提取的频率（Hz）
 *  - realOut/imagOut: 输出复数相量，表示峰值幅值 phasor (peak)
 *
 * 返回: 0 成功，负值为错误
 *
 * 说明：
 *  - 本实现使用 Hann 窗（可修改为矩形窗）。幅值恢复采用 A_peak = 2 * |X(k)| / sum_w，
 *    同时直接返回复数分量 real/imag 分别乘以 (2.0f / sum_w)。
 *  - 要求 freq 对应的 bin = round(freq*N/fs) 在 [1, N/2-1]（或处理 DC / Nyquist 特例）。
*  -  最好注入频率freq*采样点N/采样频率fs 为整数（称为相干采样/coherent），频谱没有泄露，幅值估计最准确。
      否则会发生谱泄露，需要靠窗函数、零填充、插值或 Goertzel 来补偿。
 */

int FFT_GetPhasor_CMSIS(const float *samples, int FFT_N, float fs, float freq, float *realOut, float *imagOut)
{
    if (!samples || !realOut || !imagOut) return -1;
    if (FFT_N <= 1) return -2;
    if (freq < 0.0f || fs <= 0.0f) return -3;
    if (freq >= fs * 0.5f) return -4; // 超过奈奎斯特

    // 初始化 rfft/caches
    int rc = ensure_rfft_buffers(FFT_N);
    if (rc != 0) return -5;

    //1) apply window into g_fft_in 生成窗并计算 sum_w
    float *win = NULL;
//    float sum_w = make_hann_window(FFT_N, &win);  //汉宁窗
//	  if (sum_w <= 0.0f) return -6;
//    for (int i = 0; i < FFT_N; ++i) {
//        g_fft_in[i] = samples[i] * win[i];
//    }
		
		// 矩形窗
		float sum_w = (float)FFT_N;
		for (int i = 0; i < FFT_N; ++i) {
      g_fft_in[i] = samples[i]; // 矩形窗相当于乘 1
    }

    // 2) compute real FFT (rfft fast)
    //    output layout (CMSIS): pOut[0]=Re(0), pOut[1]=Re(N/2), pOut[2]=Re(1), pOut[3]=Im(1), ...
    arm_rfft_fast_f32(&g_rfft, g_fft_in, g_fft_out, 0);

    // 3) find bin
    float kf = freq * (float)FFT_N / fs;
    int k = (int)(kf + 0.5f);
    if (k < 0) k = 0;
    if (k > FFT_N/2) k = FFT_N/2;

    float re = 0.0f, im = 0.0f;
    if (k == 0) {
        // DC bin: real at g_fft_out[0], imag = 0
        re = g_fft_out[0];
        im = 0.0f;
        // For DC, no factor 2: but our amplitude formula for peak (DC) is A = re / sum_w
        // But we keep consistent scale (we will scale below).
    } else if (k == FFT_N/2) {
        // Nyquist bin (real only)
        re = g_fft_out[1];
        im = 0.0f;
    } else {
        // general bin: Re(k) at out[2*k], Im(k) at out[2*k+1]
        re = g_fft_out[2 * k];
        im = g_fft_out[2 * k + 1];
    }

    // 4) scale to get peak phasor: phasor = (2.0 / sum_w) * (re + j*im)
    //    (Note: for k==0 or k==FFT_N/2 the factor 2 may not be appropriate, handle DC specially)
    float scale = 2.0f / sum_w;
    if (k == 0 || k == FFT_N/2) {
        // DC and Nyquist are single-sided components; for DC correct scale is 1/sum_w,
        // but user usually doesn't request DC here. We'll apply 1/sum_w to be precise.
        scale = 1.0f / sum_w;
    }
    *realOut = re * scale;
    *imagOut = im * scale;

    return 0;
}

// 简单包装：返回 peak magnitude（A_peak）
float FFT_GetMagnitude_CMSIS(const float *samples, int FFT_N, float fs, float freq, float *re, float *im)
{
  
    int rc = FFT_GetPhasor_CMSIS(samples, FFT_N, fs, freq, re, im);
    if (rc != 0){
			return 0.0f;
		}
    return sqrtf(*re * *re + *im * *im);
}



/***************************************************** 相电阻辨识 **************************************************************/

//方法一：SVPWM输出
/*
 * 函数名称: Resistance_Identify_Loop
 * 函数功能: 参数辨识初始化,将电机转子锁定在零度位置
 * 输入参数: 无
 * 输出参数: 无
**/
enum {INIT, HOLD0, RAMP_UP, HOLD1, REDUCE, HOLD2, CALC, DONE};
int R_Identify_state = INIT;

int Resistance_Identify_Loop(float vbus, float pwm_period, float lock_angle_rad, float *Phase_R)
{
	#define Lpf(a,a_last,lpf)  (a * lpf + a_last * (1 - lpf))
	#define INIT_vd 2.0f
	#define INIT_Count 4000
	#define Ud_step 0.001f
	#define I_target 1.0f  // 目标电流值
	#define hold_count 4000
 
  #define Ud   foc_dq_v.vd

	static int count;
	static float UN,IN,Irip,Uhalf;
	static float Is_filtered, Un_filtered;
	static float Ud_Sample, Uq_Sample;
	// 	1、设置电压矢量角度为0(或其他恒定值)。令Vd = n，Vq = 0，通过svpwm输出, 然后延时一段时间等电流归0。
	// 2、将采样到的相电流进行clark变换，计算
	// 3、逐渐增加幅值， 等待达到额定电流。
	// 4、保持步骤3的幅值，采集此时的电流和电压，并分别低通滤波，等待一定时间后，记录滤波后的电流和电压值 。
	// 5、减小幅值，等待达到额定电压的1/2；
	// 6、保持步骤5占空比，采集此时的电流和电压，并分别低通滤波，等待一定时间后，记录滤波后的电流和电压值0.5；
	// 7、计算相电阻 
	Angle_Sin_Cos(lock_angle_rad, &foc_sin_cos);  
	Clark(foc_abc_current_i, &foc_alpha_beta_i);

    float Is = sqrt(foc_alpha_beta_i.i_alpha*foc_alpha_beta_i.i_alpha + foc_alpha_beta_i.i_beta*foc_alpha_beta_i.i_beta);
    //float Ud = foc_dq_v.vd;    //考虑死区补偿等因素的估计Ud
    foc_dq_v.vq = 0.0f;
		
    compensate(vbus, pwm_period, &Ud_Sample, &Uq_Sample);
    // 低通滤波
    Is_filtered = Lpf(Is, Is_filtered,0.05f);
    Un_filtered = Lpf(Ud_Sample, Un_filtered, 0.05f);


    // 状态机
    switch(R_Identify_state) {
      case INIT:{
				foc_dq_v.vd = INIT_vd;

				count++;
				if (count >= INIT_Count) { 
					count = 0;
						foc_dq_v.vd = 0.0f;
					R_Identify_state = HOLD0;
				}
						break;
				}
			 case HOLD0:{                        //延时一段时间等电流归0
				count++;
				if (count >= hold_count) { 
					  count = 0;
					  R_Identify_state = RAMP_UP;
				}
						break;
				}                         
      case RAMP_UP:{
				Ud += Ud_step;
						if (Is_filtered >= I_target) 
				{  
					R_Identify_state = HOLD1; 
				}
						break;
				}
      case HOLD1:
        count++;
        if (count >= hold_count) 
				{ 
					count = 0;
					IN = Is_filtered; 
					UN = Un_filtered; 
					R_Identify_state = REDUCE; 
				}
        break;
      case REDUCE:{
						Ud = 0.5f * UN; // 直接用记录到的 UN/2
						R_Identify_state = HOLD2; 
						break;
			}   
			case HOLD2:{
				count++;
				if (count >= hold_count) 
				{ 
					count = 0;
					Irip = Is_filtered; 
					Uhalf = Un_filtered; 
					R_Identify_state = CALC; 
				}
        break;
			}
      case CALC:{
        *Phase_R = (UN - Uhalf) / (IN - Irip); // 一般 UN - 0.5UN simplifies to 0.5UN...
        R_Identify_state = DONE;
        break;
			}
      case DONE:{
        // 输出结果，回到最初的状态
				TIM1->CCR1 = pwm_period / 2;
				TIM1->CCR2 = pwm_period / 2;
				TIM1->CCR3 = pwm_period / 2;
				R_Identify_state = INIT;
				return 1;
      }
  }
	Reverse_Park(foc_sin_cos, foc_dq_v, &foc_alpha_beta_v);
	Foc_Svpwm(foc_alpha_beta_v, &foc_pwm, vbus, pwm_period);
	TIM1->CCR1 = foc_pwm.pwm_u;
	TIM1->CCR2 = foc_pwm.pwm_v;
	TIM1->CCR3 = foc_pwm.pwm_w;

	return 0;
}



/*
测试记录：
电机1：
辨识： 
电桥测量： 
*/


/***************************************************** 相电感辨识 **************************************************************/


// 高频正弦电压法
// 高频注入法：用 FFT 提取幅值计算相电感
// 注意：需保证采样频率 fs = 1/CTRL_PERIOD_S > 2 * inj_freq（奈奎斯特）
// 并确保 inj_freq 不与 PWM 载波及其显著谐波冲突。
// FFT 接口（由用户实现）：在 samples[] 中查找 freq 对应分量的幅值（峰值 amplitude）


// 状态机
#define HF_SAMPLE_N  512   // 采样点数（最好为 2 的幂）；根据内存/分辨率权衡调整
enum {HF_INIT, HF_HOLD0, HF_HOLD1, HF_SAMPLE, HF_CALC, HF_DONE};
static int L_Identify_state = HF_INIT;

float Ud_Sample = 0.0f, Uq_Sample = 0.0f;
float Imag, Vmag;
float FFT_Ire=0.0f, FFT_Iim=0.0f;
float FFT_Ure=0.0f, FFT_Uim=0.0f;

int Inductance_Identify_HF_Loop(float vbus, float pwm_period, float CTRL_PERIOD_S,
                                 float lock_angle_rad, float phase_R,
                                 float inj_freq, float inj_amp, float *Phase_L, float *Phase_R)
{
	#define INIT_vd           2.0f    // 初始小电压，唤醒电路（单位 V），可按需改
	#define INIT_COUNT        (4000 * 4)   // INIT 保持周期数（以控制周期计数）
	#define HOLD_COUNT        (4000 * 4)     // 各 HOLD 等待周期数（或改成稳态判定）
	
	
    // 基本检查
    if (Phase_L == NULL) return -10;
    if (inj_freq <= 0.0f) return -11;
    float fs = 1.0f / CTRL_PERIOD_S;
    if (inj_freq >= fs * 0.5f) return -12; // Nyquist violation

    // 内部静态/持久变量

    static float vd_samples[HF_SAMPLE_N];
	  static float id_samples[HF_SAMPLE_N];
    static int sample_idx = 0;
    static float phase = 0.0f;
    static float omega = 0.0f;
    static int count = 0;

    // 每次循环更新角度基元
    omega = 2.0f * PI * inj_freq;

    // 检查全局结构与变换（与原代码风格一致）
    Angle_Sin_Cos(lock_angle_rad, &foc_sin_cos);
    Clark(foc_abc_current_i, &foc_alpha_beta_i);
    Park(foc_alpha_beta_i, foc_sin_cos, &foc_dq_i); // 得到 signed id, iq

    // 获得实际测得的电压分量（d/q），采用你已有的 compensate 接口
    compensate(vbus, pwm_period, &Ud_Sample, &Uq_Sample); // 更新 Ud_Sample, Uq_Sample
    float Vd_inst = Ud_Sample; // 即刻测得的 d 轴电压（瞬时）

    // 状态机
    switch ( L_Identify_state) {
			 case HF_INIT:
            // 先施加一个小初始电压以唤醒系统并等待稳态
			      sample_idx = 0;
            phase = 0.0f;
            foc_dq_v.vd = INIT_vd;
			      foc_dq_v.vq = 0.0f;
            count++;
            if (count >= INIT_COUNT) {
                count = 0;
                // 将电压回 0，进入 HOLD0 等电流静止
                foc_dq_v.vd = 0.0f;
                L_Identify_state = HF_HOLD0;
            }
            break;

        case HF_HOLD0:
            // 等待电流归到零（或近零）
            count++;
            if (count >= HOLD_COUNT) {
                count = 0;
                L_Identify_state = HF_HOLD1;
            }
            break;

        case HF_HOLD1:
            // 产生注入信号（d 轴正弦），等待系统稳定
            count++;
            foc_dq_v.vd = inj_amp * sinf(phase);
            foc_dq_v.vq = 0.0f;
				    phase += omega * CTRL_PERIOD_S;
            if (phase > 2.0f * PI) phase -= 2.0f * PI;
				
            if (count >= HOLD_COUNT) {
                count = 0;
                L_Identify_state = HF_SAMPLE;
            }

            break;

        case HF_SAMPLE:
					 // 采样当前周期的 d 轴电流（signed id）和实际测电压（Vd_inst）
				    id_samples[sample_idx] = foc_dq_i.id;  // Park 已得到 id
            vd_samples[sample_idx] = Vd_inst;      // 来自 compensate 的实际 d 轴电压（瞬时）
				    sample_idx++;
				
				    // 继续产生注入信号
						foc_dq_v.vd = inj_amp * sinf(phase);
            foc_dq_v.vq = 0.0f;
            phase += omega * CTRL_PERIOD_S;
            if (phase > 2.0f * PI) phase -= 2.0f * PI;
				
            // 在下一周期继续注入，切换回 HF_INJECT
            if (sample_idx >= HF_SAMPLE_N) {
                 // 收集完成，停注入并进入计算
                sample_idx = 0;
                L_Identify_state = HF_CALC;
                // 恢复默认/中性电压（避免继续注入）
                foc_dq_v.vd = 0.0f;
                foc_dq_v.vq = 0.0f;

            }
            break;

        case HF_CALC:
        {
            // 1) 用 FFT 接口计算基波幅值（peak） (V_peak, I_peak)

            Vmag = FFT_GetMagnitude_CMSIS(vd_samples, HF_SAMPLE_N, fs, inj_freq, &FFT_Ure, &FFT_Uim);  //inj_amp ; //
            Imag = FFT_GetMagnitude_CMSIS(id_samples, HF_SAMPLE_N, fs, inj_freq, &FFT_Ire, &FFT_Iim);

            if ((Imag <= 1e-12f) ||(Vmag <= 1e-12f)) {
                // 电流太小或 FFT 失败
                *Phase_L = -1.0f; // 指示错误，可自定义
                L_Identify_state = HF_DONE;
                break;
            }

            // 2) 计算电感 L = sqrt((V/I)^2 - R^2) / omega
            float Zmag = Vmag / Imag;
            float tmp = Zmag * Zmag - phase_R * phase_R;
            if (tmp <= 0.0f) {
                // 可能 R 太大 / 信噪比太差 /测量误差，不能开根
                *Phase_L = -2.0f; // 错误码
                L_Identify_state = HF_DONE;
                break;
            }
            float L_axis = sqrtf(tmp) / omega;
            // 3) 如果你需要把轴电感换算成相电感（依据你之前用的换算 2/3）
            *Phase_L = (2.0f / 3.0f) * L_axis; // 如果你之前用的是这个因子；否则直接返回 L_axis

							
//						float Zre = (FFT_Ure * FFT_Ire + FFT_Uim * FFT_Iim) / Imag;
//						float Zim = (FFT_Uim * FFT_Ire - FFT_Ure * FFT_Iim) / Imag; // note sign
//						// --- 得到 R 与 L (轴向) ---
//						float R_meas = Zre;           // 实部就是等效串联电阻（在该频率）
//						float L_axis = Zim / omega;   // omega = 2*pi*inj_freq
//						*Phase_L = (2.0f / 3.0f) * L_axis;
//						*Phase_R = R_meas;
						
						
            L_Identify_state = HF_DONE;
            break;
        }

        case HF_DONE:
            // 恢复输出中性，占空比回中值
            TIM1->CCR1 = pwm_period / 2;
            TIM1->CCR2 = pwm_period / 2;
            TIM1->CCR3 = pwm_period / 2;
            // 重置状态以便下次运行（若需要）
            L_Identify_state = HF_INIT;
            return 1; // 完成
    }

    // 若当前不是刚返回并写过 SVPWM，则把 dq->ab->svpwm 写回硬件（保证每周期都有输出）
   
		Reverse_Park(foc_sin_cos, foc_dq_v, &foc_alpha_beta_v);
		Foc_Svpwm(foc_alpha_beta_v, &foc_pwm, vbus, pwm_period);
		TIM1->CCR1 = foc_pwm.pwm_u;
		TIM1->CCR2 = foc_pwm.pwm_v;
		TIM1->CCR3 = foc_pwm.pwm_w;

    return 0; // 进行中
}






/***************************************************** 转动惯量辨识 **************************************************************/






/***************************************************** 磁链辨识 **************************************************************/



