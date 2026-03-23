#include "filter.h"

//#define DEBUG
#define DEBUG_OUT(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

//#define USE_FREERTOS_MEM  // 取消注释使用 FreeRTOS 的动态内存，否则使用 C 标准库
#ifdef USE_FREERTOS_MEM
    #include "FreeRTOS.h"  // FreeRTOS 头文件，提供 pvPortMalloc/vPortFree
			static inline void* MyCalloc(size_t num, size_t size) {
			void *ptr =  pvPortMalloc(num * size);
			if (ptr) 
				memset(ptr, 0, num * size);
			return ptr;
    }
    #define MALLOC(size)         pvPortMalloc(size)
		#define CALLOC(num, size)    MyCalloc(num, size)
    #define FREE(ptr)            vPortFree(ptr)
#else
    #define MALLOC(size)         malloc(size)
		#define CALLOC(num, size)    calloc(num, size)
    #define FREE(ptr)            free(ptr)
#endif


/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanfilter(KFP *kfp,float input)
{
	 //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
	 kfp->Now_P = kfp->LastP + kfp->Q;
	 //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
	 kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
	 //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
	 kfp->out = kfp->out + kfp->Kg * (input - kfp->out);//因为这一次的预测值就是上一次的输出值
	 //更新协方差方程: 本次的系统协方差赋给 kfp->LastP 为下一次运算准备。
	 kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
	 return kfp->out;
}

/**
 *一阶Butterworth低通滤波器 
 *@param LPF *lpf 一阶低通滤波器结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的值
 */
float lowpass_filter(LPF *lpf,float input)
{
	lpf->Old = lpf->Fitter; 
	lpf->Fitter =lpf->K1*lpf->Old + lpf->K2*input;
	return lpf->Fitter;
}

//根据截止频率和采样时间计算K1和K2
void init_lowpass_filter(LPF *lpf, float Samp_Time, float Cut_Freq)
{
    lpf->Samp_Time = Samp_Time;
	lpf->Cut_Freq = Cut_Freq;

	float RC = (float)(1.0f/(2.0f*3.141586f*lpf->Cut_Freq));
	lpf->K1 = RC/(RC + lpf->Samp_Time);
	lpf->K2 = 1.0f - lpf->K1;
}


/**
 *滑动平均滤波器
 *@param MAF *maf 滑动平均结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的值
 */

// **初始化滑动平均滤波器**
MAF* mov_average_filter_init(int windowSize, int downsample_n) {
    MAF *maf = (MAF*)MALLOC(sizeof(MAF));
    if (!maf){
#ifdef DEBUG
			DEBUG_OUT("MAF allocation failed\n");
#endif
			return NULL;
		}

    maf->Date = (float*)CALLOC(windowSize, sizeof(float));  // 申请窗口大小的数组
    if (!maf->Date) {
        FREE(maf);
#ifdef DEBUG
			DEBUG_OUT("MAF_Buffer allocation failed\n");
#endif
        return NULL;
    }

    maf->Total_value = 0.0f;
    maf->Average_value = 0.0f;
    maf->WindowSize = windowSize;
    maf->head = 0;  // 环形队列起始索引
    maf->downsample_n = downsample_n;
    maf->downsample_cnt = 0;
    return maf;
}

// **滑动平均滤波（环形队列优化）**
float mov_average_filter(MAF *maf, float input) {

   // 降采样判断：未到周期，不处理
    maf->downsample_cnt++;
    if (maf->downsample_cnt < maf->downsample_n) {
        return maf->Average_value;  // 返回上一次的平均值
    }
    maf->downsample_cnt = 0;  // 重置降采样计数器


    int new_head = (maf->head + 1) % maf->WindowSize;  // 计算下一个新数据要插入的位置

    // 移除最旧数据
    float oldest_data = maf->Date[maf->head];
    maf->Total_value -= oldest_data;

    // 插入新数据代替最旧的数据
    maf->Date[maf->head] = input;
    maf->Total_value += input;

    // 更新平均值
    maf->Average_value = maf->Total_value / maf->WindowSize;

    // 移动 `head`（环形队列指针前进）
    maf->head = new_head;

    return maf->Average_value;
}

// **清空滑动平均滤波器**
void mov_average_filter_clear(MAF *maf) {
    if (!maf || !maf->Date) {
        return; // 安全检查
    }
    
    // 清空数据缓冲区
    for (int i = 0; i < maf->WindowSize; i++) {
        maf->Date[i] = 0.0f;
    }
    
    // 重置统计变量
    maf->Total_value = 0.0f;
    maf->Average_value = 0.0f;
    maf->downsample_cnt = 0;
    maf->head = 0;
}

// **释放滑动平均滤波器**
void mov_average_filter_deinit(MAF *maf) {
    if (maf) {
        FREE(maf->Date);
        FREE(maf);
    }
}





/**
 *中值滤波器，去除脉冲噪声
 *@param MFP *mfp 中值滤波结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的值
 */
MFP* median_filter_init(int window_size) {
    if ((window_size <= 0) ||(window_size >= MAX_WINDOW))
    {
#ifdef DEBUG
		DEBUG_OUT("MFP window_size out of range\n");
#endif
        return NULL;  // 确保窗口大小合法
    }
   
    MFP *mfp = (MFP*)MALLOC(sizeof(MFP));  // 分配 MFP 结构体
    if (!mfp)
    {
#ifdef DEBUG
			DEBUG_OUT("MFP allocation failed\n");
#endif
        return NULL;
    } 

    mfp->Date = (float*)CALLOC(window_size , sizeof(float));  // 动态分配数据数组
    if (!mfp->Date) {
        FREE(mfp);  // 如果分配失败，释放 MFP 结构体
#ifdef DEBUG
		DEBUG_OUT("MFP_Buffer allocation failed\n");
#endif
        return NULL;
    }

    memset(mfp->Date, 0, window_size * sizeof(float));  // 初始化数据数组
    mfp->Median_value = 0;
    mfp->Index = 0;
    mfp->WindowSize = window_size;

    return mfp;
}

void median_filter_deinit(MFP *mfp) {
    if (mfp) {
        free(mfp->Date);  // 释放数据数组
        free(mfp);        // 释放结构体本身
    }
}

float median_filter(MFP *mfp, float new_sample) {  
    if (!mfp || !mfp->Date) return new_sample;  // 避免空指针异常

    mfp->Date[mfp->Index] = new_sample;
    mfp->Index = (mfp->Index + 1) % mfp->WindowSize;  // 更新索引
    
    float temp[MAX_WINDOW];  //频繁申请释放太消耗性能，这里用静态申请
//     float *temp = (float*)MALLOC(mfp->WindowSize * sizeof(float));  // 临时数组
//     if (!temp) 
//     {
// #ifdef  DEBUG
// 		DEBUG_OUT("MFP_Temp_buffer allocation failed\n");
// #endif
//         return new_sample;  // 如果分配失败，返回原值
//     }
    memcpy(temp, mfp->Date, mfp->WindowSize * sizeof(float));
    qsort(temp, mfp->WindowSize, sizeof(float), compare);  // 快速排序
    mfp->Median_value = temp[mfp->WindowSize / 2];  // 取中位值

    //FREE(temp);  // 释放临时数组  
    return mfp->Median_value;
}


/* 比较函数，用于快速排序 */
int compare(const void *a, const void *b) {
    return (*(float *)a > *(float *)b) - (*(float *)a < *(float *)b);
}

/* 用指定的 seed 值填充历史样本，并把中位值设为 seed（可用于避免清零带来的瞬态）*/
void median_filter_fill(MFP *mfp, float seed)
{
    if (mfp == NULL || mfp->Date == NULL || mfp->WindowSize <= 0) return;

    for (int i = 0; i < mfp->WindowSize; ++i) {
        mfp->Date[i] = seed;
    }
    mfp->Index = 0;
    mfp->Median_value = seed;
}

//#include "arm_math.h"
//arm_fir_instance_f32 S;  //FIR实例化结构体
//uint16_t numTaps=81;    //FIR滤波器系数个数
//float pCoeffs[81] = {0};   //FIR滤波器系数：b[numTaps-1],b[numTaps-2],...,b[1],b[0];
//float pState[336]={0.0f};  //FIR滤波器状态变量暂存：numTaps+blockSize-1
//uint32_t blockSize=256;  //包处理大小
//float pSrc[256]={0.0f};   //输入数据
//float PDst[256]={0.0f};   //输出结果

//void DSP_Test(void)
//{
//	uint16_t i= 0;
//	 for(i=0;i<256;i++)
//	{
//		pSrc[i]=50+100*arm_sin_f32(100*PI*i/3200)+50*arm_sin_f32(1300*PI*i/3200);
//	}
//	arm_fir_init_f32(&S,numTaps,pCoeffs,pState,blockSize);
//	arm_fir_f32(&S,pSrc,PDst,blockSize);
//	for(i=0;i<256;i++)
//	{
//		printf("%f,%f\r\n",pSrc[i],PDst[i]);
//	}
//	
//}

//#include "arm_math.h"
//arm_biquad_casd_df1_inst_f32 S;  //直接Ⅰ型IIR实例化结构体
//uint8_t numStages=4;    //IIR滤波器二阶节个数
//float pCoeffs[20] = {   //IIR滤波器二阶节系数，每个二阶节5个系数，对应b0、b1、b2、a1、a2
//1*0.020654369564432010358823177398335246835,  2*0.020654369564432010358823177398335246835,  1*0.020654369564432010358823177398335246835,   1.901162838469254712236988780205138027668 , -0.983780316726982850816796144499676302075,
//1*0.0133069406757561504300868548966718663,  2*0.0133069406757561504300868548966718663,  1*0.0133069406757561504300868548966718663,   1.901657916216396682074218915659002959728 , -0.95488567891942133236682366259628906846, 
//1*0.004328513985533888851209294301725094556,  2*0.004328513985533888851209294301725094556,  1*0.004328513985533888851209294301725094556,   1.917563221142392748674865288194268941879 , -0.93487727708452805774896887669456191361, 
//1*0.018416790493812174034360396035481244326,  1*0.018416790493812174034360396035481244326,  0*0.018416790493812174034360396035481244326,   0.963166419012375762953581670444691553712 , -0               };   
//float pState[16]={0.0f};  //IIR滤波器状态变量暂存，每个二阶节4个状态，对应x[n-1]、x[n-2]、y[n-1]、y[n-2]
//uint32_t blockSize=256;  //包处理大小
//float pSrc[256]={0.0f};   //输入数据
//float PDst[256]={0.0f};   //输出结果

//float Scale_Value = 0.020654369564432010358823177398335246835*\
//0.0133069406757561504300868548966718663*\
//0.004328513985533888851209294301725094556*\
//0.018416790493812174034360396035481244326; //定标值

//void DSP_Test(void)
//{
//	uint16_t i= 0;
//	 for(i=0;i<256;i++)
//	{
//		pSrc[i]=50+300*arm_sin_f32(100*PI*i/3200)+100*arm_sin_f32(700*PI*i/3200);
//	}
//	arm_biquad_cascade_df1_init_f32(&S,numStages,pCoeffs,pState);
//	arm_biquad_cascade_df1_f32(&S,pSrc,PDst,blockSize);
//	for(i=0;i<256;i++)
//	{
//		printf("%f,%f\r\n",pSrc[i],PDst[i]);
//	}
//}


/**
 *PLL滤波器，去除电角度噪声
 *@param PLL *PLL_Filter 锁相环滤波结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的值
 */
// 	#define PWM_FREQ 20000.0f
//	#define SpeedPllBandwidth 250.0f   //rad/s
PLL* Angle_PLL_filter_init(float FOC_FREQ, float PllBandwidth, float deadband, float Max_Angle_Speed)
{
    // 参数合法性检查（可根据需要调整阈值）
    if (FOC_FREQ <= 0.0f || PllBandwidth <= 0.0f) {
#ifdef DEBUG
        DEBUG_OUT("PLL init param invalid: FOC_FREQ=%f, PllBandwidth=%f", FOC_FREQ, PllBandwidth);
#endif
        return NULL;
    }

    // 分配结构体
    PLL *pll = (PLL*)MALLOC(sizeof(PLL));
    if (!pll) {
#ifdef DEBUG
        DEBUG_OUT("PLL allocation failed\n");
#endif
        return NULL;
    }

    // 清零初始化
    memset(pll, 0, sizeof(PLL));

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
float Angle_PLL_filter(PLL *PLL_Filter, float eleAngle_input)
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
		//    PLL_Filter->PLL_DeltTheta = sin_error;  // 直接用 sin_error 作为误差值
		
		// 也可以用 atan2f 把近似相位差sin（a-b）转换成实际相位差a-b，范围 (-pi, pi]
    float cos_error = (PLL_Filter->eleangle_Cosx * PLL_Filter->PLLAngle_Cosx) + \
                                (PLL_Filter->eleangle_Sinx * PLL_Filter->PLLAngle_Sinx);
    float err = atan2f(sin_error, cos_error);
    PLL_Filter->PLL_DeltTheta = err;

     if (fabsf(PLL_Filter->PLL_DeltTheta) < PLL_Filter->deadband) {      // 死区处理，避免噪声在小误差下触发积分/PI动作
         PLL_Filter->PLL_DeltTheta = 0.0f;
     }
		 
//	if(Filter_Flag == 1){
//		 PLL_Filter->PLL_DeltTheta = median_filter(PLL_Filter->fliter_struct,  PLL_Filter->PLL_DeltTheta);
//	}
	
	/* 3. 计算角速度和角度  */
	PLL_Filter->PLL_DeltTheta_Sum += PLL_Filter->PLL_DeltTheta;  //PLL积分项
  Limit_Sat(PLL_Filter->PLL_DeltTheta_Sum ,PLL_Filter -> PLL_DeltTheta_Sum_Maxlimit, PLL_Filter -> PLL_DeltTheta_Sum_Minlimit) ;
		 
	PLL_Filter->PLL_Omega = (PLL_Filter -> SpeedPllKp * PLL_Filter->PLL_DeltTheta) + \
	                            (PLL_Filter -> SpeedPllKi * PLL_Filter->PLL_DeltTheta_Sum);
  Limit_Sat(PLL_Filter->PLL_Omega ,PLL_Filter -> PLL_Omega_Maxlimit, PLL_Filter -> PLL_Omega_Minlimit);
	PLL_Filter->PLL_Theta += (PLL_Filter->PLL_Omega* PLL_Filter -> OmegaToTheta);

  PLL_Filter->PLL_Theta = fmodf(PLL_Filter->PLL_Theta, 2.0f * PI);  	//最终输出限幅	 角度规范化到 [0,2pi)
  if (PLL_Filter->PLL_Theta < 0) PLL_Filter->PLL_Theta += 2.0f * PI;
		 
		 
		
	return PLL_Filter->PLL_Theta;
}

// **清空PLL滤波器**
void Angle_PLL_filter_clear(PLL *PLL_Filter) {
    if (!PLL_Filter) {
        return; // 安全检查
    }
    
    // 重置变量 
    PLL_Filter->initialized_Flag = 0;
    PLL_Filter->PLL_DeltTheta_Sum = 0.0f;
    PLL_Filter->PLL_Theta = 0.0f;
    PLL_Filter->PLL_Omega = 0.0f;
		
//		median_filter_fill(PLL_Filter->fliter_struct, 0.0f);
}

void Angle_PLL_filter_deinit(PLL *pll)
{
    if (!pll) return;
#ifdef DEBUG
    DEBUG_OUT("PLL deinit free %p", (void*)pll);
#endif
		
//	  median_filter_deinit(pll->fliter_struct);
    FREE(pll);

}



