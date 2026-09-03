#ifndef _FOC_H_
#define _FOC_H_

#include "main_user.h"
#include "arm_math.h"

#define SQRT3 1.732050807568877f
#define SQRT3_BY_2 0.8660254f
#define Two_PI 6.28318530718f
#define ONE_BY_SQRT3 0.57735027f


#define PWM_PERIOD			   ((uint16_t)6000) //10K //((uint16_t)1200)  //50K  
#define OFFSET_ADC_TRGO    ((uint16_t)-60)
#define PWM_ADC_TRGO			 ((uint16_t)PWM_PERIOD + OFFSET_ADC_TRGO)
#define PWM_HalfPerMax   ((uint16_t)PWM_PERIOD/2)
#define TIM1CoreClock  120000000
//#define PWM_FREQ  (TIM1CoreClock / PWM_PERIOD / 2)  15KHZ中断频率
#define FOC_Frequency		(TIM1CoreClock / PWM_PERIOD / 2)		
#define timFactor   (1.0f/FOC_Frequency )   //0.0001f 

#define N   10    //
#define SpeedFactor		((float)(FOC_Frequency/(2*PI) / N))
//#define Encoder_Pulse  4096
//#define Speed_Max    40.0f    // motor speed, unit: r/s
//#define mec_deta_speed_Max   2.0f * Speed_Max * 2*PI *timFactor * N   //
//#define mec_deta_speed_Min   1.0f/Encoder_Pulse * 2 *PI * 0.1f  

#define CurrentLoop_Bandwitch    400.0f    //HZ
#define SpeedLoop_Bandwitch      50.0f


typedef struct{
	 
   float  VBUS;	      //电机母线电压
	 float  Current_Rating;	      //电机额定电流
	 float  Current_Max;	      //电机最大电流
     float  Torque_Rating;  //电机额定转矩
     float  Torque_Max;     //电机最大转矩
	 float  Ld;         //d轴电感
	 float  Lq;         //q轴电感
	 float  Phase_R;    //相电阻
	 float  Flux;       //磁链
	 float  Ke;         //反电势常数 V·s/rad_mech（=P·ψf，磁链辨识）
	 float  Kt;         //转矩常数 N·m/A（=1.5·Ke，磁链辨识）
	 float  Pole_Pairs;  //极对数
	 float  Rotor_inertia;  //转动惯量

     float  Reduction_ratio;  //减速比
     float  RPM_Rating;      //额定转速
     float  RPM_Max;         //最大转速
	
}Motor_Params_t;
extern Motor_Params_t Motor_Params;
#define Motor_Params_DEFAULTS  {24,3,0,0,0,0,0}

typedef struct
{
    float ia;
    float ib;
    float ic;
	  uint16_t  adc_phase_ia, adc_phase_ib, adc_phase_ic; // Raw ADC Values （A/B/C相电流）
	  int32_t   offset_phase_ia;   // Phase a Current
	  int32_t   offset_phase_ib;   // Phase b Current
	  int32_t   offset_phase_ic;   // Phase c Current
} foc_abc_current_t;

typedef struct
{
    float i_alpha;
    float i_beta;
} foc_alpha_beta_current_t;

typedef struct
{
    float iq;
    float id;
		float iq_last;
	  float id_last;
} foc_qd_current_t;

typedef struct
{
    float v_alpha;
    float v_beta;
} foc_alpha_beta_volt_t;

typedef struct
{
    float pwm_u;
    float pwm_v;
    float pwm_w;
} foc_pwm_t;

typedef struct
{
    float sinx;
    float cosx;
} foc_sin_cos_t;
typedef struct
{
    float vd;
    float vq;
} foc_dq_volt_t;



extern foc_sin_cos_t foc_sin_cos;
extern foc_dq_volt_t foc_dq_v;
extern foc_alpha_beta_volt_t foc_alpha_beta_v;
extern foc_abc_current_t foc_abc_current_i;
extern foc_alpha_beta_current_t foc_alpha_beta_i;
extern foc_qd_current_t foc_dq_i;
extern foc_pwm_t foc_pwm;

void Angle_Sin_Cos(float theta,foc_sin_cos_t* foc_sin_cos);

void Reverse_Park(foc_sin_cos_t foc_sin_cos,foc_dq_volt_t foc_dq,foc_alpha_beta_volt_t* foc_alpha_beta_volt);

void Foc_Svpwm(foc_alpha_beta_volt_t foc_alpha_beta_volt, foc_pwm_t *foc_pwm, float Vbus, uint16_t PWM_TIM_FREQ);

void Clark(foc_abc_current_t foc_abc_temp,foc_alpha_beta_current_t* foc_alpha_beta_current);

void Park(foc_alpha_beta_current_t foc_alpha_beta_current, foc_sin_cos_t foc_sin_cos, foc_qd_current_t *foc_dq_current);


float RevPark_Circle_Limitation_cmsis(float *Voltage_Alpha, float *Voltage_Beta,
                                      float Vbus, float max_modulation);



#endif

