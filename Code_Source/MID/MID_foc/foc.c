#include "foc.h"

foc_sin_cos_t foc_sin_cos;
foc_dq_volt_t foc_dq_v;
foc_alpha_beta_volt_t foc_alpha_beta_v;
foc_abc_current_t foc_abc_current_i;
foc_alpha_beta_current_t foc_alpha_beta_i;
foc_qd_current_t foc_dq_i;
foc_pwm_t foc_pwm;
Motor_Params_t Motor_Params;

void Angle_Sin_Cos(float theta, foc_sin_cos_t *foc_sin_cos)
{
    foc_sin_cos->cosx = arm_cos_f32(theta);
    foc_sin_cos->sinx = arm_sin_f32(theta);
}
void Clark(foc_abc_current_t foc_abc_temp,foc_alpha_beta_current_t* foc_alpha_beta_current)
{
    foc_alpha_beta_current->i_alpha = 0.6666667f * foc_abc_temp.ia - 0.3333333f * (foc_abc_temp.ib + foc_abc_temp.ic);
    foc_alpha_beta_current->i_beta  = (foc_abc_temp.ib - foc_abc_temp.ic) * 0.57735027f;
}
void Park(foc_alpha_beta_current_t foc_alpha_beta_current,foc_sin_cos_t foc_sin_cos, foc_qd_current_t* foc_dq_current)
{
    foc_dq_current->id = foc_alpha_beta_current.i_alpha * foc_sin_cos.cosx + foc_alpha_beta_current.i_beta * foc_sin_cos.sinx;
    foc_dq_current->iq = -foc_alpha_beta_current.i_alpha * foc_sin_cos.sinx + foc_alpha_beta_current.i_beta * foc_sin_cos.cosx;
}
void Reverse_Park(foc_sin_cos_t foc_sin_cos, foc_dq_volt_t foc_dq, foc_alpha_beta_volt_t *foc_alpha_beta_volt)
{
    foc_alpha_beta_volt->v_alpha = foc_sin_cos.cosx * foc_dq.vd - foc_sin_cos.sinx * foc_dq.vq;
    foc_alpha_beta_volt->v_beta = foc_sin_cos.sinx * foc_dq.vd + foc_sin_cos.cosx * foc_dq.vq;
}

void Foc_Svpwm(foc_alpha_beta_volt_t foc_alpha_beta_volt, foc_pwm_t *foc_pwm, float Vbus, uint16_t PWM_TIM_FREQ)
{
    float u_alpha = foc_alpha_beta_volt.v_alpha / Vbus;   //Normalization
    float u_beta = foc_alpha_beta_volt.v_beta / Vbus;
	
	  float va = (u_alpha);                                 //Clark reverse 
    float vb = (-0.5f * u_alpha + u_beta * SQRT3_BY_2);
    float vc = (-0.5f * u_alpha - u_beta * SQRT3_BY_2);      
	
    float vmax = (va > vb) ? va : vb;    ///七段式马鞍波调制（零序分量注入）
    vmax = (vmax > vc) ? vmax : vc;
    float vmin = (va < vb) ? va : vb;
    vmin = (vmin < vc) ? vmin : vc;
    float v0 = 0.5f * (vmax + vmin);
    va -= v0;
    vb -= v0;
    vc -= v0;
	
    va *= PWM_TIM_FREQ;       //比较值计算
    vb *= PWM_TIM_FREQ;
    vc *= PWM_TIM_FREQ;
    foc_pwm->pwm_u = va + (PWM_TIM_FREQ * 0.5f);   //  PWM mode1: 小于比较值的时候MOS上开下关
    foc_pwm->pwm_v = vb + (PWM_TIM_FREQ * 0.5f);
    foc_pwm->pwm_w = vc + (PWM_TIM_FREQ * 0.5f);
		
		//  foc_pwm->pwm_u = (PWM_TIM_FREQ * 0.5f) - va;       //  PWM mode2: 小于比较值的时候MOS上开下关
    //  foc_pwm->pwm_v = (PWM_TIM_FREQ * 0.5f) - vb;
    //  foc_pwm->pwm_w = (PWM_TIM_FREQ * 0.5f) - vc;                                                    
}


/**
 * @brief  对 α–β 平面电压矢量做圆形限幅（circular limitation）。
 *
 *         如果输入的 (Voltage_Alpha, Voltage_Beta) 矢量幅值超过基于母线电压
 *         (Vbus) 与最大调制度 (max_modulation) 计算得到的允许最大幅值 v_max，
 *         则按比例缩放 α、β 分量，使得矢量幅值被限制到 v_max 以内。
 *
 * @param  Voltage_Alpha  指向 α 分量的指针（输入/输出）。函数可能原地修改该值。
 * @param  Voltage_Beta   指向 β 分量的指针（输入/输出）。函数可能原地修改该值。
 * @param  Vbus           母线电压（DC bus voltage），用于计算允许的最大幅值。
 * @param  max_modulation 最大调制度（通常取 0..1 范围），用于缩放 Vbus 得到 v_max。
 *
 * @return factor         缩放因子（float）。
 *                        - 当不需要缩放或 v_max <= 0.0f 时，返回 1.0f（未改变）。
 *                        - 若超限，返回 factor = v_max / mag (< 1.0f)，并且
 *                          *Voltage_Alpha 和 *Voltage_Beta 已按该 factor 缩放。
 */
float RevPark_Circle_Limitation_cmsis(float *Voltage_Alpha, float *Voltage_Beta,
                                      float Vbus, float max_modulation)
{
    const float INV_SQRT3 = 0.5773502691896257f;
    float v_max = Vbus * INV_SQRT3 * max_modulation;
    float v_alpha = *Voltage_Alpha;
    float v_beta  = *Voltage_Beta;

    float mag2 = v_alpha * v_alpha + v_beta * v_beta;
    float vmax2 = v_max * v_max;

    if (mag2 <= vmax2 || v_max <= 0.0f) {
        return 1.0f;
    }

    float mag;
    arm_sqrt_f32(mag2, &mag); 
    float factor = v_max / mag;   

    *Voltage_Alpha = v_alpha * factor;
    *Voltage_Beta  = v_beta  * factor;

    return factor;
}

