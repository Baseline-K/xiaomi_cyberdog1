#ifndef _PARA_IDENTIIFY_H_
#define _PARA_IDENTIIFY_H_




int Resistance_Identify_Loop(float vbus, float pwm_period, float lock_angle_rad, float *Phase_R);
int Inductance_Identify_Loop(float vbus, float pwm_period, float CTRL_PERIOD_S, float lock_angle_rad, float phase_R, float *Phase_L);

int Inductance_Identify_HF_Loop(float vbus, float pwm_period, float CTRL_PERIOD_S,
                                 float lock_angle_rad, float phase_R,
                                 float inj_freq, float inj_amp, float *Phase_L, float *Phase_R);

float FFT_GetMagnitude_CMSIS(const float *samples, int FFT_N, float fs, float freq, float *re, float *im);
#endif  /* _PARA_IDENTIIFY_H_ */






