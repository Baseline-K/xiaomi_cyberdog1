#ifndef _DEAD_COMP_h_
#define _DEAD_COMP_h_

#include "foc.h"



typedef struct {
    // sampling / SOGI params
    float Ts;       // sampling period (s)
    float k;        // SOGI gain (typical 1.0 .. 2.0)

    // SOGI states (estimate of in-phase and quadrature)
    float v_alpha;  // in-phase estimate
    float v_beta;   // quadrature estimate

    // PLL states
    float theta;    // phase estimate (rad)
    float omega;    // estimated angular frequency (rad/s)
    float Kp;       // PLL proportional gain (acts on sin_error)
    float Ki;       // PLL integral gain (discrete form, multiply error then add directly)
    float integ;    // integrator state (for PLL)
    float pll_sin;  // sin(theta) cached
    float pll_cos;  // cos(theta) cached

    // amplitude estimate
    float amp;      // magnitude estimate sqrt(v_alpha^2 + v_beta^2)
    // filters / thresholds
    float deadband; // small deadband for sin_error
    float I_min;    // minimum amplitude to enable PLL update
    // limits
    float omega_max;
    float omega_min;
    float integ_max;
    float integ_min;

    uint8_t initialized;
} SP_SOGI_PLL_t;
void SP_SOGI_PLL_init(SP_SOGI_PLL_t *p,
                                    float Ts,
                                    float initial_omega,
                                    float sogi_k,
                                    float desired_pll_bw_Hz,
                                    float deadband,
                                    float I_min);
void SP_SOGI_PLL_update(SP_SOGI_PLL_t *p, float i_in, float *i_filtered_out);
																		
/***************************** Dead Zone Compensation *************************************/
#define Use_Dead_Compensation 1
#if Use_Dead_Compensation
typedef struct {
	  float tpwm;    //PWM周期时间  单位us
	  float td;    //控制死区时间
    float ton;   //开关管开通时间
	  float toff;  //开关管关断时间
		float uCE;   //开关管导通管压降  单位V
		float uD;    //开关管反并联二极管压降
	
	  int8_t signIa;  //电流极性  1：电流为正   -1：电流为负
	  int8_t signIb; 
	  int8_t signIc;
	  float theta_fei;  //arctan(iq/id)
	  float theta_s;  //电流矢量角
}FOC_DeadCompensation_t;	
extern FOC_DeadCompensation_t FOC_DeadComp;
void deadtime_compensation_phase_polarities_based(
    float id, float iq, float theta_elec,
    float Udc, float *v_alpha, float *v_beta);
		
int DeadTime_Compensation_Init(Motor_Params_t *motor);
int DeadTime_Compensation_Deinit(void);
int get_weight_from_phasecurrent(float ia, float ib, float ic);
#else

#endif

#endif
		