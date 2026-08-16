#include "speed_pid.h"
#include "AS5600.h"

SPEED_PID_DEF speed_pid;         
SCurveProfile ScurveProfile;        
			
static float Limit_Sat( float fdata,float fmax, float fmin) 
{
    fdata = (fdata > fmax) ? fmax : fdata;
    fdata = (fdata < fmin) ? fmin : fdata;

    return fdata;
}
			

void  Speed_pid_c(SPEED_PID_DEF* speed_pid_temp)
{
		//Normal PI Controller
    speed_pid_temp->error = speed_pid_temp->ref - speed_pid_temp->fbk;
   speed_pid_temp->up  = speed_pid_temp->Kp * speed_pid_temp->error;
    speed_pid_temp->I_Sum= Limit_Sat(
															speed_pid_temp->I_Sum  +  speed_pid_temp->Ki  *speed_pid_temp->error, 
                              speed_pid_temp->Max_Output, 
                              speed_pid_temp->Min_Output);
   speed_pid_temp ->output = Limit_Sat(
                              speed_pid_temp->up + speed_pid_temp->I_Sum, 
                              speed_pid_temp->Max_Output, 
                              speed_pid_temp->Min_Output);
}

void speed_pid_init(Motor_Params_t Motor_Params, unsigned int update_counter)
{
	
	//Normal PI Controller
	speed_pid.Kp = SpeedLoop_Bandwitch  * 6.28f * Motor_Params.Rotor_inertia / (1.5f * Motor_Params.Pole_Pairs * Motor_Params.Flux) *0.8f;  //  0.08f
	speed_pid.Ki = SpeedLoop_Bandwitch  * 6.28f * speed_pid.Kp * timFactor * update_counter *0.5f;  //  0.08f
	 
	speed_pid.Max_Output =  2.0f;   
	speed_pid.Min_Output = -0.5f;
	speed_pid.up = 0.0f;
	speed_pid.I_Sum = 0.0f;

	SCurve_Init(&ScurveProfile, 200.0f, 200.0f);
}


void speedset(float speed_ref, float *speed_set,float speedupAc,float speeddownAc)
{
	if (speed_ref > *speed_set)
	{
		*speed_set += speedupAc;
		if (*speed_set > speed_ref)
			*speed_set = speed_ref;
	}
	else if(speed_ref < *speed_set)
	{
		*speed_set -= speeddownAc;
		if (*speed_set < speed_ref)
			*speed_set = speed_ref;
	}
	else
		*speed_set = speed_ref;
}

float speed_ref_true,SCurve_speed_ref_true;
void speed_Loop_Handle(float speed_ref ,SPEED_PID_DEF* speed_pid_temp, float speedupAc, float speeddownAc)
{
	speed_pid_temp->update_count++;
	if (speed_pid_temp->update_count >= N)
	{
		speed_pid_temp->update_count = 0;
		
    speedset(speed_ref, &speed_ref_true, speedupAc, speeddownAc);
    SCurve_speed_ref_true = SCurve_Update(&ScurveProfile, speed_ref, timFactor * N);
		
		bsp_as5600GetSpeed(SpeedFactor);

		speed_pid_temp->ref = SCurve_speed_ref_true;
		speed_pid_temp->fbk = Encoder_PLL_eleFilter->PLL_Omega_filtered/(2.0f * PI * Motor_Params.Pole_Pairs);//Encoder_AS5600.Speed_RPM;
		Speed_pid_c(&speed_pid);
	}
}



/**
 * @brief 初始化S型曲线结构体
 * @param profile: S型曲线结构体指针
 * @param accel_time: 加速时间 (ms)
 * @param decel_time: 减速时间 (ms)
 */
void SCurve_Init(SCurveProfile *profile, float accel_time, float decel_time) {
    profile->target_vel = 0.0f;
    profile->current_vel = 0.0f;
    profile->accel_time = accel_time/1000.0f;  // 转换为秒
    profile->decel_time = decel_time/1000.0f;  // 转换为秒

	profile->Plan_Threshold = 0.01f;
    // 默认限制值（需要根据实际电机调整）
    profile->max_vel_fwd = Motor_Params.RPM_Rating / 60;      // 正转 RPS
    profile->max_accel_fwd = Motor_Params.Torque_Rating / Motor_Params.Rotor_inertia /(2*PI);    // 加速度  RPS/s
    profile->max_jerk_fwd = 10.0f *  profile->max_accel_fwd;    // 加加速度  RPS/s²
    
    profile->max_vel_rev = profile->max_vel_fwd;      // 反转RPS
    profile->max_accel_rev = profile->max_accel_fwd;    // RPS/s
    profile->max_jerk_rev = profile->max_jerk_fwd;    // RPS/s²
    
    profile->T1 = 0.0f;
    profile->T2 = 0.0f;
    profile->T3 = 0.0f;
    
    profile->vel_start = 0.0f;
    profile->vel_end = 0.0f;
    profile->time = 0.0f;
    profile->max_accel = 0.0f;
    profile->jerk = 0.0f;
    
    profile->limit_flags = LIMIT_NONE;
    profile->is_moving = 0;
    profile->direction = 0;
}

/**
 * @brief 规划S型曲线参数（带限制检查）
 * @param profile: S型曲线结构体指针
 * @param new_target: 新的目标速度
 */
void SCurve_Plan(SCurveProfile *profile, float new_target) {
    // 清除之前的限制标志
    profile->limit_flags = LIMIT_NONE;
    
    // 确定运动方向并选择对应的限制值
    float max_vel, max_accel, max_jerk;
    if (new_target >= 0) {
        max_vel = profile->max_vel_fwd;
        max_accel = profile->max_accel_fwd;
        max_jerk = profile->max_jerk_fwd;
    } else {
        max_vel = profile->max_vel_rev;
        max_accel = profile->max_accel_rev;
        max_jerk = profile->max_jerk_rev;
    }
    
    // 速度限制检查
    if (fabsf(new_target) > max_vel) {
        profile->limit_flags |= LIMIT_VEL;
        new_target = (new_target > 0) ? max_vel : -max_vel;
    }
    
    // 如果目标速度没变化，不需要重新规划
    if (fabsf(new_target - profile->target_vel) < profile->Plan_Threshold && 
        fabsf(profile->current_vel - profile->target_vel) < profile->Plan_Threshold) {
        return;
    }
    
    profile->target_vel = new_target;
    profile->vel_start = profile->current_vel;
    profile->vel_end = new_target;
    
    float delta_v = new_target - profile->current_vel;
    
    // 判断方向
    if (fabsf(delta_v) < profile->Plan_Threshold) {
        profile->is_moving = 0;
        profile->direction = 0;
        return;
    }
    
    profile->direction = (delta_v > 0) ? 1 : -1;
    delta_v = fabsf(delta_v);
    
    // 选择加速或减速时间
    float total_time = (profile->direction > 0) ? profile->accel_time : profile->decel_time;
    
    // 初始三段式时间分配：T1 = T3 = 25%, T2 = 50%
    profile->T1 = total_time * 0.25f;
    profile->T2 = total_time * 0.5f;
    profile->T3 = total_time * 0.25f;
    
    // 根据时间和速度变化计算加加速度和最大加速度
    // delta_v = jerk*T1*(T1 + T2)
    float calculated_jerk = delta_v / (profile->T1 * (profile->T1 + profile->T2));
    float calculated_accel = calculated_jerk * profile->T1;
    
    // 加加速度限制检查
    if (calculated_jerk > max_jerk) {
        profile->limit_flags |= LIMIT_JERK;
        calculated_jerk = max_jerk;
        calculated_accel = calculated_jerk * profile->T1;
        
        // 重新计算时间以满足速度要求
        // delta_v = jerk*T1*(T1 + T2)
        // T2 = delta_v/(jerk*T1) - T1
        profile->T2 = delta_v / (calculated_jerk * profile->T1) - profile->T1;
        
        // 确保T2不为负
        if (profile->T2 < 0) {
            profile->T2 = 0;
            // 如果T2为0，需要调整T1和T3
            profile->T1 = sqrtf(delta_v / calculated_jerk);
            profile->T3 = profile->T1;
            calculated_accel = calculated_jerk * profile->T1;
        }
    }
    
    // 加速度限制检查
    if (calculated_accel > max_accel) {
        profile->limit_flags |= LIMIT_ACCEL;
        calculated_accel = max_accel;
        calculated_jerk = calculated_accel / profile->T1;
        
        // 重新计算T2
        profile->T2 = delta_v / (calculated_jerk * profile->T1) - profile->T1;
        
        if (profile->T2 < 0) {
            profile->T2 = 0;
            // 调整T1以满足加速度限制
            profile->T1 = sqrtf(delta_v / calculated_jerk);
            profile->T3 = profile->T1;
        }
    }
    
    profile->jerk = calculated_jerk;
    profile->max_accel = calculated_accel;
    
    // 重置时间计数
    profile->time = 0.0f;
    profile->is_moving = 1;
}

/**
 * @brief 更新S型曲线，计算当前目标速度
 * @param profile: S型曲线结构体指针
 * @param target_vel: 最终目标速度
 * @param dt: 时间步长 (秒)
 * @return 当前目标速度
 */
float SCurve_Update(SCurveProfile *profile, float target_vel, float dt) {
    // 如果目标改变，重新规划
    if (fabsf(target_vel - profile->target_vel) > profile->Plan_Threshold) {
        SCurve_Plan(profile, target_vel);
    }
    
    // 如果没有运动，直接返回当前速度
    if (!profile->is_moving) {
        return profile->current_vel;
    }
    
    profile->time += dt;
    float t = profile->time;
    float v0 = profile->vel_start;
    float dir = profile->direction;
    float j = profile->jerk;
    float a_max = profile->max_accel;
    
    float vel = v0;
    float t_sum = 0.0f;
    
    // 阶段1: 加加速 (jerk作用) t_sum为局部变量
    t_sum += profile->T1;
    if (t <= t_sum) {
        vel = v0 + dir * 0.5f * j * t * t;
        profile->current_vel = vel;
        return vel;
    }
    float v1 = v0 + dir * 0.5f * j * profile->T1 * profile->T1;
    
    // 阶段2: 匀加速 (加速度恒定)
    t_sum += profile->T2;
    if (t <= t_sum) {
        float t2 = t - profile->T1;
        vel = v1 + dir * a_max * t2;
        profile->current_vel = vel;
        return vel;
    }
    float v2 = v1 + dir * a_max * profile->T2;
    
    // 阶段3: 减加速 (jerk反向)
    t_sum += profile->T3;
    if (t <= t_sum) {
        float t3 = t - profile->T1 - profile->T2;
        vel = v2 + dir * (a_max * t3 - 0.5f * j * t3 * t3);
        profile->current_vel = vel;
        return vel;
    }
    
    // 完成，到达目标速度
    profile->current_vel = profile->vel_end;
    profile->is_moving = 0;
    return profile->vel_end;
}


