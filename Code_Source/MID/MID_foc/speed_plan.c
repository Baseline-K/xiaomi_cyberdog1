/**
 * 速度 S 曲线规划（移植自 AC_Control Traj_Plan.c 的 SCurve_Speed_Profile，jerk 受限三段）
 *
 * 在 10kHz FOC step 内 10:1 降采样到 1kHz：
 *   - 目标变化超过阈值 → 自动重规划（SCurve_Speed_Update 语义，可运行中改目标）
 *   - 方向反转 → 先刹到 0 再反向（两段，next_pending）
 *   - 输出：平滑 ref_speed (RPS) 与 ref_accel (RPS/s)
 * 单位：速度 RPS、加速度 RPS/s、时间 s。
 */
#include "speed_plan.h"
#include <math.h>

#define SP_DT_S      0.001f   /* 1kHz 控制周期 */
#define SP_DIV       10u      /* 10kHz -> 1kHz 分频 */
#define SP_JERK_RATIO 50.0f   /* max_jerk = ratio * max_accel */
#define SP_VEL_MAX   40.0f    /* ±RPS 限速（电机最大 40RPS） */
#define SP_ACC_MAX   800.0f   /* RPS/s，余量大则不触发 */
#define SP_DEFAULT_MS 300u

typedef struct
{
    float target_vel, current_vel, current_accel;
    float accel_time, decel_time, Plan_Threshold;
    float T1, T2, T3, vel_start, vel_end, time, max_accel, jerk;
    float max_vel_fwd, max_vel_rev, max_accel_fwd, max_accel_rev;
    float max_jerk_fwd, max_jerk_rev;
    unsigned short limit_flags;
    int is_moving, direction;
    float next_target;
    int next_pending;
} SpProfile_t;

static SpProfile_t g_sp;
static float g_accel_time_s = SP_DEFAULT_MS / 1000.0f;
static float g_decel_time_s = SP_DEFAULT_MS / 1000.0f;
static uint8_t s_div = 0, s_prev_active = 0;
static float s_ref = 0.0f, s_acc = 0.0f;

/* ---------- 内部：Plan_ToZero ---------- */
static void sp_plan_tozero(SpProfile_t *p)
{
    float new_target = 0.0f;
    p->limit_flags = 0;
    p->target_vel = new_target;
    p->vel_start  = p->current_vel;
    p->vel_end    = new_target;
    float raw_delta = new_target - p->current_vel;
    float delta_v   = fabsf(raw_delta);
    p->direction    = (raw_delta > 0.0f) ? 1 : -1;

    float total_time = p->decel_time;
    if (total_time <= 0.0f) total_time = 1e-6f;
    p->T1 = total_time * 0.25f;
    p->T2 = total_time * 0.50f;
    p->T3 = total_time * 0.25f;

    float max_accel = (p->direction > 0) ? p->max_accel_fwd : p->max_accel_rev;
    float max_jerk  = (p->direction > 0) ? p->max_jerk_fwd  : p->max_jerk_rev;
    float jk = (p->T1 * (p->T1 + p->T2) > 0.0f) ? (delta_v / (p->T1 * (p->T1 + p->T2))) : 0.0f;
    float ac = jk * p->T1;
    if (jk > max_jerk) {
        p->limit_flags |= 4u; /* LIMIT_JERK */
        jk = max_jerk; ac = jk * p->T1;
        p->T2 = delta_v / (jk * p->T1) - p->T1;
        if (p->T2 < 0.0f) { p->T2 = 0.0f; jk = delta_v / (2.0f * p->T1 * p->T1); ac = jk * p->T1; }
    }
    if (ac > max_accel) {
        p->limit_flags |= 2u; /* LIMIT_ACCEL */
        ac = max_accel; jk = ac / p->T1;
        p->T2 = delta_v / (jk * p->T1) - p->T1;
        if (p->T2 < 0.0f) { p->T2 = 0.0f; jk = delta_v / (2.0f * p->T1 * p->T1); }
    }
    p->jerk = jk;
    p->max_accel = ac;
    p->time = 0.0f;
    p->is_moving = (delta_v > p->Plan_Threshold) ? 1 : 0;
}

/* ---------- 内部：Plan ---------- */
static void sp_plan(SpProfile_t *p, float new_target, int force_replan)
{
    p->limit_flags = 0;

    float max_vel = (new_target >= 0.0f) ? p->max_vel_fwd : p->max_vel_rev;
    if (fabsf(new_target) > max_vel) {
        p->limit_flags |= 1u; /* LIMIT_VEL */
        new_target = (new_target > 0.0f) ? max_vel : -max_vel;
    }
    if (!force_replan) {
        if (fabsf(new_target - p->target_vel) < p->Plan_Threshold &&
            fabsf(p->current_vel - p->target_vel) < p->Plan_Threshold) {
            return;
        }
    }
    float raw_delta = new_target - p->current_vel;
    if (fabsf(raw_delta) < p->Plan_Threshold) {
        p->is_moving = 0; p->direction = 0;
        p->target_vel = new_target; p->vel_end = new_target;
        return;
    }
    /* 方向反转：先刹到 0，到 0 后自动去 next_target */
    if ((p->current_vel * new_target) < 0.0f) {
        if (fabsf(p->current_vel) > p->Plan_Threshold) {
            p->next_target = new_target; p->next_pending = 1;
            sp_plan_tozero(p);
            return;
        }
    }

    p->target_vel = new_target;
    p->vel_start  = p->current_vel;
    p->vel_end    = new_target;
    float delta_v = fabsf(raw_delta);
    p->direction  = (raw_delta > 0.0f) ? 1 : -1;
    int is_speed_up = (fabsf(new_target) > fabsf(p->current_vel)) ? 1 : 0;
    float total_time = is_speed_up ? p->accel_time : p->decel_time;
    if (total_time <= 0.0f) total_time = 1e-6f;
    p->T1 = total_time * 0.25f;
    p->T2 = total_time * 0.50f;
    p->T3 = total_time * 0.25f;

    float max_jerk  = (p->direction > 0) ? p->max_jerk_fwd  : p->max_jerk_rev;
    float max_accel = (p->direction > 0) ? p->max_accel_fwd : p->max_accel_rev;
    float jk = (p->T1 * (p->T1 + p->T2) > 0.0f) ? (delta_v / (p->T1 * (p->T1 + p->T2))) : 0.0f;
    float ac = jk * p->T1;
    if (jk > max_jerk) {
        p->limit_flags |= 4u;
        jk = max_jerk; ac = jk * p->T1;
        p->T2 = delta_v / (jk * p->T1) - p->T1;
        if (p->T2 < 0.0f) { p->T2 = 0.0f; jk = delta_v / (2.0f * p->T1 * p->T1); ac = jk * p->T1; }
    }
    if (ac > max_accel) {
        p->limit_flags |= 2u;
        ac = max_accel; jk = ac / p->T1;
        p->T2 = delta_v / (jk * p->T1) - p->T1;
        if (p->T2 < 0.0f) { p->T2 = 0.0f; jk = delta_v / (2.0f * p->T1 * p->T1); }
    }
    p->jerk = jk;
    p->max_accel = ac;
    p->time = 0.0f;
    p->is_moving = 1;
}

/* ---------- 内部：单步推进（更新 current_vel/current_accel） ---------- */
static void sp_update_internal(SpProfile_t *p, float dt)
{
    if (!p->is_moving) return;
    p->time += dt;
    float t = p->time;
    float T1 = p->T1, T2 = p->T2, T3 = p->T3, T_total = T1 + T2 + T3;

    if (T_total <= 0.0f) {
        p->is_moving = 0; p->time = 0.0f;
        p->current_vel = p->vel_end; p->current_accel = 0.0f;
        if (p->next_pending) { float nxt = p->next_target; p->next_pending = 0; sp_plan(p, nxt, 0); }
        return;
    }
    float v0 = p->vel_start;
    int   dir = (p->direction >= 0) ? 1 : -1;
    float j = p->jerk;
    float a_max = p->max_accel;
    float vel = v0, acc = 0.0f;
    float t1 = T1, t2 = T1 + T2, t3 = T_total;

    if (t <= 0.0f) {
        vel = v0; acc = 0.0f;
    } else if (t < t1) {
        acc = (j > 0.0f) ? ((float)dir * j * t) : 0.0f;
        vel = v0 + (float)dir * 0.5f * j * t * t;
    } else if (t < t2) {
        float tp = t - t1;
        acc = (float)dir * a_max;
        float v1 = v0 + (float)dir * 0.5f * j * t1 * t1;
        vel = v1 + (float)dir * a_max * tp;
    } else if (t < t3) {
        float tp = t - t2;
        float am = a_max - j * tp;
        if (am < 0.0f) am = 0.0f;
        if (am > a_max) am = a_max;
        acc = (float)dir * am;
        float v1 = v0 + (float)dir * 0.5f * j * t1 * t1;
        float v2 = v1 + (float)dir * a_max * T2;
        vel = v2 + (float)dir * (a_max * tp - 0.5f * j * tp * tp);
    } else {
        p->is_moving = 0; p->time = T_total;
        p->current_vel = p->vel_end; p->current_accel = 0.0f;
        if (p->next_pending) { float nxt = p->next_target; p->next_pending = 0; sp_plan(p, nxt, 0); }
        return;
    }
    p->current_vel = vel;
    p->current_accel = acc;
    if (p->time >= T_total - 1e-6f) {
        p->is_moving = 0;
        p->current_vel = p->vel_end; p->current_accel = 0.0f;
        if (p->next_pending) { float nxt = p->next_target; p->next_pending = 0; sp_plan(p, nxt, 0); }
    }
}

/* ---------- 内部：目标更新 + 推进（AC SCurve_Speed_Update 语义） ---------- */
static float sp_update(SpProfile_t *p, float target_vel, float dt)
{
    float max_vel = (target_vel >= 0.0f) ? p->max_vel_fwd : p->max_vel_rev;
    float clamped = target_vel;
    if (fabsf(target_vel) > max_vel) clamped = (target_vel > 0.0f) ? max_vel : -max_vel;

    int need_replan = 0;
    if (fabsf(clamped - p->target_vel) > p->Plan_Threshold) {
        if (p->next_pending) {
            if (fabsf(clamped - p->next_target) < p->Plan_Threshold && fabsf(p->target_vel) < p->Plan_Threshold) {
                need_replan = 0;
            } else {
                need_replan = 1;
            }
        } else {
            need_replan = 1;
        }
    }
    if (need_replan) sp_plan(p, clamped, 0);
    if (!p->is_moving) return p->current_vel;
    sp_update_internal(p, dt);
    return p->current_vel;
}

/* ---------- 对外 ---------- */
void SpeedPlan_Reset(void)
{
    g_sp.target_vel = 0.0f; g_sp.current_vel = 0.0f; g_sp.current_accel = 0.0f;
    g_sp.accel_time = g_accel_time_s; g_sp.decel_time = g_decel_time_s;
    g_sp.Plan_Threshold = 0.05f;
    g_sp.max_vel_fwd = SP_VEL_MAX; g_sp.max_vel_rev = SP_VEL_MAX;
    g_sp.max_accel_fwd = SP_ACC_MAX; g_sp.max_accel_rev = SP_ACC_MAX;
    g_sp.max_jerk_fwd = SP_JERK_RATIO * SP_ACC_MAX;
    g_sp.max_jerk_rev = SP_JERK_RATIO * SP_ACC_MAX;
    g_sp.T1 = g_sp.T2 = g_sp.T3 = 0.0f;
    g_sp.vel_start = g_sp.vel_end = 0.0f;
    g_sp.time = g_sp.max_accel = g_sp.jerk = 0.0f;
    g_sp.limit_flags = 0;
    g_sp.is_moving = 0; g_sp.direction = 0;
    g_sp.next_pending = 0;
    s_div = 0; s_prev_active = 0; s_ref = 0.0f; s_acc = 0.0f;
}

void SpeedPlan_Step10k(int active, float cmd_rps, float fbk_rps, float *ref_rps, float *acc_rps2)
{
    int en = active;

    if (en && !s_prev_active) {
        /* 进入速度模式/重新使能：从实测转速无扰起步，强制规划到当前目标 */
        g_sp.current_vel = fbk_rps;
        g_sp.current_accel = 0.0f;
        g_sp.time = 0.0f;
        g_sp.is_moving = 0;
        g_sp.next_pending = 0;
        sp_plan(&g_sp, cmd_rps, 1);
        s_div = 0;
    }
    s_prev_active = (uint8_t)en;

    if (!en) {
        *ref_rps = cmd_rps;   /* 直通=原行为 */
        *acc_rps2 = 0.0f;
        return;
    }

    s_div = (uint8_t)((s_div + 1) % SP_DIV);
    if (s_div == 0) {
        s_ref = sp_update(&g_sp, cmd_rps, SP_DT_S);
        s_acc = g_sp.current_accel;
    }
    *ref_rps  = s_ref;
    *acc_rps2 = s_acc;
}

void SpeedPlan_SetTimeMs(uint32_t accel_ms, uint32_t decel_ms)
{
    if (accel_ms < 10u) accel_ms = 10u;
    if (decel_ms < 10u) decel_ms = 10u;
    g_accel_time_s = (float)accel_ms / 1000.0f;
    g_decel_time_s = (float)decel_ms / 1000.0f;
    g_sp.accel_time = g_accel_time_s;
    g_sp.decel_time = g_decel_time_s;
    /* 时间变化后当前运动若进行中，用新时间重规划到当前目标（幅度近似不变） */
    if (g_sp.is_moving) {
        sp_plan(&g_sp, g_sp.target_vel, 1);
    }
}
