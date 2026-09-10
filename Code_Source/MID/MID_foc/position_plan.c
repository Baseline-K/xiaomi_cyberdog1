/**
 * 位置 S 曲线规划（非对称梯形/三角，闭式解）—— 移植自 AC_Control Traj_Plan.c 的位置部分
 * 绝对目标版本：以当前规划位置/速度为起点，重规划到新目标；输出 rad / RPS / RPS²。
 */
#include "position_plan.h"
#include <math.h>

#define PP_DT_S      0.001f     /* 1kHz 控制周期 */
#define PP_DIV       10u        /* 10kHz -> 1kHz */
#define PP_TWO_PI    6.283185307f
#define PP_TGT_EPS   1e-4f      /* 目标变化阈值(rad) */
#define PP_DEF_VMAX  40.0f      /* 默认规划峰值限速(RPS) */
#define PP_DEF_T_MS  300u

typedef struct {
    float target;                    /* 绝对目标(rad) */
    float start_pos, start_vel;      /* 本次规划起点(rad, rad/s) */
    float d_total; int dir;          /* 行程与方向 */
    float vpeak;                     /* 峰值速度(相对, rad/s) */
    float a_acc, a_dec;              /* 加/减速度幅值(rad/s²) */
    float Ta, Tc, Td, time;          /* 三段时长与已耗时(s) */
    float pos, vel, acc;             /* 当前规划状态(rad, rad/s, rad/s²) */
    float vmax_rps;                  /* 规划峰值限速(RPS) */
    float trise, tfall;              /* 加/减速时间(s) */
} PP_t;

static PP_t    g;
static uint8_t s_div;
static int     s_prev_active;
static float   s_pos, s_vel_rps, s_acc_rps2;

static float sgn(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

static void pp_plan(float cur_pos, float cur_vel, float target)
{
    g.target    = target;
    g.start_pos = cur_pos;
    g.start_vel = cur_vel;
    g.time      = 0.0f;

    float d = target - cur_pos;
    g.d_total = fabsf(d);
    g.dir     = (d >= 0.0f) ? 1 : -1;

    if (g.d_total < 1e-6f) {                 /* 已在目标：不动 */
        g.vpeak = 0.0f; g.Ta = g.Tc = g.Td = 0.0f;
        g.pos = target; g.vel = 0.0f; g.acc = 0.0f;
        return;
    }

    float v0   = (float)g.dir * cur_vel;     /* 方向相对速度 */
    float vend = 0.0f;                       /* 终点速度 0 */
    float v_used = g.vmax_rps * PP_TWO_PI;   /* RPS -> rad/s */
    if (v_used < 1e-6f) v_used = 1e-6f;

    float t_r = (g.trise > 1e-6f) ? g.trise : 1e-6f;
    float t_f = (g.tfall > 1e-6f) ? g.tfall : 1e-6f;
    float a_acc = fabsf(v_used - v0) / t_r;
    float a_dec = fabsf(vend  - v_used) / t_f;
    if (a_acc < 1e-6f) a_acc = 1e-6f;
    if (a_dec < 1e-6f) a_dec = 1e-6f;

    float D = g.d_total, Ta = 0.0f, Tc = 0.0f, Td = 0.0f, vp = 0.0f;
    int ok = 0;
    {   /* 先试“能达到 v_used”的梯形 */
        float Ta1 = (v_used - v0) / a_acc;
        float Td1 = (vend  - v_used) / a_dec;
        float s_acc = v0 * Ta1 + 0.5f * a_acc * Ta1 * Ta1;
        float s_dec = v_used * Td1 + 0.5f * a_dec * Td1 * Td1;
        if ((Ta1 >= 0.0f) && (Td1 >= 0.0f) && ((s_acc + s_dec) <= D)) {
            Ta = Ta1; Td = Td1; vp = v_used;
            float rem = D - (s_acc + s_dec);
            Tc = (v_used > 1e-6f) ? (rem / v_used) : 0.0f;
            ok = 1;
        }
    }
    if (!ok) {   /* 三角（或前后速度非零的非对称）解峰值 */
        float numer = 2.0f * a_acc * a_dec * D + a_dec * v0 * v0 + a_acc * vend * vend;
        float denom = a_acc + a_dec;
        vp = sqrtf(numer / denom);
        if (vp < v0)   vp = v0;
        if (vp < vend) vp = vend;
        Ta = (vp - v0)   / a_acc; if (Ta < 0.0f) Ta = 0.0f;
        Td = (vend - vp) / a_dec; if (Td < 0.0f) Td = 0.0f;
        Tc = 0.0f;
    }
    g.vpeak = vp; g.a_acc = a_acc; g.a_dec = a_dec;
    g.Ta = Ta; g.Tc = Tc; g.Td = Td;
    g.pos = cur_pos; g.vel = cur_vel; g.acc = 0.0f;
}

static void pp_eval(void)
{
    float T = g.Ta + g.Tc + g.Td;
    float t = g.time;
    if (t >= T) {
        g.pos = g.target; g.vel = 0.0f; g.acc = 0.0f;
        return;
    }
    float v0 = (float)g.dir * g.start_vel;
    float vend = 0.0f;
    float sa = sgn(g.vpeak - v0)   * g.a_acc;   /* 含符号加速度 */
    float sd = sgn(vend  - g.vpeak) * g.a_dec;

    float pos_r, vel_r, acc_r;
    float s_acc = v0 * g.Ta + 0.5f * sa * g.Ta * g.Ta;
    if (t < g.Ta) {
        acc_r = sa; vel_r = v0 + sa * t; pos_r = v0 * t + 0.5f * sa * t * t;
    } else if (t < (g.Ta + g.Tc)) {
        acc_r = 0.0f; vel_r = g.vpeak; pos_r = s_acc + g.vpeak * (t - g.Ta);
    } else {
        float td = t - g.Ta - g.Tc;
        float s_cru = g.vpeak * g.Tc;
        acc_r = sd; vel_r = g.vpeak + sd * td;
        pos_r = s_acc + s_cru + g.vpeak * td + 0.5f * sd * td * td;
    }

    g.pos = g.start_pos + (float)g.dir * pos_r;
    g.vel = (float)g.dir * vel_r;
    g.acc = (float)g.dir * acc_r;
}

void PositionPlan_Reset(void)
{
    g.target = 0.0f; g.start_pos = 0.0f; g.start_vel = 0.0f;
    g.d_total = 0.0f; g.dir = 1; g.vpeak = 0.0f;
    g.a_acc = g.a_dec = 0.0f; g.Ta = g.Tc = g.Td = g.time = 0.0f;
    g.pos = g.vel = g.acc = 0.0f;
    g.vmax_rps = PP_DEF_VMAX;
    g.trise = g.tfall = (float)PP_DEF_T_MS / 1000.0f;
    s_div = 0u; s_prev_active = 0; s_pos = 0.0f; s_vel_rps = 0.0f; s_acc_rps2 = 0.0f;
}

void PositionPlan_SetTimeMs(uint32_t ms)
{
    if (ms < 10u) ms = 10u;
    g.trise = g.tfall = (float)ms / 1000.0f;
    if (s_prev_active) pp_plan(g.pos, g.vel, g.target);   /* 时间变化后按当前状态重规划 */
}

void PositionPlan_SetMaxVelRps(float rps)
{
    if (rps > 0.0f) g.vmax_rps = rps;
}

void PositionPlan_SetTarget(float target_rad)
{
    if (!s_prev_active) { g.target = target_rad; return; }   /* 未激活：激活时以实测锚定 */
    if (fabsf(target_rad - g.target) > PP_TGT_EPS) {
        pp_plan(g.pos, g.vel, target_rad);
    }
}

void PositionPlan_Step10k(int active, float fbk_rad,
                          float *pos_ref, float *vel_ref, float *acc_ref)
{
    if (active && !s_prev_active) {
        pp_plan(fbk_rad, 0.0f, g.target);   /* 进入位置模式：从实测位置无扰起步 */
        s_div = 0u;
    }
    s_prev_active = active;

    if (!active) {
        s_pos = fbk_rad; s_vel_rps = 0.0f; s_acc_rps2 = 0.0f;
        *pos_ref = s_pos; *vel_ref = 0.0f; *acc_ref = 0.0f;
        return;
    }

    s_div = (uint8_t)((s_div + 1u) % PP_DIV);
    if (s_div == 0u) {
        g.time += PP_DT_S;
        pp_eval();
        s_pos = g.pos;
        s_vel_rps  = g.vel / PP_TWO_PI;   /* rad/s -> RPS */
        s_acc_rps2 = g.acc / PP_TWO_PI;   /* rad/s² -> RPS/s */
    }
    *pos_ref = s_pos; *vel_ref = s_vel_rps; *acc_ref = s_acc_rps2;
}
