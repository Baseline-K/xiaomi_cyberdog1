/**
  ******************************************************************************
  * @file    Encoder_Adapt.c
  * @brief   编码器服务层实现，见 Encoder_Adapt.h。
  *
  *          依赖关系：
  *            Encoder_Adapt.c  ->  Encoder_Adapt.h
  *                             ->  AS5047.h / AS5600.h   (只在下面的适配器里出现)
  *          应用层（FOC_run.c / identify_pole.c / ...）只调用本文件的 API，
  *          不再直接包含 AS5047.h 或 AS5600.h。
  ******************************************************************************
  */

#include "main.h"          /* CMSIS: __get_PRIMASK / __set_PRIMASK */

#include "Encoder_Adapt.h"
#include "AS5047.h"
#if (ENCODER_SERVICE_ENABLE_AS5600 == 1)
#include "AS5600.h"
#include "Soft_IIC.h"      /* SOFTI2C_OK */
#endif
#include "motor_state_machine.h"   /* MotorStateMachine_PostFault（ISR 安全） */
#include "Safety_Module.h"         /* FAULT_ENCODER_MASK */
#include "foc.h"                   /* FOC_Frequency（驱动速度估计/预测补帧需要 Ts） */

/*==============================================================================
 * 私有常量
 *============================================================================*/

#define ENCODER_TWO_PI              6.28318530717958647692f
#define ENCODER_DEFAULT_POLE_PAIRS  7.0f

/*==============================================================================
 * 单例
 *============================================================================*/

Encoder_t Encoder;

/*==============================================================================
 * 私有工具
 *============================================================================*/

/** @brief 归一到 [0, 2π)。输入有界，用减法循环避免中断里的库函数调用。 */
static float encoder_wrap_2pi(float a)
{
    while (a >= ENCODER_TWO_PI) {
        a -= ENCODER_TWO_PI;
    }
    while (a < 0.0f) {
        a += ENCODER_TWO_PI;
    }
    return a;
}

/** @brief 归一到 [-π, π)。 */
static float encoder_wrap_pi(float a)
{
    while (a >= ENCODER_TWO_PI * 0.5f) {
        a -= ENCODER_TWO_PI;
    }
    while (a < -ENCODER_TWO_PI * 0.5f) {
        a += ENCODER_TWO_PI;
    }
    return a;
}

/*==============================================================================
 * 编码器故障上报
 *
 * 连续 ENCODER_FAULT_STREAK_N 拍拿不到有效样本 → PostFault(FAULT_ENCODER_MASK)。
 * MotorStateMachine_PostFault() 只做"或位 + 置 pending"，可由本函数在 FOC 中断里
 * 直接调用；后续关功率、CAN 上报、以及重初始化都由状态机负责。
 * 这里不直接调 PowerStage_Disable()：关断路径单一化，避免两处抢管。
 *============================================================================*/

static void encoder_note_sample_ok(void)
{
    Encoder.fault_streak = 0U;
}

static void encoder_note_sample_bad(void)
{
    Encoder.fault_streak++;

    if (Encoder.fault_streak == ENCODER_FAULT_STREAK_N) {
        /* 10 consecutive missing samples at 10 kHz = 1 ms. */
        Encoder.fault_posts++;
        MotorStateMachine_PostFault(FAULT_ENCODER_MASK);
    } else if ((Encoder.fault_streak % ENCODER_FAULT_REPOST_CYCLES) == 0U) {
        /* 仍取不到 → 周期性重报（≈2.4 次/秒），使状态机在自动恢复后
           重新锁存，不会安静地停在 IDLE；同时避免刷爆 RTT。
           streak 只有成功取到样本才清零，所以这里不依赖状态机内部行为。 */
        Encoder.fault_posts++;
        MotorStateMachine_PostFault(FAULT_ENCODER_MASK);
    }
}

/* AS5047: default one-frame polling at each FOC tick. Thread-context alignment
 * uses a fresh two-frame read while FOC is quiescent. DMA is optional and uses
 * the same response ownership/age rules. */
static int as5047_init(void)
{
    if (bsp_as5047HwInit() != AS5047_OK) {
        return -1;
    }

    bsp_as5047SetSampleTime(1.0f / (float)FOC_Frequency);
    /* Filter only measured-sample velocity. It now also advances delayed samples
     * to control time; validate acceleration tracking with the model PLL. */
    bsp_as5047SetSpeedLpf(0.1f);

#if (AS5047_DMA_SUPPORTED == 1)
    if (bsp_as5047AsyncStart() != AS5047_OK) {
        return -1;
    }
#endif
    return 0;
}

static void as5047_deinit(void)
{
#if (AS5047_DMA_SUPPORTED == 1)
    bsp_as5047AsyncStop();
#endif
}

static Encoder_SampleResult_e as5047_update(float *mec_rad)
{
    if (!Encoder_AS5047.initialized) {
        return ENC_SAMPLE_ERROR;
    }

    /* Startup alignment reads once after a long delay. Never use a stale
     * pipeline response to establish electrical zero. FOC is not running here. */
    if (__get_IPSR() == 0U) {
        return bsp_as5047GetAngleImmediate(mec_rad) == AS5047_OK
            ? ENC_SAMPLE_NEW : ENC_SAMPLE_ERROR;
    }

#if (AS5047_DMA_SUPPORTED == 1)
    {
        int ret = bsp_as5047AsyncFetch(NULL);

        *mec_rad = bsp_as5047GetControlAngle();
        if (ret == AS5047_OK) {
            return ENC_SAMPLE_NEW;
        }
        if (ret == AS5047_PREDICTED) {
            return ENC_SAMPLE_PREDICTED;   /* 角度连续，但不是实测值 */
        }
        if (ret == AS5047_ERR_BUSY) {
            return ENC_SAMPLE_ERROR; /* No fresh response is not a healthy sample. */
        }
        return ENC_SAMPLE_ERROR;
    }
#else
    /* One frame per FOC call, with a timestamped previous-command response. */
    {
        int ret = bsp_as5047GetAngle(NULL);

        *mec_rad = bsp_as5047GetControlAngle();
        if (ret == AS5047_OK) {
            return ENC_SAMPLE_NEW;
        }
        if (ret == AS5047_PREDICTED) {
            return ENC_SAMPLE_PREDICTED;
        }
        return ENC_SAMPLE_ERROR;
    }
#endif
}

static void as5047_set_zero_offset(float offset_rad)
{
    bsp_as5047SetZeroOffset(offset_rad);
}

static int as5047_set_zero(void)
{
    return (bsp_as5047SetZeroHere() == AS5047_OK) ? 0 : -1;
}

static void as5047_reset_tracking(void)
{
    bsp_as5047ResetTracking();
}

static bool as5047_health(void)
{
    return (bsp_as5047IsMagnetOk() && bsp_as5047IsAngleValid());
}

const Encoder_Driver_t Encoder_Driver_AS5047 = {
    ENCODER_TYPE_AS5047,
    "AS5047",
    AS5047_RESOLUTION,
    100.0f,                  /* 仅作参考上限，按实际机械上限修改 */
    as5047_init,
    as5047_deinit,
    as5047_update,
    as5047_set_zero_offset,
    as5047_set_zero,
    as5047_reset_tracking,
    as5047_health
};

/*==============================================================================
 * 适配器：AS5600（软件 I2C）—— 默认不编译
 * PA5/PC4 已被输出端 6 路线性霍尔的 ADC 占用，软件 I2C 无引脚可用；
 * 同时 CMakeLists.txt 也不编译 MID_SoftIIC/ 与 BSP/AS5600.c。
 *============================================================================*/
#if (ENCODER_SERVICE_ENABLE_AS5600 == 1)

static int as5600_init(void)
{
    float probe = 0.0f;
    int ret = bsp_as5600HwInit();

    if (ret == SOFTI2C_OK) {
        return 0;
    }

    /*
     * bsp_as5600HwInit() 的返回值是"滤波配置下发"的结果。上电瞬间器件可能还没
     * 就绪（配置帧 NACK），但稍后读角是正常的——那种情况不能判定为"没有编码器"，
     * 否则整机将永远没有角度可用（迁移前的代码是照常读角，只是滤波停在默认值）。
     * 这里补一次探测：只要能读到角度就允许挂载，配置失败只降级为默认滤波；
     * 具体原因仍可从 AS5600_StartupDiagnostics.filter_config_result 查。
     */
    if (bsp_as5600GetAngle(&probe) == SOFTI2C_OK) {
        return 0;
    }
    return -1;
}

static void as5600_deinit(void)
{
    /* 旧驱动没有反初始化接口；软件 I2C 的 GPIO 保持原状即可。 */
}

static Encoder_SampleResult_e as5600_update(float *mec_rad)
{
    /* bsp_as5600GetAngle() 内部已经把零点偏移加上并归一化到 [0, 2π)。 */
    if (bsp_as5600GetAngle(&Encoder_AS5600.angle) != SOFTI2C_OK) {
        return ENC_SAMPLE_ERROR;
    }
    *mec_rad = Encoder_AS5600.angle;
    return ENC_SAMPLE_NEW;
}

static void as5600_set_zero_offset(float offset_rad)
{
    Encoder_AS5600.angle_zero_offset = offset_rad;
}

static int as5600_set_zero(void)
{
    return (bsp_AS5600_SetZero(0U) == SOFTI2C_OK) ? 0 : -1;
}

static void as5600_reset_tracking(void)
{
    bsp_as5600ResetTracking();
}

static bool as5600_health(void)
{
    /* AS5600 的状态寄存器只在启动时读过；读失败时按“健康”处理，避免误报。 */
    const uint8_t st = AS5600_StartupDiagnostics.status;

    return ((st & (uint8_t)(AS5600_STATUS_ML_MASK | AS5600_STATUS_MH_MASK)) == 0U);
}

const Encoder_Driver_t Encoder_Driver_AS5600 = {
    ENCODER_TYPE_AS5600,
    "AS5600",
    AS5600_RESOLUTION,
    100.0f,
    as5600_init,
    as5600_deinit,
    as5600_update,
    as5600_set_zero_offset,
    as5600_set_zero,
    as5600_reset_tracking,
    as5600_health
};

#endif /* ENCODER_SERVICE_ENABLE_AS5600 */

/*==============================================================================
 * 驱动查找与挂载
 *============================================================================*/

static const Encoder_Driver_t *encoder_find_driver(Encoder_Type_e type)
{
    switch (type) {
    case ENCODER_TYPE_AS5047:
        return &Encoder_Driver_AS5047;
#if (ENCODER_SERVICE_ENABLE_AS5600 == 1)
    case ENCODER_TYPE_AS5600:
        return &Encoder_Driver_AS5600;
#endif
    default:
        return NULL;
    }
}

void Encoder_Detach(void)
{
    const Encoder_Driver_t *old;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    old = Encoder.drv;
    Encoder.drv = NULL;
    Encoder.initialized = false;
    __set_PRIMASK(primask);

    if ((old != NULL) && (old->deinit != NULL)) {
        old->deinit();
    }
}

int Encoder_Attach(Encoder_Type_e type)
{
    const Encoder_Driver_t *drv = encoder_find_driver(type);
    uint32_t primask;

    if (drv == NULL) {
        return ENCODER_ERROR_INIT;
    }
    if ((Encoder.drv == drv) && Encoder.initialized) {
        return ENCODER_OK;
    }

    /* 1) 先把旧接口撤下：FOC 中断此后只会看到 drv == NULL，不会用到半拆卸的驱动。 */
    Encoder_Detach();

    /* 2) 中断开启状态下初始化新硬件（bsp_as5047HwInit 带 HAL_Delay，不能关中断）。 */
    Encoder.data = (Encoder_Data_t){0};
    Encoder.data.status = ENCODER_ERROR_INIT;
    Encoder.data.magnet_ok = false;
    Encoder.have_last = false;
    Encoder.mec_last = 0.0f;

    if ((drv->init == NULL) || (drv->init() != 0)) {
        Encoder.data.status = ENCODER_ERROR_INIT;
        return ENCODER_ERROR_INIT;
    }

    /* 3) 初始化成功后才一次性发布接口。 */
    primask = __get_PRIMASK();
    __disable_irq();
    Encoder.drv = drv;
    Encoder.initialized = true;
    Encoder.data.status = ENCODER_OK;
    __set_PRIMASK(primask);

    /* 4) 初始化可能清零了偏移，这里重新下发一次。 */
    Encoder_SetZeroOffset(Encoder.zero_offset);
    return ENCODER_OK;
}

/*==============================================================================
 * 配置
 *============================================================================*/

void Encoder_SetPolePairs(float pole_pairs)
{
    if (pole_pairs > 0.0f) {
        Encoder.pole_pairs = pole_pairs;
    }
}

void Encoder_SetDirection(float direction)
{
    Encoder.direction = (direction < 0.0f) ? -1.0f : 1.0f;
}

void Encoder_SetZeroOffset(float zero_offset_rad)
{
    Encoder.zero_offset = encoder_wrap_2pi(zero_offset_rad);
    if ((Encoder.drv != NULL) && (Encoder.drv->setZeroOffset != NULL)) {
        Encoder.drv->setZeroOffset(Encoder.zero_offset);
    }
}

int Encoder_Configure(float pole_pairs, float zero_offset)
{
    if (pole_pairs <= 0.0f) {
        return ENCODER_ERROR_INIT;
    }
    Encoder_SetPolePairs(pole_pairs);
    Encoder_SetZeroOffset(zero_offset);
    return ENCODER_OK;
}

void Encoder_ServiceInit(void)
{
    Encoder.drv = NULL;
    Encoder.initialized = false;
    Encoder.pole_pairs = ENCODER_DEFAULT_POLE_PAIRS;
    Encoder.direction = 1.0f;
    Encoder.zero_offset = 0.0f;
    Encoder.mec_last = 0.0f;
    Encoder.have_last = false;
    Encoder.fault_streak = 0U;
    Encoder.fault_posts = 0U;

    Encoder.data = (Encoder_Data_t){0};
    Encoder.data.status = ENCODER_ERROR_NOT_ATTACHED;

    if (Encoder_Attach(ENCODER_SERVICE_DEFAULT_TYPE) == ENCODER_OK) {
        return;
    }

#if (ENCODER_SERVICE_ENABLE_FALLBACK == 1)
    /* 默认编码器没挂上（未焊/接线/上电顺序）时再试另一种，让台架不至于完全
       没有角度可用。实际挂载结果由 Encoder_GetName() 查得，应用层负责打印。 */
    if (ENCODER_SERVICE_DEFAULT_TYPE != ENCODER_TYPE_AS5600) {
        (void)Encoder_Attach(ENCODER_TYPE_AS5600);
    }
#endif
}

/*==============================================================================
 * 每拍更新（FOC 中断里调用）
 *============================================================================*/

int Encoder_Update(void)
{
    const Encoder_Driver_t *drv = Encoder.drv;
    Encoder_Data_t *d = &Encoder.data;
    Encoder_SampleResult_e r;
    float mec = 0.0f;
    float sign;

    if ((drv == NULL) || !Encoder.initialized || (drv->update == NULL)) {
        d->sample_fresh = false;
        d->status = ENCODER_ERROR_NOT_ATTACHED;
        encoder_note_sample_bad();          /* 没挂上驱动 = 永远取不到角度 */
        return ENCODER_ERROR_NOT_ATTACHED;
    }

    /* DMA 模式下这一步同时"转换上一帧、发下一帧"。 */
    r = drv->update(&mec);

    if (r == ENC_SAMPLE_ERROR || r == ENC_SAMPLE_REPEAT) {
        d->error_cnt++;
        d->sample_fresh = false;
        d->status = ENCODER_ERROR_COMM;
        d->predict_streak = 0U;      /* 连预测都用不上了（额度耗尽/未使能） */
        encoder_note_sample_bad();
        return ENCODER_ERROR_COMM;
    }

    if (r == ENC_SAMPLE_PREDICTED) {
        /* 预测补帧：驱动用上一拍有效速度把角度外推了一帧 —— 角度连续，但不是实测值。
         *   1) 照常往下走、用这个连续角度更新 mec/ele，这正是本机制的目的：
         *      避免模型 PLL 看到"输入冻结"而出现过冲/塌陷；
         *   2) **仍然计入 fault_streak** —— 不能因为角度看着连续就当编码器没坏，
         *      否则彻底断线会被预测永远掩盖住、永远不报故障。
         * 预测额度与 ENCODER_FAULT_STREAK_N 对齐（都是 10 帧）：额度耗尽的同时
         * 故障也就上报了，功率级随即被关断。 */
        d->miss_cnt++;
        d->predicted_cnt++;
        d->predict_streak++;
        if (d->predict_streak > d->max_predict_streak) {
            d->max_predict_streak = d->predict_streak;
        }
        d->sample_fresh = false;
        encoder_note_sample_bad();
    } else {
        /* Only a real new sample clears the missing-sample fault streak. */
        encoder_note_sample_ok(); /* Only ENC_SAMPLE_NEW reaches here. */
        d->predict_streak = 0U;
        d->sample_fresh = (r == ENC_SAMPLE_NEW);
        if (d->sample_fresh) {
            d->cnt++;
        } else {
            d->miss_cnt++;
        }
    }

    sign = (Encoder.direction < 0.0f) ? -1.0f : 1.0f;

    /* 机械角保持驱动给出的值（已含零点偏移）；方向只体现在电角度上。 */
    d->mec = mec;
    d->ele = encoder_wrap_2pi(sign * mec * Encoder.pole_pairs);

    if (!Encoder.have_last) {
        Encoder.have_last = true;
        Encoder.mec_last = mec;
        d->total_angle = 0.0f;
    } else {
        /* 最短路径增量，天然处理 2π 绕回 */
        d->total_angle += encoder_wrap_pi(mec - Encoder.mec_last);
        Encoder.mec_last = mec;
    }

    d->magnet_ok = (drv->health != NULL) ? drv->health() : true;
    d->status = d->magnet_ok ? ENCODER_OK : ENCODER_ERROR_INVALID_DATA;
    return (d->status == ENCODER_OK) ? ENCODER_OK : d->status;
}

/*==============================================================================
 * 只读访问
 *============================================================================*/

int Encoder_GetElecAngle(float *ele)
{
    if (ele == NULL) {
        return ENCODER_ERROR_INIT;
    }
    *ele = Encoder.data.ele;
    return (Encoder.data.status == ENCODER_OK) ? ENCODER_OK : Encoder.data.status;
}

int Encoder_GetMecAngle(float *mec)
{
    if (mec == NULL) {
        return ENCODER_ERROR_INIT;
    }
    *mec = Encoder.data.mec;
    return (Encoder.data.status == ENCODER_OK) ? ENCODER_OK : Encoder.data.status;
}

int Encoder_GetTotalAngle(float *total_angle)
{
    if (total_angle == NULL) {
        return ENCODER_ERROR_INIT;
    }
    *total_angle = Encoder.data.total_angle;
    return ENCODER_OK;
}

int Encoder_SetZero(void)
{
    if ((Encoder.drv == NULL) || (Encoder.drv->setZero == NULL)) {
        return ENCODER_ERROR_NOT_ATTACHED;
    }
    return (Encoder.drv->setZero() == 0) ? ENCODER_OK : ENCODER_ERROR_CALIBRATION;
}

void Encoder_ResetTracking(void)
{
    Encoder.have_last = false;
    Encoder.mec_last = 0.0f;

    Encoder.data.total_angle = 0.0f;
    Encoder.data.miss_cnt = 0U;

    /* 注意：不动 fault_streak。故障判定必须只由"是否取到样本"驱动；
     * 若在这里清零，状态机的自动恢复（会调 Param_init → 本函数）就会顺带
     * 重置判定，导致重报节奏被状态机内部行为左右。 */

    if ((Encoder.drv != NULL) && (Encoder.drv->resetTracking != NULL)) {
        Encoder.drv->resetTracking();
    }
}

bool Encoder_IsHealthy(void)
{
    return ((Encoder.drv != NULL) && Encoder.initialized &&
            (Encoder.data.status == ENCODER_OK) && Encoder.data.magnet_ok);
}

Encoder_Status_e Encoder_GetStatus(void)
{
    return Encoder.data.status;
}

Encoder_Type_e Encoder_GetType(void)
{
    return (Encoder.drv != NULL) ? Encoder.drv->type : ENCODER_TYPE_NONE;
}

const char *Encoder_GetName(void)
{
    return (Encoder.drv != NULL) ? Encoder.drv->name : "none";
}

uint32_t Encoder_GetResolution(void)
{
    return (Encoder.drv != NULL) ? Encoder.drv->resolution : 0U;
}
