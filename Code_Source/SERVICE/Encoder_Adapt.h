/**
  ******************************************************************************
  * @file    Encoder_Adapt.h
  * @brief   编码器服务层（策略模式 + 驱动注册）。
  *
  *          结构参照 AC_Control_400W_New 的 SERVICE/Encoder_Adapt.c：
  *            1) 定义统一的编码器数据结构和驱动接口（函数指针表）；
  *            2) 每个具体编码器（AS5047 / AS5600 / ...）实现自己的适配器；
  *            3) 应用层只调用本服务层的 API，不直接碰 BSP 驱动。
  *
  *          换编码器 = 换一行注册：
  *            Encoder_Attach(ENCODER_TYPE_AS5047);   // 或 ENCODER_TYPE_AS5600
  *          或者改编译宏 ENCODER_SERVICE_DEFAULT_TYPE，其余代码不用动。
  *
  *          职责边界（重要）：
  *            - 驱动层：硬件初始化/反初始化、取本拍机械角、零点偏移与烧写、
  *              磁场健康状态。
  *            - 服务层：电角度换算、方向、连续多圈角、状态聚合、健康判定。
  *            - **服务层不计算速度，也不计算加速度。** 这两者属于观测器/
  *              控制算法的职责，本项目由 Simulink 模型里的 EncoderPLL 提供
  *              （Y.speed_meas_rps / Y.accel_meas_rads2）。这样全系统只有一条
  *              "角度→速度→加速度"链路，不会出现两路互相打架的估计值。
  ******************************************************************************
  */

#ifndef __ENCODER_ADAPT_H
#define __ENCODER_ADAPT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 配置
 *============================================================================*/

/** 上电默认挂载的编码器。改这一行即可整体切换。 */
#ifndef ENCODER_SERVICE_DEFAULT_TYPE
#define ENCODER_SERVICE_DEFAULT_TYPE    ENCODER_TYPE_AS5047
#endif

/**
 * 1 = 默认编码器挂载失败时，自动再试另一种（AS5047 <-> AS5600），
 *     并在打印里说明实际挂上的是哪一个。台架换板/换编码器时不会因为
 *     一处配置就完全没角度可用；量产可设 0 让失败直接暴露。
 */
#ifndef ENCODER_SERVICE_ENABLE_FALLBACK
#define ENCODER_SERVICE_ENABLE_FALLBACK 0
#endif

/**
 * 是否编译 AS5600（软件 I2C）这条路径。
 *
 * 2026-09-27 起置 0，原因是**物理上没引脚了**：输出端 6 路线性霍尔占用了
 * PA5（原 IIC_SDA）与 PC4（原 IIC_SCL）作为 ADC 输入，CubeMX 再生成后
 * `IIC_SDA_Pin/IIC_SCL_Pin` 宏消失，`Soft_IIC.c` 无法编译。
 * 同时 CMakeLists.txt 里也把 `MID_SoftIIC/` 与 `BSP/AS5600.c` 排除出构建
 * —— 恢复时要两边一起改。
 */
#ifndef ENCODER_SERVICE_ENABLE_AS5600
#define ENCODER_SERVICE_ENABLE_AS5600 0
#endif

/**
 * 连续多少拍"取不到有效角度"才判定编码器故障并上报 FAULT_ENCODER_MASK。
 * FOC 中断 10 kHz，10 拍 = 1 ms。
 *
 * 取值权衡：驱动层会在读失败时做**速度预测补帧**（见 AS5047.h 的
 * `AS5047_PREDICT_MAX_FRAMES`，上限同样是 10 帧），所以这 10 拍里角度仍然连续、
 * 控制不会因为一次校验失败而抖动；到第 10 拍预测额度用尽 → 立即报故障关功率。
 * 也就是说"预测到多久"与"多久后报故障"是同一个窗口。
 *
 * 注意：预测帧**仍然计入**本计数（不能因为角度看起来连续就当它没坏），
 * 否则编码器彻底断线会被预测永远掩盖。
 */
#ifndef ENCODER_FAULT_STREAK_N
#define ENCODER_FAULT_STREAK_N  10U
#endif

/**
 * 首次判定后，若"取不到样本"持续不断，每隔多少拍重报一次 FAULT_ENCODER_MASK。
 * 4096 拍 ≈ 410 ms（取 2 的幂，编译器把取模变成与运算）。
 *
 * 为什么必须重报：状态机对编码器故障会做一次定向软恢复（Param_init/deinit +
 * FOC_Generated_Reset）然后回到 IDLE；若不重报，编码器一直坏的情况下系统会
 * 安静地停在 IDLE、等着带病启动。
 * 为什么不是每 N 拍重报：那会以 ~100 Hz 触发状态机的 FAULT_NOW 打印，刷爆 RTT。
 * 410 ms 对应约 2.4 行/秒，既能表达"仍在重试"，又不淹没其它输出。
 */
#ifndef ENCODER_FAULT_REPOST_CYCLES
#define ENCODER_FAULT_REPOST_CYCLES  4096U
#endif

/*==============================================================================
 * 类型
 *============================================================================*/

/** 支持的编码器类型。 */
typedef enum {
    ENCODER_TYPE_NONE  = 0,
    ENCODER_TYPE_AS5600,
    ENCODER_TYPE_AS5047
} Encoder_Type_e;

/** 服务层状态。 */
typedef enum {
    ENCODER_OK           =  0,   /**< 工作正常                                  */
    ENCODER_CALIBRATING  =  1,   /**< 标定中（供应用层使用，服务层不产生）        */
    ENCODER_CALIBRATED   =  2,   /**< 标定完成                                  */

    ENCODER_ERROR_INIT       = -1,   /**< 初始化失败                            */
    ENCODER_ERROR_COMM       = -2,   /**< 通信失败                              */
    ENCODER_ERROR_CALIBRATION= -3,   /**< 标定失败                              */
    ENCODER_ERROR_HARDWARE   = -4,   /**< 硬件错误                              */
    ENCODER_ERROR_INVALID_DATA = -5, /**< 数据不可信（磁场异常/CORDIC 溢出）     */
    ENCODER_ERROR_NOT_ATTACHED = -6  /**< 没有注册编码器                        */
} Encoder_Status_e;

/** 单次采样的结果，由驱动适配器返回。 */
typedef enum {
    ENC_SAMPLE_NEW       =  0,   /**< 取到新样本                                   */
    ENC_SAMPLE_REPEAT    =  1,   /**< 本拍无新样本，角度沿用上一次（不是错误）        */
    ENC_SAMPLE_PREDICTED =  2,   /**< 读取失败，角度由速度外推补出（连续但不新鲜）    */
    ENC_SAMPLE_ERROR     = -1    /**< 通信/数据错误，角度冻结                       */
} Encoder_SampleResult_e;

/** 服务层输出的编码器数据（应用层只读）。 */
typedef struct {
    float ele;              /**< 电角度 [0, 2π)                                  */
    float mec;              /**< 机械角 [0, 2π)，已含零点偏移                     */
    float total_angle;      /**< 连续多圈机械角，rad，可正可负                     */
    uint32_t cnt;           /**< 有效样本计数                                     */
    uint32_t error_cnt;     /**< 通信错误计数                                     */
    uint32_t miss_cnt;      /**< 无新样本的拍数                                   */
    uint32_t predicted_cnt; /**< 其中由速度预测补出的帧数（角度连续但非实测）        */
    uint32_t predict_streak;/**< 当前连续预测帧数（取到真实样本即清零）             */
    uint32_t max_predict_streak; /**< 历史最长连续预测帧数                          */
    bool sample_fresh;      /**< 本拍是否取到新样本                                */
    bool magnet_ok;         /**< 磁/信号健康（驱动不支持时恒为 true）              */
    Encoder_Status_e status;/**< 综合状态                                         */
} Encoder_Data_t;

/* --- 驱动适配器需要实现的接口 ------------------------------------------- */

/** @brief 初始化硬件。返回 0 表示成功。 */
typedef int  (*Encoder_Init_fn)(void);

/** @brief 反初始化硬件（可为 NULL）。 */
typedef void (*Encoder_Deinit_fn)(void);

/**
 * @brief 取本拍样本。
 * @param mec_rad 输出：机械角 [0, 2π)，需已包含驱动自身的零点偏移。
 * @note  必须在中断中使用安全：不得阻塞等待长操作，不得动态分配。
 */
typedef Encoder_SampleResult_e (*Encoder_Update_fn)(float *mec_rad);

/** @brief 设置零点偏移 [rad]（可为 NULL）。 */
typedef void (*Encoder_SetZeroOffset_fn)(float offset_rad);

/** @brief 把当前机械位置写为电气零点。返回 0 表示成功（可为 NULL）。 */
typedef int  (*Encoder_SetZero_fn)(void);

/** @brief 只清软件跟踪量，不访问硬件（可为 NULL）。 */
typedef void (*Encoder_ResetTracking_fn)(void);

/** @brief 磁/信号健康（可为 NULL，NULL 视为健康）。 */
typedef bool (*Encoder_Health_fn)(void);

/** 编码器驱动描述表。 */
typedef struct {
    Encoder_Type_e type;
    const char    *name;
    uint32_t       resolution;      /**< 每圈计数                                  */
    float          max_speed_rps;   /**< 该编码器允许的最大机械转速                */

    Encoder_Init_fn          init;
    Encoder_Deinit_fn        deinit;
    Encoder_Update_fn        update;
    Encoder_SetZeroOffset_fn setZeroOffset;
    Encoder_SetZero_fn       setZero;
    Encoder_ResetTracking_fn resetTracking;
    Encoder_Health_fn        health;
} Encoder_Driver_t;

/** 服务层实例（全局单例，与参考工程的 Encoder_ops 对应）。 */
typedef struct {
    const Encoder_Driver_t *drv;    /**< 当前挂载的驱动，NULL = 未挂载           */
    Encoder_Data_t data;            /**< 输出数据，中断里实时更新                 */

    /* 配置 */
    float pole_pairs;               /**< 电机极对数                              */
    float direction;                /**< +1.0f / -1.0f                          */
    float zero_offset;              /**< 零点偏移，rad（会下发到驱动）             */

    /* 内部跟踪量 */
    float mec_last;
    bool  have_last;
    bool  initialized;

    /* 故障判定：连续取不到样本的拍数，以及累计上报 FAULT_ENCODER_MASK 的次数 */
    uint32_t fault_streak;
    uint32_t fault_posts;
} Encoder_t;

extern Encoder_t Encoder;

/*==============================================================================
 * 可注册的驱动表（供 Encoder_Attach 和直接引用）
 *============================================================================*/

extern const Encoder_Driver_t Encoder_Driver_AS5047;
#if (ENCODER_SERVICE_ENABLE_AS5600 == 1)
extern const Encoder_Driver_t Encoder_Driver_AS5600;
#endif

/*==============================================================================
 * API
 *============================================================================*/

/** @brief 复位服务层状态并挂载 ENCODER_SERVICE_DEFAULT_TYPE。上电调用一次。 */
void Encoder_ServiceInit(void);

/**
 * @brief  挂载（或切换）编码器驱动：先拆旧的，再初始化新的，成功后一次性发布。
 * @return ENCODER_OK / ENCODER_ERROR_INIT。
 * @note   可在运行中调用；发布接口时关中断，避免 FOC 中断用到半初始化的驱动。
 */
int Encoder_Attach(Encoder_Type_e type);

/** @brief 卸载当前驱动。 */
void Encoder_Detach(void);

/**
 * @brief  一次性配置常用参数（并下发零点偏移给驱动）。
 * @param  pole_pairs    电机极对数
 * @param  zero_offset   电气零点偏移，rad
 */
int Encoder_Configure(float pole_pairs, float zero_offset);

/** @brief 设置电机极对数。 */
void Encoder_SetPolePairs(float pole_pairs);

/** @brief 设置方向：+1.0f / -1.0f。只影响电角度的符号。 */
void Encoder_SetDirection(float direction);

/** @brief 设置零点偏移，rad（同时下发给驱动）。 */
void Encoder_SetZeroOffset(float zero_offset_rad);

/**
 * @brief  更新一拍。放在 FOC 采样中断里调用。
 * @return ENCODER_OK 表示 data 可用（含"本拍无新样本、沿用上一拍"的情况，
 *         此时 data.sample_fresh = false 且 data.miss_cnt 累加）；
 *         ENCODER_ERROR_COMM 表示通信失败，角度不再是可信数据；
 *         ENCODER_ERROR_INVALID_DATA 表示磁场/CORDIC 异常，角度不可信。
 * @note   无论返回值如何，data.ele 始终是最近一次有效值，中断里可以直接用，
 *         用 Encoder_IsHealthy() 决定是否要报故障。
 * @note   DMA 读取模式下，本函数同时负责"转换上一帧、发下一帧"，所以每个
 *         控制周期必须恰好调用一次（漏调会停止采样，多调会把采样周期拉长）。
 * @note   连续 ENCODER_FAULT_STREAK_N 拍拿到 ENCODER_ERROR_NOT_ATTACHED 或
 *         ENCODER_ERROR_COMM 时，本函数会调用 MotorStateMachine_PostFault()
 *         上报 FAULT_ENCODER_MASK（ISR 安全），由状态机去关功率并走恢复流程。
 *         ENCODER_ERROR_INVALID_DATA（磁场异常/CORDIC 溢出）**不**在此自动报故障，
 *         只反映在 data.status / Encoder_IsHealthy()，是否处置由应用决定。
 */
int Encoder_Update(void);

/** @brief 读电角度 [0, 2π)。中断安全，只读已更新的数据，不访问硬件。 */
int Encoder_GetElecAngle(float *ele);

/** @brief 读机械角 [0, 2π)。 */
int Encoder_GetMecAngle(float *mec);

/** @brief 读连续多圈机械角，rad。 */
int Encoder_GetTotalAngle(float *total_angle);

/** @brief 把当前位置写成电气零点（调用驱动的 SetZero，可能需要停机）。 */
int Encoder_SetZero(void);

/** @brief 只清软件跟踪量（total_angle 与计数），不访问硬件。 */
void Encoder_ResetTracking(void);

/** @brief 是否健康：已挂载、状态 OK、磁场正常。 */
bool Encoder_IsHealthy(void);

/** @brief 当前综合状态。 */
Encoder_Status_e Encoder_GetStatus(void);

/** @brief 当前挂载的编码器类型。 */
Encoder_Type_e Encoder_GetType(void);

/** @brief 当前驱动的名字（未挂载返回 "none"）。 */
const char *Encoder_GetName(void);

/**
 * @brief 当前编码器每圈计数（AS5600 = 4096，AS5047 = 16384）。
 * @note  用它来定"按分辨率"的阈值（如 PLL 死区、速度噪声门限），
 *        这样换编码器时这些阈值也自动跟着变，不用改应用层常数。
 */
uint32_t Encoder_GetResolution(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_ADAPT_H */
