/**
  ******************************************************************************
  * @file    AS5047.c
  * @brief   AMS AS5047P / AS5047D 14-bit magnetic rotary encoder driver (SPI).
  * @note    See AS5047.h for the wiring, the frame formats, the note about the
  *          GD32F303 "SPI2" == STM32F103 "SPI3" naming on PC10/PC11/PC12 and the
  *          explanation of why CS stays a software-controlled GPIO on PA15.
  ******************************************************************************
  */

#include "AS5047.h"
#include "as5047_pipeline.h"
#include <math.h>

/*==============================================================================
 * Private defines
 *============================================================================*/

#define AS5047_TWO_PI               6.28318530717958647692f
#define AS5047_PI                   3.14159265358979323846f

/* Frame bit masks. */
#define AS5047_FRAME_PARITY_MASK    0x8000U
#define AS5047_FRAME_RW_MASK        0x4000U   /* EF in a read answer                   */
#define AS5047_FRAME_ADDR_MASK      0x3FFFU

/* Default runtime configuration. */
#define AS5047_DEFAULT_POLE_PAIRS   7.0f
#define AS5047_DEFAULT_SPEED_LPF    1.0f      /* 1.0 = no filtering */
#define AS5047_DEFAULT_DIRECTION    1.0f

/* AFIO_MAPR / AFIO_PCF0 bit 28: SPI3 remap on STM32F103, SPI2 remap on
   GD32F303. Both move SCK/MISO/MOSI to PC10/PC11/PC12. */
#define AS5047_AFIO_SPI3_REMAP_BIT  (1UL << 28)

/* The register DMA backend owns SPI3 and its two DMA channels. */
#if (AS5047_DMA_SUPPORTED == 1) && (AS5047_SPI_INIT_IN_DRIVER == 1)
#define AS5047_DMA_ACTIVE           1
#else
#define AS5047_DMA_ACTIVE           0
#endif

/*==============================================================================
 * Private types
 *============================================================================*/

typedef enum {
    AS5047_BUS_IDLE = 0,      /* free                                     */
    AS5047_BUS_BLOCKING,      /* one frame in flight, caller is waiting    */
    AS5047_BUS_ASYNC          /* continuous ANGLECOM stream                */
} as5047_bus_mode_t;

/*==============================================================================
 * Private variables
 *============================================================================*/

Encoder_AS5047_t Encoder_AS5047;

#if (AS5047_SPI_INIT_IN_DRIVER == 1)
SPI_HandleTypeDef hspi3;
#endif

#if (AS5047_DMA_ACTIVE == 1)
static DMA_HandleTypeDef hdma_spi3_rx;
static DMA_HandleTypeDef hdma_spi3_tx;
#endif

static volatile as5047_bus_mode_t s_bus_mode = AS5047_BUS_IDLE;

static AS5047_Pipeline s_pipeline;
static uint32_t s_last_frame_cycles;
static uint32_t s_raw_sample_cycles;
static uint32_t s_real_cycles;
static float s_real_mech;
static bool s_have_real;
static bool s_sample_expired;

#if (AS5047_DMA_ACTIVE == 1)
static uint16_t s_async_tx;
static volatile uint16_t s_async_rx;
static volatile bool s_async_ready;
static volatile bool s_async_inflight;
static volatile int s_async_result;
static volatile uint32_t s_async_completed_cycles;
static uint32_t s_async_started_cycles;
static volatile uint32_t s_async_epoch;
#endif

/* Acquire ownership, never wait for a preempted task from an ISR. */
static bool as5047_claim(void)
{
    uint32_t key = __get_PRIMASK();
    bool ok;
    __disable_irq();
    ok = s_bus_mode == AS5047_BUS_IDLE;
    if (ok) s_bus_mode = AS5047_BUS_BLOCKING;
    __set_PRIMASK(key);
    return ok;
}

/*==============================================================================
 * Private helpers
 *============================================================================*/

/** @brief true when the number of 1-bits of the 15 low bits is odd. */
static uint16_t as5047_odd_parity(uint16_t value)
{
    value &= 0x7FFFU;
    value ^= (uint16_t)(value >> 8);
    value ^= (uint16_t)(value >> 4);
    value ^= (uint16_t)(value >> 2);
    value ^= (uint16_t)(value >> 1);
    return (uint16_t)(value & 0x0001U);
}

/** @brief Build a command frame: D15 parity, D14 R/W, D13:D0 address. */
static uint16_t as5047_build_command(uint16_t address, bool read)
{
    uint16_t frame = (uint16_t)(address & AS5047_FRAME_ADDR_MASK);

    if (read) {
        frame |= AS5047_FRAME_RW_MASK;                 /* D14 = 1 */
    }
    if (as5047_odd_parity(frame) != 0U) {
        frame |= AS5047_FRAME_PARITY_MASK;             /* D15 makes the total even */
    }
    return frame;
}

/** @brief A read of register 0x0000 is the NOP that clocks out the answer. */
static uint16_t as5047_build_nop(void)
{
    return as5047_build_command(AS5047_REG_NOP, true);  /* 0xC000 */
}

/** @brief Build a write-data frame: D15 parity, D14 = 0, D13:D0 data. */
static uint16_t as5047_build_data(uint16_t data)
{
    uint16_t frame = (uint16_t)(data & AS5047_FRAME_ADDR_MASK);

    if (as5047_odd_parity(frame) != 0U) {
        frame |= AS5047_FRAME_PARITY_MASK;
    }
    return frame;
}

/** @brief Check the even parity of a returned frame (D15 covers D14:D0). */
static bool as5047_frame_parity_ok(uint16_t frame)
{
    uint16_t expected = (uint16_t)(as5047_odd_parity(frame) << 15);

    return ((frame & AS5047_FRAME_PARITY_MASK) == expected);
}

/**
 * @brief t_CSn / t_L: the datasheet (AS5047P v2-00, p12 timing table) requires
 *        >= 350 ns between the rising edge of CSn and the next falling edge
 *        (t_CSn, "high time of CSn between two transmissions") and >= 350 ns
 *        between the falling edge of CSn and the first rising CLK edge (t_L).
 *        We wait 450 ns (~1.3x) for margin.
 *
 * @note  Why this is expressed in *cycles* instead of "a loop of N NOPs":
 *        the previous implementation ran 24 iterations of a `volatile` loop,
 *        which the compiler turns into 7 instructions with two stack accesses
 *        per iteration => ~240 cycles = **2 us**, i.e. 5.7x more than the
 *        350 ns the device asks for. With 4-6 gaps per sample that alone ate
 *        ~12 % of the 10 kHz FOC period. Deriving the count from
 *        SystemCoreClock keeps it correct across compiler settings, flash wait
 *        states and clock changes instead of depending on a cycle estimate.
 *
 *        DWT is free-running and independent of interrupt priority, so this is
 *        safe from the FOC ISR (priority 1) and from the DMA callback
 *        (priority 2). If the cycle counter is not usable we fall back to a
 *        deliberately conservative NOP loop -- slow, but never too short.
 */
#define AS5047_CS_GAP_NS            450U
#define AS5047_CS_GAP_CYCLES        ((SystemCoreClock / 1000000U) * AS5047_CS_GAP_NS / 1000U)
#define AS5047_CS_GAP_FALLBACK_NOP  16U

static void as5047_dwt_ensure(void);   /* defined below; used by as5047_cs_gap() */

static void as5047_cs_gap(void)
{
    uint32_t t0;

    if (as5047_dwt_state == 0U) {
        as5047_dwt_ensure();
    }

    if (as5047_dwt_state == 1U) {
        /* 常量先算好放寄存器：否则编译器把 (SystemCoreClock/1e6*450/1000) 的
         * 两条 umull 序列留在循环体内，每轮白烧 ~10 周期，检测粒度也从 ~8 周期
         * 粗到 ~18 周期（不影响"至少要等多久"，只是没必要）。 */
        const uint32_t gap = AS5047_CS_GAP_CYCLES;

        t0 = DWT->CYCCNT;
        uint32_t guard = 0U;
        while ((uint32_t)(DWT->CYCCNT - t0) < gap && ++guard < 128U) {
            /* busy wait, 450 ns @ 120 MHz */
        }
    } else {
        /* No usable cycle counter: must still be >= 350 ns, so stay generous. */
        volatile uint32_t i;
        for (i = 0U; i < AS5047_CS_GAP_FALLBACK_NOP; i++) {
            __NOP();
        }
    }
}

/**
 * @brief Wrap into [0, 2*pi).
 * @note  Every caller feeds a bounded value (a raw angle, a raw angle plus an
 *        offset in [0, 2*pi), or that angle times a pole-pair count), so a
 *        short subtract loop is both faster and more predictable than a call
 *        into the maths library - and it keeps the FOC ISR free of library
 *        dependencies.
 */
static float as5047_wrap_2pi(float angle)
{
    if (!isfinite(angle)) return 0.0f;
    while (angle >= AS5047_TWO_PI) {
        angle -= AS5047_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += AS5047_TWO_PI;
    }
    return angle;
}

/** @brief Measured-sample tracking only; predictions never feed back into speed. */
static void as5047_publish(float new_angle)
{
    float wrapped_delta;

    /* Shortest-path delta, so the 2*pi wrap does not look like a jump. */
    wrapped_delta = new_angle - Encoder_AS5047.angle_last;
    if (wrapped_delta > AS5047_PI) {
        wrapped_delta -= AS5047_TWO_PI;         /* crossed 2pi -> 0 backwards */
        Encoder_AS5047.rotation_circles--;
        Encoder_AS5047.rotation_circles_absolute++;
    } else if (wrapped_delta < -AS5047_PI) {
        wrapped_delta += AS5047_TWO_PI;         /* crossed 0 -> 2pi forwards */
        Encoder_AS5047.rotation_circles++;
        Encoder_AS5047.rotation_circles_absolute++;
    }

    wrapped_delta *= Encoder_AS5047.direction;
    Encoder_AS5047.angle_delta = wrapped_delta;
    Encoder_AS5047.angle_last  = new_angle;
    Encoder_AS5047.move_state  = (wrapped_delta >= 0.0f) ? 1U : 0U;

    if (wrapped_delta > 0.0f) {
        Encoder_AS5047.angle_total_ccw += wrapped_delta;
    } else if (wrapped_delta < 0.0f) {
        Encoder_AS5047.angle_total_cw += -wrapped_delta;
    }

    Encoder_AS5047.sample_seq++;
}

/**
 * @brief Convert a raw ANGLECOM sample into the whole tracking state.
 * @note  All multi-field state updates happen here, always in the context of
 *        the caller. In streaming mode the caller is the FOC ISR, which has a
 *        higher NVIC priority than the DMA callback, so the callback can never
 *        interrupt it half way through.
 */
static void as5047_track(uint16_t raw, uint32_t sample_cycles)
{
    float mech;

    mech = ((float)raw) * (AS5047_TWO_PI / (float)AS5047_RESOLUTION);
    if (s_have_real) {
        uint32_t elapsed = sample_cycles - s_real_cycles;
        float delta = mech - s_real_mech;
        if (delta > AS5047_PI) delta -= AS5047_TWO_PI;
        if (delta < -AS5047_PI) delta += AS5047_TWO_PI;
        if (elapsed != 0U && elapsed <= (SystemCoreClock / 1000000U) * AS5047_MAX_SAMPLE_AGE_US) {
            float speed = delta * ((float)SystemCoreClock / (float)elapsed) * Encoder_AS5047.direction;
            Encoder_AS5047.speed_rad_s += Encoder_AS5047.speed_lpf * (speed - Encoder_AS5047.speed_rad_s);
        } else {
            Encoder_AS5047.speed_rad_s = 0.0f;
        }
    }
    s_real_mech = mech;
    s_real_cycles = sample_cycles;
    s_have_real = true;
    s_sample_expired = false;
    Encoder_AS5047.sample_cycles = sample_cycles;
    Encoder_AS5047.valid_sample_seq++;
    Encoder_AS5047.speed_rpm = Encoder_AS5047.speed_rad_s * (60.0f / AS5047_TWO_PI);

    Encoder_AS5047.raw_angle  = raw;
    Encoder_AS5047.mech_angle = mech;
    Encoder_AS5047.angle      = as5047_wrap_2pi(mech + Encoder_AS5047.angle_zero_offset);
    Encoder_AS5047.eleangle   = as5047_wrap_2pi(Encoder_AS5047.angle *
                                                Encoder_AS5047.pole_pairs);

    Encoder_AS5047.predict_streak = 0U;   /* 真实样本回来 → 预测链结束 */

    as5047_publish(Encoder_AS5047.angle);
}

static bool as5047_sample_current(void)
{
    uint32_t age = DWT->CYCCNT - s_real_cycles;
    if (!s_have_real || s_sample_expired || as5047_dwt_state != 1U) return false;
    Encoder_AS5047.sample_age_us = age / (SystemCoreClock / 1000000U);
    if (Encoder_AS5047.sample_age_us >= AS5047_MAX_SAMPLE_AGE_US) {
        s_sample_expired = true; /* stays expired even after a DWT wrap */
        Encoder_AS5047.stale_samples++;
        return false;
    }
    return true;
}

float bsp_as5047GetControlAngle(void)
{
    if (!as5047_sample_current()) return Encoder_AS5047.angle;
    float age_s = (float)(uint32_t)(DWT->CYCCNT - s_real_cycles) / (float)SystemCoreClock;
    float direction = Encoder_AS5047.direction < 0.0f ? -1.0f : 1.0f;
    return as5047_wrap_2pi(s_real_mech + Encoder_AS5047.angle_zero_offset +
                           Encoder_AS5047.speed_rad_s * direction * age_s);
}

#if (AS5047_ENABLE_PREDICTION == 1)
static int as5047_predict(void)
{
    if (!Encoder_AS5047.initialized || !as5047_sample_current() ||
        Encoder_AS5047.predict_streak >= AS5047_PREDICT_MAX_FRAMES) {
        Encoder_AS5047.predict_blocked++;
        return AS5047_ERR_NO_DATA;
    }
    /* Leave the measured angle and measured-speed estimator untouched. */
    Encoder_AS5047.predict_streak++;
    Encoder_AS5047.predict_frames++;
    if (Encoder_AS5047.predict_streak > Encoder_AS5047.predict_max_streak)
        Encoder_AS5047.predict_max_streak = Encoder_AS5047.predict_streak;
    return AS5047_PREDICTED;
}
#endif

/*==============================================================================
 * Private helpers: bounded-timeout time base
 *============================================================================*/

/* 超时基准用 DWT 周期计数，**不用** HAL_GetTick()。
 * 本驱动是在 FOC ISR（NVIC 优先级 1）里被调用的，而 HAL 时基是 TIM6（优先级 15）——
 * TIM6 抢不过 ADC 中断，所以 ISR 里阻塞等待期间 `uwTick` 根本不前进，
 * HAL 的毫秒级超时**永远不会到期**：SPI 一旦异常（SCK 被短接、外设被误配、
 * 从机一直不返回），就会把这个 10kHz 的 FOC 中断挂死 —— 采样、编码器、快安全全停
 * （最终只能靠 IWDG 复位兜住：SafetyTask 以 g_foc_isr_cnt 是否推进作为喂狗条件）。
 * DWT->CYCCNT 是自由运行的 SystemCoreClock 周期计数器，与中断优先级无关。 */
#define AS5047_TIMEOUT_CYCLES   ((SystemCoreClock / 1000000U) * AS5047_FRAME_TIMEOUT_US)

/* 兜底：万一 DWT 周期计数器没在走（见 as5047_dwt_ensure），上面那条判据会恒不成立，
 * 就退回"最大轮询次数"——精度没了，但**保证不会死等**，而"不死等"正是本超时存在的
 * 唯一意义。每次循环至少 5~10 条指令，按 8 周期保守折算。 */
#define AS5047_TIMEOUT_POLLS    (AS5047_TIMEOUT_CYCLES / 8U)

/* 0 = 未探测，1 = 在计数，2 = 不计数（走轮询次数兜底）。非 static：RTT `get` 会打印，
 * 见 AS5047.h 的说明 —— "超时判据静默失效"是最该被看见的失败模式。 */
uint8_t as5047_dwt_state = 0U;

/** @brief 确保 DWT 周期计数器可用，并**实测**它是否真的在计数。幂等，可重复调用。
 *  @note  为什么驱动要自己管这件事：本工程里 DWT 是被 `SEGGER_RTT_PLUS_Port_Init()`
 *         （`main_user.c` 里那句 "J-Scope 调试变量曲线"）顺手使能的，
 *         而 `main_user.c` 里那个自己的 `DWT_Init()` 调用是**被注释掉的**。
 *         把**安全超时**寄托在"调试工具的初始化有没有被删掉"上太脆：
 *         一旦那行被去掉，超时判据会静默退化成死等，FOC WCET 还会静默印成 0。
 *         这里自己确保一次 + 实测，去掉这层隐式耦合。 */
static void as5047_dwt_ensure(void)
{
    uint32_t a, b, guard;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    /* 实测：读两次，中间塞足够多的 NOP，保证正常情况下的差值不可能为 0。
     * 光看 CYCCNTENA 位不够 —— 某些内核/调试状态下性能计数器会被冻结。 */
    a = DWT->CYCCNT;
    for (guard = 0U; guard < 64U; guard++) {
        __NOP();
    }
    b = DWT->CYCCNT;

    as5047_dwt_state = (b != a) ? 1U : 2U;
}

/** @brief 一次有界等待是否已超时。
 *  @param t0    起始 DWT 周期计数（只在 DWT 可用时有意义）
 *  @param polls 已轮询次数（DWT 不可用时的兜底判据） */
static inline bool as5047_timed_out(uint32_t t0, uint32_t polls)
{
    return polls > AS5047_TIMEOUT_POLLS ||
        (as5047_dwt_state == 1U && AS5047_Elapsed(DWT->CYCCNT, t0, AS5047_TIMEOUT_CYCLES));
}

/*==============================================================================
 * Frame transfer
 *============================================================================*/

/* ===== 超时基准：见文件上方的 as5047_timed_out() / as5047_dwt_ensure() ===== */

/** @brief Disable SPI and drain DR/SR to clear receive residue and OVR. */
static void as5047_spi_resync(SPI_TypeDef *spi)
{
    volatile uint32_t discard;
    spi->CR1 &= ~SPI_CR1_SPE;
    discard = spi->DR;
    discard = spi->SR; /* DR then SR clears OVR; do not assume SPE clears it. */
    (void)discard;
    spi->CR1 |= SPI_CR1_SPE;
}

/** @brief Bounded register polling; TXE is buffer-empty, BSY checks wire completion. */
static int as5047_transfer_polling(uint16_t tx_frame, uint16_t *rx_frame)
{
    SPI_TypeDef *spi = AS5047_SPI_HANDLE.Instance;
    uint32_t t0 = DWT->CYCCNT, polls = 0U;
    uint16_t rx;
    __HAL_SPI_ENABLE(&AS5047_SPI_HANDLE);
    AS5047_CS_LOW();
    as5047_cs_gap();
    while ((spi->SR & SPI_SR_TXE) == 0U)
        if (as5047_timed_out(t0, ++polls)) goto timeout;
    spi->DR = tx_frame;
    while ((spi->SR & SPI_SR_RXNE) == 0U)
        if (as5047_timed_out(t0, ++polls)) goto timeout;
    rx = (uint16_t)spi->DR;
    while ((spi->SR & SPI_SR_BSY) != 0U)
        if (as5047_timed_out(t0, ++polls)) goto timeout;
    if ((spi->SR & (SPI_SR_OVR | SPI_SR_MODF)) != 0U) goto timeout;
    as5047_cs_gap(); /* conservative last-clock to CS rising hold */
    if (as5047_timed_out(t0, polls)) goto timeout;
    AS5047_CS_HIGH();
    s_last_frame_cycles = DWT->CYCCNT;
    as5047_cs_gap();
    if (rx_frame) *rx_frame = rx;
    return AS5047_OK;
timeout:
    AS5047_CS_HIGH();
    as5047_cs_gap();
    as5047_spi_resync(spi);
    AS5047_PipelineReset(&s_pipeline);
    Encoder_AS5047.transfer_timeouts++;
    return AS5047_ERR_SPI;
}

/** @brief Bounded polling for generic register access in both builds. */
static int as5047_transfer(uint16_t tx_frame, uint16_t *rx_frame)
{
    return as5047_transfer_polling(tx_frame, rx_frame);
}

/**
 * @brief Read one register and return the raw 16-bit answer frame.
 *
 * The device pipelines its answers: the frame returned by the command transfer
 * belongs to the command issued before, so a NOP frame is needed to collect the
 * answer to the command we just sent.
 */
static int as5047_read_register_raw(uint16_t address, uint16_t *frame)
{
    uint16_t discard;
    int ret;
    if (!as5047_claim()) return AS5047_ERR_BUSY;
    AS5047_PipelineReset(&s_pipeline);
    ret = as5047_transfer(as5047_build_command(address, true), &discard);
    s_raw_sample_cycles = s_last_frame_cycles;
    if (ret == AS5047_OK) ret = as5047_transfer(as5047_build_nop(), frame);
    s_bus_mode = AS5047_BUS_IDLE;
    return ret;
}

/** @brief Drop any stale answer left in the pipeline after an error. */
static void as5047_flush_pipeline(void)
{
    uint16_t discard;
    if (__get_IPSR() != 0U || !as5047_claim()) return;
    AS5047_PipelineReset(&s_pipeline);
    if (as5047_transfer(as5047_build_nop(), &discard) == AS5047_OK)
        (void)as5047_transfer(as5047_build_nop(), &discard);
    s_bus_mode = AS5047_BUS_IDLE;
}

/*==============================================================================
 * Bus bring-up
 *============================================================================*/

#if (AS5047_SPI_INIT_IN_DRIVER == 1)

#if (AS5047_DMA_ACTIVE == 1)
static void as5047_dma_init(void)
{
    /* GD32F303 calls this controller "DMA1"; the clock bit is the same (AHBEN
       bit 1), so __HAL_RCC_DMA2_CLK_ENABLE() is correct on both parts. */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* RX: peripheral -> memory. Half-word alignment is mandatory: the SPI data
       size is 16 bit and HAL_SPI_TransmitReceive_DMA() does not adapt the DMA
       alignment by itself. */
    hdma_spi3_rx.Instance                 = AS5047_DMA_RX_CHANNEL;
    hdma_spi3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc              = DMA_MINC_DISABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_spi3_rx.Init.Mode                = DMA_NORMAL;
    hdma_spi3_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi3, hdmarx, hdma_spi3_rx);

    /* TX: memory -> peripheral. */
    hdma_spi3_tx.Instance                 = AS5047_DMA_TX_CHANNEL;
    hdma_spi3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_spi3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi3_tx.Init.MemInc              = DMA_MINC_DISABLE;
    hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_spi3_tx.Init.Mode                = DMA_NORMAL;
    hdma_spi3_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi3, hdmatx, hdma_spi3_tx);

    /* RX complete + RX/TX error interrupts; TX completion is disabled at kick. */
    HAL_NVIC_SetPriority(AS5047_DMA_RX_IRQn, AS5047_DMA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(AS5047_DMA_RX_IRQn);
    HAL_NVIC_SetPriority(AS5047_DMA_TX_IRQn, AS5047_DMA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(AS5047_DMA_TX_IRQn);
}
#endif /* AS5047_DMA_ACTIVE */

void bsp_as5047SpiInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* The AFIO clock must be on before the remap register can be written. */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * SPI3 remap moves SCK/MISO/MOSI from PB3/PB4/PB5 to PC10/PC11/PC12.
     *
     * On the GD32F303RET6 this board actually uses, the very same function is
     * called "SPI2 remap" and lives in AFIO_PCF0 bit 28, so one and the same
     * register bit selects these three pins on both parts, and
     * __HAL_RCC_SPI3_CLK_ENABLE() maps to GD32 RCU_APB1EN bit 15 == RCU_SPI2EN.
     *
     * stm32f103xe.h does not define AFIO_MAPR_SPI3_REMAP (STM32's own note says
     * the bit is documented for connectivity-line devices only), which also
     * means __HAL_AFIO_REMAP_SPI3_ENABLE() is compiled out for this part. The
     * bit is therefore written directly with the HAL helper that keeps the
     * write-only SWJ_CFG field unchanged, so the NOJTAG configuration done in
     * HAL_MspInit() (needed for CS on PA15) is preserved.
     */
#if defined(AFIO_MAPR_SPI3_REMAP)
    __HAL_AFIO_REMAP_SPI3_ENABLE();
#else
    AFIO_REMAP_ENABLE(AS5047_AFIO_SPI3_REMAP_BIT);
#endif

    /* SCK (PC10) and MOSI (PC12): alternate function push-pull.
       This intentionally replaces the CubeMX GPIO_Output setting. */
    GPIO_InitStruct.Pin   = AS5047_SCK_Pin | AS5047_MOSI_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AS5047_SCK_GPIO_Port, &GPIO_InitStruct);

    /* MISO (PC11): input floating. */
    GPIO_InitStruct.Pin  = AS5047_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AS5047_MISO_GPIO_Port, &GPIO_InitStruct);

    /* CS (PA15) as a plain GPIO output, idle high. */
    AS5047_CS_HIGH();
    GPIO_InitStruct.Pin   = AS5047_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AS5047_CS_GPIO_Port, &GPIO_InitStruct);

    /* SPI3 peripheral. */
    __HAL_RCC_SPI3_CLK_ENABLE();

    hspi3.Instance               = SPI3;
    hspi3.Init.Mode              = SPI_MODE_MASTER;
    hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize          = SPI_DATASIZE_16BIT;
    hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* CPOL = 0  */
    hspi3.Init.CLKPhase          = SPI_PHASE_2EDGE;     /* CPHA = 1 -> mode 1 */
    hspi3.Init.NSS               = SPI_NSS_SOFT;        /* SSM + SSI, PA15 stays GPIO */
    hspi3.Init.BaudRatePrescaler = AS5047_SPI_PRESCALER;
    hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial     = 10U;

    /*
     * HAL_SPI_Init() calls HAL_SPI_MspInit(), and the CubeMX generated
     * HAL_SPI_MspInit() in Core/Src/spi.c only handles SPI2, so the GPIO and
     * clock setup done above is what actually takes effect. This keeps the
     * driver independent of Core/Src/spi.c and safe against CubeMX
     * regeneration. If SPI3 is ever enabled in CubeMX, either delete
     * MX_SPI3_Init() from main.c or set AS5047_SPI_INIT_IN_DRIVER to 0 and let
     * MX_SPI3_Init() own the handle.
     */
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }

    /* HAL_SPI_Init() 结束时 **SPE 仍为 0**（见 as5047_transfer_polling 的说明）。
     * 这里显式使能一次，让上电后的外设状态确定；每帧传输前还会再检查一次，
     * 以兜住 HAL_SPI_Abort()/HAL_SPI_DMAStop() 把 SPE 清掉的情况。 */
    __HAL_SPI_ENABLE(&hspi3);

#if (AS5047_DMA_ACTIVE == 1)
    as5047_dma_init();
#endif
}

#else /* AS5047_SPI_INIT_IN_DRIVER == 0 */

void bsp_as5047SpiInit(void)
{
    /* SPI instance is created elsewhere (e.g. MX_SPI3_Init()).
       Only make sure CS idles high so the device is deselected. */
    AS5047_CS_HIGH();
}

#endif /* AS5047_SPI_INIT_IN_DRIVER */

/*==============================================================================
 * Register access
 *============================================================================*/

int bsp_as5047ReadFrameRaw(uint16_t address, uint16_t *frame)
{
    if (frame == NULL) {
        return AS5047_ERR_PARAM;
    }
    return as5047_read_register_raw(address, frame);
}

int bsp_as5047ReadRegister(uint16_t address, uint16_t *data)
{
    uint16_t frame = 0U;
    int ret;

    if (data == NULL) {
        return AS5047_ERR_PARAM;
    }

    ret = as5047_read_register_raw(address, &frame);
    if (ret == AS5047_ERR_BUSY) {
        return ret;
    }
    if (ret != AS5047_OK) {
        Encoder_AS5047.comm_errors++;
        as5047_flush_pipeline();
        return ret;
    }

    if (!as5047_frame_parity_ok(frame)) {
        Encoder_AS5047.parity_errors++;
        return AS5047_ERR_PARITY;
    }

    *data = (uint16_t)(frame & AS5047_FRAME_ADDR_MASK);

    /* EF describes the frame *before* this answer, i.e. our command frame. */
    if ((frame & AS5047_FRAME_RW_MASK) != 0U) {
        Encoder_AS5047.device_flags++;
        return AS5047_ERR_DEVICE_FLAG;
    }
    return AS5047_OK;
}

static int as5047_write_register_owned(uint16_t address, uint16_t data)
{
    uint16_t discarded = 0U;
    uint16_t frame = 0U;
    int ret;

    if (s_bus_mode == AS5047_BUS_ASYNC) {
        return AS5047_ERR_BUSY;
    }

    /*
     * A write is two frames that both belong to the same instruction:
     * first the write command, then the 16-bit data frame. The data frame is
     * followed by a NOP so the device is ready for the next command.
     */
    ret = as5047_transfer(as5047_build_command(address, false), &discarded);
    if (ret != AS5047_OK) {
        Encoder_AS5047.comm_errors++;
        as5047_flush_pipeline();
        return ret;
    }

    ret = as5047_transfer(as5047_build_data(data), &frame);
    if (ret != AS5047_OK) {
        Encoder_AS5047.comm_errors++;
        as5047_flush_pipeline();
        return ret;
    }

    (void)as5047_transfer(as5047_build_nop(), &discarded);

    if (!as5047_frame_parity_ok(frame)) {
        Encoder_AS5047.parity_errors++;
        return AS5047_ERR_PARITY;
    }
    return AS5047_OK;
}

int bsp_as5047WriteRegister(uint16_t address, uint16_t data)
{
    int ret;
    if (!as5047_claim()) return AS5047_ERR_BUSY;
    AS5047_PipelineReset(&s_pipeline);
    ret = as5047_write_register_owned(address, data);
    s_bus_mode = AS5047_BUS_IDLE;
    return ret;
}

int bsp_as5047ReadAngleRaw(uint16_t *raw_angle)
{
    uint16_t frame = 0U;
    int ret;

    if (raw_angle == NULL) {
        return AS5047_ERR_PARAM;
    }

    ret = as5047_read_register_raw(AS5047_REG_ANGLECOM, &frame);
    if (ret == AS5047_ERR_BUSY) {
        return ret;
    }
    if (ret != AS5047_OK) {
        Encoder_AS5047.comm_errors++;
        as5047_flush_pipeline();
        return ret;
    }

    if (!as5047_frame_parity_ok(frame)) {
        Encoder_AS5047.parity_errors++;
        return AS5047_ERR_PARITY;
    }

    if ((frame & AS5047_FRAME_RW_MASK) != 0U) {
        Encoder_AS5047.device_flags++;
        return AS5047_ERR_DEVICE_FLAG;
    }

    *raw_angle = (uint16_t)(frame & AS5047_FRAME_ADDR_MASK);
    Encoder_AS5047.raw_angle = *raw_angle;
    return AS5047_OK;
}

int bsp_as5047ReadAngleUncompensated(uint16_t *raw_angle)
{
    uint16_t frame = 0U;
    int ret;

    if (raw_angle == NULL) {
        return AS5047_ERR_PARAM;
    }

    ret = as5047_read_register_raw(AS5047_REG_ANGLEUNC, &frame);
    if (ret == AS5047_ERR_BUSY) {
        return ret;
    }
    if (ret != AS5047_OK) {
        Encoder_AS5047.comm_errors++;
        as5047_flush_pipeline();
        return ret;
    }
    if (!as5047_frame_parity_ok(frame)) {
        Encoder_AS5047.parity_errors++;
        return AS5047_ERR_PARITY;
    }

    *raw_angle = (uint16_t)(frame & AS5047_FRAME_ADDR_MASK);
    Encoder_AS5047.raw_angle_unc = *raw_angle;
    return AS5047_OK;
}

int bsp_as5047ReadErrorFlags(uint16_t *flags)
{
    uint16_t value = 0U;
    int ret;

    if (flags == NULL) {
        return AS5047_ERR_PARAM;
    }

    /* Reading ERRFL clears the flags inside the device. */
    ret = bsp_as5047ReadRegister(AS5047_REG_ERRFL, &value);
    Encoder_AS5047.error_flags = value;
    *flags = value;

    /* AS5047_ERR_DEVICE_FLAG from this very read is the normal "there was an
       error" indication, so it is reported as success here. */
    if (ret == AS5047_ERR_DEVICE_FLAG) {
        ret = AS5047_OK;
    }
    return ret;
}

int bsp_as5047ReadDiagnostics(uint16_t *diaagc)
{
    uint16_t value = 0U;
    int ret;

    ret = bsp_as5047ReadRegister(AS5047_REG_DIAAGC, &value);
    if (ret == AS5047_ERR_DEVICE_FLAG) {
        ret = AS5047_OK;
    }
    if (ret != AS5047_OK) {
        return ret;
    }

    Encoder_AS5047.diaagc = value;
    Encoder_AS5047.diagnostics_valid = true;
    Encoder_AS5047.agc = (uint8_t)(value & AS5047_DIAAGC_AGC_MASK);
    Encoder_AS5047.cordic_overflow  = ((value & AS5047_DIAAGC_COF_MASK) != 0U);
    Encoder_AS5047.offset_loop_settled = ((value & AS5047_DIAAGC_LF_MASK) != 0U);

    if ((value & AS5047_DIAAGC_MAGL_MASK) != 0U) {
        Encoder_AS5047.magnet_status = AS5047_MAGNET_TOO_WEAK;
    } else if ((value & AS5047_DIAAGC_MAGH_MASK) != 0U) {
        Encoder_AS5047.magnet_status = AS5047_MAGNET_TOO_STRONG;
    } else {
        Encoder_AS5047.magnet_status = AS5047_MAGNET_OK;
    }

    if (diaagc != NULL) {
        *diaagc = value;
    }
    return AS5047_OK;
}

int bsp_as5047ReadMagnitude(uint16_t *mag)
{
    uint16_t value = 0U;
    int ret;

    ret = bsp_as5047ReadRegister(AS5047_REG_MAG, &value);
    if (ret == AS5047_ERR_DEVICE_FLAG) {
        ret = AS5047_OK;
    }
    if (ret != AS5047_OK) {
        return ret;
    }

    Encoder_AS5047.mag = value;
    if (mag != NULL) {
        *mag = value;
    }
    return AS5047_OK;
}

int bsp_as5047ReadAllDiagnostics(void)
{
    uint16_t errfl = 0U;
    uint16_t diaagc = 0U;
    uint16_t mag = 0U;
    int ret;
    int first_error = AS5047_OK;

    ret = bsp_as5047ReadErrorFlags(&errfl);
    if (ret != AS5047_OK) {
        first_error = ret;
    }
    ret = bsp_as5047ReadDiagnostics(&diaagc);
    if ((ret != AS5047_OK) && (first_error == AS5047_OK)) {
        first_error = ret;
    }
    ret = bsp_as5047ReadMagnitude(&mag);
    if ((ret != AS5047_OK) && (first_error == AS5047_OK)) {
        first_error = ret;
    }
    return first_error;
}

/*==============================================================================
 * Angle tracking
 *============================================================================*/

void bsp_as5047ResetTracking(void)
{
    /* Safe at run-state transitions: performs no SPI access. */
    Encoder_AS5047.angle_last = Encoder_AS5047.angle;
    Encoder_AS5047.angle_delta = 0.0f;
    Encoder_AS5047.speed_rad_s = 0.0f;
    Encoder_AS5047.speed_rpm = 0.0f;
    Encoder_AS5047.move_state = 0U;
    Encoder_AS5047.rotation_circles = 0;
    Encoder_AS5047.rotation_circles_absolute = 0U;
    Encoder_AS5047.angle_total_ccw = 0.0f;
    Encoder_AS5047.angle_total_cw = 0.0f;
    Encoder_AS5047.sample_misses = 0U;
    /* 预测链也要断开：速度已被清零，继续按旧速度外推没有意义。 */
    Encoder_AS5047.predict_streak = 0U;
}

void bsp_as5047SetZeroOffset(float offset_rad)
{
    Encoder_AS5047.angle_zero_offset = as5047_wrap_2pi(offset_rad);
}

void bsp_as5047SetPolePairs(float pole_pairs)
{
    if (pole_pairs > 0.0f) {
        Encoder_AS5047.pole_pairs = pole_pairs;
    }
}

void bsp_as5047SetSampleTime(float sample_time_s)
{
    Encoder_AS5047.sample_time_s = (sample_time_s > 0.0f) ? sample_time_s : 0.0f;
}

void bsp_as5047SetDirection(float direction)
{
    Encoder_AS5047.direction = (direction < 0.0f) ? -1.0f : 1.0f;
}

void bsp_as5047SetSpeedLpf(float speed_lpf)
{
    if (speed_lpf < 0.0f) {
        speed_lpf = 0.0f;
    } else if (speed_lpf > 1.0f) {
        speed_lpf = 1.0f;
    }
    Encoder_AS5047.speed_lpf = speed_lpf;
}

static int as5047_consume(uint16_t command, uint16_t frame, uint32_t completed)
{
    uint16_t raw = 0U;
    uint32_t stamp = 0U;
    int result = AS5047_PipelineAccept(&s_pipeline, command, frame, completed, &raw, &stamp);
    if (result == AS5047_PIPE_ANGLE) {
        if (AS5047_Elapsed(DWT->CYCCNT, stamp,
            (SystemCoreClock / 1000000U) * AS5047_MAX_SAMPLE_AGE_US)) {
            Encoder_AS5047.stale_samples++;
            return AS5047_ERR_NO_DATA;
        }
        as5047_track(raw, stamp);
        return AS5047_OK;
    }
    if (result == AS5047_PIPE_ERROR_FLAGS) Encoder_AS5047.error_flags = raw;
    if (result == AS5047_PIPE_PARITY) Encoder_AS5047.parity_errors++;
    if (result == AS5047_PIPE_EF) Encoder_AS5047.device_flags++;
    if (result == AS5047_PIPE_WARMUP) Encoder_AS5047.priming_frames++;
    return AS5047_ERR_NO_DATA;
}

static int as5047_finish_sample(int result, volatile float *angle, uint32_t started)
{
    if (result != AS5047_OK) {
        Encoder_AS5047.sample_misses++;
#if (AS5047_ENABLE_PREDICTION == 1)
        if (as5047_predict() == AS5047_PREDICTED) result = AS5047_PREDICTED;
#endif
    }
    if (angle) *angle = bsp_as5047GetControlAngle();
    Encoder_AS5047.read_cycles = DWT->CYCCNT - started;
    if (Encoder_AS5047.read_cycles > Encoder_AS5047.max_read_cycles)
        Encoder_AS5047.max_read_cycles = Encoder_AS5047.read_cycles;
    return result;
}

int bsp_as5047GetAngle(volatile float *angle)
{
    uint32_t started = DWT->CYCCNT;
    uint16_t rx, command;
    int ret;
    if (!Encoder_AS5047.initialized) return AS5047_ERR_NOT_INIT;
    if (!as5047_claim()) return AS5047_ERR_BUSY;
    command = AS5047_PipelineCommand(&s_pipeline);
    ret = as5047_transfer_polling(command, &rx);
    if (ret == AS5047_OK) ret = as5047_consume(command, rx, s_last_frame_cycles);
    else Encoder_AS5047.comm_errors++;
    s_bus_mode = AS5047_BUS_IDLE;
    return as5047_finish_sample(ret, angle, started);
}

int bsp_as5047GetAngleImmediate(volatile float *angle)
{
    uint16_t raw;
    int ret;
    if (__get_IPSR() != 0U) return AS5047_ERR_BUSY;
    if (!Encoder_AS5047.initialized) return AS5047_ERR_NOT_INIT;
#if (AS5047_DMA_ACTIVE == 1)
    bool restart = bsp_as5047AsyncIsRunning();
    if (restart) bsp_as5047AsyncStop();
#endif
    ret = bsp_as5047ReadAngleRaw(&raw);
    if (ret == AS5047_OK) {
        as5047_track(raw, s_raw_sample_cycles);
        if (angle) *angle = Encoder_AS5047.angle;
    }
#if (AS5047_DMA_ACTIVE == 1)
    if (restart) {
        int start_ret = bsp_as5047AsyncStart();
        if (ret == AS5047_OK) ret = start_ret;
    }
#endif
    return ret;
}

int bsp_as5047Update(void)
{
    return bsp_as5047GetAngle(NULL);
}

/*==============================================================================
 * Zero position
 *============================================================================*/

int bsp_as5047SetZeroPosition(uint16_t position)
{
    int ret;

    position = (uint16_t)(position & AS5047_FRAME_ADDR_MASK);

    /* ZPOSM holds the 8 MSB, ZPOSL the 6 LSB of the 14-bit zero position. */
    ret = bsp_as5047WriteRegister(AS5047_REG_ZPOSM, (uint16_t)(position >> 6));
    if (ret != AS5047_OK) {
        return ret;
    }
    ret = bsp_as5047WriteRegister(AS5047_REG_ZPOSL, (uint16_t)(position & 0x003FU));
    if (ret != AS5047_OK) {
        return ret;
    }

    /* Datasheet: allow the new zero position to settle. */
    HAL_Delay(1U);
    as5047_flush_pipeline();
    return AS5047_OK;
}

int bsp_as5047SetZeroHere(void)
{
    uint16_t raw = 0U;
    int ret;

    ret = bsp_as5047ReadAngleRaw(&raw);
    if (ret != AS5047_OK) {
        return ret;
    }
    return bsp_as5047SetZeroPosition(raw);
}

AS5047_MagnetStatus_t bsp_as5047GetMagnetStatus(void)
{
    return Encoder_AS5047.magnet_status;
}

bool bsp_as5047IsMagnetOk(void)
{
    return (Encoder_AS5047.magnet_status == AS5047_MAGNET_OK);
}

bool bsp_as5047IsAngleValid(void)
{
    return (Encoder_AS5047.diagnostics_valid && !Encoder_AS5047.cordic_overflow);
}

bool bsp_as5047IsOffsetLoopSettled(void)
{
    return Encoder_AS5047.offset_loop_settled;
}

uint8_t bsp_as5047GetAgc(void)
{
    return Encoder_AS5047.agc;
}

/*==============================================================================
 * DMA transport: FOC owns scheduling/protocol; IRQ only completes one frame.
 * Dedicated register path avoids HAL's tick-based waits in its DMA callback.
 *============================================================================*/
#if (AS5047_DMA_ACTIVE == 1)
static void as5047_dma_cancel_locked(void)
{
    SPI_TypeDef *spi = AS5047_SPI_HANDLE.Instance;
    spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    AS5047_DMA_RX_CHANNEL->CCR &= ~DMA_CCR_EN;
    AS5047_DMA_TX_CHANNEL->CCR &= ~DMA_CCR_EN;
    DMA2->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
    s_async_epoch++;
    s_async_inflight = false;
    s_async_ready = false;
    AS5047_CS_HIGH();
    as5047_spi_resync(spi);
}

static void as5047_async_kick(void)
{
    SPI_TypeDef *spi = AS5047_SPI_HANDLE.Instance;
    if (s_async_inflight || s_async_ready || s_bus_mode != AS5047_BUS_ASYNC) return;
    s_async_tx = AS5047_PipelineCommand(&s_pipeline);
    AS5047_DMA_RX_CHANNEL->CCR &= ~DMA_CCR_EN;
    AS5047_DMA_TX_CHANNEL->CCR &= ~DMA_CCR_EN;
    DMA2->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
    AS5047_DMA_RX_CHANNEL->CPAR = (uint32_t)&spi->DR;
    AS5047_DMA_RX_CHANNEL->CMAR = (uint32_t)&s_async_rx;
    AS5047_DMA_RX_CHANNEL->CNDTR = 1U;
    AS5047_DMA_TX_CHANNEL->CPAR = (uint32_t)&spi->DR;
    AS5047_DMA_TX_CHANNEL->CMAR = (uint32_t)&s_async_tx;
    AS5047_DMA_TX_CHANNEL->CNDTR = 1U;
    /* Only RX completion and either channel's transfer error interrupt. */
    AS5047_DMA_RX_CHANNEL->CCR = DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
        DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;
    AS5047_DMA_TX_CHANNEL->CCR = DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |
        DMA_CCR_PL_1 | DMA_CCR_DIR | DMA_CCR_TEIE;
    __HAL_SPI_ENABLE(&AS5047_SPI_HANDLE);
    AS5047_CS_HIGH();
    as5047_cs_gap();
    AS5047_CS_LOW();
    as5047_cs_gap();
    s_async_started_cycles = DWT->CYCCNT;
    s_async_epoch++;
    s_async_inflight = true; /* publish ownership BEFORE enabling requests */
    __DMB();
    AS5047_DMA_RX_CHANNEL->CCR |= DMA_CCR_EN;
    spi->CR2 |= SPI_CR2_RXDMAEN;
    AS5047_DMA_TX_CHANNEL->CCR |= DMA_CCR_EN;
    spi->CR2 |= SPI_CR2_TXDMAEN;
}

int bsp_as5047AsyncStart(void)
{
    if (__get_IPSR() != 0U) return AS5047_ERR_BUSY;
    if (!Encoder_AS5047.initialized) return AS5047_ERR_NOT_INIT;
    if (s_bus_mode == AS5047_BUS_ASYNC) return AS5047_OK;
    if (!as5047_claim()) return AS5047_ERR_BUSY;
    AS5047_PipelineReset(&s_pipeline);
    s_async_ready = false;
    s_async_inflight = false;
    Encoder_AS5047.async_stopped = false;
    Encoder_AS5047.async_error_streak = 0U;
    s_bus_mode = AS5047_BUS_ASYNC;
    as5047_async_kick();
    return AS5047_OK;
}

void bsp_as5047AsyncStop(void)
{
    /* Caller must quiesce the consumer before stopping/reinitializing. */
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    if (s_bus_mode == AS5047_BUS_ASYNC) {
        as5047_dma_cancel_locked();
        s_bus_mode = AS5047_BUS_IDLE;
        AS5047_PipelineReset(&s_pipeline);
    }
    __set_PRIMASK(key);
    as5047_cs_gap();
}

bool bsp_as5047AsyncIsRunning(void) { return s_bus_mode == AS5047_BUS_ASYNC; }
bool bsp_as5047AsyncDataReady(void) { return s_async_ready; }

int bsp_as5047AsyncFetch(volatile float *angle)
{
    uint32_t started = DWT->CYCCNT;
    uint16_t rx = 0U, command = 0U;
    uint32_t completed = 0U;
    int ret = AS5047_ERR_NO_DATA;
    bool ready;
    uint32_t key;
    if (s_bus_mode != AS5047_BUS_ASYNC) return AS5047_ERR_NOT_INIT;
    key = __get_PRIMASK();
    __disable_irq();
    ready = s_async_ready;
    if (ready) {
        __DMB();
        rx = s_async_rx;
        command = s_async_tx;
        completed = s_async_completed_cycles;
        ret = s_async_result;
        s_async_ready = false;
    } else if (s_async_inflight && AS5047_Elapsed(DWT->CYCCNT,
        s_async_started_cycles, (SystemCoreClock / 1000000U) * AS5047_DMA_TIMEOUT_US)) {
        as5047_dma_cancel_locked();
        AS5047_PipelineReset(&s_pipeline);
        Encoder_AS5047.transfer_timeouts++;
        ret = AS5047_ERR_SPI;
    }
    __set_PRIMASK(key);
    if (ready && ret == AS5047_OK) ret = as5047_consume(command, rx, completed);
    if (ret == AS5047_ERR_SPI) {
        AS5047_PipelineReset(&s_pipeline);
        Encoder_AS5047.comm_errors++;
    }
    if (ret == AS5047_OK) Encoder_AS5047.async_error_streak = 0U;
    else Encoder_AS5047.async_error_streak++;
    if (Encoder_AS5047.async_error_streak >= AS5047_ASYNC_MAX_ERROR_STREAK) {
        bsp_as5047AsyncStop();
        Encoder_AS5047.async_stopped = true;
    } else {
        as5047_async_kick();
    }
    return as5047_finish_sample(ret, angle, started);
}

static void as5047_dma_irq(void)
{
    uint32_t flags = DMA2->ISR;
    uint32_t epoch = s_async_epoch;
    uint32_t t0 = DWT->CYCCNT, polls = 0U;
    uint32_t key;
    int result = AS5047_OK;
    SPI_TypeDef *spi = AS5047_SPI_HANDLE.Instance;
    if ((flags & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) == 0U) return;
    if ((flags & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) != 0U) result = AS5047_ERR_SPI;
    while (result == AS5047_OK && (spi->SR & SPI_SR_BSY) != 0U) {
        if (as5047_timed_out(t0, ++polls)) result = AS5047_ERR_SPI;
    }
    if ((spi->SR & (SPI_SR_OVR | SPI_SR_MODF)) != 0U) result = AS5047_ERR_SPI;
    if (AS5047_Elapsed(DWT->CYCCNT, s_async_started_cycles,
        (SystemCoreClock / 1000000U) * AS5047_DMA_TIMEOUT_US)) result = AS5047_ERR_SPI;
    as5047_cs_gap(); /* hold after final SPI clock */
    key = __get_PRIMASK();
    __disable_irq();
    /* A preempting FOC may have timed out and cancelled this transfer. */
    if (s_bus_mode == AS5047_BUS_ASYNC && s_async_inflight && epoch == s_async_epoch) {
        spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        AS5047_DMA_RX_CHANNEL->CCR &= ~DMA_CCR_EN;
        AS5047_DMA_TX_CHANNEL->CCR &= ~DMA_CCR_EN;
        DMA2->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
        AS5047_CS_HIGH();
        s_async_completed_cycles = DWT->CYCCNT;
        if (result != AS5047_OK) as5047_spi_resync(spi);
        s_async_result = result;
        Encoder_AS5047.async_frames++;
        __DMB();
        s_async_ready = true;
        s_async_inflight = false;
    }
    __set_PRIMASK(key);
}

#if (AS5047_DMA_IRQ_HANDLERS_IN_DRIVER == 1)
void DMA2_Channel1_IRQHandler(void) { as5047_dma_irq(); }
void DMA2_Channel2_IRQHandler(void) { as5047_dma_irq(); }
#else
#error "AS5047 register DMA backend requires driver-owned IRQ handlers"
#endif
#endif

/*==============================================================================
 * Start-up
 *============================================================================*/

int bsp_as5047HwInit(void)
{
    uint16_t frame = 0U;
    int ret;
    int first_error = AS5047_OK;

    /* Zero-initialise the runtime state but keep the compile-time defaults. */
    Encoder_AS5047.raw_angle = 0U;
    Encoder_AS5047.raw_angle_unc = 0U;
    Encoder_AS5047.error_flags = 0U;
    Encoder_AS5047.diaagc = 0U;
    Encoder_AS5047.mag = 0U;
    Encoder_AS5047.angle = 0.0f;
    Encoder_AS5047.mech_angle = 0.0f;
    Encoder_AS5047.angle_last = 0.0f;
    Encoder_AS5047.eleangle = 0.0f;
    Encoder_AS5047.angle_zero_offset = 0.0f;
    Encoder_AS5047.angle_delta = 0.0f;
    Encoder_AS5047.speed_rad_s = 0.0f;
    Encoder_AS5047.speed_rpm = 0.0f;
    Encoder_AS5047.pole_pairs = AS5047_DEFAULT_POLE_PAIRS;
    Encoder_AS5047.direction = AS5047_DEFAULT_DIRECTION;
    Encoder_AS5047.speed_lpf = AS5047_DEFAULT_SPEED_LPF;
    Encoder_AS5047.sample_time_s = 0.0f;
    Encoder_AS5047.magnet_status = AS5047_MAGNET_UNKNOWN;
    Encoder_AS5047.diagnostics_valid = false;
    Encoder_AS5047.cordic_overflow = false;
    Encoder_AS5047.offset_loop_settled = false;
    Encoder_AS5047.agc = 0U;
    Encoder_AS5047.comm_errors = 0U;
    Encoder_AS5047.parity_errors = 0U;
    Encoder_AS5047.device_flags = 0U;
    Encoder_AS5047.sample_seq = 0U;
    Encoder_AS5047.async_frames = 0U;
    Encoder_AS5047.async_error_streak = 0U;
    Encoder_AS5047.async_stopped = false;
    Encoder_AS5047.initialized = false;
    bsp_as5047ResetTracking();

    s_have_real = false;
    s_sample_expired = false;
    AS5047_PipelineReset(&s_pipeline);
    Encoder_AS5047.valid_sample_seq = 0U;
    Encoder_AS5047.transfer_timeouts = 0U;
    Encoder_AS5047.stale_samples = 0U;
    Encoder_AS5047.priming_frames = 0U;
    Encoder_AS5047.read_cycles = 0U;
    Encoder_AS5047.max_read_cycles = 0U;
    as5047_dwt_ensure();
    if (as5047_dwt_state != 1U) return AS5047_ERR_SPI;
    bsp_as5047SpiInit();
    as5047_flush_pipeline();

    /* 在所有 SPI 通信**之前**把超时用的 DWT 时基准备好并实测一次。
     * 这样第一次传输就不用付 64 个 NOP 的探测开销，而且"时基不可用"这件事
     * 在上电初始化阶段就暴露出来（可打印），而不是等到某次 SPI 异常时才发现超时判据失效。 */
    as5047_dwt_ensure();

    /* Parity alone cannot prove presence: 0x0000 and 0xFFFF both have even parity.
     * Also validate cached diagnostics below. */
    ret = bsp_as5047ReadFrameRaw(AS5047_REG_ERRFL, &frame);
    if (ret != AS5047_OK) {
        return ret;
    }
    if (!as5047_frame_parity_ok(frame)) {
        Encoder_AS5047.parity_errors++;
        return AS5047_ERR_PARITY;
    }

    /* Clear any power-on error flags, then cache the health registers. */
    ret = bsp_as5047ReadAllDiagnostics();
    if ((ret != AS5047_OK) && (first_error == AS5047_OK)) {
        first_error = ret;
    }

    /* Acquire the first angle sample and prime the tracking state. */
    ret = bsp_as5047ReadAngleRaw(&frame);
    if (ret == AS5047_OK) as5047_track(frame, s_raw_sample_cycles);
    if (ret != AS5047_OK) {
        return ret;
    }
    Encoder_AS5047.angle_last = Encoder_AS5047.angle;
    Encoder_AS5047.initialized = true;

    return first_error;
}
