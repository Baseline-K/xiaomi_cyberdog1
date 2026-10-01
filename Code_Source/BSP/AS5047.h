/**
  ******************************************************************************
  * @file    AS5047.h
  * @brief   AMS AS5047P / AS5047D 14-bit magnetic rotary encoder (SPI) driver.
  *
  *          Hardware wiring on this board (Xiaomi_CyberDog1 / GD32F303RET6):
  *            - Encoder SPI: PC10 = SCK, PC11 = MISO, PC12 = MOSI
  *            - Encoder CS : PA15
  *          These pins are reserved for the encoder; the CubeMX-generated
  *          GPIO_Output configuration of PC10/PC11/PC12/PA15 is intentionally
  *          overridden by bsp_as5047SpiInit() (which runs after MX_GPIO_Init()).
  *
  *          NOTE / IMPORTANT
  *          PC10/PC11/PC12 are NOT SPI2 pins. On both STM32F103 and GD32F303
  *          these three pins belong to the *third* SPI instance:
  *            - STM32F103 naming : SPI3  (AFIO_MAPR bit 28 = SPI3_REMAP)
  *            - GD32F303  naming : SPI2  (AFIO_PCF0 bit 28 = SPI2_REMAP)
  *          The AFIO remap bit (bit 28) and the APB1 clock-enable bit (bit 15)
  *          are bit-identical on both parts, and GD32's SPI_CTL0/CTL1/STAT/DATA
  *          are layout-compatible with STM32's SPI_CR1/CR2/SR/DR, so the stock
  *          STM32F1 HAL SPI driver can drive this peripheral directly.
  *          In this project the handle is therefore named `hspi3`.
  *
  *          The other SPI in this project (hspi2 = PB13/PB14/PB15, CS = PB12)
  *          belongs to the DRV8353 gate driver and is NOT shared with the
  *          encoder.
  *
  *          CS / NSS: why software NSS on PA15 and not hardware NSS
  *          ------------------------------------------------------
  *          The chip select must stay a plain GPIO driven by software
  *          (SPI_NSS_SOFT), for two independent reasons:
  *
  *          1) PA15 is the SPI3_NSS pin only in the *non-remapped* pinout,
  *             where SCK/MISO/MOSI are PB3/PB4/PB5. Setting SPI3_REMAP moves
  *             the whole group, and NSS moves with it:
  *                 no remap : NSS/PA15, SCK/PB3,  MISO/PB4,  MOSI/PB5
  *                 remap    : NSS/PA4,  SCK/PC10, MISO/PC11, MOSI/PC12
  *             So "SCK/MISO/MOSI on PC10/PC11/PC12 with NSS on PA15" is not a
  *             combination the peripheral can produce, on STM32F103 or on
  *             GD32F303. PA15 can only be CS as a GPIO.
  *
  *          2) Even if CS were moved to PA4, master-mode NSS output (SSOE = 1)
  *             is not a per-frame chip select: the peripheral drives NSS low
  *             while SPE = 1 and releases it when SPE = 0. The AS5047 requires
  *             one CS pulse per 16-bit frame with t_CSn >= 350 ns between the
  *             command frame and the data frame. Producing that from hardware
  *             NSS would mean toggling SPE around every frame - slower, and
  *             with no guaranteed CS timing.
  *
  *          Software NSS also releases the pin: HAL_SPI_Init() writes SSM = 1,
  *          and SPI_MODE_MASTER already contains SPI_CR1_SSI, so the peripheral
  *          ignores NSS completely and PA15 is a free GPIO.
  *
  *          Protocol summary (AS5047P datasheet):
  *            SPI mode 1 (CPOL = 0, CPHA = 1), 16-bit frames, MSB first,
  *            fSCK <= 10 MHz, CSn active low, t_CSn >= 350 ns.
  *
  *            16-bit command frame (host -> AS5047):
  *              D15    = even parity over D14:D0
  *              D14    = R/W  (1 = read, 0 = write)
  *              D13:D0 = register address
  *
  *            16-bit write-data frame (host -> AS5047):
  *              D15    = even parity over D14:D0
  *              D14    = 0 (fixed)
  *              D13:D0 = data
  *
  *            16-bit read-data frame (AS5047 -> host):
  *              D15    = even parity over D14:D0
  *              D14    = EF, error flag of the *previous* command
  *              D13:D0 = data
  *
  *          SPI is full duplex and the device pipelines its answers: the frame
  *          returned while sending command N carries the answer to command N-1.
  *          Two consequences are used by this driver:
  *            - a register read = read command, then a NOP frame to clock out
  *              the answer (bsp_as5047ReadRegister() and friends);
  *            - a continuous stream of ANGLECOM read commands yields one valid
  *              angle per frame from the second frame on, which is what the
  *              DMA streaming mode below relies on.
  ******************************************************************************
  */

#ifndef __BSP_AS5047_H
#define __BSP_AS5047_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * Design-time configuration
 *============================================================================*/

/**
 * 1 = let this driver create its own SPI3 instance on PC10/PC11/PC12
 *     (self-contained, survives CubeMX code regeneration, nothing to add
 *      to Core/Src/spi.c).
 * 0 = the SPI instance is created elsewhere (e.g. MX_SPI3_Init() generated by
 *     STM32CubeMX). The driver then only drives the protocol on top of
 *     AS5047_SPI_HANDLE.
 */
#ifndef AS5047_SPI_INIT_IN_DRIVER
#define AS5047_SPI_INIT_IN_DRIVER   1
#endif

/* SPI handle used by the encoder. When AS5047_SPI_INIT_IN_DRIVER == 1 AS5047.c
   defines it; otherwise it is defined elsewhere (e.g. MX_SPI3_Init() in
   Core/Src/spi.c) and this declaration must simply match. */
extern SPI_HandleTypeDef hspi3;
#define AS5047_SPI_HANDLE           hspi3

/* Bus pins. The SPI3 remap moves SCK/MISO/MOSI onto PC10/PC11/PC12, which the
   CubeMX project already labels Encode_SCK / Encode_MISO / Encode_MOSI. */
#define AS5047_SCK_GPIO_Port        Encode_SCK_GPIO_Port
#define AS5047_SCK_Pin              Encode_SCK_Pin
#define AS5047_MISO_GPIO_Port       Encode_MISO_GPIO_Port
#define AS5047_MISO_Pin             Encode_MISO_Pin
#define AS5047_MOSI_GPIO_Port       Encode_MOSI_GPIO_Port
#define AS5047_MOSI_Pin             Encode_MOSI_Pin

/* Chip select. PA15 is JTDI but this project already releases the JTAG pins
   with __HAL_AFIO_REMAP_SWJ_NOJTAG() in Core/Src/stm32f1xx_hal_msp.c. */
#define AS5047_CS_GPIO_Port         Encode_CS_GPIO_Port
#define AS5047_CS_Pin               Encode_CS_Pin
#define AS5047_CS_LOW()             HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_RESET)
#define AS5047_CS_HIGH()            HAL_GPIO_WritePin(AS5047_CS_GPIO_Port, AS5047_CS_Pin, GPIO_PIN_SET)

/* SPI3 is on APB1 (PCLK1 = 60 MHz at 120 MHz SYSCLK) -> /8 = 7.5 MHz.
   AS5047P allows up to 10 MHz; use /16 (3.75 MHz) if the wiring is long. */
#ifndef AS5047_SPI_PRESCALER
#define AS5047_SPI_PRESCALER        SPI_BAUDRATEPRESCALER_8
#endif

/* Total deadline for a 16-bit transfer, including CS setup/hold. */
#ifndef AS5047_FRAME_TIMEOUT_US
#define AS5047_FRAME_TIMEOUT_US     12U
#endif
/* Maximum age of a real measurement; predictions never refresh this deadline. */
#ifndef AS5047_MAX_SAMPLE_AGE_US
#define AS5047_MAX_SAMPLE_AGE_US     1000U
#endif
/* A DMA callback may be delayed until the ADC ISR exits. */
#ifndef AS5047_DMA_TIMEOUT_US
#define AS5047_DMA_TIMEOUT_US        150U
#endif

/*==============================================================================
 * Angle read mode
 *
 * AS5047_READ_MODE_BLOCKING (default)
 *     One 16-bit ANGLE command per control tick, bounded register polling.
 *     The response belongs to the preceding command; first response is discarded.
 *     Generic register APIs retain two-frame transactions and invalidate priming.
 * AS5047_READ_MODE_DMA
 *     One asynchronous frame per Fetch. RX completion only publishes transport
 *     data; the consumer owns protocol decoding and starting the next frame.
 *     Response age can approach TWO control periods, so use recorded timestamps.
 * Both modes retain measured angle separately from the control-time estimate.
 *
 * Switching modes changes the compiled code, so it is a build-time choice.
 *============================================================================*/
#define AS5047_READ_MODE_BLOCKING   0
#define AS5047_READ_MODE_DMA        1

#ifndef AS5047_READ_MODE
#define AS5047_READ_MODE            AS5047_READ_MODE_BLOCKING
#endif

/* DMA support follows from the read mode: in blocking mode the DMA code, the
   two DMA interrupt handlers and the asynchronous API are not compiled at all,
   so a DMA channel cannot be claimed by accident. */
#if (AS5047_READ_MODE == AS5047_READ_MODE_DMA)
#define AS5047_DMA_SUPPORTED        1
#else
#define AS5047_DMA_SUPPORTED        0
#endif

/**
 * 1 = this driver provides DMA2_Channel1_IRQHandler() / DMA2_Channel2_IRQHandler().
 *     They are weak defaults in startup_stm32f103xe.s, so defining them here is
 *     safe and keeps the driver self-contained against CubeMX regeneration.
 * 0 = the handlers live in Core/Src/stm32f1xx_it.c instead (which is what
 *     STM32CubeMX generates when these DMA channels are enabled in the .ioc).
 */
#ifndef AS5047_DMA_IRQ_HANDLERS_IN_DRIVER
#define AS5047_DMA_IRQ_HANDLERS_IN_DRIVER   1
#endif

/* DMA channels and request numbers for the third SPI instance.
   STM32F103/STM32F1 HAL naming      GD32F303 naming
   DMA2_Channel1 = SPI3_RX           DMA1 channel 0 = SPI2_RX
   DMA2_Channel2 = SPI3_TX           DMA1 channel 1 = SPI2_TX            */
#define AS5047_DMA_RX_CHANNEL       DMA2_Channel1
#define AS5047_DMA_TX_CHANNEL       DMA2_Channel2
#define AS5047_DMA_RX_IRQn          DMA2_Channel1_IRQn
#define AS5047_DMA_TX_IRQn          DMA2_Channel2_IRQn

/* DMA IRQ priority 2: below ADC=1, no RTOS calls. Completion never starts
 * another transaction and uses no HAL tick-based waits. */
#ifndef AS5047_DMA_IRQ_PRIORITY
#define AS5047_DMA_IRQ_PRIORITY     2U
#endif

/* Streaming mode gives up after this many consecutive bus errors. */
#ifndef AS5047_ASYNC_MAX_ERROR_STREAK
#define AS5047_ASYNC_MAX_ERROR_STREAK   5U
#endif

/*==============================================================================
 * Register map
 *============================================================================*/

/* Volatile registers (read-only unless noted). */
#define AS5047_REG_NOP              0x0000U   /* read = NOP, clocks out the previous answer */
#define AS5047_REG_ERRFL            0x0001U   /* error flags, reading clears them */
#define AS5047_REG_PROG             0x0003U   /* OTP programming (R/W) */
#define AS5047_REG_DIAAGC           0x3FFCU   /* diagnostics + AGC */
#define AS5047_REG_MAG              0x3FFDU   /* CORDIC magnitude */
#define AS5047_REG_ANGLEUNC         0x3FFEU   /* angle without DAEC */
#define AS5047_REG_ANGLECOM         0x3FFFU   /* angle with DAEC (default angle source) */

/* One-time-programmable configuration registers (R/W).
   A write is volatile until a PROG burn command is issued. */
#define AS5047_REG_ZPOSM            0x0016U   /* zero position, 8 MSB */
#define AS5047_REG_ZPOSL            0x0017U   /* zero position, 6 LSB */
#define AS5047_REG_SETTINGS1        0x0018U
#define AS5047_REG_SETTINGS2        0x0019U

/* ERRFL (0x0001) bit masks. */
#define AS5047_ERRFL_FRERR_MASK     0x0004U   /* framing error */
#define AS5047_ERRFL_INVCOMM_MASK   0x0002U   /* invalid command / unknown address */
#define AS5047_ERRFL_PARERR_MASK    0x0001U   /* parity error on the command frame */

/* DIAAGC (0x3FFC) - magnetic field / signal diagnostics.
 *
 *   Name  Bit  Description
 *   MAGL  11   magnetic field strength too low   (AGC = 0xFF)
 *   MAGH  10   magnetic field strength too high  (AGC = 0x00)
 *   COF   9    CORDIC overflow -> angle data invalid
 *   LF    8    offset compensation loop status
 *              LF = 0: internal offset loop not yet settled
 *              LF = 1: internal offset loop finished
 *   AGC   7:0  automatic gain control value
 *
 * Note that LF is *not* a field-strength flag: it only reports whether the
 * internal offset compensation has settled, and it reads 1 when things are
 * fine. Field strength is reported by MAGL / MAGH.
 */
#define AS5047_DIAAGC_MAGL_MASK     0x0800U   /* field too weak               */
#define AS5047_DIAAGC_MAGH_MASK     0x0400U   /* field too strong             */
#define AS5047_DIAAGC_COF_MASK      0x0200U   /* CORDIC overflow              */
#define AS5047_DIAAGC_LF_MASK       0x0100U   /* offset loop finished (1 = ok) */
#define AS5047_DIAAGC_AGC_MASK      0x00FFU   /* AGC value                    */

/* Number of angle steps per revolution (14 bit). */
#define AS5047_RESOLUTION           16384U
#define AS5047_CPR                  AS5047_RESOLUTION

/* Missing-frame prediction: estimate from the last REAL sample and its timestamp.
 * Bound by both PREDICT_MAX_FRAMES and MAX_SAMPLE_AGE_US. Measured fields and
 * valid_sample_seq are never advanced by predictions. The service still counts
 * predicted/missing frames towards its fault threshold. Aged samples may exhaust
 * the time budget before the frame-count budget.
 */
#ifndef AS5047_ENABLE_PREDICTION
#define AS5047_ENABLE_PREDICTION    1
#endif
#define AS5047_PREDICT_MAX_FRAMES   10U

/*==============================================================================
 * Types
 *============================================================================*/

/** Return codes of the driver API. */
typedef enum {
    AS5047_PREDICTED         =  1,   /**< 读失败，角度由速度外推补出（不是错误，但不是实测值） */
    AS5047_OK                =  0,   /**< transfer and frame integrity both fine */
    AS5047_ERR_PARAM         = -1,   /**< NULL / out-of-range argument */
    AS5047_ERR_NOT_INIT      = -2,   /**< not initialised, or streaming not running */
    AS5047_ERR_SPI           = -3,   /**< HAL timeout / bus error */
    AS5047_ERR_PARITY        = -4,   /**< even parity of the returned frame is wrong */
    AS5047_ERR_DEVICE_FLAG   = -5,   /**< EF was set in the returned frame */
    AS5047_ERR_BUSY          = -6,   /**< busy, or no fresh streaming sample yet */
    AS5047_ERR_NO_DATA       = -7    /**< 读失败且预测不可用（超上限/未使能）→ 角度冻结 */
} AS5047_Status_t;

/** Magnet field-strength state derived from DIAAGC (MAGL / MAGH). */
typedef enum {
    AS5047_MAGNET_UNKNOWN    = 0U,   /**< DIAAGC not read yet          */
    AS5047_MAGNET_OK         = 1U,   /**< neither MAGL nor MAGH set    */
    AS5047_MAGNET_TOO_WEAK   = 2U,   /**< MAGL: field too low (AGC = 0xFF)  */
    AS5047_MAGNET_TOO_STRONG = 3U    /**< MAGH: field too high (AGC = 0x00) */
} AS5047_MagnetStatus_t;

/** Runtime state of one encoder. Mirrors the layout style of Encoder_AS5600_t. */
typedef struct {
    /* --- latest raw samples --- */
    volatile uint16_t raw_angle;        /* ANGLECOM, 0 .. 16383 */
    volatile uint16_t raw_angle_unc;    /* ANGLEUNC, 0 .. 16383 (no DAEC) */
    volatile uint16_t error_flags;      /* last value read from ERRFL (reading clears it) */
    volatile uint16_t diaagc;           /* last value read from DIAAGC */
    volatile uint16_t mag;              /* last value read from MAG */

    /* --- angles (rad) --- */
    volatile float angle;               /* mechanical angle, zero offset applied, [0, 2pi) */
    volatile float mech_angle;          /* mechanical angle, no offset, [0, 2pi) */
    volatile float angle_last;
    volatile float eleangle;            /* electrical angle, [0, 2pi) */
    float angle_zero_offset;            /* mechanical/electrical zero offset, rad */
    float pole_pairs;                   /* motor pole pairs, used for eleangle */
    float direction;                    /* +1.0f or -1.0f, flips the counting direction */

    /* --- speed / multi-turn tracking --- */
    volatile float angle_delta;         /* wrapped delta of the last sample, rad */
    volatile float speed_rad_s;         /* filtered angular speed, rad/s */
    volatile float speed_rpm;           /* filtered speed, RPM */
    float speed_lpf;                    /* IIR weight of the new sample, 0..1 */
    float sample_time_s;                /* 0 = speed estimation disabled */
    uint8_t move_state;                 /* 1 = CCW/forward, 0 = CW/reverse */
    int32_t rotation_circles;           /* signed turn counter */
    uint32_t rotation_circles_absolute; /* unsigned turn counter */
    float angle_total_ccw;              /* accumulated CCW travel, rad */
    float angle_total_cw;               /* accumulated CW travel, rad (positive) */

    /* --- health --- */
    bool initialized;
    AS5047_MagnetStatus_t magnet_status;
    bool diagnostics_valid;             /* DIAAGC has been read successfully at least once */
    bool cordic_overflow;               /* COF: angle data invalid right now */
    bool offset_loop_settled;           /* LF: internal offset loop finished */
    uint8_t agc;                        /* AGC value from the last DIAAGC read */
    uint32_t comm_errors;               /* SPI/HAL failures */
    uint32_t parity_errors;             /* bad even-parity on a returned frame */
    uint32_t device_flags;              /* number of frames with EF set */

    /* --- 速度预测补帧统计 --- */
    uint32_t predict_streak;            /* 当前连续预测帧数，真实样本到来即清零 */
    uint32_t predict_frames;            /* 累计预测帧数 */
    uint32_t predict_max_streak;        /* 历史最长连续预测帧数 */
    uint32_t predict_blocked;           /* 超上限而放弃预测的次数（此后角度冻结） */

    /* --- DMA streaming mode --- */
    uint32_t valid_sample_seq;          /* real angle samples only */
    uint32_t sample_cycles;             /* command CS rising edge */
    uint32_t sample_age_us;
    uint32_t transfer_timeouts;
    uint32_t stale_samples;
    uint32_t priming_frames;
    uint32_t read_cycles;               /* consumer update cost, including preemption */
    uint32_t max_read_cycles;
    volatile uint32_t sample_seq;       /* incremented for every published sample */
    volatile uint32_t async_frames;     /* frames completed while streaming */
    uint32_t sample_misses;             /* fetch calls that found no fresh sample */
    uint32_t async_error_streak;        /* consecutive bus errors while streaming */
    bool async_stopped;                 /* streaming gave up after repeated errors */
} Encoder_AS5047_t;

extern Encoder_AS5047_t Encoder_AS5047;

/* 阻塞读超时所用时基（DWT 周期计数器）的状态，只读诊断用：
 *   0 = 还没探测，1 = 正常在计数，2 = **不计数**（超时已退化为"最大轮询次数"兜底）
 * 值为 2 时不会死等，但超时精度很差 —— 该值应长期为 1。
 * 见 AS5047.c 的 as5047_dwt_ensure()：本工程 DWT 是被 J-Scope 初始化顺手使能的，
 * 驱动自己又确保并实测了一次，去掉那层隐式耦合。 */
extern uint8_t as5047_dwt_state;

/*==============================================================================
 * API
 *============================================================================*/

/**
 * @brief  Bring up the SPI3 instance on PC10/PC11/PC12 plus the DMA channels.
 *         Software NSS, mode 1, 16 bit, MSB first, AS5047_SPI_PRESCALER.
 * @note   Only compiled when AS5047_SPI_INIT_IN_DRIVER == 1. It configures the
 *         GPIO, enables the AFIO clock and the SPI3 remap itself, so nothing has
 *         to be added to Core/Src/spi.c and a CubeMX regeneration cannot lose it.
 *         Existing CubeMX GPIO_Output settings on PC10/PC11/PC12/PA15 are
 *         intentionally replaced here.
 * @note   Safe to call more than once.
 */
void bsp_as5047SpiInit(void);

/**
 * @brief  Full start-up: SPI bring-up, first sample, state reset.
 * @return AS5047_OK, or a negative AS5047_Status_t value.
 * @note   Call once before the FOC sampling ISR starts using the encoder.
 */
int bsp_as5047HwInit(void);

/** @brief Reset the tracking state only. Performs no SPI access. */
void bsp_as5047ResetTracking(void);

/** @brief Set the mechanical zero offset in rad (wrapped internally). */
void bsp_as5047SetZeroOffset(float offset_rad);

/** @brief Set the motor pole pairs used for the electrical angle. */
void bsp_as5047SetPolePairs(float pole_pairs);

/** @brief Set the sampling period in s. 0 disables speed estimation. */
void bsp_as5047SetSampleTime(float sample_time_s);

/** @brief Set the counting direction: +1.0f (default) or -1.0f. */
void bsp_as5047SetDirection(float direction);

/** @brief Set the IIR weight of the newest speed sample, 0..1. 1 = no filter. */
void bsp_as5047SetSpeedLpf(float speed_lpf);

/*----------------------------------------------------------------------------
 * Blocking register / angle access
 *
 * These use the bus directly and therefore return AS5047_ERR_BUSY while
 * streaming mode is running. Stop streaming first to touch registers.
 *--------------------------------------------------------------------------*/

/**
 * @brief  Read one register (strict). Returns AS5047_ERR_DEVICE_FLAG when the
 *         device reported EF for the previous frame, but *data is still valid.
 */
int bsp_as5047ReadRegister(uint16_t address, uint16_t *data);

/**
 * @brief  Write one register. A trailing NOP frame is sent so the next read
 *         starts from a clean pipeline.
 */
int bsp_as5047WriteRegister(uint16_t address, uint16_t data);

/** @brief Read ANGLECOM (angle with DAEC), 0 .. 16383. */
int bsp_as5047ReadAngleRaw(uint16_t *raw_angle);

/** @brief Read ANGLEUNC (angle without DAEC), 0 .. 16383. */
int bsp_as5047ReadAngleUncompensated(uint16_t *raw_angle);

/** @brief Read ERRFL. Reading clears the flags on the device. */
int bsp_as5047ReadErrorFlags(uint16_t *flags);

/** @brief Read DIAAGC, updates Encoder_AS5047.magnet_status as a side effect. */
int bsp_as5047ReadDiagnostics(uint16_t *diaagc);

/** @brief Read MAG (CORDIC magnitude). */
int bsp_as5047ReadMagnitude(uint16_t *mag);

/** @brief Read and cache ERRFL / DIAAGC / MAG. Intended for startup or a
 *         low-rate health task, not for the FOC ISR. */
int bsp_as5047ReadAllDiagnostics(void);

/**
 * @brief  Blocking sample: read ANGLECOM and refresh the full tracking state
 *         (angle, mech_angle, eleangle, turn counters, speed when sample_time_s
 *         has been set).
 * @param  angle  optional pointer receiving the zero-offset angle in rad.
 * @return AS5047_OK on success.
 */
int bsp_as5047GetAngle(volatile float *angle);
/* Two-frame fresh read for initialization/alignment only, with FOC quiescent.
 * Temporarily stops/reprimes DMA when enabled. Never call from an ISR. */
int bsp_as5047GetAngleImmediate(volatile float *angle);
/* Extrapolate the last real mechanical sample to now, bounded by its age.
 * Does not modify raw/measured angle or count a prediction as a real sample. */
float bsp_as5047GetControlAngle(void);

/** @brief Convenience wrapper: same as bsp_as5047GetAngle(NULL). */
int bsp_as5047Update(void);

/**
 * @brief  Write a new zero position through ZPOSM/ZPOSL.
 * @param  position  raw 0 .. 16383 value that shall read as 0.
 * @note   The write is volatile: it is lost at power-down unless a PROG burn
 *         command (register 0x0003) is issued. This driver never burns OTP.
 */
int bsp_as5047SetZeroPosition(uint16_t position);

/** @brief Program the current position as the zero position (volatile). */
int bsp_as5047SetZeroHere(void);

/** @brief Field-strength state from the cached DIAAGC (MAGL / MAGH). */
AS5047_MagnetStatus_t bsp_as5047GetMagnetStatus(void);

/** @brief true when the cached DIAAGC shows neither MAGL nor MAGH. */
bool bsp_as5047IsMagnetOk(void);

/** @brief true when the angle can be trusted: DIAAGC was read and COF is clear. */
bool bsp_as5047IsAngleValid(void);

/** @brief true when the internal offset compensation loop has finished (LF = 1). */
bool bsp_as5047IsOffsetLoopSettled(void);

/** @brief Last AGC value (only meaningful when diagnostics_valid is true). */
uint8_t bsp_as5047GetAgc(void);

/** @brief Raw 16-bit frames for debugging, bypassing the frame checks. */
int bsp_as5047ReadFrameRaw(uint16_t address, uint16_t *frame);

#if (AS5047_DMA_SUPPORTED == 1)

/* DMA transport is triggered by the consumer, not free-running. A command's
 * response arrives in the next frame and is consumed at a later Fetch. Data age
 * includes both stages; it is not unconditionally one period. Missing/invalid
 * data uses bounded prediction and counts towards the service fault threshold.
 * Start/Stop and generic register access are initialization/stopped-state APIs;
 * the control consumer must be quiescent when changing transport ownership. */
int bsp_as5047AsyncStart(void);

/** @brief Stop streaming, abort any pending DMA transfer and release CS. */
void bsp_as5047AsyncStop(void);

/** @brief true while the frame pipeline is running. */
bool bsp_as5047AsyncIsRunning(void);

/** @brief true when a fresh raw sample is waiting to be fetched. */
bool bsp_as5047AsyncDataReady(void);

/**
 * @brief  Consume the newest streamed sample, refresh the tracking state and
 *         issue the next frame. Intended to be called once per FOC period.
 * @param  angle  optional pointer receiving the zero-offset angle in rad.
 * @return AS5047_OK when a new sample was published; AS5047_PREDICTED for a bounded extrapolation, otherwise an error;
 *         AS5047_ERR_NOT_INIT when the pipeline is not running.
 */
int bsp_as5047AsyncFetch(volatile float *angle);

#endif /* AS5047_DMA_SUPPORTED */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_AS5047_H */
