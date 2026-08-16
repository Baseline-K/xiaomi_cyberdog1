/**
 * CAN_bsp —— bxCAN 底层接收/发送
 * 与 GD32 CAN 补丁（stm32f1xx_hal_can_gd32.c）兼容，用标准 STM32F1 HAL_CAN_*。
 *
 * 接收：ISR(HAL_CAN_RxFifo0MsgPendingCallback) → 环形缓冲（单生产者/单消费者）
 * 发送：HAL_CAN_AddTxMessage（硬件自动重发）
 */
#include "CAN_bsp.h"
#include "can.h"          /* hcan */

/* ---- 环形缓冲 ---- */
static volatile uint8_t  rx_head = 0U;
static volatile uint8_t  rx_tail = 0U;
static CAN_Frame_t       rx_buf[CAN_RX_BUF_SIZE];

/* ---- 统计 ---- */
static volatile uint32_t rx_total  = 0U;
static volatile uint32_t rx_overrun = 0U;   /* 预留：FIFO 溢出（需开 FIFO0_OVERRUN 通知） */
static volatile uint32_t rx_drop   = 0U;    /* 环形缓冲满丢弃 */

/* 只收本节点命令帧；调通阶段可放开为全收 */
static uint8_t can_id_is_supported(uint32_t id)
{
    return (id == CAN_ID_SDO_RX);
}

void CAN_bsp_Init(void)
{
    CAN_FilterTypeDef f = {0};

    /* 过滤器：32 位掩码模式，精确收 CAN_ID_SDO_RX，映射 FIFO0 */
    f.FilterBank          = 0U;
    f.FilterMode          = CAN_FILTERMODE_IDMASK;
    f.FilterScale         = CAN_FILTERSCALE_32BIT;
    /* bxCAN 标准帧 STID 占 FilterIdHigh[15:5]，故 <<5 */
    f.FilterIdHigh        = (uint16_t)((uint32_t)CAN_ID_SDO_RX << 5U);
    f.FilterIdLow         = 0x0000U;
    f.FilterMaskIdHigh    = (uint16_t)(0x7FFU << 5U);   /* 精确匹配 STID */
    f.FilterMaskIdLow     = 0x0000U;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &f);

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan);
}

/* 强定义覆盖 GD32 补丁里的弱回调 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanp)
{
    CAN_RxHeaderTypeDef h = {0};
    CAN_Frame_t f;
    uint8_t next;

    if (HAL_CAN_GetRxMessage(hcanp, CAN_RX_FIFO0, &h, f.data) != HAL_OK) return;
    if (h.IDE != CAN_ID_STD) return;                 /* 只收标准帧 */
    if (!can_id_is_supported(h.StdId)) return;

    f.id  = h.StdId;
    f.dlc = h.DLC;
    next = (uint8_t)((rx_head + 1U) % CAN_RX_BUF_SIZE);
    if (next == rx_tail) { rx_drop++; return; }      /* 满则丢 */
    rx_buf[rx_head] = f;
    rx_head = next;
    rx_total++;
}

uint8_t CAN_bsp_PopFrame(CAN_Frame_t *frame)
{
    uint8_t ret = 0U;
    if (rx_head != rx_tail) {
        __disable_irq();                             /* 保护 tail 写，防 ISR 抢占 */
        *frame = rx_buf[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) % CAN_RX_BUF_SIZE);
        __enable_irq();
        ret = 1U;
    }
    return ret;
}

uint8_t CAN_bsp_Send(uint32_t std_id, const uint8_t data[8], uint8_t dlc)
{
    CAN_TxHeaderTypeDef h = {0};
    uint32_t mailbox;
    h.StdId = std_id;
    h.IDE = CAN_ID_STD;
    h.RTR = CAN_RTR_DATA;
    h.DLC = dlc;
    return (HAL_CAN_AddTxMessage(&hcan, &h, (uint8_t*)data, &mailbox) == HAL_OK);
}

void CAN_bsp_GetStats(uint32_t *rx_total_o, uint32_t *rx_overrun_o, uint32_t *rx_drop_o)
{
    if (rx_total_o)   *rx_total_o   = rx_total;
    if (rx_overrun_o) *rx_overrun_o = rx_overrun;
    if (rx_drop_o)    *rx_drop_o    = rx_drop;
}
