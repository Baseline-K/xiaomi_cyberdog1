#ifndef CAN_BSP_H
#define CAN_BSP_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* 节点地址：多关节板可改（0x600+node = 收，0x580+node = 发） */
#define CAN_NODE_ID        1U
#define CAN_ID_SDO_RX      (0x600U + CAN_NODE_ID)   /* 0x601 命令收 */
#define CAN_ID_SDO_TX      (0x580U + CAN_NODE_ID)   /* 0x581 回复发 */

#define CAN_RX_BUF_SIZE    16U

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

/* 底层初始化：过滤器 + 通知 + Start（在 main 里调用一次） */
void    CAN_bsp_Init(void);

/* 主循环取一帧，1=成功 */
uint8_t CAN_bsp_PopFrame(CAN_Frame_t *frame);

/* 发送标准帧 */
uint8_t CAN_bsp_Send(uint32_t std_id, const uint8_t data[8], uint8_t dlc);

/* 统计（调试/诊断） */
void    CAN_bsp_GetStats(uint32_t *rx_total, uint32_t *rx_overrun, uint32_t *rx_drop);

#endif /* CAN_BSP_H */
