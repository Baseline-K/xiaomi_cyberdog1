#ifndef SEGGER_RTT_PORT_H
#define SEGGER_RTT_PORT_H

#include <stdint.h>
#include "SEGGER_RTT.h"


typedef uint32_t tb_tick_t;    // 原始计数器宽度（32位）
typedef uint64_t tb_us_t;      // 微秒类型（64位）

void SEGGER_RTT_TimeStamp_reset(void);
void SEGGER_RTT_Port_Init(void);
void SEGGER_RTT_Port_Write(void);

void SEGGER_RTT_PLUS_Port_Init(void);
void JS_RTT_PLUS_WaitForConfig(void);
void JS_RTT_PLUS_SendData(void); 
#endif

