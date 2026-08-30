/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
*         SEGGER SystemView  * Real-time application analysis        *
*              https://github.com/SEGGERMicro/SystemView             *
*                                                                    *
**********************************************************************

---------------------------END-OF-HEADER------------------------------

Purpose : SEGGER SystemView configuration file.
          Set defines which deviate from the defaults (see SEGGER_SYSVIEW_ConfDefaults.h) here.          

Additional information:
  Required defines which must be set are:
    SEGGER_SYSVIEW_GET_TIMESTAMP
    SEGGER_SYSVIEW_GET_INTERRUPT_ID
  For known compilers and cores, these might be set to good defaults
  in SEGGER_SYSVIEW_ConfDefaults.h.
  
  SystemView needs a (nestable) locking mechanism.
  If not defined, the RTT locking mechanism is used,
  which then needs to be properly configured.
*/

#ifndef SEGGER_SYSVIEW_CONF_H
#define SEGGER_SYSVIEW_CONF_H

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

/*********************************************************************
*
*       Define: SEGGER_SYSVIEW_SECTION
*
*  Description
*    Section to place the SystemView RTT Buffer into.
*  Default
*    undefined: Do not place into a specific section.
*  Notes
*    If SEGGER_RTT_SECTION is defined, the default changes to use
*    this section for the SystemView RTT Buffer, too.
*/
#if !(defined SEGGER_SYSVIEW_SECTION) && (defined SEGGER_RTT_BUFFER_SECTION)
  #define SEGGER_SYSVIEW_SECTION                  SEGGER_RTT_BUFFER_SECTION
#endif


/*********************************************************************
*       Defines for this project
**********************************************************************
*/
// SystemView 独占 RTT 通道 3（0=终端/printf，1=J-Scope 数据，2=J-Scope 配置）。
#define SEGGER_SYSVIEW_RTT_CHANNEL                3

// 初始化后等待主机 Start 命令，避免主机连接前填满缓冲区。
#define SEGGER_SYSVIEW_START_ON_INIT              0

// 8 KiB 独立缓冲区用于吸收 10 kHz FOC 中断的短时记录突发。
#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE            8192

// 时间戳：默认 DWT->CYCCNT（120MHz，见 ConfDefaults.h），本工程已由 SEGGER_RTT_Port 使能 DWT
// 中断 ID：默认读 Cortex-M ICSR（见 ConfDefaults.h）
// CPU 时钟 / tick 频率见 SEGGER_SYSVIEW_Config_FreeRTOS.c（用 configCPU_CLOCK_HZ）


#endif  // SEGGER_SYSVIEW_CONF_H

/*************************** End of file ****************************/
