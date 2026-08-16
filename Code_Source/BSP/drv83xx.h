/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV835X.c
 * Description        :  DRV835X driver 
 ******************************************************************************
 * @attention
 *
* COPYRIGHT:    Copyright (c) 2025
* CREATED BY:   Baseline
* DATE:         October 22th, 2025
 ******************************************************************************
 */
/* USER CODE END Header */
 
/* Includes ------------------------------------------------------------------*/
#ifndef DRV835X_H
#define DRV835X_H
 
#ifdef _cplusplus
extern "C"
{
#endif
 
#include "main.h"
#include <stdio.h>
 
/* --- Check whether the user has specified a model --- */
//#define DRV8350   
//#define DRV8353   
//#define DRV8320   
#define DRV8323      //Must ： Select your Specifically DRV83xx Chip

#if !defined(DRV8350) && !defined(DRV8353) && !defined(DRV8320) && !defined(DRV8323)
  #error "Please define a driver model macro (e.g. -DDRIVER_DRV8353A) in build options"
#endif

/* --- Map model types to feature macros --- */
/* For models with OCP_ACT / DIS_GDUV */
#if defined(DRV8350) || defined(DRV8353)
  #define HAS_OCP_ACT    1
  #define HAS_DIS_GDUV   1
	#define HAS_DRV835X_VDS_LVL  1
#endif

/* For models with DIS_CPUV */
#if defined(DRV8320) || defined(DRV8323)
  #define HAS_DIS_CPUV   1
	#define HAS_DRV832X_VDS_LVL  1
#endif
 
#if defined(DRV8323) || defined(DRV8353)
  #define HAS_CSA_Control_Fields 1
#endif
 
#if defined(DRV8353)
  #define HAS_CAL_MODE   1
#endif 
/* --- Check whether the user has specified a model End--- */ 
 
 
#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

#define  W_MODE              0
#define  R_MODE              1

/**************************************************************/ 
/*
    INPUT data structure  
*/
typedef struct
{
    uint16_t DATA      : 11;    // 11 data bits, D (bits B11 through B0)
    uint16_t ADDRESS   : 4;     // 4 address bits, A (bits B14 through B11)
    
    uint16_t WR        : 1;     // 1 read or write bit, W (bit B15)
}Input_WrReg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Input_WrReg_bit inputRegObj;
   };
} Input_WrReg ;
 
/*
    Fault Status Register 1 (address = 0x00h)
*/
typedef struct
{
    uint16_t VDS_LC    : 1;
    uint16_t VDS_HC    : 1;
    uint16_t VDS_LB    : 1;
    uint16_t VDS_HB    : 1;
    
    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;
    uint16_t OTSD      : 1;
    uint16_t UVLO      : 1;
    
    uint16_t GDF       : 1;
    uint16_t VDS_OCP   : 1;
    uint16_t FAULT     : 1;
    
    uint16_t res       : 5; 
    
}Fault_StatusReg1_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Fault_StatusReg1_bit fault1RegObj;
   };
} Fault_StatusReg1 ;
 
/*
    Fault Status Register 2 (address = 0x01h)
*/
typedef struct
{
    uint16_t VGS_LC    : 1;
    uint16_t VGS_HC    : 1;
    
    uint16_t VGS_LB    : 1;
    uint16_t VGS_HB    : 1;
    
    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;
    
    uint16_t GDUV      : 1;
    uint16_t OTW       : 1;
    
    uint16_t SC_OC     : 1;
    uint16_t SB_OC     : 1;
    uint16_t SA_OC     : 1;
    
    uint16_t res       : 5; 
    
}Fault_StatusReg2_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Fault_StatusReg2_bit fault2RegObj;
   };
} Fault_StatusReg2;
 
/*
    Driver Control Register (address = 0x02h)
*/
typedef struct
{
    uint16_t CLR_FLT  : 1;
    uint16_t BRAKE    : 1;
    uint16_t COAST    : 1;
    uint16_t PWM1_DIR : 1;
    
    uint16_t PWM1_COM : 1;
    uint16_t PWM_MODE : 2;
    uint16_t OTW_REP  : 1;
    
    uint16_t DIS_GDF  : 1;
	
#ifdef  HAS_DIS_GDUV
    uint16_t DIS_GDUV : 1;
#elif   HAS_DIS_CPUV	  
	  uint16_t DIS_CPUV : 1;
#else
	  uint16_t bit9_RESERVE : 1;
#endif
	
#ifdef  HAS_OCP_ACT
    uint16_t OCP_ACT  : 1;
#else
    uint16_t bit10_RESERVE : 1;
#endif	    
	
    uint16_t res      : 5; 
    
}Drv_CtrlReg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_CtrlReg_bit ctrlRegObj;
   };
} Drv_CtrlReg ;
 
/*
    Gate Drive HS Register (address = 0x03h)
*/
typedef struct
{
    uint16_t IDRIVEN_HS  : 4;
    uint16_t IDRIVEP_HS  : 4;
    
    uint16_t LOCK        : 3;
    uint16_t res         : 5; 
    
}Drv_GateHS_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_GateHS_bit gateHSRegObj;
   };
} Drv_GateHS;
 
 
/*
    Gate Drive LS Register (address = 0x04h)
*/
typedef struct
{
    uint16_t IDRIVEN_LS  : 4;
    uint16_t IDRIVEP_LS  : 4;
    
    uint16_t TDRIVE      : 2;
    uint16_t CBC         : 1;
    
    uint16_t res         : 5; 
    
}Drv_GateLS_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_GateLS_bit gateLSRegObj;
   };
} Drv_GateLS;
 
/*
    OCP Control Register (address = 0x05h)
*/
typedef struct
{
    uint16_t VDS_LVL     : 4;
    uint16_t OCP_DEG     : 2;
    
    uint16_t OCP_MODE    : 2;
    uint16_t DEAD_TIME   : 2;
    
    uint16_t TRETRY      : 1;
    
    uint16_t res         : 5; 
    
}Drv_OCP_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_OCP_bit ocpObj;
   };
} Drv_OCP;
 
/*
    CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
*/
typedef struct
{
#ifdef HAS_CSA_Control_Fields
    uint16_t SEN_LVL     : 2;
    
    uint16_t CSA_CAL_C   : 1;
    uint16_t CSA_CAL_B   : 1;
    uint16_t CSA_CAL_A   : 1;
    uint16_t DIS_SEN     : 1;
    
    uint16_t CSA_GAIN    : 2;
    uint16_t LS_REF      : 1;
    uint16_t VREF_DIV    : 1;
    
    uint16_t CSA_FET     : 1;
#else
	  uint16_t bit0_10_RESERVE : 11;
#endif
    uint16_t res         : 5; 
}Drv_CSA_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_CSA_bit csaObj;
   };
} Drv_CSA;
 
/*
    Driver Configuration Register (DRV8353 and DRV8353R Only) (address = 0x07h)
*/
typedef struct
{
#if HAS_CAL_MODE
    uint16_t SEN_LVL        : 1;
#else
	  uint16_t bit0_RESERVE   : 1;  
#endif
    uint16_t bit1_10_RESERVE     : 10; 
    uint16_t res         : 5; 
    
}Drv_Cfg_bit;
 
typedef struct
{
   union
   {
      uint16_t data;
      Drv_Cfg_bit cfgObj;
   };
} Drv_Cfg;
 
/**************************************************************/
typedef struct
{
    Fault_StatusReg1  faultStatusReg1_obj;
    Fault_StatusReg2  faultStatusReg2_obj; 
	
	  Drv_CtrlReg drvCtrl_obj; 
    Drv_GateLS drvGateLS_obj;
    Drv_GateHS drvGateHS_obj;
    
    Drv_OCP drvOcp_obj;
	  Drv_CSA drvCsa_obj;
	  Drv_Cfg drvCfg_obj; 
}Stru_DRV83XX;        
extern Stru_DRV83XX stru_DRV8323Obj;
 
 
typedef struct
{
   // Driver Control Register (address = 0x02h)
   uint8_t PWM_MODE;


   //Gate Drive HS Register (address = 0x03h)
   uint8_t IDRIVEP_HS;
   uint8_t IDRIVEN_HS;
   uint8_t LOCK;
    
   // Gate Drive LS Register (address = 0x04h) 
   uint8_t IDRIVEN_LS;
   uint8_t IDRIVEP_LS;
   uint8_t TDRIVE;
   uint8_t CBC;
	 
   // OCP Control Register (address = 0x05h)
   uint8_t VDS_LVL;
   uint8_t OCP_DEG;
   uint8_t OCP_MODE;
   uint8_t DEAD_TIME;
    
   // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
   uint8_t SEN_LVL;
   uint8_t CSA_GAIN;
   uint8_t VREF_DIV;
   uint8_t CSA_CAL_A;
	 uint8_t CSA_CAL_B;
   uint8_t CSA_CAL_C;
} StruDRV83XXCfgPara;
 
 
void DRV83XX_Init(void);
int DRV83XX_updateCfgPara(Stru_DRV83XX *stru_DRV83xxObj, StruDRV83XXCfgPara stru_config);
 
void DRV83XX_read_ALLReg(Stru_DRV83XX *stru_DRV83xxObj);
void DRV83XX_read_FaultStatusReg1(Stru_DRV83XX *stru_DRV83xxObj);
void DRV83XX_read_FaultStatusReg2(Stru_DRV83XX *stru_DRV83xxObj);
int DRV8323_COAST_Disable(Stru_DRV83XX *stru_DRV83xxObj);
int DRV8323_COAST_enable(Stru_DRV83XX *stru_DRV83xxObj);
int DRV8323_CurrentCAL_enable(Stru_DRV83XX *stru_DRV83xxObj);
int DRV8323_CurrentCAL_disable(Stru_DRV83XX *stru_DRV83xxObj);
 
#ifdef _cplusplus
}
#endif
 
 
#endif  /* DRV835X_H */

