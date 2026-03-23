/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          :  drv_DRV83XX.c
 * Description        :  DRV83XX driver 
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
#ifndef __DRV_DRV83XX_REG_H
#define __DRV_DRV83XX_REG_H
 
#ifdef _cplusplus
extern "C"
{
#endif
 
#include "main.h"
#include "drv83xx.h"



/// ****************** Registers ******************///
#define FSR1             0x0     /// Fault Status Register 1   --  only read
#define FSR2             0x1     /// Fault Status Register 2   --  only read
#define DCR              0x2     /// Drive Control Register    --  read and write
#define HSR              0x3     /// Gate Drive HS Register    --  read and write
#define LSR              0x4     /// Gate Drive LS Register    --  read and write
#define OCPCR            0x5     /// OCP Control Register      --  read and write
#define CSACR            0x6     /// CSA Control Register      --  read and write
#define DFGCR            0x7     /// Driver Configuration Register   --  read and write
 
/// ****** Drive Control Fields ******///(address = 0x02h)
#ifdef HAS_OCP_ACT
	#define OCP_ACT_Associated  0x0     //Associated half-bridge is shutdown in response to  VDS_OCP and SEN_OCP
	#define OCP_ACT_Global      0x1     //All three half-bridges are shutdown in response to VDS_OCP and SEN_OCP
#endif
#ifdef  HAS_DIS_GDUV
	#define DIS_GDUV_EN         0x0     ///VCP and VGLS undervoltage lockout fault is enabled
	#define DIS_GDUV_DIS        0x1
#endif

#ifdef HAS_DIS_CPUV
	#define DIS_CPUV_EN         0x0     /// Charge pump UVLO fault
	#define DIS_CPUV_DIS        0x1
#endif

#define DIS_GDF_EN          0x0     /// Gate drive fault
#define DIS_GDF_DIS         0x1
#define OTW_REP_DIS         0x0
#define OTW_REP_EN          0x1     /// Over temp warning reported on nFAULT/FAULT bit
 
#define PWM_MODE_6X         0x0     /// PWM Input Modes  Chip not allow Upper and lower pipes straight through
#define PWM_MODE_3X         0x1     /// 3X INHx PWM Control,in this mode all INLx should be High level
#define PWM_MODE_1X         0x2     /// Typically used for six-position reversing control
#define PWM_MODE_IND        0x3     /// 6X PWM but Chip allow Upper and lower pipes straight through
 
#define PWM_1X_COM_SYNC     0x0     /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC    0x1
 
#define PWM_1X_DIR_0        0x0     /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1        0x1
 
#define COAST_DIS           0x0     
#define COAST_EN            0x1      ///put all MOSFETs in the Hi-Z state, Allow the motor to coast freely

#define BRAKE_DIS           0x0     
#define BRAKE_EN            0x1      ///turn on all three low-side MOSFETs, Apply short-circuit braking to the motor

#define CLR_FLT_DIS         0x0
#define CLR_FLT__EN         0x1      ///clear latched fault bits.This bit automatically resets after being writen


/// ****** Gate Drive HS Fields ******///(address = 0x03h)
#define LOCK_ON             0x6    //Lock all register write operations except for specific registers (such as bits [0–2] at address 0x02).
#define LOCK_OFF            0x3    //Unlocked state enabled; all writable registers can now be modified.
 
#define IDRIVEP_HS_10MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_HS_30MA     0x1
#define IDRIVEP_HS_60MA     0x2
#define IDRIVEP_HS_80MA     0x3
#define IDRIVEP_HS_120MA    0x4
#define IDRIVEP_HS_140MA    0x5
#define IDRIVEP_HS_170MA    0x6
#define IDRIVEP_HS_190MA    0x7
#define IDRIVEP_HS_260MA    0x8
#define IDRIVEP_HS_330MA    0x9
#define IDRIVEP_HS_370MA    0xA
#define IDRIVEP_HS_440MA    0xB
#define IDRIVEP_HS_570MA    0xC
#define IDRIVEP_HS_680MA    0xD
#define IDRIVEP_HS_820MA    0xE
#define IDRIVEP_HS_1000MA   0xF     /// Default :
 
#define IDRIVEN_HS_20MA     0x0     /// High side turn off current
#define IDRIVEN_HS_60MA     0x1     
#define IDRIVEN_HS_120MA    0x2
#define IDRIVEN_HS_160MA    0x3
#define IDRIVEN_HS_240MA    0x4
#define IDRIVEN_HS_280MA    0x5
#define IDRIVEN_HS_340MA    0x6
#define IDRIVEN_HS_380MA    0x7
#define IDRIVEN_HS_520MA    0x8
#define IDRIVEN_HS_660MA    0x9
#define IDRIVEN_HS_740MA    0xA
#define IDRIVEN_HS_880MA    0xB
#define IDRIVEN_HS_1140MA   0xC
#define IDRIVEN_HS_1360MA   0xD
#define IDRIVEN_HS_1640MA   0xE
#define IDRIVEN_HS_2000MA   0xF    /// Default :
 
/// ****** Gate Drive LS Fields ****** /// (address = 0x04h)

                                 //PWM_GIVER_ENABLE/DISABLE Active only when OCP_MODE = 01b
#define  PWM_GIVER_DISABLE  0x0  //Overcurrent faults will automatically clear after a fixed delay of tRETRY .
#define  PWM_GIVER_ENABLE   0x1  /// Default :the fault is cleared when a new PWM input is given or after tRETRY 

#define TDRIVE_500NS        0x0     /// Peak gate-current drive time
#define TDRIVE_1000NS       0x1
#define TDRIVE_2000NS       0x2
#define TDRIVE_4000NS       0x3     /// Default :
 
#define IDRIVEP_LS_10MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_LS_30MA     0x1
#define IDRIVEP_LS_60MA     0x2
#define IDRIVEP_LS_80MA     0x3
#define IDRIVEP_LS_120MA    0x4
#define IDRIVEP_LS_140MA    0x5
#define IDRIVEP_LS_170MA    0x6
#define IDRIVEP_LS_190MA    0x7
#define IDRIVEP_LS_260MA    0x8
#define IDRIVEP_LS_330MA    0x9
#define IDRIVEP_LS_370MA    0xA
#define IDRIVEP_LS_440MA    0xB
#define IDRIVEP_LS_570MA    0xC
#define IDRIVEP_LS_680MA    0xD
#define IDRIVEP_LS_820MA    0xE
#define IDRIVEP_LS_1000MA   0xF     /// Default :
 
#define IDRIVEN_LS_20MA     0x0     /// High side turn off current
#define IDRIVEN_LS_60MA     0x1     
#define IDRIVEN_LS_120MA    0x2
#define IDRIVEN_LS_160MA    0x3
#define IDRIVEN_LS_240MA    0x4
#define IDRIVEN_LS_280MA    0x5
#define IDRIVEN_LS_340MA    0x6
#define IDRIVEN_LS_380MA    0x7
#define IDRIVEN_LS_520MA    0x8
#define IDRIVEN_LS_660MA    0x9
#define IDRIVEN_LS_740MA    0xA
#define IDRIVEN_LS_880MA    0xB
#define IDRIVEN_LS_1140MA   0xC
#define IDRIVEN_LS_1360MA   0xD
#define IDRIVEN_LS_1640MA   0xE
#define IDRIVEN_LS_2000MA   0xF    /// Default :

 
 
/// ****** OCP Control Fields ******///  (address = 0x05h)
#define TRETRY_4MS          0x0     /// VDS OCP and SEN OCP retry time
#define TRETRY_50US         0x1
 
#define DEADTIME_50NS       0x0     /// Deadtime
#define DEADTIME_100NS      0x1     /// Default :
#define DEADTIME_200NS      0x2
#define DEADTIME_400NS      0x3
 

#define OCP_Mode_LATCH           0x0     ///Overcurrent causes a latched fault
#define OCP_Mode_RETRY           0x1     ///Default :Overcurrent causes an automatic retrying fault
#define OCP_Mode_REPORT          0x2     ///Overcurrent is report only but no action is taken
#define OCP_Mode_NONE            0x3     ///Overcurrent is not reported and no action is taken
 
#define OCP_DEG_2US         0x0     /// OCP Deglitch Time
#define OCP_DEG_4US         0x1     /// Default :
#define OCP_DEG_6US         0x2
#define OCP_DEG_8US         0x3
 
 

#ifdef HAS_DRV832X_VDS_LVL  //VDS_LVL :MOSFET Drain-Source Voltage OverCurrent Protect Detection Threshold
	#define VDS_OCP_LVL_0_06        0x0
	#define VDS_OCP_LVL_0_13        0x1
	#define VDS_OCP_LVL_0_2         0x2
	#define VDS_OCP_LVL_0_26        0x3
	#define VDS_OCP_LVL_0_31        0x4
	#define VDS_OCP_LVL_0_45        0x5
	#define VDS_OCP_LVL_0_53        0x6
	#define VDS_OCP_LVL_0_6         0x7
	#define VDS_OCP_LVL_0_68        0x8
	#define VDS_OCP_LVL_0_75        0x9     //Default : VDS_OCP_LVL = 0.75 V
	#define VDS_OCP_LVL_0_94        0xA
	#define VDS_OCP_LVL_1_13        0xB
	#define VDS_OCP_LVL_1_3         0xC
	#define VDS_OCP_LVL_1_5         0xD
	#define VDS_OCP_LVL_1_7         0xE
	#define VDS_OCP_LVL_1_88        0xF
#endif

#ifdef HAS_DRV835X_VDS_LVL
	#define VDS_OCP_LVL_0_06        0x0
	#define VDS_OCP_LVL_0_07        0x1
	#define VDS_OCP_LVL_0_08        0x2
	#define VDS_OCP_LVL_0_09        0x3
	#define VDS_OCP_LVL_0_1         0x4
	#define VDS_OCP_LVL_0_2         0x5
	#define VDS_OCP_LVL_0_3         0x6
	#define VDS_OCP_LVL_0_4         0x7
	#define VDS_OCP_LVL_0_5         0x8
	#define VDS_OCP_LVL_0_6         0x9     
	#define VDS_OCP_LVL_0_7         0xA
	#define VDS_OCP_LVL_1_8         0xB
	#define VDS_OCP_LVL_1_9         0xC
	#define VDS_OCP_LVL_1_0         0xD   //Default : VDS_OCP_LVL = 1.0 V
	#define VDS_OCP_LVL_1_5         0xE
	#define VDS_OCP_LVL_2_0         0xF
#endif
 
/// ****** CSA Control Fields ******///  (address = 0x06h)
#ifdef HAS_CSA_Control_Fields
	#define CSA_FET_SP          0x0     /// Current sense amplifier positive input, set if Use Low-side sampling resistor
	#define CSA_FET_SH          0x1     /* set if Use Use the equivalent resistance of the MOSFET as the Low-side sampling resistor.
	                                       also automatically sets the LS_REF bit to 1 */
	 
	#define VREF_DIV_1          0x0     /// Amplifier reference voltage is VREV/1
	#define VREF_DIV_2          0x1     /// Default : Amplifier reference voltage is VREV/2
	
	#define LS_REF_SHtoSP       0x0     ///VDS_OCP for the low-side MOSFET is measured across SHx to SPx
	#define LS_REF_SHtoSN       0x1     
	
	#define CSA_GAIN_5          0x0     /// Current sensor gain
	#define CSA_GAIN_10         0x1
	#define CSA_GAIN_20         0x2     /// Default : 
	#define CSA_GAIN_40         0x3
	 
	#define DIS_SEN_EN          0x0     /// Sense over current fault is enabled
	#define DIS_SEN_DIS         0x1
	
	#define CSA_CAL_A_DIS       0x0        
	#define CSA_CAL_A_EN        0x1     ///Short inputs to sense amplifier A for offset calibration, should set COAST_EN or BRAKE_EN
	#define CSA_CAL_B_DIS       0x0        
	#define CSA_CAL_B_EN        0x1
	#define CSA_CAL_C_DIS       0x0        
	#define CSA_CAL_C_EN        0x1
	 
	#define SEN_OCP_LVL_0_25        0x0     /// Sense OCP voltage level 
	#define SEN_OCP_LVL_0_5         0x1
	#define SEN_OCP_LVL_0_75        0x2
	#define SEN_OCP_LVL_1_0         0x3     /// Default : Sense OCP voltage level = 1.0V
#endif    
    
/// ****** Driver Configuration Fields ******///  (address = 0x07h)
#ifdef HAS_CAL_MODE
	#define CAL_MODE_Manual    0x0    //Set CSA_CAL_A/B/C_EN and Manually read the ADC, calculate the offset, and save/compensate it in software.
	#define CAL_MODE_Auto      0x1    //Amplifier calibration uses internal auto calibration routine
#endif	

 
#ifdef _cplusplus
}
#endif
 
 
#endif  /* __DRV_DRV83XX_REG_H */


