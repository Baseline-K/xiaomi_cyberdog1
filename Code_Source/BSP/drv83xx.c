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
#include "drv83xx.h"
#include "drv83xx_reg.h"
#include <stdio.h>          /* DRV83XX_PrintConfig 打印寄存器真值 */ 
 

 
/***********************************   SPI Interface Begin *********************************/
/* Private macro -------------------------------------------------------------*/
extern SPI_HandleTypeDef        hspi2;
 
/* Private define ------------------------------------------------------------*/
/* CMakeLists.txt 在 Debug 配置里已经用 -DDEBUG 定义过（第 8 行），
 * 这里加 #ifndef 只是兜底，否则每次编译都会报 "DEBUG redefined"。 */
#ifndef DEBUG
#define DEBUG
#endif
#define DEBUG_OUT(fmt, ...)   //printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

#define TIME_OUT                10
#define DEFAULT_GAIN            10
#define SYSCLK  120000000
#define Drv83xx_Commu_delayus   50     //The interval us between two communications

// DRV8353 SPI CS PIN 
#define DRV835X_CS_EN           HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define DRV835X_CS_DIS          HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
 
// DRV8353 ENABLE PIN 
/*
    Gate driver enable. When this pin is logic low the device goes to a low power sleep mode. 
    An 8 to 40-µs low pulse can be used to reset fault conditions.
*/
#define DRV835X_ENABLE_LOW      HAL_GPIO_WritePin(DRV8323_EN_GPIO_Port,DRV8323_EN_Pin,GPIO_PIN_RESET)
#define DRV835X_ENABLE_HIGH     HAL_GPIO_WritePin(DRV8323_EN_GPIO_Port,DRV8323_EN_Pin,GPIO_PIN_SET)
 


static int spi_transmit(uint16_t data) {
  int status;
	uint16_t data_response;
  //status = HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, 1, TIME_OUT);  //16-bit Mode
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&data, (uint8_t*)&data_response, 1, HAL_MAX_DELAY);
  return status;
}

static int spi_receive(uint16_t data, uint16_t *data_response) {
  int status;
  //status = HAL_SPI_Receive(&hspi2, (uint8_t *)&data, 1, TIME_OUT);   //16-bit Mode
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&data, (uint8_t*)data_response, 1, HAL_MAX_DELAY);
  return status;
}

static void Delay_ms(int ms)
{
	HAL_Delay(ms);
}


static void Delay_us(int _us)
{
	 uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
 
    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
    ticks = _us * (SYSCLK / 1000000);
 
    // 获取当前的SysTick值
    told = SysTick->VAL;
 
    while (1)
    {
        // 重复刷新获取当前的SysTick值
        tnow = SysTick->VAL;
 
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;
 
            told = tnow;
 
            // 如果达到了需要的时钟数，就退出循环
            if (tcnt >= ticks)
                break;
        }
    }
}
/***********************************   SPI Interface End *********************************/

 

/* Private variables ---------------------------------------------------------*/
Stru_DRV83XX stru_DRV8323Obj;
 
StruDRV83XXCfgPara stru_config = 
{
    // Driver Control Register (address = 0x02h)
   .PWM_MODE = PWM_MODE_3X,

    
   //Gate Drive HS Register (address = 0x03h)
   .IDRIVEP_HS = IDRIVEP_HS_1000MA,
   .IDRIVEN_HS = IDRIVEN_HS_2000MA,
   .LOCK = LOCK_OFF,
    
   // Gate Drive LS Register (address = 0x04h) 
	 .IDRIVEP_LS = IDRIVEP_LS_1000MA,
   .IDRIVEN_LS = IDRIVEN_LS_2000MA,
   .TDRIVE = TDRIVE_4000NS,
   .CBC = PWM_GIVER_ENABLE,  // 1b = For VDS_OCP and SEN_OCP, the fault is cleared when
                             // a new PWM input is given or after tRETRY
	
    // OCP Control Register (address = 0x05h)
    // 三层过流保护的分工（从快到慢）：
    //   1. DRV VDS_OCP（本寄存器）：硬件比较器，~几 us 内自己关栅极，不依赖 MCU。
    //      判据是 Rds(on)+分流上的压降。
    //   2. DRV SEN_OCP（CSACR.SEN_LVL）：比较放大后的 SOx 电压，另一路硬件兜底。
    //   3. 软件过流（Safety_FastStep，10kHz）：48A 过流 / 60A 短路，最灵活但要等一个 ISR 周期。
    // 注意 .CBC 只在 OCP_MODE=RETRY(01b) 时对 VDS_OCP/SEN_OCP 有意义；
    // 本工程 OCP_MODE=LATCH，所以 CBC 不参与 OCP 行为（保留默认值即可）。
   .VDS_LVL =  VDS_OCP_LVL_0_45,   // 0x5 = 0.45V。
                                   // STL140N6F7(60V, Rds(on) 2.4mΩ typ) + 1mΩ 分流：
                                   // 热态 0.45V/(4.3~6.0mΩ+1mΩ) ≈ 69A，冷态 ≈ 132A。
                                   // 落在软件过流 48A 之上，作为"MCU 没来得及反应"时的硬件兜底。
   .OCP_DEG =  OCP_DEG_6US,        // 6us 消隐，躲开开关瞬间的电流尖峰
   .OCP_MODE = OCP_Mode_LATCH,     // 0b00 = 过流锁存：DRV 自己把栅极关死并保持，
                                   // 只有读 FSR1/FSR2 才能解锁（motor_state_machine.c 的
                                   // state_FAULT_do 会读）。这样即使 MCU 侧卡死，硬件也已断开。
   .DEAD_TIME = DEADTIME_400NS,
	    
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
#ifdef HAS_CSA_Control_Fields
   .SEN_LVL = SEN_OCP_LVL_1_0,    // 0x3 = 1.0V（本字段最高档）。
                                  // 阈值是 SOx 引脚（放大后）电压：增益 20 + Rs 1mΩ → 20mV/A，
                                  // 所以 1.0V ≈ 50A，正好落在软件过流 48A 之上、Current_Max×1.25，
                                  // 作为第二层硬件兜底。
                                  // 原值 0.25V 在旧硬件(增益5)下可用，但换成 1mΩ+增益20 后
                                  // 只对应 ~12.5A，正常 20A 峰值运行就会持续误报 —— 必须抬到最高档。
   .CSA_GAIN = CSA_GAIN_20,   //  10b = 20-V/V 电流放大器增益
                              //  !! 必须与 FOC_run.h 的 CSA_GAIN_VV = 20.0f 一致（1mΩ 分流）。
                              //  不一致 → I_SCALE 错 → 所有电流读数与过流阈值成比例错。
   .VREF_DIV = VREF_DIV_2,    //  1b = Sense amplifier reference voltage is VREF divided by 2
	 .CSA_CAL_A = CSA_CAL_A_DIS,
	 .CSA_CAL_B = CSA_CAL_B_DIS,
	 .CSA_CAL_C = CSA_CAL_C_DIS,
	  
#endif    
};
 
/* Private function prototypes -----------------------------------------------*/
static uint16_t read_reg(uint16_t address);
static int write_reg(uint16_t address, uint16_t data);
 
int DRV83XX_Init( void )
{
    int ret = 0;

    DRV835X_ENABLE_LOW;
    Delay_ms(10);
    DRV835X_ENABLE_HIGH;
    Delay_ms(10);

    if(0 != DRV83XX_updateCfgPara(&stru_DRV8323Obj, stru_config))
		{
			ret = -1;   /* write_reg 内部逐寄存器回读校验，非 0 就是配置没真正写进去 */
#ifdef DEBUG
			DEBUG_OUT("DRV83XX write reg failed\n");
#endif
		}
	  Delay_ms(100);
		DRV83XX_read_ALLReg(&stru_DRV8323Obj);

    return ret;
}

/* 启动期打印"寄存器真值"（重新读回来的，不是代码意图）。
 * 最要紧的是 CSA_GAIN：它决定 FOC_run.h 的 I_SCALE 对不对，
 * 也决定所有电流保护阈值对不对。main_user 会把它和 CSA_GAIN_VV 做一致性检查。 */
void DRV83XX_PrintConfig( void )
{
    printf("[DRV] DCR=0x%04X  PWM_MODE=%u COAST=%u\r\n",
           (unsigned)stru_DRV8323Obj.drvCtrl_obj.data,
           (unsigned)stru_DRV8323Obj.drvCtrl_obj.ctrlRegObj.PWM_MODE,
           (unsigned)stru_DRV8323Obj.drvCtrl_obj.ctrlRegObj.COAST);
    printf("[DRV] CSACR=0x%04X  CSA_GAIN=%u SEN_LVL=%u VREF_DIV=%u DIS_SEN=%u\r\n",
           (unsigned)stru_DRV8323Obj.drvCsa_obj.data,
           (unsigned)stru_DRV8323Obj.drvCsa_obj.csaObj.CSA_GAIN,
           (unsigned)stru_DRV8323Obj.drvCsa_obj.csaObj.SEN_LVL,
           (unsigned)stru_DRV8323Obj.drvCsa_obj.csaObj.VREF_DIV,
           (unsigned)stru_DRV8323Obj.drvCsa_obj.csaObj.DIS_SEN);
    printf("[DRV] OCPCR=0x%04X  VDS_LVL=%u OCP_DEG=%u OCP_MODE=%u DEAD_TIME=%u\r\n",
           (unsigned)stru_DRV8323Obj.drvOcp_obj.data,
           (unsigned)stru_DRV8323Obj.drvOcp_obj.ocpObj.VDS_LVL,
           (unsigned)stru_DRV8323Obj.drvOcp_obj.ocpObj.OCP_DEG,
           (unsigned)stru_DRV8323Obj.drvOcp_obj.ocpObj.OCP_MODE,
           (unsigned)stru_DRV8323Obj.drvOcp_obj.ocpObj.DEAD_TIME);
    printf("[DRV] FSR1=0x%04X FSR2=0x%04X\r\n",
           (unsigned)stru_DRV8323Obj.faultStatusReg1_obj.data,
           (unsigned)stru_DRV8323Obj.faultStatusReg2_obj.data);
}

/* 读 FSR1/FSR2 —— 读操作本身就会清除 DRV 内部的故障锁存。
 * OCP_MODE=LATCH 时这是唯一的解锁途径（否则栅极一直被 DRV 自己按住不放）。
 *
 * 返回 1 = nFAULT(PA12) 已释放 → 故障确实清了；
 *      0 = 仍为低        → 真故障还在，调用方（state_FAULT_do）下拍重试。
 *
 * 必须在**任务上下文**调用：内部有阻塞 SPI 和 printf。
 * 现场只有这一份打印 —— 寄存器读一次就清一次，读完就再也拿不到原始内容了。 */
int DRV83XX_ReadAndClearFaults( void )
{
    Fault_StatusReg1_bit *f1;
    Fault_StatusReg2_bit *f2;
    uint8_t nfault;

    stru_DRV8323Obj.faultStatusReg1_obj.data = read_reg( FSR1 );   /* 读即清锁存 */
    stru_DRV8323Obj.faultStatusReg2_obj.data = read_reg( FSR2 );

    f1 = &stru_DRV8323Obj.faultStatusReg1_obj.fault1RegObj;
    f2 = &stru_DRV8323Obj.faultStatusReg2_obj.fault2RegObj;

    printf("[DRV] FAULT read: FSR1=0x%04X FSR2=0x%04X\r\n",
           (unsigned)stru_DRV8323Obj.faultStatusReg1_obj.data,
           (unsigned)stru_DRV8323Obj.faultStatusReg2_obj.data);

    /* ---- FSR1：功率级 / 供电类 ---- */
    if (f1->FAULT)   printf("[DRV]   FSR1.FAULT   =1 器件报故障\r\n");
    if (f1->VDS_OCP) printf("[DRV]   FSR1.VDS_OCP =1 VDS 过流（LATCH 已锁存）\r\n");
    if (f1->GDF)     printf("[DRV]   FSR1.GDF     =1 栅极驱动故障\r\n");
    if (f1->UVLO)    printf("[DRV]   FSR1.UVLO    =1 VM 欠压\r\n");
    if (f1->OTSD)    printf("[DRV]   FSR1.OTSD    =1 过温关断\r\n");
    if (f1->VDS_HA || f1->VDS_LA || f1->VDS_HB || f1->VDS_LB || f1->VDS_HC || f1->VDS_LC) {
        printf("[DRV]   FSR1 VDS 具体管 HA=%u LA=%u HB=%u LB=%u HC=%u LC=%u\r\n",
               (unsigned)f1->VDS_HA, (unsigned)f1->VDS_LA,
               (unsigned)f1->VDS_HB, (unsigned)f1->VDS_LB,
               (unsigned)f1->VDS_HC, (unsigned)f1->VDS_LC);
    }

    /* ---- FSR2：栅极驱动 / 电流放大器类 ----
     * 字段名照抄 drv83xx.h 的 Fault_StatusReg2_bit（8323 的 FSR2 与 8353 不同，
     * 这里没有 VGS_HA/VGS_LA，A 相那两位叫 VDS_HA/VDS_LA）。 */
    if (f2->GDUV)  printf("[DRV]   FSR2.GDUV  =1 栅极驱动欠压\r\n");
    if (f2->OTW)   printf("[DRV]   FSR2.OTW   =1 过温预警\r\n");
    if (f2->SA_OC || f2->SB_OC || f2->SC_OC) {
        printf("[DRV]   FSR2 SEN 过流 SA=%u SB=%u SC=%u\r\n",
               (unsigned)f2->SA_OC, (unsigned)f2->SB_OC, (unsigned)f2->SC_OC);
    }
    if (f2->VGS_HC || f2->VGS_LC || f2->VGS_HB || f2->VGS_LB || f2->VDS_HA || f2->VDS_LA) {
        printf("[DRV]   FSR2 逐管 VGS_HC=%u VGS_LC=%u VGS_HB=%u VGS_LB=%u VDS_HA=%u VDS_LA=%u\r\n",
               (unsigned)f2->VGS_HC, (unsigned)f2->VGS_LC,
               (unsigned)f2->VGS_HB, (unsigned)f2->VGS_LB,
               (unsigned)f2->VDS_HA, (unsigned)f2->VDS_LA);
    }

    nfault = (HAL_GPIO_ReadPin(DRV8232_nFAULT_GPIO_Port, DRV8232_nFAULT_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    printf("[DRV]   nFAULT(PA12)=%u -> %s\r\n", (unsigned)nfault,
           (nfault != 0U) ? "released (fault cleared)" : "still low (retry next tick)");

    return (int)nfault;
}
 

int DRV83XX_updateCfgPara(Stru_DRV83XX *stru_DRV83xxObj, StruDRV83XXCfgPara stru_config)
{
    int Status;
    uint16_t data;
    // Driver Control Register (address = 0x02h)
    stru_DRV83xxObj->drvCtrl_obj.ctrlRegObj.PWM_MODE  = stru_config.PWM_MODE;
    data = stru_DRV83xxObj->drvCtrl_obj.data;
    Status = Status | write_reg( DCR, data);
 
    //Gate Drive HS Register (address = 0x03h)
    stru_DRV83xxObj->drvGateHS_obj.gateHSRegObj.IDRIVEP_HS = stru_config.IDRIVEP_HS;
    stru_DRV83xxObj->drvGateHS_obj.gateHSRegObj.IDRIVEN_HS = stru_config.IDRIVEN_HS;
    stru_DRV83xxObj->drvGateHS_obj.gateHSRegObj.LOCK = stru_config.LOCK;
    data = stru_DRV83xxObj->drvGateHS_obj.data;
    Status = Status | write_reg( HSR, data);

    // Gate Drive LS Register (address = 0x04h)
    stru_DRV83xxObj->drvGateLS_obj.gateLSRegObj.IDRIVEN_LS = stru_config.IDRIVEN_LS;
    stru_DRV83xxObj->drvGateLS_obj.gateLSRegObj.IDRIVEP_LS = stru_config.IDRIVEP_LS;
    stru_DRV83xxObj->drvGateLS_obj.gateLSRegObj.TDRIVE = stru_config.TDRIVE;
    stru_DRV83xxObj->drvGateLS_obj.gateLSRegObj.CBC = stru_config.CBC;
    data = stru_DRV83xxObj->drvGateLS_obj.data;
    Status = Status | write_reg( LSR, data);
 
    // OCP Control Register (address = 0x05h)
    stru_DRV83xxObj->drvOcp_obj.ocpObj.VDS_LVL =  stru_config.VDS_LVL;
    stru_DRV83xxObj->drvOcp_obj.ocpObj.OCP_DEG =  stru_config.OCP_DEG;
    stru_DRV83xxObj->drvOcp_obj.ocpObj.OCP_MODE = stru_config.OCP_MODE;
    stru_DRV83xxObj->drvOcp_obj.ocpObj.DEAD_TIME = stru_config.DEAD_TIME;
    data = stru_DRV83xxObj->drvOcp_obj.data;
    Status = Status | write_reg( OCPCR, data);
 
 
	    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
#ifdef HAS_CSA_Control_Fields
    stru_DRV83xxObj->drvCsa_obj.csaObj.SEN_LVL  = stru_config.SEN_LVL;
    stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_GAIN = stru_config.CSA_GAIN;
    stru_DRV83xxObj->drvCsa_obj.csaObj.VREF_DIV = stru_config.VREF_DIV;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_A = stru_config.CSA_CAL_A;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_B = stru_config.CSA_CAL_B;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_C = stru_config.CSA_CAL_C;
		data = stru_DRV83xxObj->drvCsa_obj.data;
    Status = Status | write_reg( CSACR, data);
#endif   

    return Status;
} 

static uint16_t read_reg(uint16_t address)
{
	  uint16_t data, data_response;
    Input_WrReg stru_Input_WrRegObj;
    stru_Input_WrRegObj.inputRegObj.WR =  R_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    data = stru_Input_WrRegObj.data;
    
    DRV835X_CS_EN;
	  spi_receive(data, &data_response);
    DRV835X_CS_DIS;
    Delay_us(Drv83xx_Commu_delayus);
    
    return (data_response & 0x7FF);
}
 

static int write_reg(uint16_t address, uint16_t data)
{
    Input_WrReg stru_Input_WrRegObj;
    uint16_t data_response;
    stru_Input_WrRegObj.inputRegObj.WR =  W_MODE;
    stru_Input_WrRegObj.inputRegObj.ADDRESS = address;
    stru_Input_WrRegObj.inputRegObj.DATA = data;
    data = stru_Input_WrRegObj.data;

		DRV835X_CS_EN;
		spi_transmit(data);
		DRV835X_CS_DIS;
		Delay_us(Drv83xx_Commu_delayus);
	  data_response = read_reg(address);
		if(data_response != (data & 0x7FF))
		{
			 return -1;
		} 
    
    return 0;
}
 

void DRV83XX_read_ALLReg(Stru_DRV83XX *stru_DRV83xxObj)
{
	  stru_DRV83xxObj->faultStatusReg1_obj.data = read_reg( FSR1 );
    stru_DRV83xxObj->faultStatusReg2_obj.data = read_reg( FSR2 );
	
    stru_DRV83xxObj->drvCtrl_obj.data = read_reg( DCR );
	  stru_DRV83xxObj->drvGateHS_obj.data = read_reg( HSR );
    stru_DRV83xxObj->drvGateLS_obj.data = read_reg( LSR );
	
    stru_DRV83xxObj->drvOcp_obj.data = read_reg( OCPCR );
    stru_DRV83xxObj->drvCsa_obj.data = read_reg( CSACR );
    stru_DRV83xxObj->drvCfg_obj.data = read_reg( DFGCR );
}
 
void DRV83XX_read_FaultStatusReg1(Stru_DRV83XX *stru_DRV83xxObj)
{
    stru_DRV83xxObj -> faultStatusReg1_obj .data = read_reg( FSR1 );
}
 
void DRV83XX_read_FaultStatusReg2(Stru_DRV83XX *stru_DRV83xxObj)
{
    stru_DRV83xxObj -> faultStatusReg2_obj.data = read_reg( FSR2 );
}
 
int DRV8323_COAST_Disable(Stru_DRV83XX *stru_DRV83xxObj)
{
		stru_DRV83xxObj->drvCtrl_obj.data = read_reg(DCR);
	  stru_DRV83xxObj->drvCtrl_obj.ctrlRegObj.COAST  = COAST_DIS;
    return write_reg(DCR, stru_DRV83xxObj->drvCtrl_obj.data);
}

int DRV8323_COAST_enable(Stru_DRV83XX *stru_DRV83xxObj)
{
	  stru_DRV83xxObj->drvCtrl_obj.data = read_reg(DCR);
	  stru_DRV83xxObj->drvCtrl_obj.ctrlRegObj.COAST  = COAST_EN;
    return write_reg(DCR, stru_DRV83xxObj->drvCtrl_obj.data);
}

int DRV8323_CurrentCAL_enable(Stru_DRV83XX *stru_DRV83xxObj)
{
		stru_DRV83xxObj->drvCsa_obj.data = read_reg(CSACR);
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_A = CSA_CAL_A_EN;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_B = CSA_CAL_B_EN;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_C = CSA_CAL_C_EN;
    return write_reg(DCR, stru_DRV83xxObj->drvCtrl_obj.data);
}

int DRV8323_CurrentCAL_disable(Stru_DRV83XX *stru_DRV83xxObj)
{
		stru_DRV83xxObj->drvCsa_obj.data = read_reg(CSACR);
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_A = CSA_CAL_A_DIS;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_B = CSA_CAL_B_DIS;
		stru_DRV83xxObj->drvCsa_obj.csaObj.CSA_CAL_C = CSA_CAL_C_DIS;
		return write_reg(DCR, stru_DRV83xxObj->drvCtrl_obj.data);
}

/* End of this file */
 