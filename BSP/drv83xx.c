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
 

 
/***********************************   SPI Interface Begin *********************************/
/* Private macro -------------------------------------------------------------*/
extern SPI_HandleTypeDef        hspi2;
 
/* Private define ------------------------------------------------------------*/
#define DEBUG
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
   .VDS_LVL =  VDS_OCP_LVL_0_94,
   .OCP_DEG =  OCP_DEG_6US,
   .OCP_MODE = OCP_Mode_REPORT,
   .DEAD_TIME = DEADTIME_400NS,
	    
    // CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
#ifdef HAS_CSA_Control_Fields
   .SEN_LVL = SEN_OCP_LVL_0_25,   //  00b = Sense OCP 0.25 V
   .CSA_GAIN = CSA_GAIN_5,   //  03b = 40-V/V shunt amplifier gain
   .VREF_DIV = VREF_DIV_2,    //  1b = Sense amplifier reference voltage is VREF divided by 2
	 .CSA_CAL_A = CSA_CAL_A_DIS,
	 .CSA_CAL_B = CSA_CAL_B_DIS,
	 .CSA_CAL_C = CSA_CAL_C_DIS,
	  
#endif    
};
 
/* Private function prototypes -----------------------------------------------*/
static uint16_t read_reg(uint16_t address);
static int write_reg(uint16_t address, uint16_t data);
 
void DRV83XX_Init( void )
{
    DRV835X_ENABLE_LOW;
    Delay_ms(10);
    DRV835X_ENABLE_HIGH;
    Delay_ms(10);

    if(0 != DRV83XX_updateCfgPara(&stru_DRV8323Obj, stru_config))
		{
#ifdef DEBUG
			DEBUG_OUT("DRV83XX write reg failed\n");
#endif
		}
	  Delay_ms(100);
		DRV83XX_read_ALLReg(&stru_DRV8323Obj);

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
 