#include "AS5600.h"
#include "arm_math.h"
#include "Soft_IIC.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f


/********************* IIC Adapter *************************/
static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  //status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, (dev_addr<<1), pData, count, i2c_time_out);
	
	status = softi2c_master_write(&g_softi2c, dev_addr, pData, count);

  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  //status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, ((dev_addr<<1) | 1), pData, count, i2c_time_out);
	
	status = softi2c_master_read(&g_softi2c, dev_addr, pData, count);
	
  return status;
}

static int as5600_select_raw_angle(void)
{
    uint8_t reg = AS5600_RAW_ANGLE_REGISTER;
    return i2cWrite(AS5600_RAW_ADDR, &reg, 1U);
}

/*
 * Configuration accesses change the AS5600 internal address pointer. The fast
 * angle path intentionally uses address-pointer readback, so always restore the
 * pointer to RAW_ANGLE before returning. These APIs must not run concurrently
 * with the FOC ISR angle read.
 */
int AS5600_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    int ret;
    int restore_ret;

    if ((data == NULL) || (length == 0U)) return SOFTI2C_ARG_ERROR;

    ret = softi2c_master_mem_read(&g_softi2c, AS5600_RAW_ADDR,
                                  reg, 1U, data, length);
    restore_ret = as5600_select_raw_angle();
    return (ret != SOFTI2C_OK) ? ret : restore_ret;
}

int AS5600_WriteRegisters(uint8_t reg, const uint8_t *data, uint16_t length)
{
    int ret;
    int restore_ret;

    if ((data == NULL) || (length == 0U) ||
        (reg == AS5600_BURN_REGISTER)) return SOFTI2C_ARG_ERROR;

    ret = softi2c_master_mem_write(&g_softi2c, AS5600_RAW_ADDR,
                                   reg, 1U, data, length);
    restore_ret = as5600_select_raw_angle();
    return (ret != SOFTI2C_OK) ? ret : restore_ret;
}

int AS5600_ReadConfigRaw(uint16_t *raw_config)
{
    uint8_t data[2];
    int ret;

    if (raw_config == NULL) return SOFTI2C_ARG_ERROR;

    ret = AS5600_ReadRegisters(AS5600_CONF_HIGH_REGISTER, data, 2U);
    if (ret != SOFTI2C_OK) return ret;

    *raw_config = ((uint16_t)data[0] << 8U) | data[1];
    return SOFTI2C_OK;
}

int AS5600_WriteConfigRaw(uint16_t raw_config, bool verify)
{
    uint8_t data[2];
    uint16_t current;
    uint16_t value_to_write;
    uint16_t readback;
    int ret;

    /* Datasheet: blank CONF fields can contain factory settings. Preserve them. */
    ret = AS5600_ReadConfigRaw(&current);
    if (ret != SOFTI2C_OK) return ret;
    value_to_write = (current & AS5600_CONF_RESERVED_MASK) |
                     (raw_config & AS5600_CONF_MASK);

    data[0] = (uint8_t)(value_to_write >> 8U);
    data[1] = (uint8_t)value_to_write;
    ret = AS5600_WriteRegisters(AS5600_CONF_HIGH_REGISTER, data, 2U);
    if (ret != SOFTI2C_OK) return ret;

    /* Datasheet requires at least 1 ms for a new configuration to take effect. */
    HAL_Delay(1U);
    if (!verify) return SOFTI2C_OK;

    ret = AS5600_ReadConfigRaw(&readback);
    if (ret != SOFTI2C_OK) return ret;
    return (readback == value_to_write) ? SOFTI2C_OK : SOFTI2C_BUS_ERROR;
}

int AS5600_ReadConfig(AS5600_Config_t *config)
{
    uint16_t raw;
    int ret;

    if (config == NULL) return SOFTI2C_ARG_ERROR;
    ret = AS5600_ReadConfigRaw(&raw);
    if (ret != SOFTI2C_OK) return ret;

    config->power_mode = (AS5600_PowerMode_t)(raw & 0x3U);
    config->hysteresis = (AS5600_Hysteresis_t)((raw >> 2U) & 0x3U);
    config->output_stage = (AS5600_OutputStage_t)((raw >> 4U) & 0x3U);
    config->pwm_frequency = (AS5600_PwmFrequency_t)((raw >> 6U) & 0x3U);
    config->slow_filter = (AS5600_SlowFilter_t)((raw >> 8U) & 0x3U);
    config->fast_filter_threshold =
        (AS5600_FastFilterThreshold_t)((raw >> 10U) & 0x7U);
    config->watchdog_enable = ((raw & (1U << 13U)) != 0U);
    return SOFTI2C_OK;
}

int AS5600_WriteConfig(const AS5600_Config_t *config, bool verify)
{
    uint16_t raw;

    if (config == NULL) return SOFTI2C_ARG_ERROR;
    if (((uint32_t)config->power_mode > 3U) ||
        ((uint32_t)config->hysteresis > 3U) ||
        ((uint32_t)config->output_stage > 2U) ||
        ((uint32_t)config->pwm_frequency > 3U) ||
        ((uint32_t)config->slow_filter > 3U) ||
        ((uint32_t)config->fast_filter_threshold > 7U)) {
        return SOFTI2C_ARG_ERROR;
    }

    raw = (uint16_t)config->power_mode |
          ((uint16_t)config->hysteresis << 2U) |
          ((uint16_t)config->output_stage << 4U) |
          ((uint16_t)config->pwm_frequency << 6U) |
          ((uint16_t)config->slow_filter << 8U) |
          ((uint16_t)config->fast_filter_threshold << 10U);
    if (config->watchdog_enable) raw |= (uint16_t)(1U << 13U);

    return AS5600_WriteConfigRaw(raw, verify);
}

int AS5600_SetFilterConfig(AS5600_SlowFilter_t slow_filter,
                           AS5600_FastFilterThreshold_t fast_filter_threshold,
                           bool verify)
{
    AS5600_Config_t config;
    int ret;

    if (((uint32_t)slow_filter > 3U) ||
        ((uint32_t)fast_filter_threshold > 7U)) {
        return SOFTI2C_ARG_ERROR;
    }

    ret = AS5600_ReadConfig(&config);
    if (ret != SOFTI2C_OK) return ret;
    config.slow_filter = slow_filter;
    config.fast_filter_threshold = fast_filter_threshold;
    return AS5600_WriteConfig(&config, verify);
}


/******************* General Function **********************/
static inline int bsp_as5600GetRawAngle(uint16_t *raw_angle) {
	int ret;
	
  uint8_t buffer[2] = {0};
  if (raw_angle == NULL) return SOFTI2C_ARG_ERROR;
//  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
//  ret = i2cWrite(AS5600_RAW_ADDR, &raw_angle_register, 1);
  ret = i2cRead(AS5600_RAW_ADDR, buffer, 2);
  if (ret != SOFTI2C_OK) return ret;

  *raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];

  return SOFTI2C_OK;
}

int bsp_as5600GetAngle(volatile float *Angle) {
	int ret;
  uint16_t angle_data;
	float angle;
	if (Angle == NULL) return SOFTI2C_ARG_ERROR;
	ret =	bsp_as5600GetRawAngle(&angle_data);
	if (ret != SOFTI2C_OK) return ret;

	angle = (angle_data / (float)AS5600_RESOLUTION)*_2PI;
	angle = fmodf(angle + Encoder_AS5600.angle_zero_offset, _2PI);  // fmodf 保留符号，所以我们要修正负值
	if (angle < 0.0f) angle += _2PI;
	
	*Angle = angle;
  return ret;
}



// Read STATUS register (1 byte)
int AS5600_ReadStatus(uint8_t *status)
{
	if (status == NULL) return SOFTI2C_ARG_ERROR;
    return AS5600_ReadRegisters(AS5600_STATUS_REGISTER, status, 1U);
}

// Execute BURN_ANGLE (write 0x80 into 0xFF). Caller must ensure MD=1 and hardware power requirements.
// WARNING: irreversible and limited number of times. Check datasheet before using.
int AS5600_BurnAngle(void)
{
    uint8_t buf[2];
    int ret;

    buf[0] = AS5600_BURN_REGISTER;
    buf[1] = 0x80U; // BURN_ANGLE command

    ret = i2cWrite(AS5600_RAW_ADDR, buf, 2);
    if (ret != SOFTI2C_OK) {
        (void)as5600_select_raw_angle();
        return ret;
    }

    // datasheet recommends waiting a bit and optionally verifying; wait small time
    HAL_Delay(5);
    return as5600_select_raw_angle();
}


//配置AS5600的起始位置（ZPOS）
int bsp_AS5600_SetZero(uint8_t Burn)
{
	int ret;
	uint16_t RAWangle, ZPOS_RAWangle; 
	
	uint8_t zpos[2];
	ret = bsp_as5600GetRawAngle(&RAWangle);
	if (ret != SOFTI2C_OK) return ret;
	uint8_t msb = (RAWangle >> 8) & 0x0F; // ZPOS(11:8)
  uint8_t lsb = RAWangle & 0xFF;        // ZPOS(7:0)
	
  zpos[0] = msb;
  zpos[1] = lsb;
  ret = AS5600_WriteRegisters(AS5600_ZPOS11_8_REGISTER, zpos, 2U);
	if (ret != SOFTI2C_OK) return ret;
  HAL_Delay(2);
	
	/******* Verify *******/
	uint8_t buffer[2] = {0};
  ret = AS5600_ReadRegisters(AS5600_ZPOS11_8_REGISTER, buffer, 2U);
  if (ret != SOFTI2C_OK) return ret;
  ZPOS_RAWangle = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]) & 0x0FFFU;
	
	if((Burn == 1)&&(ZPOS_RAWangle == RAWangle)){
		uint8_t status;
		if (AS5600_ReadStatus(&status) == SOFTI2C_OK) {
			 if (status & AS5600_STATUS_MD_MASK) {
				 // magnet detected -> allowed to burn
				 // double-check power, caps, magnet position before calling:
				 ret = AS5600_BurnAngle();
			 } 
			 else {
				 // magnet not detected: do not burn
			 }
	  }
  } 
	
  return ret;
}


/******************* User Function **********************/

Encoder_AS5600_t   Encoder_AS5600;
AS5600_StartupDiagnostics_t AS5600_StartupDiagnostics;

void bsp_as5600ResetTracking(void)
{
	/* This function is safe for run-state transitions: it never touches I2C. */
	Encoder_AS5600.angle_last = Encoder_AS5600.angle;
	Encoder_AS5600.Speed_angle_deta = 0.0f;
	Encoder_AS5600.Speed_angle_deta_last = 0.0f;
	Encoder_AS5600.Speed_RPM = 0.0f;
	Encoder_AS5600.Speed_RPM_last = 0.0f;
	Encoder_AS5600.Move_State = 0U;
	Encoder_AS5600.rotation_circles = 0U;
	Encoder_AS5600.rotation_circles_absolute = 0U;
	Encoder_AS5600.CCW_angle_total = 0.0f;
	Encoder_AS5600.CW_angle_total = 0.0f;
}

int bsp_as5600HwInit(void)
{
	int angle_ret;

	/* Hardware setup and non-angle register access are startup-only operations. */
	SoftI2C_RegisterAndInit(1000000U, SystemCoreClock);

	/*
	 * AS5600 power-on default SF=16x/FTH=slow-only has about 2.2 ms latency.
	 * With 7 pole pairs this is about 89 electrical degrees at 16 RPS and
	 * 111 electrical degrees at 20 RPS. The delayed rotor angle rotates q-axis
	 * voltage/current into the d axis and caused the observed high-speed runaway
	 * and flux-identification failure.
	 *
	 * SF=2x reduces the specified latency to about 0.286 ms (about 14.4
	 * electrical degrees at 20 RPS). Keep FTH=slow-only so the delay remains
	 * deterministic and does not jump when a fast-filter threshold is crossed.
	 * PM=NOM, HYST=OFF and WD=OFF are preserved by the read-modify-write API.
	 * This changes volatile CONF only (no OTP burn), so apply it on every boot.
	 */
	AS5600_StartupDiagnostics.filter_config_result =
		AS5600_SetFilterConfig(AS5600_SLOW_FILTER_2X,
		                       AS5600_FAST_FILTER_SLOW_ONLY,
		                       true);
	AS5600_StartupDiagnostics.config_read_result =
		AS5600_ReadConfigRaw(&AS5600_StartupDiagnostics.config_raw);
	AS5600_StartupDiagnostics.status_read_result =
		AS5600_ReadStatus(&AS5600_StartupDiagnostics.status);

	/* Configuration/status reads restore RAW_ANGLE; acquire the first sample. */
	angle_ret = bsp_as5600GetAngle(&Encoder_AS5600.angle);
	if (angle_ret != SOFTI2C_OK) return angle_ret;

	Encoder_AS5600.Speed_angle_deta_Min = 1.0f / 4096.0f * 2.0f * PI * 0.1f;
	Encoder_AS5600.n_timeout = 4U;
	Encoder_AS5600.dma_transfer_complete = true;
	bsp_as5600ResetTracking();

	if (AS5600_StartupDiagnostics.filter_config_result != SOFTI2C_OK) {
		return AS5600_StartupDiagnostics.filter_config_result;
	}
	if (AS5600_StartupDiagnostics.config_read_result != SOFTI2C_OK) {
		return AS5600_StartupDiagnostics.config_read_result;
	}
	return AS5600_StartupDiagnostics.status_read_result;
}

//计算转动了多少机械角度
float bsp_as5600_GetRotationAngle(float *RotationAngle) {
//  if (Encoder_AS5600. Move_State == 1){
//		Encoder_AS5600.rotation_angle = (Encoder_AS5600.rotation_circles * _2PI - Encoder_AS5600.angle_init + Encoder_AS5600.angle);
//		Encoder_AS5600.rotation_angle_absolute =  (Encoder_AS5600.rotation_circles_absolute * _2PI - Encoder_AS5600.angle_init + Encoder_AS5600.angle);
//  }
//	else{
//		Encoder_AS5600.rotation_angle = (Encoder_AS5600.rotation_circles * _2PI + Encoder_AS5600.angle_init - Encoder_AS5600.angle);
//		Encoder_AS5600.rotation_angle_absolute =  (Encoder_AS5600.rotation_circles_absolute * _2PI + Encoder_AS5600.angle_init - Encoder_AS5600.angle);
//	}	
	return 0;
}

#define Lpf(a,a_last,lpf)  (a * lpf + a_last * (1 - lpf))    
float bsp_as5600GetSpeed(float AS5600_SpeedFactor) {
 
	Encoder_AS5600.Speed_angle_deta_last = Encoder_AS5600.Speed_angle_deta;
	Encoder_AS5600.Speed_angle_deta = Encoder_AS5600.angle - Encoder_AS5600.angle_last;
	Encoder_AS5600.angle_last = Encoder_AS5600.angle;
	if(Encoder_AS5600.Speed_angle_deta > 0){   //初步判断方向
		Encoder_AS5600.Move_State = 1;
	}
	else{
		Encoder_AS5600.Move_State = 0;
	}
	
	if(Encoder_AS5600.Speed_angle_deta < - PI)  //正转
	{
		Encoder_AS5600.Move_State = 1;
		Encoder_AS5600.Speed_angle_deta =  (Encoder_AS5600.Speed_angle_deta + _2PI);
		Encoder_AS5600.rotation_circles_absolute++;
		Encoder_AS5600.rotation_circles++;
	}
	if(Encoder_AS5600.Speed_angle_deta >	PI)  //反转
	{
		Encoder_AS5600.Move_State = 0;
		Encoder_AS5600.Speed_angle_deta =  (Encoder_AS5600.Speed_angle_deta - _2PI);
		Encoder_AS5600.rotation_circles_absolute++;
		Encoder_AS5600.rotation_circles--;
	}
//	if(fabsf(Encoder_AS5600.Speed_angle_deta) >= mec_deta_speed_Max){
//			Encoder_AS5600.Speed_angle_deta = Encoder_AS5600.Speed_angle_deta_last;
//		}
	if(fabsf(Encoder_AS5600.Speed_angle_deta) <= Encoder_AS5600.Speed_angle_deta_Min) {   // 噪声门限：小于阈值视为 0，避免震荡导致正/负累计反复
			Encoder_AS5600.Speed_angle_deta = 0.0f;
	}
	
	if (Encoder_AS5600.Speed_angle_deta > 0.0f) {   // 更新单方向累计
			Encoder_AS5600.CCW_angle_total += Encoder_AS5600.Speed_angle_deta;
	} else if (Encoder_AS5600.Speed_angle_deta < 0.0f) {
			Encoder_AS5600.CW_angle_total += -Encoder_AS5600.Speed_angle_deta; // 存为正数：总的反转角度
	}	

	Encoder_AS5600.Speed_RPM_last = Encoder_AS5600.Speed_RPM;
	Encoder_AS5600.Speed_RPM = Encoder_AS5600.Speed_angle_deta * AS5600_SpeedFactor;
	Encoder_AS5600.Speed_RPM = Lpf(Encoder_AS5600.Speed_RPM,Encoder_AS5600.Speed_RPM_last,0.8f);
	return Encoder_AS5600.Speed_RPM;
}


//int bsp_as5600_DMAGetAngle(volatile float *Angle) {
//	HAL_StatusTypeDef st;
//	
//	/*DMA 通讯*/
//	if(Encoder_AS5600.dma_transfer_complete)
//  {
//    Encoder_AS5600.DMA_cnt = 0;
//    Encoder_AS5600.dma_transfer_complete = false;
//    // 使用 HAL 的 Mem_Read DMA（会在内部发送寄存器地址再 repeated start 读取）
//    st = HAL_I2C_Master_Receive_DMA(&AS5600_I2C_HANDLE, (AS5600_RAW_ADDR << 1), Encoder_AS5600.DMA_rxbuffer, 2);
//	}
//	else{
//		Encoder_AS5600.DMA_cnt++;
//	}
//	
//	/*DMA 超时判断*/
//	if(Encoder_AS5600.DMA_cnt >= Encoder_AS5600.n_timeout){
//		Encoder_AS5600.DMA_cnt = 0;
//		// 简单复位 I2C 状态（依 HAL 版本而异）
//		__HAL_I2C_DISABLE(&AS5600_I2C_HANDLE);
//		__HAL_I2C_ENABLE(&AS5600_I2C_HANDLE);
//		st = HAL_I2C_Master_Receive_DMA(&AS5600_I2C_HANDLE, (AS5600_RAW_ADDR << 1), Encoder_AS5600.DMA_rxbuffer, 2);
//	}
//	
//	//*Angle = Encoder_AS5600.angle;
//	/*获取编码器角度*/
//	return st;
//}

// 
//// DMA 传输完成回调函数  
//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if (hi2c == &AS5600_I2C_HANDLE) // 确保是我们的 I²C 设备
//    {
//        uint16_t raw_angle = ((uint16_t)Encoder_AS5600.DMA_rxbuffer[0] << 8) | (uint16_t)Encoder_AS5600.DMA_rxbuffer[1];
//	      Encoder_AS5600.angle = (raw_angle/(float)AS5600_RESOLUTION) * _2PI;
//			
//			  //获取到最新角度后要一次性在这里处理完，其他地方就不要再修改Encoder_AS5600.angle了
//        Encoder_AS5600.angle = fmodf(Encoder_AS5600.angle + Encoder_AS5600.angle_zero_offset, _2PI);  // fmodf 保留符号，所以我们要修正负值
//			  if (Encoder_AS5600.angle < 0.0f) Encoder_AS5600.angle += _2PI;
//			
//			  Encoder_AS5600.dma_transfer_complete = true;
//    }
//}

