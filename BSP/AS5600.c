#include "AS5600.h"
#include "arm_math.h"
#include "Soft_IIC.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f


/********************* IIC Adapter *************************/
static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE; 
  //status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, (dev_addr<<1), pData, count, i2c_time_out);
	
	status = softi2c_master_write(&g_softi2c, dev_addr, pData, count);

  return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  //status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, ((dev_addr<<1) | 1), pData, count, i2c_time_out);
	
	status = softi2c_master_read(&g_softi2c, dev_addr, pData, count);
	
  return status;
}

static inline int i2c_Mem_Read(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
  int status;
  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
  //status = HAL_I2C_Mem_Read(&AS5600_I2C_HANDLE, (dev_addr<<1), AS5600_RAW_ANGLE_REGISTER, I2C_MEMADD_SIZE_8BIT, pData, count, i2c_time_out);
	
	status = softi2c_master_mem_read(&g_softi2c, dev_addr, AS5600_RAW_ANGLE_REGISTER, 1, pData, count);
	
  return status;
}


/******************* General Function **********************/
static inline int bsp_as5600GetRawAngle(uint16_t *raw_angle) {
	int ret;
	
  uint8_t buffer[2] = {0};
//  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
//  ret = i2cWrite(AS5600_RAW_ADDR, &raw_angle_register, 1);
  ret = i2cRead(AS5600_RAW_ADDR, buffer, 2);
	
  *raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	
  return ret;
}

int bsp_as5600GetAngle(volatile float *Angle) {
	int ret;
  uint16_t angle_data;
	float angle;
	ret =	bsp_as5600GetRawAngle(&angle_data);

	angle = (angle_data / (float)AS5600_RESOLUTION)*_2PI;
	angle = fmodf(angle + Encoder_AS5600.angle_zero_offset, _2PI);  // fmodf 保留符号，所以我们要修正负值
	if (angle < 0.0f) angle += _2PI;
	
	*Angle = angle;
  return ret;
}



// Read STATUS register (1 byte)
int AS5600_ReadStatus(uint8_t *status)
{
	  int ret;
    uint8_t reg = AS5600_STATUS_REGISTER;
  
    ret = i2cWrite(AS5600_RAW_ADDR, &reg, 1);
    if (ret != HAL_OK) return ret;

    ret = i2cRead(AS5600_RAW_ADDR, status, 1);
    return ret;
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
    if (ret != HAL_OK) return ret;

    // datasheet recommends waiting a bit and optionally verifying; wait small time
    HAL_Delay(5);
    return HAL_OK;
}


//配置AS5600的起始位置（ZPOS）
int bsp_AS5600_SetZero(uint8_t Burn)
{
	int ret;
	uint16_t RAWangle, ZPOS_RAWangle; 
	
	uint8_t buf[3];
	ret = bsp_as5600GetRawAngle(&RAWangle);
	if (ret != HAL_OK) return ret;
	uint8_t msb = (RAWangle >> 8) & 0x0F; // ZPOS(11:8)
  uint8_t lsb = RAWangle & 0xFF;        // ZPOS(7:0)
	
  buf[0] = AS5600_ZPOS11_8_REGISTER; // start register
	buf[1] = msb;
	buf[2] = lsb;
  ret = i2cWrite(AS5600_RAW_ADDR, buf, 3);
 	if (ret != HAL_OK) return ret;
  HAL_Delay(2);
	
	/******* Verify *******/
	uint8_t buffer[2] = {0};
  uint8_t raw_angle_register = AS5600_ZPOS11_8_REGISTER;
  ret = i2cWrite(AS5600_RAW_ADDR, &raw_angle_register, 1);
  ret = i2cRead(AS5600_RAW_ADDR, buffer, 2);
  ZPOS_RAWangle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
	
	if((Burn == 1)&&(ret == HAL_OK)&&(ZPOS_RAWangle == RAWangle)){
		uint8_t status;
		if (AS5600_ReadStatus(&status) == HAL_OK) {
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
	
void bsp_as5600Init(void) {

  /* init i2c interface */
	SoftI2C_RegisterAndInit(1000000,SystemCoreClock);
	
	int IIC_status;
	uint8_t buffer[2] = {0};
	IIC_status = i2c_Mem_Read(AS5600_RAW_ADDR, buffer, 2);  //先定位到RAWangle寄存器，后续直接iic_read

  /* init var */
  Encoder_AS5600.rotation_circles = 0;
	//Encoder_AS5600.Speed_angle_deta_Max = 
	Encoder_AS5600.Speed_angle_deta_Min = 1.0f/4096 * 2 *PI * 0.1f;
	bsp_as5600GetAngle(&Encoder_AS5600.angle_last);
	
	Encoder_AS5600.n_timeout = 4;
	Encoder_AS5600.dma_transfer_complete = true;
	
	
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

