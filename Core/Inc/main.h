/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_period PWM_PERIOD
#define PWM_ADC_trgo PWM_ADC_TRGO
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Current_C_Pin GPIO_PIN_0
#define Current_C_GPIO_Port GPIOC
#define Current_B_Pin GPIO_PIN_1
#define Current_B_GPIO_Port GPIOC
#define Voltage_BUS_Pin GPIO_PIN_3
#define Voltage_BUS_GPIO_Port GPIOC
#define IIC_SDA_Pin GPIO_PIN_5
#define IIC_SDA_GPIO_Port GPIOA
#define IIC_SCL_Pin GPIO_PIN_4
#define IIC_SCL_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define PWM_C_Pin GPIO_PIN_10
#define PWM_C_GPIO_Port GPIOA
#define DRV8323_EN_Pin GPIO_PIN_11
#define DRV8323_EN_GPIO_Port GPIOA
#define DRV8232_nFAULT_Pin GPIO_PIN_12
#define DRV8232_nFAULT_GPIO_Port GPIOA
#define Encode_CS_Pin GPIO_PIN_15
#define Encode_CS_GPIO_Port GPIOA
#define Encode_SCK_Pin GPIO_PIN_10
#define Encode_SCK_GPIO_Port GPIOC
#define Encode_MISO_Pin GPIO_PIN_11
#define Encode_MISO_GPIO_Port GPIOC
#define Encode_MOSI_Pin GPIO_PIN_12
#define Encode_MOSI_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
