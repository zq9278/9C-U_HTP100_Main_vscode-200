/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"  // ���ź�����ص� API
//#include "communication.h"
//#include "Button.h"
//#include "pid.h"
//#include "delay.h"
//#include "iic.h"
//#include "bq25895.h"
//#include "bq27441.h"
//#include "tmp112.h"
//#include "24cxx.h"
//#include "tmc5130.h"
//#include "heat.h"
//#include "ws2812b.h"
//#include "app_sys.h"
//#include "ads1220.h"
//#include <stdio.h>
//#include <math.h>
//#include "UserApp.h"
//#include "interface_uart.h"
//#include "time_callback.h"
//#include "string.h"
//#include "device_lifetime.h"
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
#define CHG_QON_Pin GPIO_PIN_13
#define CHG_QON_GPIO_Port GPIOC
#define WS2812_PW_Pin GPIO_PIN_1
#define WS2812_PW_GPIO_Port GPIOA
#define PWR_SENSE_Pin GPIO_PIN_4
#define PWR_SENSE_GPIO_Port GPIOA
#define PWR_SENSE_EXTI_IRQn EXTI4_15_IRQn
#define WS2812B_Pin GPIO_PIN_6
#define WS2812B_GPIO_Port GPIOA
#define HEAT_CTRL_Pin GPIO_PIN_1
#define HEAT_CTRL_GPIO_Port GPIOB
#define SW_CNT_Pin GPIO_PIN_2
#define SW_CNT_GPIO_Port GPIOB
#define SW_CNT_EXTI_IRQn EXTI2_3_IRQn
#define EYE_SCL_Pin GPIO_PIN_10
#define EYE_SCL_GPIO_Port GPIOB
#define EYE_SDA_Pin GPIO_PIN_11
#define EYE_SDA_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define EE_SDA_Pin GPIO_PIN_8
#define EE_SDA_GPIO_Port GPIOA
#define EE_SCL_Pin GPIO_PIN_9
#define EE_SCL_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_6
#define LED0_GPIO_Port GPIOC
#define ADS1220_DRDY_Pin GPIO_PIN_7
#define ADS1220_DRDY_GPIO_Port GPIOC
#define LCD_PW_Pin GPIO_PIN_10
#define LCD_PW_GPIO_Port GPIOA
#define TMC_SDO_Pin GPIO_PIN_11
#define TMC_SDO_GPIO_Port GPIOA
#define TMC_SDI_Pin GPIO_PIN_12
#define TMC_SDI_GPIO_Port GPIOA
#define TMC_CSN_Pin GPIO_PIN_15
#define TMC_CSN_GPIO_Port GPIOA
#define TMC_ENN_Pin GPIO_PIN_0
#define TMC_ENN_GPIO_Port GPIOD
#define CHG_CE_Pin GPIO_PIN_3
#define CHG_CE_GPIO_Port GPIOD
#define TMC_SCK_Pin GPIO_PIN_3
#define TMC_SCK_GPIO_Port GPIOB
#define CHG_INT_Pin GPIO_PIN_5
#define CHG_INT_GPIO_Port GPIOB
#define CHG_INT_EXTI_IRQn EXTI4_15_IRQn
#define CHG_SCL_Pin GPIO_PIN_8
#define CHG_SCL_GPIO_Port GPIOB
#define CHG_SDA_Pin GPIO_PIN_9
#define CHG_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
