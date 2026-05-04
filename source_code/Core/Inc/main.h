/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define NBUZ_Pin LL_GPIO_PIN_13
#define NBUZ_GPIO_Port GPIOC
#define BNO085_SPI_CS_Pin LL_GPIO_PIN_1
#define BNO085_SPI_CS_GPIO_Port GPIOC
#define MPU6000_SPI_CS_Pin LL_GPIO_PIN_2
#define MPU6000_SPI_CS_GPIO_Port GPIOC
#define MPU6000_INT_Pin LL_GPIO_PIN_3
#define MPU6000_INT_GPIO_Port GPIOC
#define MPU6000_INT_EXTI_IRQn EXTI3_IRQn
#define RC_TX_Pin LL_GPIO_PIN_2
#define RC_TX_GPIO_Port GPIOA
#define RC_RX_Pin LL_GPIO_PIN_3
#define RC_RX_GPIO_Port GPIOA
#define MPU6000_SPI_SCK_Pin LL_GPIO_PIN_5
#define MPU6000_SPI_SCK_GPIO_Port GPIOA
#define MPU6000_SPI_MISO_Pin LL_GPIO_PIN_6
#define MPU6000_SPI_MISO_GPIO_Port GPIOA
#define MPU6000_SPI_MOSI_Pin LL_GPIO_PIN_7
#define MPU6000_SPI_MOSI_GPIO_Port GPIOA
#define BAT_Pin LL_GPIO_PIN_5
#define BAT_GPIO_Port GPIOC
#define BNO085_INT_Pin LL_GPIO_PIN_0
#define BNO085_INT_GPIO_Port GPIOB
#define BNO085_PS0_WAKE_Pin LL_GPIO_PIN_2
#define BNO085_PS0_WAKE_GPIO_Port GPIOB
#define ESC_M3_Pin LL_GPIO_PIN_6
#define ESC_M3_GPIO_Port GPIOC
#define ESC_M2_Pin LL_GPIO_PIN_7
#define ESC_M2_GPIO_Port GPIOC
#define ESC_M4_Pin LL_GPIO_PIN_8
#define ESC_M4_GPIO_Port GPIOC
#define ESC_M1_Pin LL_GPIO_PIN_9
#define ESC_M1_GPIO_Port GPIOC
#define DEBUG_TX_Pin LL_GPIO_PIN_12
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin LL_GPIO_PIN_2
#define DEBUG_RX_GPIO_Port GPIOD
#define BNO085_SPI_SCK_Pin LL_GPIO_PIN_3
#define BNO085_SPI_SCK_GPIO_Port GPIOB
#define BNO085_SPI_MISO_Pin LL_GPIO_PIN_4
#define BNO085_SPI_MISO_GPIO_Port GPIOB
#define BNO085_SPI_MOSI_Pin LL_GPIO_PIN_5
#define BNO085_SPI_MOSI_GPIO_Port GPIOB
#define BMP280_I2C_SCL_Pin LL_GPIO_PIN_6
#define BMP280_I2C_SCL_GPIO_Port GPIOB
#define BMP280_I2C_SDA_Pin LL_GPIO_PIN_7
#define BMP280_I2C_SDA_GPIO_Port GPIOB
#define BNO085_RST_Pin LL_GPIO_PIN_8
#define BNO085_RST_GPIO_Port GPIOB
#define LED_Pin LL_GPIO_PIN_9
#define LED_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
