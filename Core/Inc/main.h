/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm303dlhc.h"
#include "mpu6050.h"
#include "EERTOS.h"
#include "stdio.h"
#include "math.h"
#include "position_calc.h"


#define DEBUG 1
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ABS(x)         (x < 0) ? (-x) : x
#define BitIsSet(reg, bit)	((reg & (1<<(bit))) != 0)
#define BitIsClear(reg, bit)	((reg & (1<<(bit))) == 0)
#define u32 uint32_t
#define s32 int32_t
#define u16 uint16_t
#define s16 int16_t
#define u8 uint8_t
#define s8 int8_t

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B1_Pin GPIO_PIN_8
#define LED_B1_GPIO_Port GPIOE
#define LED_R1_Pin GPIO_PIN_9
#define LED_R1_GPIO_Port GPIOE
#define LED_O1_Pin GPIO_PIN_10
#define LED_O1_GPIO_Port GPIOE
#define LED_G1_Pin GPIO_PIN_11
#define LED_G1_GPIO_Port GPIOE
#define LED_B2_Pin GPIO_PIN_12
#define LED_B2_GPIO_Port GPIOE
#define LED_R2_Pin GPIO_PIN_13
#define LED_R2_GPIO_Port GPIOE
#define LED_O2_Pin GPIO_PIN_14
#define LED_O2_GPIO_Port GPIOE
#define LED_G2_Pin GPIO_PIN_15
#define LED_G2_GPIO_Port GPIOE
#define MPU_INT1_Pin GPIO_PIN_4
#define MPU_INT1_GPIO_Port GPIOB
#define MPU_INT1_EXTI_IRQn EXTI4_IRQn
#define MPU_INT0_Pin GPIO_PIN_5
#define MPU_INT0_GPIO_Port GPIOB
#define MPU_INT0_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
