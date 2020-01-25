/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char *ptr, int len)
    {
    int i = 0;
    for (i = 0; i < len; i++)
	ITM_SendChar(*ptr++);
    return len;
    }
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern LSM_DATA LSM303_data;
#include "string.h"

extern UART_HandleTypeDef huart1;

uint8_t buf2[14];

char str1[50];
void Task_led(void)
    {
    float theta, phi, rho;
    SetTimerTask(Task_led, 49);
    LED_B1_GPIO_Port->ODR ^= LED_B1_Pin;
    theta = atan2f(LSM303_data.accX, LSM303_data.accY) * 57.2957795f; //оригинал
    phi = atan2f(LSM303_data.accY, -LSM303_data.accZ) * 57.2957795f; //оригинал
    rho = atan2f(LSM303_data.accZ, LSM303_data.accX) * 57.2957795f; //оригинал

    sprintf(str1, "%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n", LSM303_data.accX,
	    LSM303_data.accY, LSM303_data.accZ, LSM303_data.magX * 10,
	    LSM303_data.magY * 10, LSM303_data.magZ * 10, (int) theta,
	    (int) phi, (int) rho);

    HAL_UART_Transmit(&huart1, (uint8_t*) str1, strlen(str1), 0x1000);
    }

volatile u32 count = 0;

void Acc_read(void)
    {
    count++;
    SetTimerTask(Acc_read, 100);
    //Accel_GetXYZ(buf);
    printf("acc X=%d, Y=%d, Z=%d\n", LSM303_data.accX, LSM303_data.accY,
	    LSM303_data.accZ);
    //Mag_GetXYZ(buf);
    printf("mag X=%d, Y=%d, Z=%d\n", LSM303_data.magX, LSM303_data.magY,
	    LSM303_data.magZ);
    }
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
    {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    InitRTOS();
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    Accel_Init();
    Task_led();
    Acc_read();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
	{
	TaskManager();
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	}
    /* USER CODE END 3 */
    }

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
    {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
	{
	Error_Handler();
	}
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
	{

	}
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1)
	{

	}
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
	{

	}
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
	{

	}
    LL_SetSystemCoreClock(72000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
	{
	Error_Handler();
	};
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
    }

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
    {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
    }

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
