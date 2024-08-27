/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "ST7920_lib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern char tx_buffer[128]; //Буфер для отправки текста на дисплей
extern uint8_t Frame_buffer[1024]; //Буфер кадра

static const uint8_t TCS3472_ADDR = 0x29<<1;
uint8_t ENABLE_ADDR = 0x00;
uint8_t COMAND_BIT = 0x80;
uint8_t WHITE_COLOR_ADDR = 0x14;
uint8_t RED_COLOR_ADDR = 0x16;
uint8_t GREEN_COLOR_ADDR = 0x18;
uint8_t BLUE_COLOR_ADDR = 0x1A;
uint8_t COMAND_BIT_INC = 0xA0;
uint8_t PON = 0x01;
uint8_t AEN = 0x02;
uint8_t A_TIM_ADDR = 0x01;
uint8_t A_TIM_VALUE154 = 0x00;

uint8_t RxBufferRed[2];
uint8_t RxBufferGreen[2];
uint8_t RxBufferBlue[2];
uint8_t RxBufferWhite[2];
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  ST7920_Init();
  ST7920_Graphic_mode(1);
  HAL_Delay(100);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

  	  HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (ENABLE_ADDR|COMAND_BIT), 1, &PON, 1,1000);
  	  HAL_Delay(10);
      uint8_t K = (PON|AEN);
      HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (ENABLE_ADDR|COMAND_BIT), 1, &K, 1,1000);
      HAL_Delay(50);
      HAL_I2C_Mem_Write(&hi2c1, TCS3472_ADDR, (A_TIM_ADDR|COMAND_BIT), 1, &A_TIM_VALUE154, 1,1000);
      HAL_Delay(50);
      uint16_t Red = 0;
      uint16_t Green = 0;
      uint16_t Blue = 0;
      uint16_t White = 0;

      uint16_t K1 = 23;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ST7920_Clean();
	  ST7920_Clean_Frame_buffer();

      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, RED_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferRed, 2);
      HAL_Delay(10);
      Red = (((uint16_t)(RxBufferRed[1]<<8))|(uint16_t)RxBufferRed[0])/K1;
	  sprintf(tx_buffer,"Красный:%ld",Red);
	  ST7920_Decode_UTF8(0, 0, 0, tx_buffer);

      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, GREEN_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferGreen, 2);
      HAL_Delay(10);
      Green = (((uint16_t)(RxBufferGreen[1]<<8))|(uint16_t)RxBufferGreen[0])/K1;
	  sprintf(tx_buffer,"Зеленый:%ld",Green);
	  ST7920_Decode_UTF8(0, 1, 0, tx_buffer);

      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, BLUE_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferBlue, 2);
      HAL_Delay(10);
      Blue = (((uint16_t)(RxBufferBlue[1]<<8))|(uint16_t)RxBufferBlue[0])/K1;
	  sprintf(tx_buffer,"Синий:%ld",Blue);
	  ST7920_Decode_UTF8(0, 2, 0, tx_buffer);

      HAL_I2C_Mem_Read_IT(&hi2c1, TCS3472_ADDR, WHITE_COLOR_ADDR|COMAND_BIT_INC , 1, RxBufferWhite, 2);
      HAL_Delay(10);
      White = (((uint16_t)(RxBufferWhite[1]<<8))|(uint16_t)RxBufferWhite[0])/K1;
	  sprintf(tx_buffer,"Освещенность:%ld",White);
	  ST7920_Decode_UTF8(0, 3, 0, tx_buffer);
	  ST7920_Update();

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
