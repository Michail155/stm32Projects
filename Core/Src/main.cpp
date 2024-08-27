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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ibus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t ibus_data[IBUS_USER_CHANNELS];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

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

class Recieve {

public:

	Recieve(){
		channel.Num1 = 0;
		channel.Num2 = 0;
		channel.Num3 = 0;
		channel.Num4 = 0;
		channel.Num5 = 0;
		channel.Num6 = 0;
		channel.Num7 = 0;
		channel.Num8 = 0;
		channel.Num9 = 0;
		channel.Num10 = 0;
	}
protected:

	struct Channel{
	uint16_t Num1;
	uint16_t Num2;
	uint16_t Num3;
	uint16_t Num4;
	uint16_t Num5;
	uint16_t Num6;
	uint16_t Num7;
	uint16_t Num8;
	uint16_t Num9;
	uint16_t Num10;
	};

	Channel channel;

	void RecieveData(void){
		channel.Num1 = ibus_data[0];
		channel.Num2 = ibus_data[1];
		channel.Num3 = ibus_data[2];
		channel.Num4 = ibus_data[3];
		channel.Num5 = ibus_data[4];// отвечает за разрешение/запрещение работы робота;
		channel.Num6 = ibus_data[5];
		channel.Num7 = ibus_data[6];
		channel.Num8 = ibus_data[7];// Отвечает за Reverse ходовой
		channel.Num9 = ibus_data[8];
		channel.Num10 = ibus_data[9];
	}
	void ibusRead(void){
		 ibus_read(ibus_data);
		 ibus_soft_failsafe(ibus_data, 10);
	}


private:

};
class Motor{

protected:

	void MotorInit(void){
		MotorWriteLeftGun(2000);
		MotorWriteRightGun2000;
		MotorWriteLeft(2000);
		MotorWriteRight(2000);
		  HAL_Delay(2000);
		MotorWriteLeftGun(1000);
		MotorWriteRightGun1000;
		MotorWriteLeft(1000);
		MotorWriteRight(1000);
		  HAL_Delay(2000);
	}
	void MotorWriteLeftGun(uint16_t val){
		TIM1->CCR1 = val;//Подача скорости на левый двигатель оружия
	}
	void MotorWriteRightGun(uint16_t val){
		TIM1->CCR2 = val;//Подача скорости на правый двигатель оружия
	}
	void MotorWriteLeft(uint16_t val){
		TIM1->CCR3 = val;//Подача скорости на левый двигатель ходовой
	}
	void MotorWriteRight(uint16_t val){
		TIM1->CCR4 = val;//Подача скорости на правый двигатель ходовой
	}
};
class Pilot:protected Motor,Recieve {

public:

	Pilot(){
		Reverse = 0;
		PermitWork = 0;
	}
	void PilotInit(void){
		ibus_init();
		MotorInit();
	}
	void PilotMotorControl(void){

		if (PermitWork){// проверка. если переменная PermitWork =1, то работа робота разрешена
			MotorWriteLeftGun(channel.Num5);
			MotorWriteRightGun(channel.Num5);
			if (Reverse == 0){// проверка, если Reverse = 0, то ходовая рабоатет в обычном режиме
				MotorWriteLeft(channel.Num1);
				MotorWriteRight(channel.Num3);
			}
			else if (Reverse >= 1){// ходовая работает реверсивно
				MotorWriteLeft(2000-channel.Num1+1000);
				MotorWriteRight(2000-channel.Num3+1000);
			}
		}
		else {
			MotorWriteLeftGun(1000);
			MotorWriteRightGun(1000);
			MotorWriteLeft(1000);
			MotorWriteRight(1000);
		}

	}
	void PilotRecieve(void){
		channel.Num8>1000&&channel.Num8<1100?Reverse = 0:0;// Ходовая рабоатет в обычном режиме
		channel.Num8>1900?Reverse = 1:0;// Ходовая работает в реверсном режиме

		channel.Num5>1000&&channel.Num5<1100?PermitWork = 0:0;// Работа робота запрещена
		channel.Num5>1900?PermitWork = 1:0;// Работа робота разрешена
	}
private:
	int Reverse;
	int PermitWork;

};
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  Pilot pilot;
  pilot.PilotInit();
  HAL_Delay(2000);

  uint32_t T = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_GetTick()-T>50){
		  T = HAL_GetTick();
		  pilot.PilotRecieve();
	  }
	  pilot.PilotMotorControl();
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart == IBUS_UART)
 ibus_reset_failsafe();
}
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
