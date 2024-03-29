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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
  /* USER CODE BEGIN 2 */
	
	//LEDs
	// turn on GPIO C clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	//config GPIO for LEDs
	// green:8 orange:9
	// output mode
	GPIOC->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	GPIOC->MODER &= ~GPIO_MODER_MODER9_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	
	GPIOC->BSRR |= GPIO_BSRR_BS_9; // start green on
	
	
	//enable clock to timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC = (short)7999; // divide clock to ms
	TIM2->ARR = 250; // 250 ms
	TIM2->DIER |= TIM_DIER_UIE; // update inturrupt
	NVIC_EnableIRQ(TIM2_IRQn); // enable the inturrupt controler
	TIM2->CR1 |= TIM_CR1_CEN; // start the timer
	
	
	//enable clock to timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->PSC = (short)99; // divide clock to 8000 khz
	TIM3->ARR = 100;
	
	// set capture/compare to output mode for channel 1 and 2
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S_Msk);
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S_Msk);
	
	// set PWM mode 2 on channel 1 (RED, PC6)
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); // clear
	TIM3->CCMR1 |= (0x7 << TIM_CCMR1_OC1M_Pos); // PWM mode 2
	
	// set PWM mode 1 on channel 2 
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_Msk); // clear
	TIM3->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1
	
	// enable preload for both channels
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
	
	// output enable both channels
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	
	// 20% duty cycle
	TIM3->CCR1 = 20;
	TIM3->CCR2 = 20;
	
	TIM3->CR1 |= TIM_CR1_CEN; // start the timer
	
	// Blue and red LEDs, set to alternate function mode
	GPIOC->MODER &= ~GPIO_MODER_MODER6_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER6_1;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER7_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER7_1;
	
	// choose alternate function 0 (which is the TIM3)
	GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL6_Msk;
	GPIOC->AFR[0] &= ~GPIO_AFRL_AFRL7_Msk;
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int count = 0;
  while (1)
  {
    /* USER CODE END WHILE */
		HAL_Delay(50);
		
		// slowly vary the PWM period
		count = (count + 1) % 100;
		
		TIM3->CCR1 = count;
		TIM3->CCR2 = count;
		
		
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

