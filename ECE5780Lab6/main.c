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

static void enableLEDs() {
	//LEDs
	// turn on GPIO C clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// set push pull output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9;
	
	// low speed
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_Msk;
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_Msk;
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_Msk;
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_Msk;
	
	// no pull resistors
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR6_Msk;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7_Msk;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_Msk;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9_Msk;
	
	// Set output mode
	GPIOC->MODER &= ~GPIO_MODER_MODER6_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER7_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER9_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
}

__attribute__((unused))
static void setRedLED(int val) {
	GPIOC->ODR &= ~GPIO_ODR_6;
	if (val) {
		GPIOC->ODR |= GPIO_ODR_6;
	}
}

__attribute__((unused))
static void setBlueLED(int val) {
	GPIOC->ODR &= ~GPIO_ODR_7;
	if (val) {
		GPIOC->ODR |= GPIO_ODR_7;
	}
}

__attribute__((unused))
static void setOrangeLED(int val) {
	GPIOC->ODR &= ~GPIO_ODR_8;
	if (val) {
		GPIOC->ODR |= GPIO_ODR_8;
	}
}

__attribute__((unused))
static void setGreenLED(int val) {
	GPIOC->ODR &= ~GPIO_ODR_9;
	if (val) {
		GPIOC->ODR |= GPIO_ODR_9;
	}
}

__attribute__((unused))
static void toggleRedLED() {
	GPIOC->ODR ^= GPIO_ODR_6;
}

__attribute__((unused))
static void toggleBlueLED() {
	GPIOC->ODR ^= GPIO_ODR_7;
}

__attribute__((unused))
static void toggleOrangeLED() {
	GPIOC->ODR ^= GPIO_ODR_8;
}

__attribute__((unused))
static void toggleGreenLED() {
	GPIOC->ODR ^= GPIO_ODR_9;
}


void initADC() {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// analog mode pin C0
	// connected to ADC_IN10
	GPIOC->MODER |= GPIO_MODER_MODER0_Msk;
	
	// continous
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	
	// 8 bit mode
	ADC1->CFGR1 |= ADC_CFGR1_RES_1;
	
	// channel 10
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	
	setOrangeLED(1);
	
	//calibrate
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL_Msk);
	
	
	
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY_Msk));
	
	ADC1->CR |= ADC_CR_ADSTART;
	
	setOrangeLED(0);
	
	
}

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

	enableLEDs();
	
	initADC();
	
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

		uint16_t analogValue = ADC1->DR;
		
		setRedLED(0);
		setBlueLED(0);
		setOrangeLED(0);
		setGreenLED(0);
		
		if (analogValue > 50) {
			setBlueLED(1);
		}
		if (analogValue > 100) {
			setOrangeLED(1);
		}
		if(analogValue > 150) {
			setRedLED(1);
		}
		if(analogValue > 200) {
			setGreenLED(1);
		}
		
		HAL_Delay(20);
		
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

