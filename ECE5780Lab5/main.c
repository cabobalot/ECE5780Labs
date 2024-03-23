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

  // Clock to GPIO B and C
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// enable alternate function on pin B11
	GPIOB->MODER &= ~GPIO_MODER_MODER11_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	
	// alternate function 1 is SDA for pin B11
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOB->AFR[1] |= GPIO_AF1_I2C2 << GPIO_AFRH_AFSEL11_Pos;
	
	// enable alternate function on pin B13
	GPIOB->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER13_1;
	
	// alternate function 5 is SCL for pin B13
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13_Msk;
	GPIOB->AFR[1] |= GPIO_AF5_I2C2 << GPIO_AFRH_AFSEL13_Pos;
	
	// output mode on pin B14
	GPIOB->MODER &= ~GPIO_MODER_MODER14_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	
	// output mode on pin C0
	GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	
	// open-drain for SDA and SCL on pins B11, B13
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	// Pull up DA and SCL on pins B11, B13
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_0;
	
	// start pins B14 and C0 high
	GPIOB->ODR |= GPIO_ODR_14;
	GPIOC->ODR |= GPIO_ODR_0;
	
	
	// RCC clock to I2C2
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Configure I2C timing for 100 kHz
	I2C2->TIMINGR |= (0x1  << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF  << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2  << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4  << I2C_TIMINGR_SCLDEL_Pos);
	
	// lastly, enable the I2C
	I2C2->CR1 |= I2C_CR1_PE;
	
	enableLEDs();
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		
		setBlueLED(1);
		
		I2C2->CR2 = 0;
		
		// set send address
		I2C2->CR2 |= (0x69 << 1) << I2C_CR2_SADD_Pos;
		// Send 1 byte
		I2C2->CR2 |= (1) << I2C_CR2_NBYTES_Pos;
		// write operation
		I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;
		// set start bit
		I2C2->CR2 |= I2C_CR2_START;
		
		// wait for send to complete
		while (!(I2C2->ISR & I2C_ISR_TXIS_Msk) && !(I2C2->ISR & I2C_ISR_NACKF_Msk));
		if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
			// NACK
			setRedLED(1);
			I2C2->ICR |= I2C_ICR_NACKCF;
		}
		else if (I2C2->ISR & I2C_ISR_TXIS_Msk) {
			// transmission good
			setGreenLED(1);
			
			//request WHO_AM_I address
			I2C2->TXDR = 0x0F;
			while (!(I2C2->ISR & I2C_ISR_TC_Msk));
			setGreenLED(0);
			
			// set send address
			I2C2->CR2 |= (0x69 << 1) << I2C_CR2_SADD_Pos;
			// read 1 byte
			I2C2->CR2 |= (1) << I2C_CR2_NBYTES_Pos;
			// read operation
			I2C2->CR2 |= I2C_CR2_RD_WRN;
			// set start bit
			I2C2->CR2 |= I2C_CR2_START;
			
			// wait for read to complete
			while (!(I2C2->ISR & I2C_ISR_RXNE_Msk) && !(I2C2->ISR & I2C_ISR_NACKF_Msk));
			if (I2C2->ISR & I2C_ISR_NACKF_Msk) {
				// NACK
				setRedLED(1);
				I2C2->ICR |= I2C_ICR_NACKCF;
			}
			else {
				// transfer good
				setGreenLED(1);
				uint16_t data = I2C2->RXDR;
				while (!(I2C2->ISR & I2C_ISR_TC_Msk));
				I2C2->CR2 |= I2C_CR2_STOP;
				
				if (data == 0xD3) {
					setOrangeLED(1);
				}
			}
			
		}
			
			
		HAL_Delay(1000);
		
		setBlueLED(0);
		setRedLED(0);
		setGreenLED(0);
		setOrangeLED(0);
		
		HAL_Delay(1000);
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

