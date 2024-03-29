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

void transmitChar(char c) {
	while (!(USART3->ISR & USART_ISR_TXE_Msk)); // wait for transmit register to be empty
	
	USART3->TDR = c;
	
}

void transmitString(char* str) {
	int i = 0;
	while(str[i] != NULL) {
		transmitChar(str[i]);
		i++;
	}
}

static int dataReadyFlag = 0;
static char charBuffer = 0;

void USART3_4_IRQHandler() {
	charBuffer = USART3->RDR;
	dataReadyFlag = 1;
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
	
	// turn on usart clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// USART3 is on PB10 - 14. Alternate function 4
	//PB10: TX
	//PB11: RX
	//PB12: CK
	//PB13: CTS
	//PB14: RTS
	
	// enable alternate function on pin 10
	GPIOB->MODER &= ~GPIO_MODER_MODER10_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER10_1;
	
	// enable alternate function on pin 11
	GPIOB->MODER &= ~GPIO_MODER_MODER11_Msk;
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	
	// choose alternate function 4
	//pin 10
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
	GPIOB->AFR[1] |= GPIO_AF4_USART3 << GPIO_AFRH_AFSEL10_Pos;
	
	//pin 11
	GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11_Msk;
	GPIOB->AFR[1] |= GPIO_AF4_USART3 << GPIO_AFRH_AFSEL11_Pos;
	
	
	// set baudrate 115200
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	//enable recieve interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
	
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	USART3->CR1 |= USART_CR1_UE;
	
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
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	transmitString("\n\rCMD?");
  while (1)
  {
    /* USER CODE END WHILE */
	
		HAL_Delay(1);
		
		if (dataReadyFlag) {
			char c = charBuffer;
			transmitChar(c); //echo to prompt
			
			//state machine state
			static int chosenLED = 0;
			
			if (chosenLED == 0) { // first letter
				if (c == 'r') {
					chosenLED = 1;
				}
				else if (c == 'b') {
					chosenLED = 2;
				}
				else if (c == 'o') {
					chosenLED = 3;
				}
				else if (c == 'g') {
					chosenLED = 4;
				}
				else {
					transmitString("\n\rUnknown LED");
					transmitString("\n\rCMD?");
					chosenLED = 0;
				}
			}
			else { // seccond letter
				if (c == '0') { //turn off
					GPIOC->ODR &= ~(GPIO_ODR_6 << (chosenLED - 1));
				}
				else if (c == '1') { //turn on
					GPIOC->ODR |= (GPIO_ODR_6 << (chosenLED - 1));
				}
				else if (c == '2') { //toggle
					GPIOC->ODR ^= (GPIO_ODR_6 << (chosenLED - 1));
				}
				else {
					transmitString("\n\rUnknown command");
				}
				transmitString("\n\rCMD?");
				chosenLED = 0;
			}
			
			dataReadyFlag = 0;
		}
		
		
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

