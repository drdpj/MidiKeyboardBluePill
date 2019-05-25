/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *  USER CODE segments
 *  Copyright (C) 2019  Daniel Jameson
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 ******************************************************************************
  5V Tolerant Pins on STM32F103C
  PB-10
  PB-11
  PB-12
  PB-13
  PB-14
  PB-15

  PA-8    
  PA-9
  PA-10
  PA-11
  PA-12
  PA-13
  PA-14
  PA-15
  
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utilities.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIDI_MASK 0b11110000
#define MIDI_ON 0b10010000
#define MIDI_OFF 0b10000000
#define MIDI_CONT 0b10110000
#define MIDI_PRES 0b11010000
#define MIDI_PROG 0b11000000
#define MIDI_IGNORE 255
#define MIDI_PED 64
#define PEDAL_ZONE 0
#define FALSE 0
#define TRUE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t keyboardMatrix[8];
uint8_t m4kZone;
uint8_t m4kNote;
uint8_t rawZone;
uint8_t tempInt;
/* ring buffer */
volatile struct rb ringBuffer;

/* midi command struct */
struct mc {
	volatile uint8_t status;
	volatile uint8_t data[2];
	volatile uint8_t dataSize;
	volatile uint8_t dataCounter;
	volatile uint8_t complete;
};

volatile struct mc midiCommand;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
inline void Note_On(uint8_t);
inline void Note_Off(uint8_t);
inline uint8_t ReadRingBuffer(void);
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
	uint8_t midiByte = 0;
	uint8_t i = 0;

	/* all keys high to start with */
	for (i = 0; i < 8; i++) {
		keyboardMatrix[i] = 255;
	}

	/* zero some things */
	ringBuffer.readIndex = 0;
	ringBuffer.writeIndex = 0;
	midiCommand.complete = 0;
	midiCommand.dataCounter = 0;
	midiCommand.dataSize = 2;
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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* enable the SPI and UART - we're not using the HAL functions so need to turn them on */
  __HAL_SPI_ENABLE(&hspi2);
  __HAL_UART_ENABLE(&huart3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /**
		 * Zones 1-7 have notes, zone 8 has the control pedal
		 * middle C is 1st note of zone 4 (int 127, midi 60)
		 */

		 /* get a midi command from the ring buffer */

	  while (midiCommand.complete == 0)
	  {
		  if (ringBuffer.readIndex != ringBuffer.writeIndex)
		  {
			  /* only if there's a byte available */
			  midiByte = ReadRingBuffer();
			  if (midiByte > 127)
			  {
				  /* a status byte */
				  switch (midiByte & MIDI_MASK) {
				  case MIDI_ON:
					  midiCommand.status = MIDI_ON;
					  midiCommand.dataSize = 2;
					  midiCommand.dataCounter = 0;
					  break;
				  case MIDI_OFF:
					  midiCommand.status = MIDI_OFF;
					  midiCommand.dataSize = 2;
					  midiCommand.dataCounter = 0;
					  break;
				  case MIDI_CONT:
					  midiCommand.status = MIDI_CONT;
					  midiCommand.dataSize = 2;
					  midiCommand.dataCounter = 0;
					  break;
				  case MIDI_PROG:
					  midiCommand.status = MIDI_PROG;
					  midiCommand.dataSize = 1;
					  midiCommand.dataCounter = 0;
					  break;
				  case MIDI_PRES:
					  midiCommand.status = MIDI_PROG;
					  midiCommand.dataSize = 1;
					  midiCommand.dataCounter = 0;
					  break;
				  default:
					  midiCommand.status = MIDI_IGNORE;
					  midiCommand.dataSize = 2;
					  midiCommand.dataCounter = 0;
				  }
			  }
			  else {
				  /* a data byte */
				  midiCommand.data[midiCommand.dataCounter] = midiByte;
				  midiCommand.dataCounter++;
			  }
			  if (midiCommand.dataCounter >= midiCommand.dataSize)
				  /* now have a complete command */
			  {
				  midiCommand.complete = 1;
			  }

		  }
	  }

	  /* Process the midi command */
	  if (midiCommand.complete == 1) {
		  switch (midiCommand.status)
		  {
		  case MIDI_ON:

			  if (midiCommand.data[1] != 0) {
				  Note_On(midiCommand.data[0]);
			  }
			  else {
				  Note_Off(midiCommand.data[0]);
			  }
			  break;
		  case MIDI_OFF:
			  Note_Off(midiCommand.data[0]);
			  break;
		  case MIDI_CONT:
			  if (midiCommand.data[0] == MIDI_PED) {
				  if (midiCommand.data[1] < 64) {
					  /**
					   * Turn it on
					   */
					  keyboardMatrix[PEDAL_ZONE] |= (1 << 6);
				  }
				  else {
					  /**
					   * Turn it off
					   */

					  keyboardMatrix[PEDAL_ZONE] &= ~(1 << 6);
				  }
			  }
			  break;
		  default:
			  break;
		  }
		  /*reset the command */
		  midiCommand.complete = 0;
		  midiCommand.dataCounter = 0;
	  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* set up interrupt*/
  hspi2.Instance->CR2 |= SPI_CR2_RXNEIE;

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 31250;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* set up interrupt */
  huart3.Instance->CR1 |= USART_CR1_RXNEIE;


  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PB0_Pin|PB1_Pin|PB2_Pin|PB3_Pin 
                          |PB4_Pin|PB5_Pin|PB6_Pin|PB7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0_Pin PB1_Pin PB2_Pin PB3_Pin 
                           PB4_Pin PB5_Pin PB6_Pin PB7_Pin */
  GPIO_InitStruct.Pin = PB0_Pin|PB1_Pin|PB2_Pin|PB3_Pin 
                          |PB4_Pin|PB5_Pin|PB6_Pin|PB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* We're only allowing midi notes between 36 and 84 otherwise
 * we'll overshoot the keyboard matrix array...
 */

 /**
  * @brief Set Keyboard Key
  * @param uint_8_t MIDInote
  * @retval None
  */

inline void Note_On(uint8_t note)
{
	if ((note > 35) && (note < 85)) {
		rawZone = ((note - 36) / 8);
		m4kZone = 7 - rawZone;
		m4kNote = note - 36 - (8 * rawZone);
		keyboardMatrix[m4kZone] &= ~(1 << m4kNote);
	}
}

/**
 * @brief Set Keyboard Key
 * @param uint_8_t MIDInote
 * @retval None
 */

inline void Note_Off(uint8_t note)
{
	if ((note > 35) && (note < 85)) {
		rawZone = ((note - 36) / 8);
		m4kZone = 7 - rawZone;
		m4kNote = note - 36 - (8 * rawZone);
		keyboardMatrix[m4kZone] |= (1 << m4kNote);
	}
}

inline uint8_t ReadRingBuffer(void)
{
	tempInt = ringBuffer.readIndex;
	ringBuffer.readIndex++;
	ringBuffer.readIndex &= 127;
	return ringBuffer.data[tempInt];
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
