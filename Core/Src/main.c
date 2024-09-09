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
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOW 		(GPIO_PIN_RESET)
#define HIGH 		(GPIO_PIN_SET)
#define DATA_PINS	((uint32_t)(DATA_1_Pin | DATA_2_Pin | DATA_3_Pin | DATA_4_Pin | DATA_5_Pin | DATA_6_Pin | DATA_7_Pin | DATA_8_Pin))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */

void Set_Command_Mode(void);
void Set_Address_Input_Mode(void);
void Write_Command(uint8_t Command);
void Write_Address(uint8_t Address);
uint8_t Read_Data(void);
void Write_Data(uint8_t Data);
uint8_t Read_EEPROM_Bytes(uint8_t* ReadBuffer, uint16_t NumOfBytes, uint32_t PageAddress, uint16_t ColumnAddress);
uint8_t Write_EEPROM_Bytes(uint8_t* WriteBuffer, uint16_t NumOfBytes, uint32_t PageAddress, uint16_t ColumnAddress);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|WE_Pin|RE_Pin|CLE_Pin
                          |ALE_Pin|WP_Pin|DATA_1_Pin|DATA_2_Pin
                          |DATA_3_Pin|DATA_4_Pin|DATA_5_Pin|DATA_6_Pin
                          |DATA_7_Pin|DATA_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin WE_Pin RE_Pin CLE_Pin
                           ALE_Pin WP_Pin DATA_1_Pin DATA_2_Pin
                           DATA_3_Pin DATA_4_Pin DATA_5_Pin DATA_6_Pin
                           DATA_7_Pin DATA_8_Pin */
  GPIO_InitStruct.Pin = CE_Pin|WE_Pin|RE_Pin|CLE_Pin
                          |ALE_Pin|WP_Pin|DATA_1_Pin|DATA_2_Pin
                          |DATA_3_Pin|DATA_4_Pin|DATA_5_Pin|DATA_6_Pin
                          |DATA_7_Pin|DATA_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BY_RY_Pin */
  GPIO_InitStruct.Pin = BY_RY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BY_RY_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Set_Command_Mode(void)
{
	HAL_GPIO_WritePin(GPIOA, CE_Pin, LOW);		//turn the Chip Enable Pin to Low
	HAL_GPIO_WritePin(GPIOA, CLE_Pin, HIGH);	//turn the Command Latch Enable Pin High
	HAL_GPIO_WritePin(GPIOA, ALE_Pin, LOW);		//turn the Address Latch Enable Pin Low
}

void Set_Address_Input_Mode(void)
{
	HAL_GPIO_WritePin(GPIOA, CE_Pin, LOW);		//turn the Chip Enable Pin to Low
	HAL_GPIO_WritePin(GPIOA, CLE_Pin, LOW);		//turn the Command Latch Enable Pin Low
	HAL_GPIO_WritePin(GPIOA, ALE_Pin, HIGH);	//turn the Address Latch Enable Pin High
}

void Write_Command(uint8_t Command)
{
	Set_Command_Mode();
	HAL_GPIO_WritePin(GPIOA, WE_Pin, LOW);

	GPIOA->ODR = (GPIOA->ODR & (~DATA_PINS)) | ( ((uint32_t)Command) << 7 );

	HAL_GPIO_WritePin(GPIOA, WE_Pin, HIGH);
}

void Write_Address(uint8_t Address)
{
	Set_Address_Input_Mode();

	HAL_GPIO_WritePin(GPIOA, WE_Pin, LOW);
	GPIOA->ODR = (GPIOA->ODR & (~DATA_PINS)) | ( ((uint32_t)Address) << 7 );
	HAL_GPIO_WritePin(GPIOA, WE_Pin, HIGH);
}

uint8_t Read_Data(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DATA_PINS;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, RE_Pin, LOW);

	uint8_t Data = (uint8_t)((GPIOA->IDR & DATA_PINS) >> 7);

	HAL_GPIO_WritePin(GPIOA, RE_Pin, HIGH);

	return Data;
}

void Write_Data(uint8_t Data)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DATA_PINS;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, WE_Pin, LOW);

	GPIOA->ODR = (GPIOA->ODR & (~DATA_PINS)) | ( ((uint32_t)Data) << 7 );

	HAL_GPIO_WritePin(GPIOA, WE_Pin, HIGH);
}

/*	This function Read bytes from a single page of the EEPROM.
	There are 2^12 = 4096 bytes readable/writable bytes in 1 page.
	There are total 2^18 = 262144 pages in the EEPROM.
	functions returns 0 if read operation passed and 1 if failed	*/

uint8_t Read_EEPROM_Bytes(uint8_t* ReadBuffer, uint16_t NumOfBytes, uint32_t PageAddress, uint16_t ColumnAddress)
{
	uint8_t columnaddressL = (uint8_t)(ColumnAddress & 0x00FF);
	uint8_t columnaddressH = (uint8_t)((ColumnAddress & 0xFF00)>>8);

	uint8_t pageaddressL = (uint8_t)(PageAddress & 0x000000FF);
	uint8_t pageaddressM = (uint8_t)((PageAddress & 0x0000FF00) >> 8);
	uint8_t pageaddressH = (uint8_t)((PageAddress & 0x00FF0000) >> 16);

	Write_Command(0x00);

	Write_Address(columnaddressL);
	Write_Address(columnaddressH);

	Write_Address(pageaddressL);
	Write_Address(pageaddressM);
	Write_Address(pageaddressH);

	Write_Command(0x30);

	while(HAL_GPIO_ReadPin(GPIOA, BY_RY_Pin) == 0);

	Write_Command(0x70);

	HAL_GPIO_WritePin(GPIOA, CE_Pin, LOW);
	HAL_GPIO_WritePin(GPIOA, CLE_Pin, LOW);
	HAL_GPIO_WritePin(GPIOA, ALE_Pin, LOW);

	if( (Read_Data() & 0x01) == 1)
		return 1;

	for(uint16_t i=0; i<NumOfBytes; i++)
	{
		ReadBuffer[i] = Read_Data();
	}

	return 0;
}

/*	This function Write bytes to a single page of the EEPROM.
	There are 2^12 = 4096 bytes readable/writable bytes in 1 page.
	There are total 2^18 = 262144 pages in the EEPROM.
	functions returns 0 if write operation passed and 1 if failed	*/

uint8_t Write_EEPROM_Bytes(uint8_t* WriteBuffer, uint16_t NumOfBytes, uint32_t PageAddress, uint16_t ColumnAddress)
{
	uint8_t columnaddressL = (uint8_t)(ColumnAddress & 0x00FF);
	uint8_t columnaddressH = (uint8_t)((ColumnAddress & 0xFF00)>>8);

	uint8_t pageaddressL = (uint8_t)(PageAddress & 0x000000FF);
	uint8_t pageaddressM = (uint8_t)((PageAddress & 0x0000FF00) >> 8);
	uint8_t pageaddressH = (uint8_t)((PageAddress & 0x00FF0000) >> 16);

	Write_Command(0x80);

	Write_Address(columnaddressL);
	Write_Address(columnaddressH);

	Write_Address(pageaddressL);
	Write_Address(pageaddressM);
	Write_Address(pageaddressH);

	HAL_GPIO_WritePin(GPIOA, CE_Pin, LOW);
	HAL_GPIO_WritePin(GPIOA, CLE_Pin, LOW);
	HAL_GPIO_WritePin(GPIOA, ALE_Pin, LOW);

	for(uint16_t i=0; i<NumOfBytes; i++)
	{
		Write_Data(WriteBuffer[i]);
	}

	Write_Command(0x10);

	while(HAL_GPIO_ReadPin(GPIOA, BY_RY_Pin) == 0);  //wait for the RY/BY Pin to get on ready state i.e. (== 1)

	Write_Command(0x70);

	return (Read_Data() & 0x01);	//return 0 means operation was successfull & 1 means failed.
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
