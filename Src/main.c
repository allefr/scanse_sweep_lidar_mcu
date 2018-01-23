/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
#include <string.h>

/* Private variables ---------------------------------------------------------*/
// define scanse sweep settings
#define SCANSE_SAMPLE_RATE		1000	// [Hz]
#define SCANSE_SWEEP_RATE		5		// [Hz]
#define SCANSE_I2C_ID			0x73	// random number.. can be changed if conflicting

// buffer for UART interrupt from scanse sweep
extern volatile uint16_t head;
volatile uint16_t tail;
extern uint8_t rx_buff_in[1024];

struct response_scan_packet_s {
	uint8_t sync:1;
    uint8_t communication_error:1;
    uint8_t reserved2:1;
	uint8_t reserved3:1;
	uint8_t reserved4:1;
	uint8_t reserved5:1;
	uint8_t reserved6:1;
	uint8_t reserved7:1;

	uint16_t angle;     	// little endian, must be divided by 16.0f [deg]
	uint16_t distance;		// little endian [cm]
	uint8_t signal_strength;
	uint8_t checksum;
} __attribute__((packed, aligned(1)));


// uart out buff
uint8_t tx_buff[8];
uint8_t tx2_buff[32];

/*
 * create buffer big enough to store 6 bytes (distance and angle)
 * max specs: 1000Hz sample rate for 5Hz -> up to 200 measurements
 * better be safe and consider up to 340 measurements.
 * leave first 2 bytes for scanse ID and measurement counter
 */
uint8_t i2c_buff_out[2048];

uint16_t scanse_count;

// err counter to know if any checksum error on receiving..
uint8_t err_crc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 36;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
volatile uint16_t calculatedSum;
volatile uint8_t i_coun;
uint8_t scanse_checksum( uint8_t * read_in ) {
	calculatedSum = 0;
	for(i_coun = 0; i_coun < 6; i_coun++){
		//add each status byte to the running sum
		calculatedSum += read_in[i_coun];
	}

	return (calculatedSum % 255) == read_in[6] ? 1 : 0;
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	portTickType xLastWakeTime;

	float task_freq = SCANSE_SWEEP_RATE * 10;	// [Hz]
	//float task_freq = 1.;	// [Hz]

	// initialize variables
	head = 0;
	tail = 0;
	scanse_count = 0;

	// define decoding variables
	int16_t offset;
	uint16_t distance;
	float angle;

	err_crc = 0;

	// send message to scanse sweep lidar to set settings and start acquisition
	// TODO: send commands for settings. default values are 5Hz sweep and 500Hz sample rate

	sprintf((char *)tx2_buff, "\r\nstart lidar\r\n");
	HAL_UART_Transmit(&huart2, tx2_buff, 15, 10);

	// make sure the sensor (re)starts as well
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_Delay(2000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

	// start acquisition:
	// the sensor return error till everything is set properly
	// and motor spinning at correct frequency
	while (1) {
		sprintf((char *)tx_buff, "DS\n");
		HAL_UART_Transmit(&huart1, tx_buff, 3, 10);

		HAL_UART_Receive(&huart1, rx_buff_in, 6, 100);

		// if everything ok on the sensor side, enable uart interrupt to start acquisition
		if ( strstr((char *)rx_buff_in, "DS00") != NULL ) {
			// enable usart interrupt
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
			break;
		}
		// toggle led
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		// wait 50ms before send start acquisition command again.
		HAL_Delay(50);
	}

	sprintf((char *)tx2_buff, "acquiring..\r\n");
	HAL_UART_Transmit(&huart2, tx2_buff, 13, 10);

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();

	while (1) {

		// get absolute offset between head and tail
		offset = (head - tail) >= 0 ? head - tail : 1024 + head - tail;

		while ( offset >= 7 ) {
			// checksum check
			calculatedSum = 0;
			for (i_coun = 0; i_coun < 6; i_coun++) {
				//add each status byte to the running sum
				calculatedSum += rx_buff_in[(tail + i_coun) % 1024];
			}
			if ( (calculatedSum % 255) != rx_buff_in[(tail + 6) % 1024] ) {
				// error checksum.. could be cause wrong data received, or cause lost some data
				tail = (++tail) % 1024;
				--offset;
				++err_crc;

				continue;
			}

			// if here, good checksum

			// if synch packet, send previous data through i2c
			if ( rx_buff_in[tail] & 0x01 ) {
				sprintf((char *)tx2_buff, "mcou: %3d\r\n", scanse_count);
				HAL_UART_Transmit(&huart2, tx2_buff, 11, 10);

				// send data through i2c
				i2c_buff_out[0] = SCANSE_I2C_ID;
				memcpy(&i2c_buff_out[2], &scanse_count, sizeof(uint16_t));
				HAL_I2C_Master_Transmit(&hi2c1, 0x11<<1, i2c_buff_out, scanse_count + 4, 20);

				scanse_count = 0;
			}

			// now read packet and store in i2c buff
			angle = ( (rx_buff_in[(tail + 2) % 1024] << 8) + rx_buff_in[(tail + 1) % 1024] )/16.;
			distance = (rx_buff_in[(tail + 4) % 1024] << 8) + rx_buff_in[(tail + 3) % 1024];

			// copy in i2c buff out
			memcpy(&i2c_buff_out[4 + scanse_count*6], &angle, sizeof(float));
			memcpy(&i2c_buff_out[8 + scanse_count*6], &distance, sizeof(uint16_t));

			++scanse_count;
			tail = (tail + 7) % 1024;
			offset -= 7;
		} // end while


		if ( err_crc != 0 ) {
			sprintf((char *)tx2_buff, "err: %3d\r\n", err_crc);
			HAL_UART_Transmit(&huart2, tx2_buff, 10, 10);

			err_crc = 0;

			// blink led 1 times
			/*
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			vTaskDelay(20 / portTICK_RATE_MS);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			vTaskDelay(10 / portTICK_RATE_MS);
			//*/
		}

		// toggle led
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, 1000 / task_freq / portTICK_RATE_MS );
	}

	vTaskDelete(NULL);

  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
