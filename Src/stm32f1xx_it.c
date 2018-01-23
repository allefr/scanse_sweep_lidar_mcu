/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include <string.h>

// variables declared as external on main
uint8_t read_in[7];
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

/*
 * create buffer big enough to store 4 bytes (distance and angle)
 * max specs: 1000Hz sample rate for 5Hz -> up to 200 measurements
 * better be safe and consider up to 210 measurements.
 * leave first 2 bytes for scanse ID and measurement counter
 */
uint8_t i2c_buff_out[842];

uint8_t scanse_count;

// define backup buffer (must be big enough)
uint8_t scanse_backup_buff[200];

/*
 *  bool var to use in 2 cases:
 *  - to flag we received a new sweep value -> send data through i2c
 *  - when i2c transfer done (should take abt 17ms), can start putting data on buffer again
 */
uint8_t sweep_completed;

// err counter to know if any checksum error on receiving..
/*
 * if so and next one still error, than it most likely means we are reading wrong data
 * NOTE: the scanse sweep does not use any start sequence, so if any byte is missed,
 * we are reading shifted bytes, so everything is wrong..
 */
uint32_t err_crc;

uint8_t count_fin;
uint16_t err_fin;


// function prototype
uint8_t scanse_checksum( void );
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

    /*
  	HAL_UART_Receive_IT(&huart1, &in_char, 1);

  	rx_buff_in[head] = in_char;
	head++;
	//*/

    //*
  	HAL_UART_Receive_IT(&huart1, read_in, 7);

	// checksum check
	if ( scanse_checksum() ) {
		// check synch bit
		if ( read_in[0] & 0x01 ) {
			sweep_completed = 1;	// this is let the task know we can send data through i2c
			// debug
			count_fin = scanse_count;
			err_fin = err_crc;
			err_crc = 0;

			// correct
			scanse_count = 0;
		}

		if ( sweep_completed ) {
			// if data not sent through i2c yet, then store in backup buffer first
			//memcpy(&scanse_backup_buff[scanse_count*4], &read_in[1], 4*sizeof(uint8_t));
		} else {
			// store in i2c output buffer and increase measurement counter
			//memcpy(&i2c_buff_out[2 + scanse_count*4], &read_in[1], 4*sizeof(uint8_t));
			i2c_buff_out[1] = scanse_count + 1;
		}
		++scanse_count;
	} else {
		// wrong checksum.. fuck.. increase error counter
		++err_crc;
	}//*/

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
volatile uint16_t calculatedSum;
volatile uint8_t i_coun;
uint8_t scanse_checksum( void ) {
	calculatedSum = 0;
	for(i_coun = 0; i_coun < 6; i_coun++){
		//add each status byte to the running sum
		calculatedSum += read_in[i_coun];
	}

	return (calculatedSum % 255) == read_in[6] ? 1 : 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	// re-enable interrupt
	__HAL_UART_ENABLE_IT(huart, UART_IT_PE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	//sprintf(tx_buff_deb, "uartErCb: %d\r\n", huart->ErrorCode);
	//HAL_UART_Transmit(&huart1, tx_buff_deb, 20, 1);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
