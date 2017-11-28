/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "drivetrain.h"
#include "vehicle_messages.h"
#include "string.h"
#include "bme280_monti.h"
#include "lis3dh_driver.h"
#include "non_i2c_sensors.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char msg_rx[MSG_TX_BUFFER_SIZE];
char msg_tx[MSG_TX_BUFFER_SIZE];
uint16_t msg_tx_count = 0;
uint16_t msg_rx_count = 0;
uint8_t msg_rx_type = MSG_HEADER_CONFIG;

struct motor motors[NUM_MOTORS_ENABLED];
struct holonomic3 holonomic3_system;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	AxesRaw_t data;
	LIS3DH_Monti_Init();

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
  MX_DMA_Init();
  // MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  uint16_t wheel_size = 22;

  configure_motors(motors);
  initialize_drivetrain(motors,
		  	  	  	  	&holonomic3_system,
						drivetrains_holonomic3,
						wheel_size);

  //uint16_t inc_duty_cycle = 0;
  //drive_motor(&motors[0], 0, 1, 0);

  drive_system_holonomic3(&holonomic3_system, 0, DEG_CW);
  //drive_motors_holonomic3(&holonomic3_system, 10, 10, 10);

  HAL_UART_Receive_DMA(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE);

  //char* msg_fail = "Transmission Failed\r\n";
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

  //char *msg_loop = "Hello World";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LIS3DH_GetAccAxesRaw(&data);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  // Constantly polls for new information to fill the rx buffer
	  HAL_UART_Receive_IT(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE); //MSG_BUFFER_SIZE

	  /*
	   * Example speed dynamic control
	   * NOTE: Bug where it is incrementing by the (1, in this case)
	   * 	during debug just fine, but during continuous runtime,
	   * 	the PWM output is "jittery" and not smooth updates.
	   */
	  if(HAL_GetTick() % 1000 == 0)
	  {
//		  assemble_message_from_vehicle(msg_tx, MSG_TX_BUFFER_SIZE);
//		  HAL_UART_Transmit(&huart2, (uint8_t*)msg_tx, MSG_TX_BUFFER_SIZE, 0xFFFF);

		  //strcpy(msg_tx, msg_tx);
		  //HAL_UART_Transmit(&huart2, (uint8_t*)msg_loop, strlen(msg_loop), 0xFFFF);
		  //HAL_UART_Transmit(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE, 0xFFFF);
		  //HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg_loop, strlen(msg_loop));
		  //HAL_UART_Transmit(&huart2, huart2.pRxBuffPtr, MSG_BUFFER_SIZE, 0xFFFF);
		  /*
		  if(HAL_UART_Transmit(&huart2, fs, huart2.RxXferSize, 0xFFFF) != HAL_OK)
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)msg_fail, strlen(msg_fail), 0xFFFF);
		  }
		  */
		  //huart2.RxXferSize
		  /*
		  if(HAL_UART_Transmit(&huart2, (uint8_t*)msg_rx, strlen(msg_rx), 10)!= HAL_OK)
		  {
		    Error_Handler();
		  }
		  */
		  //HAL_UART_Transmit(&huart2, (uint8_t*)msg_tx, strlen(msg_tx), 0xFFFF);
		  //drive_motor(&motors[0], inc_duty_cycle, 1, 0);
		  //inc_duty_cycle = (inc_duty_cycle + 1) % 100;
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		msg_rx_type = vehicle_message_receive(msg_rx);
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE, 0xFFFF);

		switch(msg_rx_type)
		{
		case MSG_HEADER_CONFIG:
			// TODO
			break;
		case MSG_HEADER_COMMAND:
			drive_system_holonomic3(&holonomic3_system,
									msg_to_vehicle.throttle,
									msg_to_vehicle.direction);
			break;
		default:
			// Nothing
			break;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {

  };
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
