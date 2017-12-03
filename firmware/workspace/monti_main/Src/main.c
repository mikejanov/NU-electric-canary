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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "drivetrain.h"
#include "holonomic3.h"
#include "differential2wd.h"

#include "vehicle_messages.h"

#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// UART Transmission
#define MAX_TIME_SINCE_RX	1000

char msg_rx[MSG_TX_BUFFER_SIZE];
char msg_tx[MSG_TX_BUFFER_SIZE];
uint16_t msg_tx_count = 0;
uint16_t msg_rx_count = 0;
uint8_t msg_rx_type = MSG_HEADER_CONFIG;
uint32_t time_last_rx = 0;

// Motors
struct motor motors[NUM_MOTORS_ENABLED];
drivetrain_options_t current_drivetrain = drivetrains_holonomic3;

// Drivetrain Wheel Sizes
#define HOLONOMIC3_WHEEL_DIA_MM			22
#define DIFFERENTIAL2WD_WHEEL_DIA_MM	83
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void drive_system(drivetrain_options_t _drivetrain,
				  uint8_t system_speed,
				  direction_t _direction);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void drive_system(drivetrain_options_t _drivetrain,
				  uint8_t system_speed,
				  direction_t _direction)
{
	switch (_drivetrain)
	{
	case drivetrains_holonomic3:
		drive_system_holonomic3(msg_to_vehicle.throttle,
								msg_to_vehicle.direction);
		break;
	case drivetrains_differential2wd:
		drive_system_differential2wd(msg_to_vehicle.throttle,
									 msg_to_vehicle.direction);
		break;
	}
}

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
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  uint32_t current_time = 0;

  configure_motors(motors);
  initialize_drivetrain(motors,
						drivetrains_holonomic3,
						HOLONOMIC3_WHEEL_DIA_MM);
  initialize_drivetrain(motors,
						drivetrains_differential2wd,
						DIFFERENTIAL2WD_WHEEL_DIA_MM);

  ////#DEBUG START
  //drive_system_holonomic3(0, DEG_0);
  //drive_motors_holonomic3(10, 10, 10);
  ////#DEBUG END

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
		  assemble_message_from_vehicle(msg_tx, MSG_TX_BUFFER_SIZE);
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg_tx, MSG_TX_BUFFER_SIZE, 0xFFFF);

		  ////#DEBUG START
		  //HAL_UART_Transmit(&huart2, (uint8_t*)msg_loop, strlen(msg_loop), 0xFFFF);
		  //HAL_UART_Transmit(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE, 0xFFFF);
		  ////#DEBUG END
	  }

	  /**
	   * Gather encoder values and calculate speed feedback
	   * TODO: 1kHz is too slow. Need at least 10kHz
	   */
	  if(HAL_GetTick() % 1 == 0)
	  {
		  update_encoders(motors);
		  current_time = HAL_GetTick();
		  for(int ii = 0; ii < NUM_MOTORS_ENABLED; ii++)
		  {
			  update_speed_feedback(&motors[ii],
					  	  	  	 	HAL_GetTick(),
									current_drivetrain);
			  msg_from_vehicle.encoders[ii] = motors[ii].linear_speed;
		  }
	  }

	  /**
	   * Stop the robot if it hasn't received a command in a while,
	   * 	but only check every half second.
	   * 	TODO: Make this a flag flip instead
	   */
	  if(HAL_GetTick() % 500 == 0)
	  {
		  if((HAL_GetTick() - time_last_rx) > MAX_TIME_SINCE_RX)
		  {
				drive_system(current_drivetrain,
							 0,
							 msg_to_vehicle.direction);
		  }
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
/**
 * UART Callbacks
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		// Do nothing
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart2.Instance)
	{
		msg_rx_type = vehicle_message_receive(msg_rx);

		////#DEBUG START
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg_rx, MSG_RX_BUFFER_SIZE, 0xFFFF);
		////#DEBUG END

		time_last_rx = HAL_GetTick(); // Reset RX timer

		switch(msg_rx_type)
		{
		case MSG_HEADER_CONFIG:
			current_drivetrain = msg_vehicle_config.drive_type;
			break;
		case MSG_HEADER_COMMAND:
			drive_system(current_drivetrain,
						 msg_to_vehicle.throttle,
						 msg_to_vehicle.direction);
			break;
		default:
			// Nothing
			break;
		}
	}
}

/**
 * GPIO Callbacks
 */
// Rising edge callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	switch(GPIO_Pin)
	{
	case MOTOR_A_ENC_A_Pin:
		//bool_motor_a_enc_a = !bool_motor_a_enc_a;
		break;
	case MOTOR_A_ENC_B_Pin:

		break;
	case MOTOR_B_ENC_A_Pin:

		break;
	case MOTOR_B_ENC_B_Pin:

		break;
	case MOTOR_C_ENC_A_Pin:

		break;
	}
	*/
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
