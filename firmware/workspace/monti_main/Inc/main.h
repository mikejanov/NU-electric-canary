/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MOTOR_A_PWM_Pin GPIO_PIN_0
#define MOTOR_A_PWM_GPIO_Port GPIOC
#define MOTOR_B_PWM_Pin GPIO_PIN_1
#define MOTOR_B_PWM_GPIO_Port GPIOC
#define MOTOR_C_PWM_Pin GPIO_PIN_2
#define MOTOR_C_PWM_GPIO_Port GPIOC
#define AIN_0_Pin GPIO_PIN_4
#define AIN_0_GPIO_Port GPIOA
#define AIN_1_Pin GPIO_PIN_5
#define AIN_1_GPIO_Port GPIOA
#define AIN_2_Pin GPIO_PIN_6
#define AIN_2_GPIO_Port GPIOA
#define AIN_3_Pin GPIO_PIN_7
#define AIN_3_GPIO_Port GPIOA
#define DIN_0_Pin GPIO_PIN_4
#define DIN_0_GPIO_Port GPIOC
#define DIN_1_Pin GPIO_PIN_5
#define DIN_1_GPIO_Port GPIOC
#define DIN_2_Pin GPIO_PIN_0
#define DIN_2_GPIO_Port GPIOB
#define DIN_3_Pin GPIO_PIN_1
#define DIN_3_GPIO_Port GPIOB
#define DIN_4_Pin GPIO_PIN_2
#define DIN_4_GPIO_Port GPIOB
#define DIN_5_Pin GPIO_PIN_10
#define DIN_5_GPIO_Port GPIOB
#define MOTOR_C_ENC_A_Pin GPIO_PIN_11
#define MOTOR_C_ENC_A_GPIO_Port GPIOB
#define MOTOR_C_ENC_B_Pin GPIO_PIN_12
#define MOTOR_C_ENC_B_GPIO_Port GPIOB
#define MOTOR_A_ENC_A_Pin GPIO_PIN_13
#define MOTOR_A_ENC_A_GPIO_Port GPIOB
#define MOTOR_A_POS_Pin GPIO_PIN_14
#define MOTOR_A_POS_GPIO_Port GPIOB
#define MOTOR_A_NEG_Pin GPIO_PIN_15
#define MOTOR_A_NEG_GPIO_Port GPIOB
#define DIN_6_Pin GPIO_PIN_7
#define DIN_6_GPIO_Port GPIOC
#define MOTOR_B_POS_Pin GPIO_PIN_8
#define MOTOR_B_POS_GPIO_Port GPIOC
#define MOTOR_B_NEG_Pin GPIO_PIN_9
#define MOTOR_B_NEG_GPIO_Port GPIOC
#define DIN_7_Pin GPIO_PIN_8
#define DIN_7_GPIO_Port GPIOA
#define DIN_8_Pin GPIO_PIN_9
#define DIN_8_GPIO_Port GPIOA
#define DIN_9_Pin GPIO_PIN_10
#define DIN_9_GPIO_Port GPIOA
#define MOTOR_C_POS_Pin GPIO_PIN_11
#define MOTOR_C_POS_GPIO_Port GPIOA
#define MOTOR_C_NEG_Pin GPIO_PIN_12
#define MOTOR_C_NEG_GPIO_Port GPIOA
#define MOTOR_B_ENC_A_Pin GPIO_PIN_12
#define MOTOR_B_ENC_A_GPIO_Port GPIOC
#define MOTOR_B_ENC_B_Pin GPIO_PIN_2
#define MOTOR_B_ENC_B_GPIO_Port GPIOD
#define MOTOR_A_ENC_B_Pin GPIO_PIN_5
#define MOTOR_A_ENC_B_GPIO_Port GPIOB
#define LIMIT_SWITCH_1_Pin GPIO_PIN_8
#define LIMIT_SWITCH_1_GPIO_Port GPIOB
#define LIMIT_SWITCH_2_Pin GPIO_PIN_9
#define LIMIT_SWITCH_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
