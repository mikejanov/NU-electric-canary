/*
 * drivetrain.c
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#include "drivetrain.h"

void initialize_drivetrain(uint8_t _drivetrain_type, uint16_t _wheel_diameter)
{
	  //HAL_TIM_Base_Start(&htim1);
	  //HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	  //HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);

	  /**
	   * Call correct motor initialization
	   */
	  switch (_drivetrain_type){
	  case holonomic3:
		  initialize_holonomic3(_wheel_diameter);
	  }
}

void configure_motors(struct motor _motors[])
{
	  // Configure _motors[0]
	  _motors[0].enc_a_bus = MOTOR_A_ENC_A_GPIO_Port;
	  _motors[0].enc_a_pin = MOTOR_A_ENC_A_Pin;
	  _motors[0].enc_b_bus = MOTOR_A_ENC_B_GPIO_Port;
	  _motors[0].enc_b_pin = MOTOR_A_ENC_B_Pin;
	  _motors[0].pwm_duty_base = &htim1;
	  _motors[0].pwm_duty_channel = TIM_CHANNEL_1;
	  _motors[0].in_pos_bus = MOTOR_A_POS_GPIO_Port;
	  _motors[0].in_pos_pin = MOTOR_A_POS_Pin;
	  _motors[0].in_neg_bus = MOTOR_A_NEG_GPIO_Port;
	  _motors[0].in_neg_pin = MOTOR_A_NEG_Pin;

	  // Configure _motors[1]
	  _motors[1].enc_a_bus = MOTOR_B_ENC_A_GPIO_Port;
	  _motors[1].enc_a_pin = MOTOR_B_ENC_A_Pin;
	  _motors[1].enc_b_bus = MOTOR_B_ENC_B_GPIO_Port;
	  _motors[1].enc_b_pin = MOTOR_B_ENC_B_Pin;
	  _motors[1].pwm_duty_base = &htim1;
	  _motors[1].pwm_duty_channel = TIM_CHANNEL_2;
	  _motors[1].in_pos_bus = MOTOR_B_POS_GPIO_Port;
	  _motors[1].in_pos_pin = MOTOR_B_POS_Pin;
	  _motors[1].in_neg_bus = MOTOR_B_NEG_GPIO_Port;
	  _motors[1].in_neg_pin = MOTOR_B_NEG_Pin;

	  // Configure _motors[2]
	  _motors[2].enc_a_bus = MOTOR_C_ENC_A_GPIO_Port;
	  _motors[2].enc_a_pin = MOTOR_C_ENC_A_Pin;
	  _motors[2].enc_b_bus = MOTOR_C_ENC_B_GPIO_Port;
	  _motors[2].enc_b_pin = MOTOR_C_ENC_B_Pin;
	  _motors[2].pwm_duty_base = &htim1;
	  _motors[2].pwm_duty_channel = TIM_CHANNEL_3;
	  _motors[2].in_pos_bus = MOTOR_C_POS_GPIO_Port;
	  _motors[2].in_pos_pin = MOTOR_C_POS_Pin;
	  _motors[2].in_neg_bus = MOTOR_C_NEG_GPIO_Port;
	  _motors[2].in_neg_pin = MOTOR_C_NEG_Pin;

	  // Configure _motors[3]
	  // { Empty }
	  // No known configuraton with 4 motors yet

	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	  // Start motors
//	  for (int i = 0; i < 4; i++)
//	  {
//		  start_motor(&_motors[i]);
//	  }
}

void start_motor(struct motor *_motor)
{
	  HAL_TIM_Base_Start(_motor->pwm_duty_base);
	  HAL_TIM_PWM_Start(_motor->pwm_duty_base, _motor->pwm_duty_channel);
}

void drive_motor(struct motor *_motor)
{
	/**
	 * Set new PWM duty cycling using the provided HAL macros
	 */
	//__HAL_TIM_GET_AUTORELOAD(_motor->pwm_duty_base); //gets the Period set for PWm
	__HAL_TIM_GET_AUTORELOAD(&htim1); //gets the Period set for PWm
	// Sets the new PWM duty cycle (Capture Compare Value)
	//__HAL_TIM_SET_COMPARE(_motor->pwm_duty_base,
	//					  _motor->pwm_duty_channel,
	//					  _motor->pwm_duty);
	__HAL_TIM_SET_COMPARE(&htim1,
						  TIM_CHANNEL_1,
						  (uint16_t) 50);

	if(_motor->in_pos)
	{
		HAL_GPIO_WritePin(_motor->in_pos_bus, _motor->in_pos_pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_motor->in_pos_bus, _motor->in_pos_pin, GPIO_PIN_RESET);
	}

	if(_motor->in_neg)
	{
		HAL_GPIO_WritePin(_motor->in_neg_bus, _motor->in_neg_pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(_motor->in_neg_bus, _motor->in_neg_pin, GPIO_PIN_RESET);
	}
}

void drive_motor_overload(struct motor *_motor, uint8_t _pwm_duty, uint8_t _in_pos, uint8_t _in_neg)
{
	_motor->pwm_duty = _pwm_duty;
	_motor->in_pos = _in_pos;
	_motor->in_neg = _in_neg;

	drive_motor(_motor);
}

uint8_t map_speed_to_duty(uint8_t _speed, uint8_t _duty)
{

}

uint8_t convert_enc_to_wheel_speed(uint8_t _enc_a, uint8_t _enc_b)
{

}

uint8_t convert_wheel_speed_to_rpm(uint8_t _wheel_speed)
{

}



