/*
 * drivetrain.c
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#include "drivetrain.h"

void initialize_drivetrain(struct motor _motors[],
							void *_drivetrain,
						   	drivetrain_options_t _drivetrain_type,
		   	   	   	   	    uint16_t _wheel_diameter)
{
	  /**
	   * Call correct motor initialization
	   */
	  switch (_drivetrain_type){
	  case drivetrains_holonomic3:
		  initialize_holonomic3(_wheel_diameter,
				  	  	  	  	_drivetrain,
								&_motors[0], &_motors[1], &_motors[2]);
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
	  _motors[0].pwm_duty_channel = TIM_CHANNEL_4;
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
	  // No known configuration with 4 motors yet

	  // Start motors
	  for (int i = 0; i < NUM_MOTORS_ENABLED; i++)
	  {
		  start_motor(&_motors[i]);
	  }
}

void start_motor(struct motor *_motor)
{
	  HAL_TIM_Base_Start(_motor->pwm_duty_base);
	  HAL_TIM_PWM_Start(_motor->pwm_duty_base, _motor->pwm_duty_channel);
}

void drive_motor(struct motor *_motor,
				 uint16_t _pwm_duty,
				 uint8_t _in_pos,
				 uint8_t _in_neg)
{
	/**
	 * Update struct values
	 */
	_motor->pwm_duty = _pwm_duty % 100;
	_motor->in_pos = _in_pos;
	_motor->in_neg = _in_neg;

	/**
	 * Set new PWM duty cycling using the provided HAL macros
	 * 	CompareValue = Period * DutyCycle / 100
	 */
	uint16_t timer_compare = _motor->pwm_duty_base->Init.Period *
							 _motor->pwm_duty /
							 100;
	__HAL_TIM_GET_AUTORELOAD(_motor->pwm_duty_base);
	__HAL_TIM_SET_COMPARE(_motor->pwm_duty_base,
			  	  	  	  _motor->pwm_duty_channel,
						  timer_compare);

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

void drive_motor_struct(struct motor *_motor)
{
	drive_motor(_motor, _motor->pwm_duty,
				_motor->in_pos, _motor->in_neg);
}

void set_motor_stopped(struct motor *_motor)
{
	_motor->in_pos = 0;
	_motor->in_neg = 0;
}

void set_motor_negative(struct motor *_motor)
{
	_motor->in_pos = 1;
	_motor->in_neg = 0;
}

void set_motor_positive(struct motor *_motor)
{
	_motor->in_pos = 0;
	_motor->in_neg = 1;
}

void throttle_motor(uint8_t _throttle, struct motor *_motor)
{
	_motor->pwm_duty = (uint8_t)(_motor->pwm_duty * _throttle / 100);
}

void update_encoders(struct motor _motors[])
{
	_motors[0].enc_a = HAL_GPIO_ReadPin(MOTOR_A_ENC_A_GPIO_Port, MOTOR_A_ENC_A_Pin);
	_motors[0].enc_b = HAL_GPIO_ReadPin(MOTOR_A_ENC_B_GPIO_Port, MOTOR_A_ENC_B_Pin);
	_motors[1].enc_a = HAL_GPIO_ReadPin(MOTOR_B_ENC_A_GPIO_Port, MOTOR_B_ENC_A_Pin);
	_motors[1].enc_b = HAL_GPIO_ReadPin(MOTOR_B_ENC_B_GPIO_Port, MOTOR_B_ENC_B_Pin);
	_motors[2].enc_a = HAL_GPIO_ReadPin(MOTOR_C_ENC_A_GPIO_Port, MOTOR_C_ENC_A_Pin);
	_motors[2].enc_b = HAL_GPIO_ReadPin(MOTOR_C_ENC_B_GPIO_Port, MOTOR_C_ENC_B_Pin);
}

uint8_t update_speed_feedback(struct motor *_motor, uint32_t _systick)
{
	uint32_t time_diff = 0;
	// Check if there is a rising edge on the XOR
	if(_motor->enc_a ^ _motor->enc_b)
	{
		// There are 4 XOR rising edges in one revolution
		time_diff = _motor->enc_last_rise - _systick;
		// Update the last rise
		_motor->enc_last_rise  = _systick;
	}
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



