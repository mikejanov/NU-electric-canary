/*
 * drivetrain.h
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 *
 *  Brief:
 * 		Contains all generic structs and functions suitable for most
 * 			drivetrain types.
 */

#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include "gpio.h"
#include "tim.h"
#include "math.h"

#define NUM_MOTORS_ENABLED	3

struct motor
{
	// Control Values

	uint8_t 	enc_a;			// Boolean output value of Encoder A
	uint8_t		enc_b;			// Boolean output value of Encoder B
	uint8_t		pwm_duty;		// Integer between 0-100 representing the duty cycle of the motor
	uint8_t		in_pos;			// Boolean clockwise motor enable pin
	uint8_t		in_neg;			// Boolean counter-clockwise motor enable pin

	// Feedback Values
	uint32_t	enc_last_rise;	// SysTick of the last (enc_a XOR enc_b)
	uint32_t	enc_last_count;	// Counts how many total (runtime) rising edges at the last poll
	uint32_t	enc_count;		// Counts how many rising edges there have been since runtime
	//uint8_t		enc_a_last;		// Keeps track of change for encoder a

	// TODO: decouple linear_speed from motor struct
	uint8_t		linear_speed;	// Coupled with the wheel size of currently-active drivetrain

	// GPIO Control Pins
	GPIO_TypeDef*	enc_a_bus;		// GPIO_x bus of pin
	uint16_t		enc_a_pin;		// GPIO_Pin_x pin

	GPIO_TypeDef*	enc_b_bus;		// GPIO_x bus of pin
	uint16_t		enc_b_pin;		// GPIO_Pin_x pin

	TIM_HandleTypeDef*	pwm_duty_base;		// htim_x base of timer peripheral
	uint16_t			pwm_duty_channel;	// TIM_CHANNEL_x timer channel

	GPIO_TypeDef*	in_pos_bus;		// GPIO_x bus of pin
	uint16_t		in_pos_pin;		// GPIO_Pin_x pin

	GPIO_TypeDef*	in_neg_bus;		// GPIO_x bus of pin
	uint16_t		in_neg_pin;		// GPIO_Pin_x pin

};

/**
 * Maps out the 10 possible directions that the drive system can make.
 * If one of the 10 possible directions was not selected, the '11'th default
 * 	is selected.
 * The Degree number (0 Degrees, etc) does not actually matter in this
 * 	implementation.
 */
typedef enum direction
{
	DEG_0	= 0,
	DEG_45 	= 1,
	DEG_90	= 2,
	DEG_135 = 3,
	DEG_180 = 4,
	DEG_225 = 5,
	DEG_270 = 6,
	DEG_315 = 7,
	DEG_360 = 8,
	DEG_CW	= 9,
	DEG_CCW	= 10
}direction_t;

typedef enum drivetrain_options
{
	drivetrains_differential2wd,
	drivetrains_holonomic3
}drivetrain_options_t;

void initialize_drivetrain(struct motor _motors[],
						   drivetrain_options_t _drivetrain_type,
		   	   	   	   	   uint16_t _wheel_diameter);

void configure_motors(struct motor _motors[]);

void start_motor(struct motor *_motor);

void drive_motor(struct motor *_motor,
				 uint16_t _pwm_duty,
				 uint8_t _in_pos,
				 uint8_t _in_neg);
void drive_motor_struct(struct motor *_motor);

void set_motor_stopped(struct motor *_motor);
void set_motor_negative(struct motor *_motor);
void set_motor_positive(struct motor *_motor);

void throttle_motor(uint8_t _throttle, struct motor *_motor);

void update_encoders(struct motor _motors[]);
void update_speed_feedback(struct motor *_motor, uint32_t _systick, drivetrain_options_t _drivetrain_type);
uint8_t calculate_wheel_linear_speed(struct motor* _motor, uint32_t _time_diff_ms, drivetrain_options_t _drivetrain_type);

uint8_t map_speed_to_duty(uint8_t _speed, uint8_t _duty);

uint8_t convert_enc_to_wheel_speed(uint8_t _enc_a, uint8_t _enc_b);

uint8_t convert_wheel_speed_to_rpm(uint8_t _wheel_speed);



#endif /* DRIVETRAIN_H_ */
