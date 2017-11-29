/*
 * holonomic3.c
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#include "holonomic3.h"

void initialize_holonomic3(uint16_t _wheel_diameter,
						   struct motor *_motor_front,
						   struct motor *_motor_right,
						   struct motor *_motor_left)
{
	holonomic3_system.wheel_diameter = _wheel_diameter;

	holonomic3_system.motors[0] = _motor_front;
	holonomic3_system.motors[1] = _motor_right;
	holonomic3_system.motors[2] = _motor_left;
}

void drive_system_holonomic3(uint8_t _system_speed,
							 direction_t _direction)
{
	/**
	 * Set parameters before driving motors
	 *******************************************
	 * Note: if code-golf was the goal, these directionalities could be compressed on an as-called-for basis.
	 * 	However, we felt it was more readable to explicitly name what each directionality was in each
	 * 	option for movement instead.
	 *
	 * 	TODO: PWM speeds have a max of 99. 100 breaks it.
	 */

	// Forward or backward
	if((_direction == DEG_0) || (_direction == DEG_180))
	{
		set_motor_stopped(holonomic3_system.motors[0]);
		if(_direction == DEG_0)
		{
			set_motor_negative(holonomic3_system.motors[1]);
			set_motor_positive(holonomic3_system.motors[2]);
		}
		if(_direction == DEG_180)
		{
			set_motor_positive(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 0;
		holonomic3_system.motors[1]->pwm_duty = 99;
		holonomic3_system.motors[2]->pwm_duty = 99;
	}
	// Right-side angles
	else if((_direction == DEG_45) || (_direction == DEG_135))
	{
		set_motor_positive(holonomic3_system.motors[0]);
		if(_direction == DEG_45)
		{
			set_motor_negative(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}
		if(_direction == DEG_135)
		{
			set_motor_positive(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 99;
		holonomic3_system.motors[1]->pwm_duty = 99;
		holonomic3_system.motors[2]->pwm_duty = 45;
	}
	// Left-side angles
	else if((_direction == DEG_225) || (_direction == DEG_315))
	{
		set_motor_negative(holonomic3_system.motors[0]);
		if(_direction == DEG_225)
		{
			set_motor_negative(holonomic3_system.motors[1]);
			set_motor_positive(holonomic3_system.motors[2]);
		}
		if(_direction == DEG_315)
		{
			set_motor_positive(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 99;
		holonomic3_system.motors[1]->pwm_duty = 45;
		holonomic3_system.motors[2]->pwm_duty = 99;
	}
	// Parallel strafing
	else if((_direction == DEG_90) || (_direction == DEG_270))
	{
		if(_direction == DEG_90)
		{
			set_motor_positive(holonomic3_system.motors[0]);
			set_motor_negative(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}
		if(_direction == DEG_270)
		{
			set_motor_negative(holonomic3_system.motors[0]);
			set_motor_positive(holonomic3_system.motors[1]);
			set_motor_negative(holonomic3_system.motors[2]);
		}

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 99;
		holonomic3_system.motors[1]->pwm_duty = 42;
		holonomic3_system.motors[2]->pwm_duty = 42;
	}
	else if(_direction == DEG_CW)
	{
		// Set directionality
		// 	0 = POS
		//	1 = POS
		//	2 = POS
		set_motor_positive(holonomic3_system.motors[0]);
		set_motor_positive(holonomic3_system.motors[1]);
		set_motor_positive(holonomic3_system.motors[2]);

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 99;
		holonomic3_system.motors[1]->pwm_duty = 99;
		holonomic3_system.motors[2]->pwm_duty = 99;
	}
	else if(_direction == DEG_CCW)
	{
		// Set directionality
		// 	0 = NEG
		//	1 = NEG
		//	2 = NEG
		set_motor_negative(holonomic3_system.motors[0]);
		set_motor_negative(holonomic3_system.motors[1]);
		set_motor_negative(holonomic3_system.motors[2]);

		// Set speed
		holonomic3_system.motors[0]->pwm_duty = 99;
		holonomic3_system.motors[1]->pwm_duty = 99;
		holonomic3_system.motors[2]->pwm_duty = 99;
	}
	else
	{
		// Do nothing
	}

	// Finally, throttle and drive the motors
	for(int imotor = 0; imotor < 3; imotor ++)
	{
		throttle_motor(_system_speed, holonomic3_system.motors[imotor]);
		drive_motor_struct(holonomic3_system.motors[imotor]);
	}
}

void drive_motors_holonomic3(uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3)
{
	uint16_t pwm[3] = {pwm1, pwm2, pwm3};

	for(int imotor = 0; imotor < 3; imotor ++)
	{
		drive_motor(holonomic3_system.motors[imotor], pwm[imotor], 1, 0);
	}
}
