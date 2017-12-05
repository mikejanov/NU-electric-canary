/*
 * differential2wd.c
 *
 *  Created on: Nov 18, 2017
 *      Author: michaeljanov
 */

#include "differential2wd.h"

void initialize_differential2wd(uint16_t _wheel_diameter,
								struct motor *_motor_left,
								struct motor *_motor_right)
{
	differential2wd_system.wheel_diameter = _wheel_diameter;

	differential2wd_system.motors[0] = _motor_left;
	differential2wd_system.motors[1] = _motor_right;
}

void drive_system_differential2wd(uint8_t system_speed,
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

	// Forward
	if(_direction == DEG_0)
	{
		// Set directionality
		set_motor_positive(differential2wd_system.motors[0]);
		set_motor_negative(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	// Bank Right Forward
	else if(_direction == DEG_45)
	{
		// Set directionality
		set_motor_positive(differential2wd_system.motors[0]);
		set_motor_negative(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 50;
	}
	// Bank Right Backwards
	else if(_direction == DEG_135)
	{
		// Set directionality
		set_motor_negative(differential2wd_system.motors[0]);
		set_motor_positive(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 50;
	}
	// Reverse
	else if(_direction == DEG_180)
	{
		// Set directionality
		set_motor_negative(differential2wd_system.motors[0]);
		set_motor_positive(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	// Bank Left Backwards
	else if(_direction == DEG_180)
	{
		// Set directionality
		set_motor_negative(differential2wd_system.motors[0]);
		set_motor_positive(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 50;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	// Bank Left Forward
	else if(_direction == DEG_315)
	{
		// Set directionality
		set_motor_positive(differential2wd_system.motors[0]);
		set_motor_negative(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 50;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	// Turn Clockwise
	else if((_direction == DEG_90) || (_direction == DEG_CW))
	{
		// Set directionality
		set_motor_negative(differential2wd_system.motors[0]);
		set_motor_negative(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	// Turn Counter-Clockwise
	else if((_direction == DEG_225) || (_direction == DEG_CCW))
	{
		// Set directionality
		set_motor_positive(differential2wd_system.motors[0]);
		set_motor_positive(differential2wd_system.motors[1]);

		// Set speed
		differential2wd_system.motors[0]->pwm_duty = 99;
		differential2wd_system.motors[1]->pwm_duty = 99;
	}
	else
	{
		// Do nothing
	}

	// Finally, drive the motors
	for(int imotor = 0; imotor < 2; imotor ++)
	{
		throttle_motor(system_speed, differential2wd_system.motors[imotor]);
		drive_motor_struct(differential2wd_system.motors[imotor]);
	}
}

void drive_motors_differential2wd(uint16_t pwm1,
								  uint16_t pwm2)
{
	uint16_t pwm[2] = {pwm1, pwm2};

	for(int imotor = 0; imotor < 2; imotor ++)
	{
		drive_motor(differential2wd_system.motors[imotor], pwm[imotor], 1, 0);
	}
}
