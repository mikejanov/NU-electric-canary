/*
 * holonomic3.c
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#include "holonomic3.h"

void initialize_holonomic3(uint16_t _wheel_diameter,
						   void *_drivetrain,
						   struct motor *_motor_front,
						   struct motor *_motor_right,
						   struct motor *_motor_left)
{
	struct holonomic3 *this_drivetrain = (struct holonomic3*)_drivetrain;

	this_drivetrain->wheel_diameter = _wheel_diameter;

	this_drivetrain->motors[0] = _motor_front;
	this_drivetrain->motors[1] = _motor_right;
	this_drivetrain->motors[2] = _motor_left;
}

void drive_system_holonomic3(void *_drivetrain,
							 uint8_t _system_speed,
							 direction_t _direction)
{
	struct holonomic3 *this_drivetrain = (struct holonomic3*)_drivetrain;

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
		set_motor_stopped(this_drivetrain->motors[0]);
		if(_direction == DEG_0)
		{
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_positive(this_drivetrain->motors[2]);
		}
		if(_direction == DEG_180)
		{
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 0;
		this_drivetrain->motors[1]->pwm_duty = 99;
		this_drivetrain->motors[2]->pwm_duty = 99;
	}
	// Right-side angles
	else if((_direction == DEG_45) || (_direction == DEG_135))
	{
		set_motor_positive(this_drivetrain->motors[0]);
		if(_direction == DEG_45)
		{
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}
		if(_direction == DEG_135)
		{
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
		this_drivetrain->motors[2]->pwm_duty = 45;
	}
	// Left-side angles
	else if((_direction == DEG_225) || (_direction == DEG_315))
	{
		set_motor_negative(this_drivetrain->motors[0]);
		if(_direction == DEG_225)
		{
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_positive(this_drivetrain->motors[2]);
		}
		if(_direction == DEG_315)
		{
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 45;
		this_drivetrain->motors[2]->pwm_duty = 99;
	}
	// Parallel strafing
	else if((_direction == DEG_90) || (_direction == DEG_270))
	{
		if(_direction == DEG_90)
		{
			set_motor_positive(this_drivetrain->motors[0]);
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}
		if(_direction == DEG_270)
		{
			set_motor_negative(this_drivetrain->motors[0]);
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 42;
		this_drivetrain->motors[2]->pwm_duty = 42;
	}
	else if(_direction == DEG_CW)
	{
		// Set directionality
		// 	0 = POS
		//	1 = POS
		//	2 = POS
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);
		set_motor_positive(this_drivetrain->motors[2]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
		this_drivetrain->motors[2]->pwm_duty = 99;
	}
	else if(_direction == DEG_CCW)
	{
		// Set directionality
		// 	0 = NEG
		//	1 = NEG
		//	2 = NEG
		set_motor_negative(this_drivetrain->motors[0]);
		set_motor_negative(this_drivetrain->motors[1]);
		set_motor_negative(this_drivetrain->motors[2]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
		this_drivetrain->motors[2]->pwm_duty = 99;
	}
	else
	{
		// Do nothing
	}

	// Finally, throttle and drive the motors
	for(int imotor = 0; imotor < 3; imotor ++)
	{
		throttle_motor(_system_speed, this_drivetrain->motors[imotor]);
		drive_motor_struct(this_drivetrain->motors[imotor]);
	}
}

void drive_motors_holonomic3(void *_drivetrain,
							 uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3)
{
	struct holonomic3 *this_drivetrain = (struct holonomic3*)_drivetrain;
	uint16_t pwm[3] = {pwm1, pwm2, pwm3};

	for(int imotor = 0; imotor < 3; imotor ++)
	{
		drive_motor(this_drivetrain->motors[imotor], pwm[imotor], 1, 0);
	}
}
