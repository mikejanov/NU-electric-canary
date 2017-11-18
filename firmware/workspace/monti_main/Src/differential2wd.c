/*
 * differential2wd.c
 *
 *  Created on: Nov 18, 2017
 *      Author: michaeljanov
 */

#include "differential2wd.h"

void initialize_differential2wd(uint16_t _wheel_diameter,
						   	   	void *_drivetrain,
								struct motor *_motor_left,
								struct motor *_motor_right)
{
	struct differential2wd *this_drivetrain = (struct differential2wd*)_drivetrain;

	this_drivetrain->wheel_diameter = _wheel_diameter;

	this_drivetrain->motors[0] = _motor_left;
	this_drivetrain->motors[1] = _motor_right;
}

void drive_system_differential2wd(void *_drivetrain,
							 	  uint8_t system_speed,
								  direction_t _direction)
{
	struct differential2wd *this_drivetrain = (struct differential2wd*)_drivetrain;

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
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	// Bank Right Forward
	else if(_direction == DEG_45)
	{
		// Set directionality
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 50;
	}
	// Bank Right Backwards
	else if(_direction == DEG_135)
	{
		// Set directionality
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_negative(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 50;
	}
	// Reverse
	else if(_direction == DEG_180)
	{
		// Set directionality
		set_motor_negative(this_drivetrain->motors[0]);
		set_motor_negative(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	// Bank Left Backwards
	else if(_direction == DEG_180)
	{
		// Set directionality
		set_motor_negative(this_drivetrain->motors[0]);
		set_motor_negative(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 50;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	// Bank Left Forward
	else if(_direction == DEG_315)
	{
		// Set directionality
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 50;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	// Turn Clockwise
	else if((_direction == DEG_90) || (_direction == DEG_CW))
	{
		// Set directionality
		set_motor_positive(this_drivetrain->motors[0]);
		set_motor_negative(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	// Turn Counter-Clockwise
	else if((_direction == DEG_225) || (_direction == DEG_CCW))
	{
		// Set directionality
		set_motor_negative(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);

		// Set speed
		this_drivetrain->motors[0]->pwm_duty = 99;
		this_drivetrain->motors[1]->pwm_duty = 99;
	}
	else
	{
		// Do nothing
	}

	// Finally, drive the motors
	for(int imotor = 0; imotor < 3; imotor ++)
	{
		drive_motor_struct(this_drivetrain->motors[imotor]);
	}
}

void drive_motors_differential2wd(void *_drivetrain,
							 	  uint16_t pwm1,
								  uint16_t pwm2)
{
	struct differential2wd *this_drivetrain = (struct differential2wd*)_drivetrain;
	uint16_t pwm[2] = {pwm1, pwm2};

	for(int imotor = 0; imotor < 2; imotor ++)
	{
		drive_motor(this_drivetrain->motors[imotor], pwm[imotor], 1, 0);
	}
}
