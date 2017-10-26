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

	/*
	this_drivetrain->motor_front = _motor_front;
	this_drivetrain->motor_right = _motor_right;
	this_drivetrain->motor_left = _motor_left;
	*/
}

void drive_system_holonomic3(void *_drivetrain,
							 uint8_t _system_speed,
							 direction_t _direction)
{
	struct holonomic3 *this_drivetrain = (struct holonomic3*)_drivetrain;
	uint16_t motor_pwm[3];

	/**
	 * Set parameters before driving motors
	 *******************************************
	 * Note: if code-golf was the goal, these directionalities could be compressed on an as-called-for basis.
	 * 	However, we felt it was more readable to explicitly name what each directionality was in each
	 * 	option for movement instead.
	 *
	 * 	TODO: PWM speeds are placeholder, but proportionally roughly accurate
	 */

	// Forward or backward
	if((_direction == DEG_0) || (_direction == DEG_180))
	{
		// Set directionality
		// 	0 = {stop}
		//	1 = POS
		//	2 = POS / NEG
		set_motor_stopped(this_drivetrain->motors[0]);
		set_motor_positive(this_drivetrain->motors[1]);
		if(_direction == DEG_0)
		{
			set_motor_positive(this_drivetrain->motors[2]);
		}
		else
		{
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		motor_pwm[0] = 0;
		motor_pwm[1] = 100;
		motor_pwm[2] = 100;
	}
	// Right-side angles
	else if((_direction == DEG_45) || (_direction == DEG_135))
	{
		// Set directionality
		// 	0 = POS
		//	1 = NEG / POS
		//	2 = POS / NEG
		set_motor_positive(this_drivetrain->motors[0]);
		if(_direction == DEG_45)
		{
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_positive(this_drivetrain->motors[2]);
		}
		else
		{
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		motor_pwm[0] = 100;
		motor_pwm[1] = 100;
		motor_pwm[2] = 45;
	}
	// Left-side angles
	else if((_direction == DEG_225) || (_direction == DEG_315))
	{
		// Set directionality
		// 	0 = NEG
		//	1 = POS / NEG
		//	2 = NEG / POS
		set_motor_negative(this_drivetrain->motors[0]);
		if(_direction == DEG_225)
		{
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}
		else
		{
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_positive(this_drivetrain->motors[2]);
		}

		// Set speed
		motor_pwm[0] = 100;
		motor_pwm[1] = 45;
		motor_pwm[2] = 100;
	}
	// Parallel strafing
	else if((_direction == DEG_90) || (_direction == DEG_270))
	{
		// Set directionality
		// 	0 = POS / NEG
		//	1 = NEG / POS
		//	2 = POS / NEG
		if(_direction == DEG_90)
		{
			set_motor_positive(this_drivetrain->motors[0]);
			set_motor_negative(this_drivetrain->motors[1]);
			set_motor_positive(this_drivetrain->motors[2]);
		}
		else
		{
			set_motor_negative(this_drivetrain->motors[0]);
			set_motor_positive(this_drivetrain->motors[1]);
			set_motor_negative(this_drivetrain->motors[2]);
		}

		// Set speed
		motor_pwm[0] = 100;
		motor_pwm[1] = 42;
		motor_pwm[2] = 42;
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
		motor_pwm[0] = 100;
		motor_pwm[1] = 100;
		motor_pwm[2] = 100;
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
		motor_pwm[0] = 100;
		motor_pwm[1] = 100;
		motor_pwm[2] = 100;
	}
	else
	{
		// Do nothing
	}

	// Finally, drive the motors
	for(int imotor = 0; imotor < 3; imotor ++)
	{
		drive_motor(this_drivetrain->motors[imotor], motor_pwm[imotor],
					this_drivetrain->motors[imotor]->in_pos, this_drivetrain->motors[imotor]->in_neg);
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
