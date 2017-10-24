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

	this_drivetrain->motor_front = _motor_front;
	this_drivetrain->motor_right = _motor_right;
	this_drivetrain->motor_left = _motor_left;
}

void drive_motors_holonomic3(void *_drivetrain,
							 uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3)
{
	struct holonomic3 *this_drivetrain = (struct holonomic3*)_drivetrain;

	drive_motor(this_drivetrain->motor_front, pwm1, 1, 0);
	drive_motor(this_drivetrain->motor_right, pwm2, 1, 0);
	drive_motor(this_drivetrain->motor_left, pwm3, 1, 0);
}
