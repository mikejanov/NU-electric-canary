/*
 * holonomic3.h
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#ifndef HOLONOMIC3_H_
#define HOLONOMIC3_H_

#include "drivetrain.h"

struct holonomic3
{
	uint16_t wheel_diameter;	// Wheel diameter in millimeters

	struct motor *motor_front;	// Pointer to front motor
	struct motor *motor_right;	// Pointer to the clockwise motor relative to front motor
	struct motor *motor_left;	// Pointer to the counter-clockwise motor relative to front motor
};

void initialize_holonomic3(uint16_t _wheel_diameter,
						   void *_drivetrain,
						   struct motor *_motor_front,
						   struct motor *_motor_right,
						   struct motor *_motor_left);

void drive_system_holonomic3(void *_drivetrain,
							 uint8_t system_speed,
							 uint8_t direction);

void drive_motors_holonomic3(void *_drivetrain,
							 uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3);

#endif /* HOLONOMIC3_H_ */
