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

	struct motor *motors[3];	// Array of motor pointers: [0] == front; [1] == right; [2] == left
};

void initialize_holonomic3(uint16_t _wheel_diameter,
						   void *_drivetrain,
						   struct motor *_motor_front,
						   struct motor *_motor_right,
						   struct motor *_motor_left);

// TODO: The compiler does not like the function definition here for some reason.
/*
void drive_system_holonomic3(void *_drivetrain,
							 uint8_t system_speed,
							 direction_t _direction);
*/

void drive_motors_holonomic3(void *_drivetrain,
							 uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3);

#endif /* HOLONOMIC3_H_ */
