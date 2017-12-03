/*
 * holonomic3.h
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#ifndef HOLONOMIC3_H_
#define HOLONOMIC3_H_

#include "drivetrain.h"
#include "math.h"

struct holonomic3
{
	uint16_t wheel_diameter;	// Wheel diameter in millimeters

	struct motor *motors[3];	// Array of motor pointers: [0] == front; [1] == right; [2] == left
}holonomic3_system;

void initialize_holonomic3(uint16_t _wheel_diameter,
						   struct motor *_motor_front,
						   struct motor *_motor_right,
						   struct motor *_motor_left);

/*
void drive_system_holonomic3(uint8_t system_speed,
							 direction_t _direction);
*/

void drive_motors_holonomic3(uint16_t pwm1,
							 uint16_t pwm2,
							 uint16_t pwm3);

#endif /* HOLONOMIC3_H_ */
