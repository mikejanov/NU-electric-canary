/*
 * differential2wd.h
 *
 *  Created on: Nov 18, 2017
 *      Author: michaeljanov
 */

#ifndef DIFFERENTIAL2WD_H_
#define DIFFERENTIAL2WD_H_

#include "drivetrain.h"

struct differential2wd
{
	uint16_t wheel_diameter;	// Wheel diameter in millimeters

	struct motor *motors[2];	// Array of motor pointers: [0] == left; [1] == right
}differential2wd_system;

void initialize_differential2wd(uint16_t _wheel_diameter,
								struct motor *_motor_left,
								struct motor *_motor_right);

// TODO: The compiler does not like the function definition here for some reason.
/*
void drive_system_differential2wd(uint8_t system_speed,
								  direction_t _direction);
*/


void drive_motors_differential2wd(uint16_t pwm1,
								  uint16_t pwm2);

#endif /* DIFFERENTIAL2WD_H_ */
