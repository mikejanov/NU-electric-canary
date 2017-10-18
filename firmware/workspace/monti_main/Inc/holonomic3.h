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
	struct motor motor[3];		// 3 motor structs used in drive configuration

	uint16_t wheel_diameter;	// Wheel diameter in millimeters
};

uint8_t initialize_holonomic3(uint16_t _wheel_diameter, struct holonomic3 *_holonomic3);

void drive_system_holonomic3(struct holonomic3 *_holonomic3,
							 uint8_t system_speed,
							 uint8_t direction);

#endif /* HOLONOMIC3_H_ */
