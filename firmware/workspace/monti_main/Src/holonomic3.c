/*
 * holonomic3.c
 *
 *  Created on: Oct 17, 2017
 *      Author: michaeljanov
 */

#include "holonomic3.h"

uint8_t initialize_holonomic3(uint16_t _wheel_diameter, struct holonomic3 *_holonomic3)
{
	for(int i = 0; i < 3; i++)
	{
		start_motor(&_holonomic3->motor[i]);
	}
}
