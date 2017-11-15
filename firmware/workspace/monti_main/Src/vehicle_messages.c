/*
 * vehicle_messages.c
 *
 *  Created on: Nov 10, 2017
 *      Author: michaeljanov
 */

#include "vehicle_messages.h"
#include "string.h"

void vehicle_message_initialize(void)
{
	// Header initialization (# modules, drivetrain type, ids of modules)
}

void assemble_message_from_vehicle(char* message, uint16_t buf_size)
{
	char tempChar;
	message = "";

	tempChar = (char)msg_from_vehicle.header;
	strcat(message, &tempChar);

	tempChar = (char)msg_from_vehicle.error;
	strcat(message, &tempChar);

	for(int ii = 0; ii < NUM_TOTAL_ENCODER_VALUES; ii++)
	{
		tempChar = (char)msg_from_vehicle.encoders[ii];
		strcat(message, &tempChar);
	}

	for(int ii = 0; ii < NUM_TOTAL_ACCEL_AXIS; ii++)
	{
		tempChar = (char)(msg_from_vehicle.accelerometer[ii]);
		strcat(message, &tempChar);

		tempChar = (char)(msg_from_vehicle.accelerometer[ii] >> 8);
		strcat(message, &tempChar);
	}

	for(int ii = 0; ii < NUM_TOTAL_DIGITAL_SENSORS; ii++)
	{
		tempChar = (char)(msg_from_vehicle.sensors[ii] >> 8);
		strcat(message, &tempChar);
	}

	strcat(message, "\r\n");
}
