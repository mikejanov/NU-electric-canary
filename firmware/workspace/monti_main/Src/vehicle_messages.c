/*
 * vehicle_messages.c
 *
 *  Created on: Nov 10, 2017
 *      Author: michaeljanov
 */

#include "vehicle_messages.h"

void message_error_handler(char message[], char error_id)
{
	message[0] = '\0';
	message[1] = (char)error_id;
	for(int ii = 2; ii < MSG_TX_BUFFER_SIZE; ii++)
	{
		message[ii] = '\0';
	}
}

uint8_t vehicle_message_initialize(void)
{
	// Header initialization (# modules, drivetrain type, ids of modules)
	return 0;
}

uint8_t assemble_message_from_vehicle(char message[], uint16_t buf_size)
{
	// Make sure the message not too big
	if(buf_size > MSG_TX_BUFFER_SIZE)
	{
		message_error_handler(message, ERROR_MSG_SIZE);
		return 1;
	}

	// Initialize the counter
	uint16_t ii_message = 0;

	// Begin assembling the message
	message[ii_message++] = (char)msg_from_vehicle.header;
	message[ii_message++] = (char)msg_from_vehicle.error;

	for(int ii = 0; ii < NUM_TOTAL_ENCODER_VALUES; ii++)
	{
		message[ii_message++] = (char)msg_from_vehicle.encoders[ii];
	}

	for(int ii = 0; ii < NUM_TOTAL_ACCEL_AXIS; ii++)
	{
		message[ii_message++] = (char)(msg_from_vehicle.accelerometer[ii]);
		message[ii_message++] = (char)(msg_from_vehicle.accelerometer[ii] >> 8);
	}

	for(int ii = 0; ii < NUM_TOTAL_DIGITAL_SENSORS; ii++)
	{
		message[ii_message++] = 'c'; //(char)(msg_from_vehicle.sensors[ii])
	}

	// Fill the remainder of the message with NULL characters
	for(int ii = ii_message; ii < buf_size; ii++)
	{
		message[ii] = '\0';
	}

	return 0;
}
