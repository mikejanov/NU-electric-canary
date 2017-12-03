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

uint8_t vehicle_message_receive(char message[])
{
	// Read header and perform operation
	switch (message[0])
	{
	case(MSG_HEADER_CONFIG):
		vehicle_message_parse_config(message);
		return MSG_HEADER_CONFIG;
	case(MSG_HEADER_COMMAND):
		vehicle_message_parse_command(message);
		return MSG_HEADER_COMMAND;
	default:
		message_error_handler(message, ERROR_MSG_HEADER);
		return 0x00;
	}
}

uint8_t vehicle_message_parse_config(char message[])
{
	uint16_t ii_message = 0;

	// Store the message into the struct
	msg_vehicle_config.header = message[ii_message++];
	msg_vehicle_config.num_modules = message[ii_message++];
	msg_vehicle_config.drive_type = message[ii_message++];

	for(int ii = 0; ii < NUM_MAX_PODS; ii++)
	{
		msg_vehicle_config.pod_ids[ii] = message[ii_message++];
	}

	return 0;
}

uint8_t vehicle_message_parse_command(char message[])
{
	uint16_t ii_message = 0;

	// Store the message into the struct
	msg_to_vehicle.header = (uint8_t)message[ii_message++];
	msg_to_vehicle.direction = (direction_t)message[ii_message++];
	msg_to_vehicle.throttle = (uint8_t)message[ii_message++];
	msg_to_vehicle.actuation_time = (uint8_t)message[ii_message++];

	return 0;
}

uint8_t assemble_message_from_vehicle(char message[], uint16_t buf_size)
{
	// Make sure the message not too big
	if(buf_size > MSG_TX_BUFFER_SIZE)
	{
		message_error_handler(message, ERROR_MSG_SIZE);
		return 1;
	}else{
		message[MSG_ERROR_INDEX] = '\0';
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
		message[ii_message++] = (char)(msg_from_vehicle.sensors[ii]);
	}

	// Fill the remainder of the message with NULL characters
	for(int ii = ii_message; ii < buf_size; ii++)
	{
		message[ii] = '\0';
	}

	return 0;
}
