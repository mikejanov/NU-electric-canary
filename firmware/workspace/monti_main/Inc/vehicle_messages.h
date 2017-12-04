/*
 * vehicle_messages.h
 *
 *  Created on: Nov 10, 2017
 *      Author: michaeljanov
 */

#ifndef VEHICLE_MESSAGES_H_
#define VEHICLE_MESSAGES_H_

#include "string.h"

#include "drivetrain.h"
#include "sensors.h"

#define NUM_TOTAL_ENCODER_VALUES	6

#define MSG_TX_BUFFER_SIZE sizeof(msg_from_vehicle)
#define MSG_RX_BUFFER_SIZE sizeof(msg_vehicle_config)

#define MSG_ERROR_INDEX		1
#define ERROR_MSG_SIZE		0x01
#define	ERROR_MSG_HEADER	0x02

#define MSG_HEADER_CONFIG	0x01
#define MSG_HEADER_COMMAND	0x02

/*
 * @header
 * 	Contains configuration options according to the following
 * 		MSB scheme:
 * 		(0) - Config
 * 		(1) - Command
 * 		(2) -
 * 		(3) -
 * 		(4) -
 * 		(5) -
 * 		(6) -
 * 		(7) -
 * @error
 * 	Error messages are bit-by-bit according to the following
 * 		MSB scheme:
 * 		(0) - Buffer too small
 * 		(1) - Unknown header
 * 		(2) -
 * 		(3) -
 * 		(4) -
 * 		(5) -
 * 		(6) -
 * 		(7) -
 * @encoders[6]
 * 	2 encoder values per motor, for a max of 3 motors in the system.
 * @accelerometer
 * 	One accelerometer chip sends 4 16-bit values as follows:
 * 		[0] - X?
 * 		[1] - Y?
 * 		[2] - Z?
 * 		[3] - ???
 * @sensors
 *	Agnostically-configurable 8-bit sensor data.
 */
struct message_from_vehicle
{
	uint8_t header;
	uint8_t error;
	uint8_t encoders[NUM_TOTAL_ENCODER_VALUES];
	uint16_t accelerometer[NUM_TOTAL_ACCEL_AXIS];
	uint8_t sensors[NUM_TOTAL_DIGITAL_SENSORS];
}msg_from_vehicle;

/*
 * @header
 * 	Contains configuration options according to the following
 * 		MSB scheme:
 * 		(0) - Config
 * 		(1) - Command
 * 		(2) -
 * 		(3) -
 * 		(4) -
 * 		(5) -
 * 		(6) -
 * 		(7) -
 * @direction
 * 	10 options for directionality control as determined by
 *   drivetrain.h
 * @throttle
 * 	0 to 100%. Amount of total system motor voltage applied to vehicle
 * 		motors.
 * @actuation_time
 *	Amount of time that motors will actuate per message. Prevents
 *		endlessly-running motors in case of loss-of-signal or packet
 *		losses.
 */
struct message_to_vehicle
{
	uint8_t header;
	direction_t direction;
	uint8_t throttle;
	uint8_t actuation_time;
}msg_to_vehicle;

/*
 * @header
 * 	Contains configuration options according to the following
 * 		MSB scheme:
 * 		(0) -
 * 		(1) -
 * 		(2) -
 * 		(3) -
 * 		(4) -
 * 		(5) -
 * 		(6) -
 * 		(7) -
 *  @num_modules
 *   The number of modules being actively sensed from.
 *  @drive_type
 *   Drivetrain type to be used.
 *  @pod_ids[8]
 *   Array of pods being used and what is being specifically
 *   referenced from base station side.
 */
struct message_vehicle_config
{
	uint8_t header;
	uint8_t num_modules;
	uint8_t drive_type;
	uint8_t pod_ids[NUM_MAX_PODS];
}msg_vehicle_config;


/**
 * Called on UART RX callback. Determines which message type is being received.
 */
uint8_t vehicle_message_receive(char message[]);
uint8_t vehicle_message_parse_config(char message[]);
uint8_t vehicle_message_parse_command(char message[]);

uint8_t assemble_message_from_vehicle(char message[], uint16_t buf_size);

/**
 * Changes the message to all NULL except for the error message
 * TODO: The error isn't actually saved in the struct, it's just transmitted in the message.
 */
void message_error_handler(char message[], char error_id);

#endif /* VEHICLE_MESSAGES_H_ */
