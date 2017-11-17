/*
 * vehicle_messages.h
 *
 *  Created on: Nov 10, 2017
 *      Author: michaeljanov
 */

#ifndef VEHICLE_MESSAGES_H_
#define VEHICLE_MESSAGES_H_

#include "drivetrain.h"
#include "string.h"

#define NUM_TOTAL_ENCODER_VALUES	6
#define NUM_TOTAL_ACCEL_AXIS		4
#define NUM_TOTAL_DIGITAL_SENSORS	16

#define MSG_TX_BUFFER_SIZE sizeof(msg_from_vehicle)
#define MSG_RX_BUFFER_SIZE sizeof(msg_vehicle_config)

#define ERROR_MSG_SIZE 0x01

void message_error_handler(char message[], char error_id);

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
 * @error
 * 	Error messages are bit-by-bit according to the following
 * 		MSB scheme:
 * 		(0) -
 * 		(1) -
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

uint8_t assemble_message_from_vehicle(char message[], uint16_t buf_size);

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

/**
 * Called on UART RX callback with a message to change the current
 *  vehicle configuration. Always uses the msg_to_vehicle register
 *  table.
 */
uint8_t vehicle_message_initialize(void);

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
	uint8_t pod_ids[8];
}msg_vehicle_config;

#endif /* VEHICLE_MESSAGES_H_ */
