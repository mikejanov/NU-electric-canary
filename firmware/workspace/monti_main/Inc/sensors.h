/*
 * sensors.h
 *
 *  Created on: Dec 4, 2017
 *      Author: michaeljanov
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "stdint.h"

/**
 * Sensor Definitions
 */

// Pod IDs
typedef enum pod_ids
{
	POD_ID_BME280 		= 0x01,
	POD_ID_LIS3DH 		= 0x02,
	POD_ID_HALL 		= 0x03,
	POD_ID_GAS 			= 0x04,
	POD_ID_ULTRASONIC 	= 0x05
}pod_ids_t;

// Pod Sizes
// Values in Bytes
typedef enum pod_sizes
{
	POD_SIZE_BME280 	= 5,
	POD_SIZE_LIS3DH 	= 6,
	POD_SIZE_HALL	 	= 1,
	POD_SIZE_GAS 		= 1,
	POD_SIZE_ULTRASONIC	= 1
}pod_sizes_t;

/**
 * System Definitions
 */

#define NUM_MAX_PODS				8
#define NUM_TOTAL_DIGITAL_SENSORS	16

#define NUM_TOTAL_ACCEL_AXIS		4

// Pod Register Tables
uint8_t pod_sizes[NUM_MAX_PODS];  // Holds the (Byte) size of each pod
uint8_t pod_starts[NUM_MAX_PODS]; // Holds the corresponding pod-to-sensor[] message start index

/**
 * Functions
 */
void update_sensors(uint8_t *pod_ids, uint8_t *sensor_list,
					uint8_t pod_id, uint8_t *data);

uint8_t get_pod_size(pod_ids_t pod_id);

void config_map_pods(uint8_t *pod_ids);

#endif /* SENSORS_H_ */
