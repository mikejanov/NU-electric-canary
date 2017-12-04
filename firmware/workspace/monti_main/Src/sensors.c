/*
 * sensors.c
 *
 *  Created on: Dec 4, 2017
 *      Author: michaeljanov
 */

#include "sensors.h"

void update_sensors(uint8_t *pod_ids, uint8_t *sensor_list,
					uint8_t pod_id, uint8_t *data)
{
	for(int ii_pod = 0; ii_pod < NUM_MAX_PODS; ii_pod++)
	{
		// Search through the pod list to find the index that contains the sensor
		// you want...
		if(pod_ids[ii_pod] == pod_id)
		{
			for(int ii_size = 0; ii_size < pod_sizes[ii_pod]; ii_size++)
			{
				sensor_list[pod_starts[ii_pod] + ii_size] = data[ii_size];
			}
		}
	}
}

uint8_t get_pod_size(pod_ids_t pod_id)
{
	switch (pod_id)
	{
	case POD_ID_BME280:
		return POD_SIZE_BME280;
	case POD_ID_LIS3DH:
		return POD_SIZE_LIS3DH;
	case POD_ID_HALL:
		return POD_SIZE_HALL;
	case POD_ID_GAS:
		return POD_SIZE_GAS;
	case POD_ID_ULTRASONIC:
		return POD_SIZE_ULTRASONIC;
	default:
		return 0x00;
	}
}

void config_map_pods(uint8_t *pod_ids)
{
	// TODO: Ultrasonic kludgey bugfix
	uint8_t pod_index = 1; // Starting pod index must be 1. Slot 0 is Ultrasonic
	for(int ii_pod = 0; ii_pod < NUM_MAX_PODS; ii_pod++)
	{
		// Map Size
		pod_sizes[ii_pod] = get_pod_size(pod_ids[ii_pod]);

		// Map Starting index
		pod_starts[ii_pod] = pod_index;
		pod_index = pod_index + pod_sizes[ii_pod]; // Prepare index for next pod
		// TODO: Throw error if the pod_index tries to place after the max size
	}
}
