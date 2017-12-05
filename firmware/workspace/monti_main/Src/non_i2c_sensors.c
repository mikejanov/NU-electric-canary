/*
 * non_i2c_sensors.c
 *
 *  Created on: Nov 24, 2017
 *      Author: Gilbert Yap
 */

// Includes
#include "non_i2c_sensors.h"

/*
 ** Gas Sensor
 */
uint8_t Read_Gas_Sensor()
{
	// Read the analog pin that gas sensor is on
	uint8_t gas_data = HAL_GPIO_ReadPin(AIN_0_GPIO_Port, AIN_0_Pin);
	return gas_data;
}

/*
 ** Hall Effect
 */
uint8_t Read_Hall_Sensor()
{
	// Read the digital pin that the hall effect sensor is on
	uint8_t hall_state = HAL_GPIO_ReadPin(DIN_0_GPIO_Port, DIN_0_Pin);
	return hall_state;
}


/*
 ** Ultrasonic Sensors
 */
// todo: better error checking
void ultrasonic_calculate(uint8_t *distance_1, uint8_t *distance_2) {
	// Max distance detectable is 4m, min is 2cm
	// Max time is 20ms (23200 us), min time is 0.12 ms?
	uint8_t timer_1 = 0;
	uint8_t timeout = 20; // ms necessary to check 1ft distance (2ft in calculation)

	uint8_t duration_1 = 0;
	uint8_t timer_2 = 0;
	uint8_t timer_3 = 0;
	uint8_t duration_2 = 0;
	uint8_t timer_4= 0;

	// -----Sensor 1 detect-----
	// Send pulse for 10uS
	HAL_GPIO_WritePin(DIN_1_GPIO_Port, DIN_1_Pin, 1);
	HAL_Delay(0.01); // 10us
	HAL_GPIO_WritePin(DIN_1_GPIO_Port, DIN_1_Pin, 0); // need to set pins

	// Wait in increments of 1us while there is nothing detected on both sensors
	while(HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) != 1 && timer_1 < timeout) {
		HAL_Delay(0.001);
		timer_1+=0.001;
	}

	// Record how long the sensors receive 1s
	while(HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) == 1) {
		HAL_Delay(0.001);
		timer_2 +=0.001;
	}

	// -----Sensor 2 Detect-----
	HAL_GPIO_WritePin(DIN_2_GPIO_Port, DIN_2_Pin, 1);
	HAL_Delay(0.01); // 10us
	HAL_GPIO_WritePin(DIN_2_GPIO_Port, DIN_2_Pin, 0); // need to set pins

	// Wait in increments of 1us while there is nothing detected on both sensors
	while(HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) != 1 && timer_3 < timeout) {
		HAL_Delay(0.001);
		timer_3 += 0.001;
	}

	// Record how long the sensors receive 1s
	while(HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) == 1) {
		HAL_Delay(0.001);
		timer_4 +=0.001;
	}

	duration_1 = timer_2-timer_1;
	duration_2 = timer_4-timer_3;
	// Todo: check these pointers
	distance_1 = (uint8_t *) (duration_1*343/2); // D = speed of sound/2 * duration
	distance_2 = (uint8_t *) (duration_2*343/2);
}

void ultrasonic_check(uint8_t *ultrasonic_distances[]) {
	uint8_t distance_1;
	uint8_t distance_2;
	uint8_t threshold_distance = 30.48; // cm

	// Get distances
	ultrasonic_calculate(&distance_1, &distance_2);
	// Send data back
	if(distance_1 <= threshold_distance) {
		ultrasonic_distances[0] = (uint8_t *) 1;
	} else {
		ultrasonic_distances[0] = (uint8_t *) 0;
	}

	if(distance_2 <= threshold_distance) {
		ultrasonic_distances[1] = (uint8_t *) 1;
	} else {
		ultrasonic_distances[1] = (uint8_t *) 0;
	}
}
