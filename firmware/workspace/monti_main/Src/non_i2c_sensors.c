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
uint8_t ultrasonic_calculate() {
	// Max distance detectable is 4m, min is 2cm
	// Max time is 23ms (23200 us), min time is 0.12 ms?
	uint8_t timer_1 = 0;
	uint8_t timeout = 20; // ms necessary to check 1ft distance (2ft in calculation)
	uint8_t timer_2 = 0;
	uint8_t duration_1 = 0;

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

	// Did the timeout test get passed?
	if(timer_1 < timeout) {
		// Record how long the sensors receive 1s
		while(HAL_GPIO_ReadPin(DIN_1_GPIO_Port, DIN_1_Pin) == 1) {
			HAL_Delay(0.001);
			timer_2 +=0.001;

			duration_1 = timer_2-timer_1;
			return (duration_1*343/2); // D = speed of sound/2 * duration
		}
	} else {
		return 1;
	}
}

uint8_t ultrasonic_check() {
	uint8_t threshold_distance = 30.48; // cm
	uint8_t ultrasonic_distance = 0;

	// Get distances
	ultrasonic_distance = ultrasonic_calculate();

	// Send data back
	if(ultrasonic_distance <= threshold_distance) {
		return 1;
	} else {
		return 0;
	}
}
