/*
 * non_i2c_sensors.c
 *
 *  Created on: Nov 24, 2017
 *      Author: Gilbert Yap
 */

// Includes
#include "non_i2c_sensors.h"

// Gas sensor
uint8_t Read_Gas_Sensor()
{
	uint8_t gas_data;
	// Read the analog pin that gas sensor is on
	gas_data = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	return gas_data;
}

// Hall effect sensor
uint8_t Read_Hall()
{
	uint8_t hall_state;

	// Read the digital pin that the hall effect sensor is on
	hall_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);

	return hall_state;
}


// Ultrasonic Sensor

uint8_t ultrasonic_detect() {
	uint8_t ultrasonic_distance[2];

	// Send pulse for 10uS
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); // need to set pins
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1); // need to set pins

	HAL_Delay(0.01); // 10us

	// Turn off trigger
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // need to set pins
	// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0); // need to set pins

	// Max distance detectable is 4m, min is 2cm
	// Max time is 20ms, min time is 116us?
	// Time is is ~ time it takes to detect 4m
	uint8_t time = 0; // in uS
	while() {

		HAL_Delay(0.001);
		time += 0.001;
	}

	ultrasonic_distance = time*340/2; // in meters

	ultrasonic_distance[0] = ;
	ultrasonic_distance[1] = ;

	return detect_set;
}

