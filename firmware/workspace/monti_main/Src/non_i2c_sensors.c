/*
 * non_i2c_sensors.c
 *
 *  Created on: Nov 24, 2017
 *      Author: Gilbert Yap
 */

// Includes
#include "non_i2c_sensors.h"

// Gas sensor
int8_t Read_Gas_Sensor()
{
	int8_t gas_data;
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


