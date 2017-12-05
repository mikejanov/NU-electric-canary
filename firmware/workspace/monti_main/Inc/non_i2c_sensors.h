/*
 * non_i2c_sensors.h
 *
 *  Created on: Nov 24, 2017
 *      Author: Gilbert Yap
 */

#ifndef NON_I2C_SENSORS_H_
#define NON_I2C_SENSORS_H_

#include "stm32f3xx_hal.h"

uint8_t Read_Gas_Sensor();
uint8_t Read_Hall_Sensor();
void ultrasonic_detect(uint8_t *distance_1, uint8_t *distance_2);
void ultrasonic_check(uint8_t *ultrasonic_distances[]);

#endif /* NON_I2C_SENSORS_H_ */
