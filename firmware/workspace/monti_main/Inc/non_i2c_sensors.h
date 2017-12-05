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
uint8_t ultrasonic_calculate();
uint8_t ultrasonic_check();

#endif /* NON_I2C_SENSORS_H_ */
