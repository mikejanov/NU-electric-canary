/**
  ******************************************************************************
  * File Name          : bme280_sensor.h
  * Description        : Bosch BME280 header file for MoNTI
  ******************************************************************************
  **/
#ifndef BME280_MONTI_H_
#define BME280_MONTI_H_

#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"

// Wait n milliseconds
void user_delay_ms(uint32_t period);

// i2c write instruction for the bme280 lib
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr,
                      uint8_t *reg_data, uint16_t len);

// i2c read instruction for the bme280 lib
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr,
                     uint8_t *reg_data, uint16_t len);

// Initialize the BME280 for normal mode operation
int8_t sensor_init(struct bme280_dev *dev);

// Set the oversampling for temperature and pressure, and data mode
int8_t set_normal_mode(uint8_t dev_id);

// Get temperature, humidity, and pressure data from the bme280
int8_t get_bme280_all_data(struct bme280_dev *dev, struct bme280_data *comp_data);

int8_t get_chip_id(uint8_t dev_id, uint8_t *reg_data);

#endif /* BME280_MONTI_H_ */
