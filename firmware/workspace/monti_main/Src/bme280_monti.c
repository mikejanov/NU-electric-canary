/**
  ******************************************************************************
  * File Name          : bme280_sensor.c
  * Description        : Bosch BME280 source file for MoNTI
  ******************************************************************************
  **/
#include "bme280_monti.h"

int8_t sensor_init(struct bme280_dev *dev)
{
	int8_t rslt;

	dev->dev_id = BME280_I2C_ADDR_SEC; // 0x77
	dev->intf = BME280_I2C_INTF;
	dev->read = user_i2c_read;
	dev->write = user_i2c_write;
	dev->delay_ms = user_delay_ms;

	rslt = bme280_init(dev);

	uint8_t settings_sel;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

	return rslt;
}

void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rtrn_rslt = 1;

    // Transmit slave id, reg_data needs to be register addr and register data (16 bits)
    HAL_I2C_Master_Transmit_IT(&hi2c1, dev_id<<1, &reg_addr, len);
    // HAL_Delay(100);

    return rtrn_rslt;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rtrn_rslt = 1;

    // Write the register addr to the slave id, should be 8bits
    HAL_I2C_Master_Transmit_IT(&hi2c1, dev_id<<1, &reg_addr, sizeof(reg_addr));
    // HAL_Delay(100);

    // Read a number of bytes that is expected from
    HAL_I2C_Master_Receive_IT(&hi2c1, dev_id<<1, reg_data, len);
    // HAL_Delay(100);
    /*
    rslt = 1;
    if (rslt == HAL_ERROR || rslt == HAL_TIMEOUT) {
    	rtrn_rslt = 1;
    } else {
    	return 0;
    }
    */
	return rtrn_rslt;
}

int8_t set_normal_mode(uint8_t dev_id)
{
	// 0x3B sets the osrs_t as 001, osrs_p as 110, and mode as 11 (normal mode)
	// ^^ "Indoor Navigation Mode"
	uint8_t reg_data[] = {BME280_CTRL_MEAS_ADDR, 0x3B};
	HAL_StatusTypeDef rslt;
	int8_t rtrn_rslt = 1;

	HAL_I2C_Master_Transmit_IT(&hi2c1, dev_id<<1, reg_data, sizeof(reg_data));
	HAL_Delay(100);

	return rtrn_rslt;
}

int8_t get_bme280_all_data(struct bme280_dev *dev, struct bme280_data *comp_data)
{
	int8_t rslt;
	rslt = bme280_get_sensor_data(BME280_ALL, comp_data, dev);
	// dev->delay_ms(250);
	return rslt;
}
