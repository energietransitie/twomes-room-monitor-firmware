/**
 * Library for reading the Si7051 temperature sensor on the Twomes room sensor node
 *
 * Author: Sjors
 * Date: July 2021
 */

#ifndef _TWOMES_ROOMSENSOR_H
#define _TWOMES_ROOMSENSOR_H

 //Debug mode:
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "twomes_i2c.h"

 /**
  * TODO: Implement CRC
  */

  //SI7051 Defines
#define SI7051_I2C_ADDRESS 0x40



/**
 * @brief read the temperature from the Si7051 sensor
 *
 * @return raw temperature value
 */
uint16_t read_si7051();

/**
 * @brief read the temperature from the Si7051 sensor
 * @param attempts amount of retries
 * @return raw temperature value
 */
uint16_t read_si7051_with_retries(uint8_t attempts);

/**
 * @brief convert a raw temperature measurement to a value in degrees Celsius
 *
 * @param raw raw temperature value
 *
 * @return the temperature in degrees Celsius
 */
double si7051_raw_to_celsius(uint16_t raw);

/**
 * CRC8 code taken from Sensirion SCD41 Datasheet: https://nl.mouser.com/datasheet/2/682/Sensirion_CO2_Sensors_SCD4x_Datasheet-2321195.pdf
 * @brief calculate the CRC for an SCD41 I2C message
 *
 * @param data pointer to the received i2c data
 * @param count size of the buffer
 *
 * @return calculated CRC8
 */
uint8_t si7051_crc8(const uint8_t *data, uint16_t count);

#endif // #ifdef _TWOMES_ROOMSENSOR_H
