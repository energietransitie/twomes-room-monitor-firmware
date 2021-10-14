/**
 * Library for reading the SCD41 CO2 sensor on the Twomes room sensor node
 *
 * Author: Sjors
 * Date: July 2021
 */

#ifndef _TWOMES_CO2SENSOR_H
#define _TWOMES_CO2SENSOR_H

#include "esp_sleep.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "twomes_i2c.h"

 /**
  * TODO: Everything...
  *
  */

#define SCD41_INIT_DELAY        1000 // milliseconds
#define SCD41_WAIT_MILLISECOND  2 // milliseconds
#define SCD41_SINGLE_SHOT_DELAY 5000 // ms

#define SCD41_ADDR              0x62

#define SCD41_CMD_SERIALNUM     0x36, 0x82 //0x3682
#define SCD41_CMD_SET_ASC_EN    0x24, 0x16 //0x2416
#define SCD41_CMD_GET_ASC_EN    0x23, 0x13 //0x2313
#define SCD41_CMD_READMEASURE   0xec, 0x05 //0xec05
#define SCD41_CMD_SINGLESHOT    0x21, 0x9d //0x219d
#define SCD41_CMD_LOWPOWER_PERIODIC 0x21, 0xac //0x21b1
#define SCD41_SELFTEST          0x36, 0x39 //0x3639

#define SCD41_CMD_GET_TEMP_OFF  0x23, 0x18  //0x2318

  //CRC defines
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

/**
 * CRC8 code taken from Sensirion SCD41 Datasheet: https://nl.mouser.com/datasheet/2/682/Sensirion_CO2_Sensors_SCD4x_Datasheet-2321195.pdf
 * @brief calculate the CRC for an SCD41 I2C message
 *
 * @param data pointer to the received i2c data
 * @param count size of the buffer
 *
 * @return calculated CRC8
 */
uint8_t scd41_crc8(const uint8_t *data, uint16_t count);

/**
 * @brief read the serial number of the CO2 sensor
 *
 * @param address i2c address of the device
 *
 * @return the serial number
 */
uint64_t co2_get_serial(uint8_t address);

/**
 * @brief disable the SCD41 ASC (Automatic Self Calibration)
 * Device performs a read immeadiately after to check for disable success
 * @param address i2c address of the device
 *
 * @return value of ASC after write (0 == success)
 */
uint8_t co2_disable_asc(uint8_t address);


void co2_read(uint8_t address, uint16_t *buffer);

void co2_read_periodic(uint8_t address, uint8_t *buffer);

void co2_perform_selftest(uint8_t address, uint8_t *buffer);

void co2_init(uint8_t address);

#endif //_TWOMES_CO2SENSOR_H