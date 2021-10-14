/**
 * Library for reading the Si7051 temperature sensor on the Twomes room sensor node
 *
 * Author: Sjors
 * Date: July 2021
 */

#ifndef _TWOMES_ROOMSENSOR_H
#define _TWOMES_ROOMSENSOR_H

#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "twomes_i2c.h"

 /**
  * TODO: Implement CRC
  */

  //SI7051 Defines
#define SI7051_I2C_ADDRESS 0x40

#define SI7051_CMD_MEASURE_TEMP_HOLDMASTER      0xE3
#define SI7051_CMD_MEASURE_TEMP_NOHOLDMASTER    0xF3
#define SI7051_CMD_RESET                        0xFE
#define SI7051_CMD_WRITE_USER_REG               0xE6
#define SI7051_CMD_READ_USER_REG                0xE7
#define SI7051_CMD_READ_ELEC_ID1                [0xFA,0x0F]
#define SI7051_CMD_READ_ELEC_ID2                [0xFC,0xC9]
#define SI7051_CMD_READ_FIRMWARE_VERSION        [0x84,0xB8]

/**
 * @brief read the temperature from the Si7051 sensor
 *
 * @return raw temperature value
 */
uint16_t read_si7051();

#endif // #ifdef _TWOMES_ROOMSENSOR_H
