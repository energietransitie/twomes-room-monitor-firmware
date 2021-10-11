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

uint16_t read_si7051() {

  uint8_t cmd = SI7051_CMD_MEASURE_TEMP_HOLDMASTER;
  twomes_i2c_write(SI7051_I2C_ADDRESS, &cmd, 1, I2C_SEND_STOP);
  //Short delay, according to conversion time from si705x datasheet (max 10.8ms)
  vTaskDelay(11 / portTICK_PERIOD_MS);


  //Set up to read:
  uint8_t i2c_data[2] = { 0,0 };
  ESP_ERROR_CHECK(twomes_i2c_read(SI7051_I2C_ADDRESS, i2c_data, sizeof(i2c_data)));

  //Store in single uint16_t
  uint16_t tempRaw;
  tempRaw = i2c_data[0] << 8;
  tempRaw |= i2c_data[1] & 0xff;
  ESP_LOGD("READTEMP", "Read I2C data %02X %02X, tempRaw=%4X", i2c_data[0], i2c_data[1], tempRaw);

  return tempRaw;
} //uint16_t read_si7051

#endif // #ifdef _TWOMES_ROOMSENSOR_H
