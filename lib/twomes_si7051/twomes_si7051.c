#include "twomes_si7051.h"

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
