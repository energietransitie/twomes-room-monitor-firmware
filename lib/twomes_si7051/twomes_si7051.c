#include "twomes_si7051.h"

#define SI7051_CMD_MEASURE_TEMP_HOLDMASTER      0xE3
#define SI7051_CMD_MEASURE_TEMP_NOHOLDMASTER    0xF3
#define SI7051_CMD_RESET                        0xFE
#define SI7051_CMD_WRITE_USER_REG               0xE6
#define SI7051_CMD_READ_USER_REG                0xE7
#define SI7051_CMD_READ_ELEC_ID1                [0xFA,0x0F]
#define SI7051_CMD_READ_ELEC_ID2                [0xFC,0xC9]
#define SI7051_CMD_READ_FIRMWARE_VERSION        [0x84,0xB8]

//CRC defines
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0x00


uint16_t read_si7051() {

  uint8_t cmd = SI7051_CMD_MEASURE_TEMP_HOLDMASTER;
  twomes_i2c_write(SI7051_I2C_ADDRESS, &cmd, 1, I2C_SEND_STOP);
  //Short delay, according to conversion time from si705x datasheet (max 10.8ms)
  vTaskDelay(11 / portTICK_PERIOD_MS);


  //Set up to read:
  uint8_t i2c_data[3] = { 0,0,0 };
  ESP_ERROR_CHECK(twomes_i2c_read(SI7051_I2C_ADDRESS, i2c_data, sizeof(i2c_data)));

  //Check CRC:
  ESP_LOGD("SI7051", "Received CRC: 0x%02X, calculated CRC: 0x%02X", i2c_data[2], si7051_crc8(&i2c_data[0], 2));

  //Store in single uint16_t
  uint16_t tempRaw;
  tempRaw = i2c_data[0] << 8;
  tempRaw |= i2c_data[1] & 0xff;
  ESP_LOGD("READTEMP", "Read I2C data %02X %02X, tempRaw=%4X", i2c_data[0], i2c_data[1], tempRaw);

  return tempRaw;
} //uint16_t read_si7051

double si7051_raw_to_celsius(uint16_t raw) {
  return (((175.7200) * raw) / 65536.0f) - 46.85f;
}

/**
 * CRC8 code taken from Sensirion SCD41 Datasheet: https://nl.mouser.com/datasheet/2/682/Sensirion_CO2_Sensors_SCD4x_Datasheet-2321195.pdf
 * @brief calculate the CRC for an SCD41 I2C message
 *
 * @param data pointer to the received i2c data
 * @param count size of the buffer
 *
 * @return calculated CRC8
 */
uint8_t si7051_crc8(const uint8_t *data, uint16_t count) {

  uint16_t current_byte;
  uint8_t crc = CRC8_INIT;
  uint8_t crc_bit;

  for (current_byte = 0; current_byte < count; ++current_byte) {
    crc ^= (data[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }

  return crc;
}