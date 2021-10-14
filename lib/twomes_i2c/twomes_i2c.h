#ifndef _TWOMES_I2C_H
#define _TWOMES_I2C_H

#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"

//I2C defines:
#define I2C_SDA         GPIO_NUM_21
#define I2C_SCL         GPIO_NUM_22 
#define I2C_MASTER_FREQ 40000L     
#define I2C_PORT        I2C_NUM_0

#define I2C_SEND_STOP    true
#define I2C_SEND_NO_STOP false

/**
 * @brief initialise the i2c bus with the pre-defined settings
 *
 * @return error code (0/ESP_OK on success)
 */
esp_err_t twomes_i2c_init(void);

/**
 * @brief Send data on the I2C port
 *
 * @param address i2c address to write to
 * @param buffer data to send
 * @param len size of buffer (in bytes)
 * @param sendStop enable/disable stop command (required for SCD41 read)
 *
 * @return esp_err_t - error check
 */
esp_err_t twomes_i2c_write(uint8_t address, uint8_t *buffer, uint8_t len, bool sendStop);

/**
 * @brief Read data from the I2C port
 *
 * @param address i2c address to read from
 * @param buffer buffer for the data
 * @param len size of buffer (in bytes)
 * @return esp_err_t - error check
 */
esp_err_t twomes_i2c_read(uint8_t address, uint8_t *buffer, uint8_t len);

#endif //_TWOMES_I2C_H