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

esp_err_t twomes_i2c_init(void) {
    //Setup the I2C:
    i2c_config_t i2c_config = {
        //master mode
        .mode = I2C_MODE_MASTER,
        //Pin 21 and 22
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        //Board has pull-ups
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        //Default I2C frequency
        .master.clk_speed = I2C_MASTER_FREQ,
    };

    //Configure I2C driver
    esp_err_t configErr = i2c_param_config(I2C_PORT, &i2c_config);
    if (configErr != ESP_OK) {
        return configErr;
    }
    configErr = i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, ESP_INTR_FLAG_LEVEL3);
    return configErr;
} //esp_err_t twomes_i2c_init;

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
esp_err_t twomes_i2c_write(uint8_t address, uint8_t *buffer, uint8_t len, bool sendStop) {
    //Setup i2c communication:
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);

    //Write the address and command to the i2c output buffer:
    i2c_master_write_byte(i2c_cmd, (address << 1) | I2C_MASTER_WRITE, true);
    for (uint8_t i = 0; i < len; i++) {
        i2c_master_write_byte(i2c_cmd, buffer[i], 1);
    }

    //End transmission (if stop is enabled):
    if (sendStop) i2c_master_stop(i2c_cmd);

    //Begin the command and return the esp_err_t code:
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, i2c_cmd, 1350);
    //Clear i2c resources:
    i2c_cmd_link_delete(i2c_cmd);

    return err;
}

/**
 * @brief Read data from the I2C port
 *
 * @param address i2c address to read from
 * @param buffer buffer for the data
 * @param len size of buffer (in bytes)
 * @return esp_err_t - error check
 */
esp_err_t twomes_i2c_read(uint8_t address, uint8_t *buffer, uint8_t len) {
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd);
    i2c_master_write_byte(i2c_cmd, (address << 1) | I2C_MASTER_READ, true);
    for (uint8_t i = 0; i < (len - 1); i++) {  //i<(buffer-1) to send NACK on last read
        i2c_master_read_byte(i2c_cmd, &buffer[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(i2c_cmd, &buffer[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd);
    return i2c_master_cmd_begin(I2C_PORT, i2c_cmd, 1350);

}

#endif //_TWOMES_I2C_H