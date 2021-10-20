/**
 * Library for pairing a Twomes sensor with a Twomes Gateway device using ESP-Now
 *
 * Author: Sjors
 * Date: July 2021
 */

#ifndef _TWOMES_SENSOR_PAIRING_H
#define _TWOMES_SENSOR_PAIRING_H

#include <stdlib.h>
#include <esp_log.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_now.h>
#include <sensor_IO.h>

#define ESPNOW_PAIRING_CHANNEL 1
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG


 /**
  * @brief read the P1 Gateway Mac and channel from NVS
  *
  * @param macAddress buffer to store the MAC address
  * @param len  length of the buffer (SHOULD ALWAYS BE 6!)
  * @param channel pointer to uint8_t to store channel
  *
  * @return error code
  */
int getGatewayData(uint8_t *macAddress, size_t len, uint8_t *channel);

//Set this as onDataRecv calback when you want to pair. Remove the callback after pairing is done
//ESP automatically reboots when done.
void onDataReceive(const uint8_t *macAddress, const uint8_t *payload, int length);



#endif //_TWOMES_SENSOR_PAIRING_H