#include "twomes_sensor_pairing.h"

const char *TAG = "ESPNOW_PAIRING";

/**
 * @brief read the P1 Gateway Mac and channel from NVS
 *
 * @param macAddress buffer to store the MAC address
 * @param len  length of the buffer (SHOULD ALWAYS BE 6!)
 * @param channel pointer to uint8_t to store channel
 *
 * @return error code
 */
int getGatewayData(uint8_t *macAddress, size_t len, uint8_t *channel) {
    esp_err_t err;
    nvs_handle gatewayHandle;

    //Open the NVS partition:
    err = nvs_open("gatewaySettings", NVS_READONLY, &gatewayHandle);
    if (err != ESP_OK) {
        return err; //return the error
    }

    //read the channel and MAC address:
    err = nvs_get_blob(gatewayHandle, "MAC", macAddress, &len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read Mac Address!");
        return err;
    }
    err = nvs_get_u8(gatewayHandle, "channel", channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read channel!");
        return err;
    }
    return ESP_OK; //Return OK on succes.
}

//Set this as onDataRecv calback when you want to pair. Remove the callback after pairing is done
//ESP automatically reboots when done.
void onDataReceive(const uint8_t *macAddress, const uint8_t *payload, int length) {
    //Read the MAC of the sender and store it in NVS:
    esp_err_t err;
    nvs_handle gatewayHandle;
    ESP_LOGD("Pairing", "Received a pairing message!");
    //Open the NVS partition:
    err = nvs_open("gatewaySettings", NVS_READWRITE, &gatewayHandle);
    if (err == ESP_OK) {
        ESP_ERROR_CHECK(nvs_set_blob(gatewayHandle, "MAC", macAddress, 6));
        ESP_ERROR_CHECK(nvs_set_u8(gatewayHandle, "channel", *payload));
        ESP_LOGD("ESP-Now", "stored MAC address: %02X:%02X:%02X:%02X:%02X:%02X, and Channel %u", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5], *payload);
        uint8_t blinkArgs[2] = { 3, LED_STATUS };
        xTaskCreatePinnedToCore(blink, "ChRecvBlink", 768, (void *)blinkArgs, 5, NULL, 1); //Blink LED to indicate receive of data
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGD("Pairing", "Stored Pairing data");
        esp_restart(); //restart the device after pairing
    }
}