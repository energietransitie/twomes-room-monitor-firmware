/**
 * Firmware for the Twomes Room-sensor
 * Currently supports: Si7051, scd41
 * TODO:
 *      - Add time-calibration features
 *      - test CO2 and Si7051 ESP-Now transmission
 *
 *  Author: Sjors
 *  Date: August 2021
 */

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include <stdlib.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/i2c.h"
#include "esp_sleep.h"
#include "esp_log.h"

 //Run in Debug mode:
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "twomes_i2c.h"
#include "sensor_IO.h"
#include "twomes_sensor_pairing.h"
#include "twomes_si7051.h"
#include "twomes_scd41.h"


#define uS_TO_S_FACTOR 1000000ULL                       /* Conversion factor for micro seconds to seconds */

#define MAX_TEMP_SAMPLES    120                         //Buffer size for data (data loss after it fills up and fails transmission)
#define MAX_CO2_SAMPLES     40
#define ESPNOW_SEND_MINIMUM 10                          //Minimum amount to start sending 
#define RETRY_INTERVAL 5                                /* Amount of measurements before a new ESP-Now attempt after a Fail To Send */
#define PAIRING_TIMEOUT_uS (20*uS_TO_S_FACTOR)          //timeout for pairing

#define TIME_TO_SLEEP 300                                /* Time between measurements in seconds (300 seconds = 5 minutes)*/
#define INTERVAL_US (TIME_TO_SLEEP * uS_TO_S_FACTOR)    /* desired interval between measurements in us */


typedef enum systemStates { //Different to boilersensor, no need for conversion sleep
    UNKNOWN,
    MEASURE_TEMP,
    MEASURE_CO2,
    SEND_ESPNOW
} systemStates;
//RTC Data, variables that remain during deep-sleep:

RTC_DATA_ATTR systemStates systemState = UNKNOWN;
RTC_DATA_ATTR uint8_t RoomTempCount = 0;       //variable in RTC memory where number of current measurement is stored
RTC_DATA_ATTR uint8_t scd41Count = 0;
RTC_DATA_ATTR uint64_t previousTime = 0;            //REVIEW
RTC_DATA_ATTR uint64_t time_correction = 200000;    //REVIEW, initial time correction
RTC_DATA_ATTR uint16_t burstNumber = 0;             //Store the amount of databursts that have been done with ESP-Now, for syncing with gateway

RTC_DATA_ATTR bool PowerUpBoot = false;

//Twomes Measurement type enum:
typedef enum ESPNOWdataTypes {
    BOILERTEMP,
    ROOMTEMP,
    CO2,
} ESPNOWdataTypes;

//NEW int16_t version to increase amount of measurements in single burst
typedef struct Roomtemp_Message {
    uint8_t measurementType;                            //Type of measurements
    uint8_t numberofMeasurements;                       //number of measurements in burst
    uint16_t index;                                     //Number identifying the message, only increments on receiving an ACK from Gateway. Could be uint8_t since overflows are ignored?
    uint16_t intervalTime;                              //Interval between measurements, for timestamping in gateway
    int16_t roomTemps[MAX_TEMP_SAMPLES];                //measurements of the Si7051
} Roomtemp_Message;
typedef struct CO2_Message {
    uint8_t measurementType;                            //Type of measurements
    uint8_t numberofMeasurements;                       //number of measurements in burst
    uint16_t index;                                     //Number identifying the message, only increments on receiving an ACK from Gateway. Could be uint8_t since overflows are ignored?
    uint16_t intervalTime;                              //Interval between measurements, for timestamping in gateway
    int16_t co2ppm[MAX_CO2_SAMPLES];                    //measurements of the CO2 concentration
    int16_t co2temp[MAX_CO2_SAMPLES];                   //measurements of the temperature by SCD41
    int16_t co2humid[MAX_CO2_SAMPLES];                  //measurements of the humidity
} CO2_Message;

RTC_DATA_ATTR uint16_t roomTemperatures[MAX_TEMP_SAMPLES];  //Si7051 temp
RTC_DATA_ATTR uint16_t scd41ppm[MAX_CO2_SAMPLES];
RTC_DATA_ATTR uint16_t scd41temp[MAX_CO2_SAMPLES];
RTC_DATA_ATTR uint16_t scd41hum[MAX_CO2_SAMPLES];

bool callbackFinished; //to keep the processor awake while waiting for the ESP-Now onDataSent Callback

//Functions:
esp_err_t send_esp_now_roomtemp(void);
esp_err_t send_esp_now_co2(void);
esp_err_t pair_sensor(void);
void onDataSent(const uint8_t *, esp_now_send_status_t);

void app_main() {
    twomes_init_gpio();


    //disable Supercap on PMOS, active low:
    gpio_set_level(PIN_SUPERCAP_ENABLE, 1);

    //If log level is set to 4(D) or 5(V):
    ESP_LOGD("DEBUG", "DEBUGGING MODE IS ENABLED\n");

    //Check if P2 is held down to enter pairing mode:
    if (!gpio_get_level(BUTTON_P2)) {
        int err = pair_sensor();

        //Check for pairing success and indicate to the user, then reboot
        if (err != ESP_OK) {
            //Error LED on failure
            ESP_LOGD("PAIRING", "Pairing returned with error code %i", err);
            uint8_t args[2] = { 3,LED_ERROR };
            xTaskCreatePinnedToCore(blink, "pairing_fail", 768, (void *)args, 5, NULL, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }
    }

    esp_err_t i2cErr = twomes_i2c_init();
    ESP_LOGD("DEBUG", "init twomes i2c with code: %s", esp_err_to_name(i2cErr));

    //give extra wakeup time to SCD41:
    co2_init(SCD41_ADDR);

    gpio_set_level(PIN_SUPERCAP_ENABLE, 0);
    gpio_hold_en(PIN_SUPERCAP_ENABLE);
    //co2_perform_selftest(SCD41_ADDR, NULL);
    uint16_t scd41Measurement[3] = { 0,0,0 };
    co2_read(SCD41_ADDR, scd41Measurement);
    //Store each measurement in it's array:
    scd41ppm[scd41Count] = scd41Measurement[0];
    scd41temp[scd41Count] = scd41Measurement[1];
    scd41hum[scd41Count] = scd41Measurement[2];
    scd41Count++;
    ESP_LOGD("CO2", "Read SCD41: co2: %u, T: %3.2f, RH: %3.1f", scd41Measurement[0], scd41_temp_raw_to_celsius(scd41Measurement[1]), scd41_rh_raw_to_percent(scd41Measurement[2]));

    gpio_hold_dis(PIN_SUPERCAP_ENABLE);
    gpio_set_level(PIN_SUPERCAP_ENABLE, 1);

    //read the temperature sensor:
    uint16_t temp = read_si7051();
    //Store the temperature in the buffer and increase the index:
    roomTemperatures[RoomTempCount] = temp;
    RoomTempCount++;

    //print the temperature to monitor for debugging:
    ESP_LOGD("TEMPERATURE", "Read temperature %3.4f", si7051_raw_to_celsius(temp));
    ESP_LOGD(" DATA", " Gathered %u measurements so far", RoomTempCount);


    //If RoomTempCount is larger than the send minimum and the interval
    if (RoomTempCount >= ESPNOW_SEND_MINIMUM && ((RoomTempCount % RETRY_INTERVAL) == 0)) {

        ESP_LOGI("ESP-Now", "esp-now transmission returned code %s", esp_err_to_name(send_esp_now_roomtemp()));
        callbackFinished = false;
        while (!callbackFinished) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        ESP_LOGD("ESP-Now", "Transmission ended, going back to sleep");
    }
    //After transmission, check RoomTempCount value. If transmission failed and amount of measurements >= MAX_SAMPLES (theoretically this could just be RoomTempCount == MAX_SAMPLES, but just to be safe...)
    if (RoomTempCount >= MAX_TEMP_SAMPLES) {
        //Dump old temps to debug:
        for (uint8_t i = 0; i < RoomTempCount; i++) {
            ESP_LOGD("TEMPDUMP", "Temp[%u]: %u", i, roomTemperatures[i]);
        }
        //Move the samples so that there are "RETRY_INTERVAL" amount of available memory for samples. Overwriting the oldest "RETRY_INTERVAL" amount of measurements
        ESP_LOGD("Memory", "Moving memory samples! was %d", RoomTempCount);
        memmove(roomTemperatures, &roomTemperatures[RoomTempCount - MAX_TEMP_SAMPLES + RETRY_INTERVAL], (MAX_TEMP_SAMPLES - RETRY_INTERVAL) * sizeof(roomTemperatures[0]));
        RoomTempCount = MAX_TEMP_SAMPLES - RETRY_INTERVAL; //Set RoomTempCount to the adjusted amount
        ESP_LOGD("Memory", "Now %d", RoomTempCount);
    }

    //If CO2 count is larger than the send minimum and the interval
    if (scd41Count >= ESPNOW_SEND_MINIMUM && ((scd41Count % RETRY_INTERVAL) == 0)) {

        ESP_LOGI("ESP-Now", "esp-now transmission returned code %s", esp_err_to_name(send_esp_now_co2()));
        callbackFinished = false;
        while (!callbackFinished) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        ESP_LOGD("ESP-Now", "Transmission ended, going back to sleep");
    }
    //After transmission, check RoomTempCount value. If transmission failed and amount of measurements >= MAX_SAMPLES (theoretically this could just be RoomTempCount == MAX_SAMPLES, but just to be safe...)
    if (scd41Count >= MAX_CO2_SAMPLES) {
        //Move the samples so that there are "RETRY_INTERVAL" amount of available memory for samples. Overwriting the oldest "RETRY_INTERVAL" amount of measurements
        memmove(scd41ppm, &scd41ppm[scd41Count - MAX_CO2_SAMPLES + RETRY_INTERVAL], (MAX_CO2_SAMPLES - RETRY_INTERVAL) * sizeof(scd41ppm[0]));
        memmove(scd41temp, &scd41temp[scd41Count - MAX_CO2_SAMPLES + RETRY_INTERVAL], (MAX_CO2_SAMPLES - RETRY_INTERVAL) * sizeof(scd41temp[0]));
        memmove(scd41hum, &scd41hum[scd41Count - MAX_CO2_SAMPLES + RETRY_INTERVAL], (MAX_CO2_SAMPLES - RETRY_INTERVAL) * sizeof(scd41hum[0]));
        scd41Count = MAX_CO2_SAMPLES - RETRY_INTERVAL; //Set RoomTempCount to the adjusted amount
    }

    //Calibrate time:
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t time_us = (int64_t)tv.tv_sec * 1000000L + (int64_t)tv.tv_usec;
    int32_t time_delta = time_us - previousTime;
    previousTime = time_us;
    time_correction += (time_delta - INTERVAL_US);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR - time_correction);


    ESP_LOGD("MAIN", "Going into deepsleep for %llu s", INTERVAL_US / 1000000);
    esp_deep_sleep_start();
}


esp_err_t send_esp_now_roomtemp(void) {
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    //enable the SuperCap:
    gpio_set_level(PIN_SUPERCAP_ENABLE, 0);

    //Setup peer
    esp_now_peer_info_t peer = {
        .ifidx = ESP_IF_WIFI_STA,
        .channel = 0,
        .encrypt = false,
    };

    uint8_t peer_address[6];
    uint8_t peer_channel;
    esp_err_t err = getGatewayData(peer_address, sizeof(peer_address), &peer_channel);
    ESP_LOGD("ESP-Now", "Read peer data from NVS error code: %s", esp_err_to_name(err));
    memcpy(peer.peer_addr, peer_address, 6);
    peer.channel = peer_channel;
    ESP_LOGD("ESP-Now", "sending data to MAC address: %02X:%02X:%02X:%02X:%02X:%02X, on Channel %u", peer.peer_addr[0], peer.peer_addr[1], peer.peer_addr[2], peer.peer_addr[3], peer.peer_addr[4], peer.peer_addr[5], peer.channel);

    //Setup Wi-Fi in station mode for ESP-Now:
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(peer_channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    //register on send callback for ACK:
    esp_now_register_send_cb(onDataSent);

    //For debug, validate the Address and channel:

    //Set up the struct:
    Roomtemp_Message roomTemp_ESPNow = {
        .measurementType = ROOMTEMP,
        .index = burstNumber,
        .intervalTime = TIME_TO_SLEEP,
        .numberofMeasurements = RoomTempCount,
    };
    //Copy the read temperatures into the struct memory:
    memcpy(roomTemp_ESPNow.roomTemps, roomTemperatures, MAX_TEMP_SAMPLES * sizeof(roomTemperatures[0]));

    esp_err_t result = esp_now_send(peer_address, (uint8_t *)&roomTemp_ESPNow, sizeof(roomTemp_ESPNow));

    return result;
}

esp_err_t send_esp_now_co2(void) {
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    //enable the SuperCap:
    gpio_set_level(PIN_SUPERCAP_ENABLE, 0);

    //Setup peer
    esp_now_peer_info_t peer = {
        .ifidx = ESP_IF_WIFI_STA,
        .channel = 0,
        .encrypt = false,
    };

    uint8_t peer_address[6];
    uint8_t peer_channel;
    esp_err_t err = getGatewayData(peer_address, sizeof(peer_address), &peer_channel);
    ESP_LOGD("ESP-Now", "Read peer data from NVS error code: %s", esp_err_to_name(err));
    memcpy(peer.peer_addr, peer_address, 6);
    peer.channel = peer_channel;
    ESP_LOGD("ESP-Now", "sending data to MAC address: %02X:%02X:%02X:%02X:%02X:%02X, on Channel %u", peer.peer_addr[0], peer.peer_addr[1], peer.peer_addr[2], peer.peer_addr[3], peer.peer_addr[4], peer.peer_addr[5], peer.channel);

    //Setup Wi-Fi in station mode for ESP-Now:
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(peer_channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    //register on send callback for ACK:
    esp_now_register_send_cb(onDataSent);

    //For debug, validate the Address and channel:

    //Set up the struct:
    CO2_Message CO2_ESPNow = {
        .measurementType = CO2,
        .index = burstNumber,
        .intervalTime = TIME_TO_SLEEP,
        .numberofMeasurements = scd41Count,
    };
    //Copy the read temperatures into the struct memory:
    memcpy(CO2_ESPNow.co2ppm, scd41ppm, scd41Count);
    memcpy(CO2_ESPNow.co2temp, scd41temp, scd41Count);
    memcpy(CO2_ESPNow.co2humid, scd41hum, scd41Count);

    esp_err_t result = esp_now_send(peer_address, (uint8_t *)&CO2_ESPNow, sizeof(CO2_ESPNow));

    return result;
}

void onDataSent(const uint8_t *mac_address, esp_now_send_status_t status) {
    //de-init ESP-Now
    esp_now_unregister_send_cb();
    esp_now_deinit();
    esp_wifi_set_mode(WIFI_MODE_NULL);
    //Close supercap
    gpio_set_level(PIN_SUPERCAP_ENABLE, 1);

    ESP_LOGD("ESP-Now", "Received status %s", status ? "fail" : "success");

    //If transmission was successful, reset the RoomTempCount count:
    if (status == ESP_NOW_SEND_SUCCESS) {
        RoomTempCount = 0;
    }

    callbackFinished = true;
}

int pair_sensor(void) {
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGD("Pairing", "Entered pairing mode");
    //Get time for timeout:
    int64_t startTime = esp_timer_get_time(); //Get time since boot in us
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //Setup ESP-Now:
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_now_init());
    //Set the channel to 0
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_PAIRING_CHANNEL, WIFI_SECOND_CHAN_NONE));
    //register the data receive callback function:
    ESP_ERROR_CHECK(esp_now_register_recv_cb(onDataReceive));

    //a timeout will return the function:
    while ((startTime + PAIRING_TIMEOUT_uS > esp_timer_get_time())) {
        gpio_set_level(LED_STATUS, 1);
        vTaskDelay(500);
        gpio_set_level(LED_STATUS, 0);
        vTaskDelay(500);
    }
    return -1; //Exit with code -1 to indicate pairing fail
}