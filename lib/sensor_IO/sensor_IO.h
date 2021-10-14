#ifndef _SENSOR_IO_H
#define _SENSOR_IO_H

#include "driver/gpio.h"
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//IO:
//Inputs
#define BUTTON_P1           GPIO_NUM_0         //Button P1 on GPIO0
#define BUTTON_P2           GPIO_NUM_15        //Button P2 on GPIO15
#define INPUT_BITMASK       (1ULL << BUTTON_P2) | (1ULL << BUTTON_P1)
//Outputs:
#define PIN_SUPERCAP_ENABLE GPIO_NUM_27
#define LED_ERROR           GPIO_NUM_14        //Error LED on GPIO 14
#define LED_STATUS          GPIO_NUM_12       //Status LED on GPIO12
#define OUTPUT_BITMASK       (1ULL << PIN_SUPERCAP_ENABLE) | (1ULL << LED_ERROR) | (1ULL << LED_STATUS)

/**
 * @brief init the GPIO with the predefined settings
 */
void twomes_init_gpio();


/**
 * @brief Blink LEDs
 * Pass two arguments in uint8_t array:
 * @param argument[0]  amount of blinks
 * @param argument[1]  pin to blink on (LED_STATUS or LED_ERROR)
 */
void blink(void *args);


#endif //_SENSOR_IO_H