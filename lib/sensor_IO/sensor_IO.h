#ifndef _SENSOR_IO_H
#define _SENSOR_IO_H

#include "driver/gpio.h"
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//IO:
//Inputs
#define BUTTON_GPIO_SW2          GPIO_NUM_0         //Button SW2 on GPIO0
#define BUTTON_GPIO15_SW3        GPIO_NUM_15        //Button SW3 on GPIO15
#define INPUT_BITMASK       (1ULL << BUTTON_GPIO15_SW3) | (1ULL << BUTTON_GPIO_SW2)
//Outputs:
#define PIN_SUPERCAP_ENABLE GPIO_NUM_27
#define RED_LED_ERROR_D1           GPIO_NUM_14        //Error LED on GPIO 14
#define GREEN_LED_STATUS_D2          GPIO_NUM_12       //Status LED on GPIO12
#define OUTPUT_BITMASK       (1ULL << PIN_SUPERCAP_ENABLE) | (1ULL << RED_LED_ERROR_D1) | (1ULL << GREEN_LED_STATUS_D2)

/**
 * @brief init the GPIO with the predefined settings
 */
void twomes_init_gpio();


/**
 * @brief Blink LEDs
 * Pass two arguments in uint8_t array:
 * @param argument[0]  amount of blinks
 * @param argument[1]  pin to blink on (GREEN_LED_STATUS_D2 or RED_LED_ERROR_D1)
 */
void blink(void *args);


#endif //_SENSOR_IO_H