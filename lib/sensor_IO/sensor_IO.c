#include "sensor_IO.h"

void twomes_init_gpio() {
    gpio_config_t io_conf;
    //CONFIGURE OUTPUTS:
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = OUTPUT_BITMASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    //CONFIGURE INPUTS:
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = INPUT_BITMASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}


/**
 * @brief Blink LEDs
 * Pass two arguments in uint8_t array:
 * @param argument[0]  amount of blinks
 * @param argument[1]  pin to blink on (GREEN_LED_ERROR_D2 or RED_LED_ERROR_D1)
 */
void blink(void *args) {
    uint8_t *arguments = (uint8_t *)args;
    uint8_t amount = arguments[0];
    uint8_t pin = arguments[1];
    uint8_t i;
    for (i = 0; i < amount; i++) {
        gpio_set_level((gpio_num_t)pin, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_set_level((gpio_num_t)pin, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    } //for(i<amount)
    //Delete the blink task after completion:
    vTaskDelete(NULL);
} //void blink;