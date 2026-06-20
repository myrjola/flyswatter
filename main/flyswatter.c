#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO 8   /* onboard LED on most ESP32-C3 boards */

void app_main(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    bool on = false;
    while (1) {
        on = !on;
        gpio_set_level(BLINK_GPIO, on);
        printf("LED %s\n", on ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
