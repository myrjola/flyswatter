idf_component_register(SRCS "flyswatter.c"
                    INCLUDE_DIRS "."
                    REQUIRES led_strip esp_adc esp_driver_gpio esp_timer)
