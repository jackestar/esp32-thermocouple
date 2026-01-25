idf_component_register(SRCS "esp32-thermocouple.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc esp_driver_dac)