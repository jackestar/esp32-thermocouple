idf_component_register(SRCS "esp32-thermocouple.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc esp_driver_dac esp_wifi nvs_flash esp_http_server) 