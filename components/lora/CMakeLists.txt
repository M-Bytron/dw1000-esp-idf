idf_component_register(SRCS "lora.c"
                    INCLUDE_DIRS "include"
                    REQUIRES sx127x packet device_identity driver esp_driver_spi esp_driver_gpio nvs_flash freertos esp_timer esp_system esp_hw_support)
