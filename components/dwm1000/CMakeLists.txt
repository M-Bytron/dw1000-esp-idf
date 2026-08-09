idf_component_register(
    SRCS "dwm1000.c"
    INCLUDE_DIRS "include"
    REQUIRES arduino_dw1000 esp_timer driver freertos
)
