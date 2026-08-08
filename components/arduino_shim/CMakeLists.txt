idf_component_register(
    SRCS "src/Arduino.cpp" "src/SPI.cpp" "src/Interrupt.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_rom freertos soc
)
