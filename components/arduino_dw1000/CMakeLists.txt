idf_component_register(
    SRCS "src/DW1000.cpp" "src/DW1000Time.cpp" "src/dw1000_c.cpp"
    INCLUDE_DIRS "src" "include"
    REQUIRES arduino_shim
)

# The arduino-dw1000 library uses `#if defined(ESP32)` to select the custom
# SPI-pin code path and to skip Arduino-only bits (e.g. SPI.usingInterrupt).
# The Arduino core defines ESP32, so we do the same here.
target_compile_definitions(${COMPONENT_LIB} PRIVATE "ESP32=1")

# This is vendored third-party code built with ESP-IDF's strict -Werror flags.
# Its debug pretty-print helpers trigger format/truncation warnings that are
# not errors in the original Arduino build, so downgrade them here.
target_compile_options(${COMPONENT_LIB} PRIVATE
    "-Wno-error=format-truncation"
    "-Wno-error=format-overflow"
    "-Wno-error=maybe-uninitialized"
)
