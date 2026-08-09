# Pure-C DW1000 component library.
#
# Fully self-contained: the driver (dw1000_lowlevel.c) and the public API
# (my_dw1000.c) are plain C on top of the ESP-IDF SPI / GPIO drivers. No C++,
# no external library.
#
# To reuse this component in another project, copy this DW1000 folder into
# that project and add "DW1000" to its REQUIRES list.
idf_component_register(
    SRCS "my_dw1000.c" "dw1000_lowlevel.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_hw_support
)
