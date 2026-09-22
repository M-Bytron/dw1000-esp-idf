idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES DW1000 device_identity sx127x lora packet
)
