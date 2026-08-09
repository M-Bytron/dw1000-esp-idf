# Pure-C DW1000 component library.
#
# This component provides a C-only API for the Decawave DW1000 / DWM1000
# UWB transceiver. The actual register-level driver is the C++ arduino-dw1000
# library (components/arduino_dw1000), reached through its C bridge header
# dw1000_c.h. All functions exposed here are plain C - no classes, no objects.
#
# To reuse this component in another project, copy the components/
# arduino_dw1000, arduino_shim and DW1000 folders into that project.
idf_component_register(
    SRCS "my_dw1000.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES arduino_dw1000
)
