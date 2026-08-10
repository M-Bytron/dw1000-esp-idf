# Device-B example: ANCHOR (ranging responder).
#
# This is a complete, standalone ESP-IDF project. It reuses the DW1000
# component from the repository root:
#
#   set(EXTRA_COMPONENT_DIRS ../../components)
#
# Build it with:
#   cd Examples/Device-B
#   idf.py set-target esp32
#   idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../../components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(device_b)
