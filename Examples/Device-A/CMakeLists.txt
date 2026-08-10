# Device-A example: TAG (ranging initiator).
#
# This is a complete, standalone ESP-IDF project. It reuses the DW1000
# component from the repository root:
#
#   set(EXTRA_COMPONENT_DIRS ../../components)
#
# Build it with:
#   cd Examples/Device-A
#   idf.py set-target esp32
#   idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ../../components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(device_a)
