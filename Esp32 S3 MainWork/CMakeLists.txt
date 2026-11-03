cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/App"
    "${CMAKE_CURRENT_LIST_DIR}/components/BSP")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gear_rack_esp32s3_main)
