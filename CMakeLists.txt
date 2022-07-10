cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()
project(segement_controller_project)
enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(segment_controller
        segment_controller.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(segment_controller
        pico_stdlib
        hardware_i2c
        hardware_spi
        hardware_irq
        pico_multicore)

# create map/bin/hex file etc.
pico_add_extra_outputs(segment_controller)

pico_enable_stdio_usb(segment_controller 1)
pico_enable_stdio_uart(segment_controller 0)