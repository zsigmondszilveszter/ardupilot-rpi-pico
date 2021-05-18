set(PICO_PLATFORM "rp2040")

cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)

project(empty C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

if (TARGET tinyusb_device)
    add_executable(empty
        empty.cpp
    )

    pico_enable_stdio_uart(empty 0)
    # enable pico stdio over USB in order to link and include every required libraries and files
    # but we don't initialize and use the Pico built in stdio
    pico_enable_stdio_usb(empty 1)

    target_link_libraries(
        empty
        pico_stdlib
    )
elseif(PICO_ON_DEVICE)
    message(WARNING "not building ardupilot-rpi-pico because TinyUSB submodule is not initialized in the SDK")
endif()


