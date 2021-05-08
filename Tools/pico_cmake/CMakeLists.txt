set(PICO_PLATFORM "rp2040")

cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)

project(ardupilot_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(empty
    empty.cpp
)

pico_enable_stdio_uart(empty 1)

target_link_libraries(
    empty
    pico_stdlib
)

