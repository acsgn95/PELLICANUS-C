cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pellicanus C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pellicanus
    src/main.c
    src/iam20680.c
    src/mmc5983ma.c
    src/ms5637.c
    src/sl871_gnss.c
)

target_include_directories(pellicanus PRIVATE include)

target_link_libraries(pellicanus
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_gpio
    hardware_timer
)

pico_enable_stdio_usb(pellicanus 1)
pico_enable_stdio_uart(pellicanus 0)

# pico_add_extra_outputs(pellicanus)  -- requires picotool; use flash.ps1 instead
