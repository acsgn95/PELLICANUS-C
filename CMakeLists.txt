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
    src/imu_stream.c
)

target_include_directories(pellicanus PRIVATE include)

target_link_libraries(pellicanus
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_uart
    hardware_gpio
    hardware_timer
)

pico_enable_stdio_usb(pellicanus 1)
pico_enable_stdio_uart(pellicanus 0)

# The UF2 and the other outputs need picotool, which the SDK fetches and builds when it is not
# installed (it needs libusb-1.0). On Windows flash.ps1 makes the UF2 instead.
if(NOT CMAKE_HOST_WIN32)
    pico_add_extra_outputs(pellicanus)
endif()
