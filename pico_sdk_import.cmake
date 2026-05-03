# This is a copy of <PICO_SDK_PATH>/external/pico_sdk_import.cmake
# Place the Pico SDK path in the PICO_SDK_PATH environment variable,
# or pass -DPICO_SDK_PATH=<path> to cmake.

if (DEFINED ENV{PICO_SDK_PATH} AND (NOT PICO_SDK_PATH))
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment: '${PICO_SDK_PATH}'")
endif ()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH not set. "
        "Pass -DPICO_SDK_PATH=<path> or set the PICO_SDK_PATH environment variable.")
endif ()

get_filename_component(PICO_SDK_PATH "${PICO_SDK_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Directory '${PICO_SDK_PATH}' not found")
endif ()

set(PICO_SDK_INIT_CMAKE_FILE ${PICO_SDK_PATH}/pico_sdk_init.cmake)
if (NOT EXISTS ${PICO_SDK_INIT_CMAKE_FILE})
    message(FATAL_ERROR "'${PICO_SDK_INIT_CMAKE_FILE}' does not exist. "
        "Is '${PICO_SDK_PATH}' a valid Pico SDK directory?")
endif ()

include(${PICO_SDK_INIT_CMAKE_FILE})
