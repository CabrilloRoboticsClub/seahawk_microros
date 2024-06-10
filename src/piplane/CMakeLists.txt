cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(piplane C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


# add_library(dshot INTERFACE)

# target_sources(dshot INTERFACE ${CMAKE_CURRENT_LIST_DIR}/dshot.c)
# target_includpico_generate_pio_header(piplane ${CMAKE_CURRENT_LIST_DIR}/dshot_encoder.pio)
# e_directories(dshot INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

link_directories($ENV{MICRO_ROS_RPI_PICO_PATH}/libmicroros)

add_executable(piplane
    piplane.c
    dshot.c
    pico_uart_transport.c
)
target_link_libraries(piplane
    pico_stdlib
    microros
    hardware_pwm
    hardware_pio
    pico_multicore
)

target_include_directories(piplane PUBLIC
    $ENV{MICRO_ROS_RPI_PICO_PATH}/libmicroros/include
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(piplane 1)
pico_enable_stdio_uart(piplane 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(piplane)

pico_generate_pio_header(piplane ${CMAKE_CURRENT_LIST_DIR}/dshot.pio)

