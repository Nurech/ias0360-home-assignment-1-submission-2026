cmake_minimum_required(VERSION 3.13)

# pico w. has to be set before the sdk import, the sdk picks the board as soon as it is included
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(ha1_imu C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(ha1_imu main.c)

# stdlib for printf/sleep/time, hardware_i2c for the icm-20948, m for sinf/cosf/sqrtf/log10f
target_link_libraries(ha1_imu pico_stdlib hardware_i2c m)
target_compile_options(ha1_imu PRIVATE -Wall -Wextra)

# usb serial only. it also gives picotool a reset interface, so "picotool load -f" works without the button
pico_enable_stdio_usb(ha1_imu 1)
pico_enable_stdio_uart(ha1_imu 0)

# makes ha1_imu.uf2 next to the .elf
pico_add_extra_outputs(ha1_imu)
