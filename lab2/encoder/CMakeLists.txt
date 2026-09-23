# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(encoder C CXX ASM)

# initialize the sdk
pico_sdk_init()

# add executable
add_executable(encoder)

# add sources
target_sources(encoder PRIVATE encoder.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(encoder pico_stdlib pico_bootsel_via_double_reset)

# create map/bin/hex file etc.
pico_add_extra_outputs(encoder)