# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Configure the build for the Raspberry Pi Pico 2 (RP2350)
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Define the project name
project(adc_dds_augmented C CXX ASM)

# Add the single, unified executable compiled from your merged code
add_executable(adc_dds_augmented
    ADC_demo.c
)

# Pull in all dependencies for BOTH Protothreads (clocks/sync) and DDS (ADC/SPI/Timer/IRQ)
target_link_libraries(adc_dds_augmented 
    pico_stdlib 
    pico_sync
    hardware_sync
    hardware_clocks
    hardware_vreg
    hardware_adc                    # Required to read the slide potentiometer
    hardware_spi                    # Required to write to the MCP4802/MCP4822 DAC
    hardware_timer                  # Required for the 50 kHz timer interrupt 
    hardware_irq                    # Required to register the exclusive alarm ISR
    pico_bootsel_via_double_reset   # Standard course bootloader utility
)

# Enable extra outputs (generate .bin, .hex, and .uf2 files to load onto the board)
pico_add_extra_outputs(adc_dds_augmented)