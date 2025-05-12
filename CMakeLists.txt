# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated CMake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(temperatura_display_dma C CXX ASM)

pico_sdk_init()

add_executable(temperatura_display_dma 
    temperatura_display_dma.c 
    inc/ssd1306_i2c.c  # Inclua todos os .c necessários da biblioteca ssd1306
)

pico_set_program_name(temperatura_display_dma "temperatura_display_dma")
pico_set_program_version(temperatura_display_dma "0.1")

pico_enable_stdio_uart(temperatura_display_dma 0)
pico_enable_stdio_usb(temperatura_display_dma 1)

target_include_directories(temperatura_display_dma PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR} 
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

target_link_libraries(temperatura_display_dma 
    pico_stdlib
    hardware_adc
    hardware_dma
    hardware_i2c
    hardware_gpio
    hardware_clocks
)

pico_add_extra_outputs(temperatura_display_dma)
