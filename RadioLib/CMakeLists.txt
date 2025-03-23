cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

add_definitions(-DRADIOLIB_STATIC_ONLY)
enable_language(C CXX ASM)

add_library(RadioLib STATIC
    Hal.cpp
    stm32RadioHal.cpp
    Module.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/SX126x/SX1262.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/SX126x/SX126x.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/SX126x/SX1261.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/SX126x/SX126x_LR_FHSS.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols/PhysicalLayer/PhysicalLayer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/CRC.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/FEC.cpp
    )

target_include_directories(RadioLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/SX126x
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols/PhysicalLayer
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
)

target_link_libraries(RadioLib PUBLIC
    stm32cubemx
)