cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME H723VGT6_Demo)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings (必须在 project() 之后)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 现在可以安全地添加编译选项了（目标已存在）
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -fno-exceptions      # 禁用异常，减少Flash占用
    -fno-rtti           # 禁用运行时类型识别
    -Os                # 体积优化（调试时可以先注释）
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
# CMSIS DSP library for STM32H7 (Cortex-M7, little-endian, single-precision FPU)
set(CMSIS_DSP_LIB_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/ARM)
set(CMSIS_DSP_LIB     ${CMSIS_DSP_LIB_DIR}/arm_cortexM7lfsp_math.lib)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    ${CMSIS_DSP_LIB_DIR}
)

# Add sources to executable
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User_alg/*.cpp
    ${CMAKE_SOURCE_DIR}/User_alg/*.c
    ${CMAKE_SOURCE_DIR}/User_drv/*.cpp
    ${CMAKE_SOURCE_DIR}/User_drv/*.c
    ${CMAKE_SOURCE_DIR}/User_dvc/*.cpp
    ${CMAKE_SOURCE_DIR}/User_dvc/*.c
    ${CMAKE_SOURCE_DIR}/User_chariot/*.cpp
    ${CMAKE_SOURCE_DIR}/User_chariot/*.c
    ${CMAKE_SOURCE_DIR}/User_ita/*.cpp
    ${CMAKE_SOURCE_DIR}/User_ita/*.c
    ${CMAKE_SOURCE_DIR}/User_tsk/*.cpp
    ${CMAKE_SOURCE_DIR}/User_tsk/*.c
)

# drv_spi depends on CubeMX-generated spi.h/spi.c. Keep it out until SPI is
# enabled in the .ioc and Core/Inc/spi.h exists.
if(NOT EXISTS ${CMAKE_SOURCE_DIR}/Core/Inc/spi.h)
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User_drv/drv_spi\\.cpp$")
    list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User_drv/drv_spi\\.c$")
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/User_alg
    ${CMAKE_SOURCE_DIR}/User_drv
    ${CMAKE_SOURCE_DIR}/User_dvc
    ${CMAKE_SOURCE_DIR}/User_chariot
    ${CMAKE_SOURCE_DIR}/User_chariot/Chassis
    ${CMAKE_SOURCE_DIR}/User_chariot/KFS
    ${CMAKE_SOURCE_DIR}/User_chariot/Weapons
    ${CMAKE_SOURCE_DIR}/User_chariot/Lift
    ${CMAKE_SOURCE_DIR}/User_ita
    ${CMAKE_SOURCE_DIR}/User_tsk
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # CMSIS DSP library for STM32H7
    ${CMSIS_DSP_LIB}

    # Add user defined libraries
)