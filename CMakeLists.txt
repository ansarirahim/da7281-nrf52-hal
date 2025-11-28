cmake_minimum_required(VERSION 3.15)

# ======================================================================
# Auto-select ARM toolchain if not provided
# ======================================================================

if (NOT CMAKE_C_COMPILER)
    set(ARM_TOOLCHAIN arm-none-eabi-gcc)
    set(ARM_ASM_TOOLCHAIN arm-none-eabi-gcc)

    message(STATUS "Using default ARM compiler: ${ARM_TOOLCHAIN}")

    # Tell CMake we are cross-compiling
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR ARM)

    # Direct compiler selection
    set(CMAKE_C_COMPILER   ${ARM_TOOLCHAIN})
    set(CMAKE_ASM_COMPILER ${ARM_ASM_TOOLCHAIN})

    # Needed to avoid running test binaries
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif()

# Set toolchain before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Toolchain settings
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# Skip compiler tests (embedded targets can't run test programs)
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# Project configuration
project(da7281_hal
    VERSION 1.0.0
    DESCRIPTION "DA7281 Haptic Driver HAL for nRF52833 with FreeRTOS"
    LANGUAGES C ASM
)

# Compiler settings
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-parameter")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

# nRF52833 specific flags
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${CPU_FLAGS}")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")

# nRF5 SDK path (set this to your SDK location)
set(NRF5_SDK_PATH "${CMAKE_SOURCE_DIR}/../nRF5_SDK" CACHE PATH "Path to nRF5 SDK")

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${NRF5_SDK_PATH}/components/drivers_nrf/nrf_soc_nosd
    ${NRF5_SDK_PATH}/components/libraries/util
    ${NRF5_SDK_PATH}/components/libraries/log
    ${NRF5_SDK_PATH}/components/libraries/log/src
    ${NRF5_SDK_PATH}/components/drivers_nrf/hal
    ${NRF5_SDK_PATH}/components/drivers_nrf/twi_master
    ${NRF5_SDK_PATH}/modules/nrfx
    ${NRF5_SDK_PATH}/modules/nrfx/hal
    ${NRF5_SDK_PATH}/modules/nrfx/mdk
    ${NRF5_SDK_PATH}/modules/nrfx/drivers/include
    ${NRF5_SDK_PATH}/external/freertos/source/include
    ${NRF5_SDK_PATH}/external/freertos/portable/GCC/nrf52
    ${NRF5_SDK_PATH}/external/freertos/portable/CMSIS/nrf52
    ${NRF5_SDK_PATH}/integration/nrfx
    ${NRF5_SDK_PATH}/components/toolchain/cmsis/include
)

# Preprocessor definitions
add_definitions(
    -DNRF52833_XXAA
    -DBOARD_CUSTOM
    -DCONFIG_GPIO_AS_PINRESET
    -DFLOAT_ABI_HARD
    -DFREERTOS
)

# Source files
set(HAL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/da7281.c
    ${CMAKE_SOURCE_DIR}/src/da7281_init.c
)

# Create static library
add_library(da7281_hal STATIC ${HAL_SOURCES})

target_include_directories(da7281_hal PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# Example executable (optional)
option(BUILD_EXAMPLES "Build example applications" OFF)

if(BUILD_EXAMPLES)
    set(EXAMPLE_SOURCES
        ${CMAKE_SOURCE_DIR}/examples/haptics_test_task.c
        # Add main.c and other required sources here
    )
    
    add_executable(haptics_example ${EXAMPLE_SOURCES})
    target_link_libraries(haptics_example da7281_hal)
endif()

# Installation
install(TARGETS da7281_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Documentation target
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

# Print configuration
message(STATUS "")
message(STATUS "DA7281 HAL Configuration:")
message(STATUS "  Version: ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C Flags: ${CMAKE_C_FLAGS}")
message(STATUS "  nRF5 SDK: ${NRF5_SDK_PATH}")
message(STATUS "")

