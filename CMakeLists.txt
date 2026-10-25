cmake_minimum_required(VERSION 3.15)

# ======================================================================
# Build modes
#   default            : cross-compile the library for the nRF52833 (arm-none-eabi-gcc)
#   -DDA7281_HOST_TESTS=ON : native build + host tests against the register model (test/)
# ======================================================================
option(DA7281_HOST_TESTS "Build natively and run the host tests against the DA7281 register model" OFF)
option(BUILD_EXAMPLES "Compile the example application code" OFF)

if(NOT DA7281_HOST_TESTS AND NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_AR arm-none-eabi-ar)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_SIZE arm-none-eabi-size)
    # Embedded targets cannot run test programs during configuration
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
endif()

project(da7281_hal
    VERSION 2.0.0
    DESCRIPTION "DA7281 Haptic Driver HAL for nRF52833 with FreeRTOS"
    LANGUAGES C
)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(DA7281_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Werror)

# ======================================================================
# nRF5 SDK and FreeRTOS headers
# ======================================================================
set(NRF5_SDK_PATH "${CMAKE_SOURCE_DIR}/../nRF5_SDK" CACHE PATH "Path to nRF5 SDK")
set(DA7281_SDK_CONFIG_DIR "" CACHE PATH "Directory with your application's sdk_config.h and FreeRTOSConfig.h")

if(DA7281_HOST_TESTS OR NOT EXISTS "${NRF5_SDK_PATH}/components")
    if(NOT DA7281_HOST_TESTS)
        message(STATUS "nRF5 SDK not found at ${NRF5_SDK_PATH}: compiling against the API stand-ins in test/stubs (compile check only)")
    endif()
    set(DA7281_SDK_INCLUDES ${CMAKE_SOURCE_DIR}/test/stubs)
else()
    set(DA7281_SDK_INCLUDES
        ${DA7281_SDK_CONFIG_DIR}
        ${NRF5_SDK_PATH}/components/libraries/util
        ${NRF5_SDK_PATH}/components/libraries/log
        ${NRF5_SDK_PATH}/components/libraries/log/src
        ${NRF5_SDK_PATH}/components/libraries/experimental_section_vars
        ${NRF5_SDK_PATH}/components/libraries/strerror
        ${NRF5_SDK_PATH}/components/drivers_nrf/nrf_soc_nosd
        ${NRF5_SDK_PATH}/integration/nrfx
        ${NRF5_SDK_PATH}/integration/nrfx/legacy
        ${NRF5_SDK_PATH}/modules/nrfx
        ${NRF5_SDK_PATH}/modules/nrfx/hal
        ${NRF5_SDK_PATH}/modules/nrfx/mdk
        ${NRF5_SDK_PATH}/modules/nrfx/drivers/include
        ${NRF5_SDK_PATH}/external/freertos/source/include
        ${NRF5_SDK_PATH}/external/freertos/portable/GCC/nrf52
        ${NRF5_SDK_PATH}/external/freertos/portable/CMSIS/nrf52
        ${NRF5_SDK_PATH}/components/toolchain/cmsis/include
    )
endif()

set(HAL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/da7281.c
    ${CMAKE_SOURCE_DIR}/src/da7281_init.c
)

# ======================================================================
# Library
# ======================================================================
add_library(da7281_hal STATIC ${HAL_SOURCES})
target_include_directories(da7281_hal PUBLIC ${CMAKE_SOURCE_DIR}/include ${DA7281_SDK_INCLUDES})
target_compile_options(da7281_hal PRIVATE ${DA7281_WARNINGS})

if(NOT DA7281_HOST_TESTS)
    # nRF52833: Cortex-M4F with single-precision hardware floating point
    set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfloat-abi=hard -mfpu=fpv4-sp-d16)
    target_compile_options(da7281_hal PUBLIC ${CPU_FLAGS} -ffunction-sections -fdata-sections)
    target_compile_definitions(da7281_hal PUBLIC NRF52833_XXAA FLOAT_ABI_HARD FREERTOS)
endif()

if(BUILD_EXAMPLES)
    # Object library: the example has no main(); this checks that it compiles against the API
    add_library(haptics_example OBJECT ${CMAKE_SOURCE_DIR}/examples/haptics_test_task.c)
    target_link_libraries(haptics_example PRIVATE da7281_hal)
    target_compile_options(haptics_example PRIVATE ${DA7281_WARNINGS})
endif()

# ======================================================================
# Host tests: the HAL against a register model of the DA7281 datasheet, at three tick rates
# ======================================================================
if(DA7281_HOST_TESTS)
    enable_testing()
    foreach(rate 1024 1000 100)
        add_executable(test_da7281_${rate} ${CMAKE_SOURCE_DIR}/test/test_da7281.c ${CMAKE_SOURCE_DIR}/test/da7281_model.c ${HAL_SOURCES})
        target_include_directories(test_da7281_${rate} PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/test ${CMAKE_SOURCE_DIR}/test/stubs)
        target_compile_definitions(test_da7281_${rate} PRIVATE configTICK_RATE_HZ=${rate}U)
        target_compile_options(test_da7281_${rate} PRIVATE -Wall -Wextra -Werror)
        add_test(NAME da7281_tick_${rate} COMMAND test_da7281_${rate})
    endforeach()
endif()

# ======================================================================
# Installation and documentation
# ======================================================================
install(TARGETS da7281_hal ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include FILES_MATCHING PATTERN "*.h")

find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

message(STATUS "DA7281 HAL ${PROJECT_VERSION}: ${CMAKE_C_COMPILER} (host tests: ${DA7281_HOST_TESTS})")
