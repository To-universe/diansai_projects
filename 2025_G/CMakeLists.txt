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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 2025_G)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/Src/*.c
)

add_library(app OBJECT ${APP_SOURCES})
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/APP/Inc
)
target_link_libraries(app PUBLIC
    stm32cubemx
)

set(CMSIS_DSP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP)
if(EXISTS ${CMSIS_DSP_ROOT}/Source)
    file(GLOB_RECURSE CMSIS_DSP_SOURCES CONFIGURE_DEPENDS
        ${CMSIS_DSP_ROOT}/Source/*.c
    )
    list(FILTER CMSIS_DSP_SOURCES INCLUDE REGEX ".*/arm_.*\\.c$")
    list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*[Ff]16.*\\.c$")
    list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*[Ff]64.*\\.c$")
    list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*[Nn][Ee][Oo][Nn].*\\.c$")
    list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*[Mm][Vv][Ee].*\\.c$")
    list(FILTER CMSIS_DSP_SOURCES EXCLUDE REGEX ".*/_.*\\.c$")

    add_library(cmsis_dsp OBJECT ${CMSIS_DSP_SOURCES})
    target_include_directories(cmsis_dsp PUBLIC
        ${CMSIS_DSP_ROOT}/Include
        ${CMSIS_DSP_ROOT}/PrivateInclude
    )
    target_compile_definitions(cmsis_dsp PUBLIC
        ARM_MATH_CM4
        ARM_MATH_LOOPUNROLL
    )
    target_link_libraries(cmsis_dsp PUBLIC
        stm32cubemx
    )
else()
    message(WARNING "CMSIS-DSP source directory not found: ${CMSIS_DSP_ROOT}/Source")
endif()

if(TARGET cmsis_dsp)
    target_link_libraries(app PUBLIC
        cmsis_dsp
    )
endif()

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    app
    $<$<TARGET_EXISTS:cmsis_dsp>:cmsis_dsp>

    # Add user defined libraries
)
