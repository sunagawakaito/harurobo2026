cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_CXX_STANDARD 17)  # C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME NUCLEO_446_board_ver1.x_template_vscode)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and CXX languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Remove main.c entries added by the generated stm32cubemx subdirectory. (and main.cpp will be added later instead.)
# It is done here to avoid editing the generated `cmake/stm32cubemx/CMakeLists.txt` file.
get_target_property(_proj_sources ${CMAKE_PROJECT_NAME} SOURCES)
if(_proj_sources)
    set(_filtered_sources)
    foreach(_s IN LISTS _proj_sources)
        # Remove any Core/Src/main.c
        if(_s MATCHES ".*Core[/\\]Src[/\\]main\.c$")
            message(STATUS "Excluding generated main from build: ${_s}")
        else()
            list(APPEND _filtered_sources "${_s}")
        endif()
    endforeach()
    # Replace the target sources with the filtered list
    set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY SOURCES "${_filtered_sources}")
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.cpp  # Added instead of generated main.c entry.

    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)

# Add Library_KURT_STM32 as a subdirectory (modular library)
set(KURT_ENABLE_CAN ON)
set(KURT_ENABLE_I2C ON)
set(KURT_ENABLE_UART ON)
set(KURT_ENABLE_TIM ON)
set(KURT_BOARD_NUCLEO_F446 ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Library_KURT_STM32)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    kurt_library
)

# Set the linker flags for printf float support
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
